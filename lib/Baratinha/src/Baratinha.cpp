#include "Baratinha.h"

#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <stdarg.h>
#include <stdio.h>

namespace {
constexpr char kDefaultApSsid[] = "Baratinha-OTA";
constexpr char kDefaultApPassword[] = "12345678";
}

Baratinha::Baratinha()
    : _tof(),
      _leds{},
      _tofReady(false),
      _running(true),
      _telemetryEnabled(false),
      _telemetryDivider(5),
      _telemetryCounter(0),
      _buttonLast(0),
      _buttonCurrent(0),
      _lastControlMicros(0),
      _controlIntervalUs(kDefaultControlIntervalUs),
      _otaEnabled(true),
      _otaConfigured(false),
      _telnetServer(23),
      _telnetClient(),
      _telnetEnabled(true),
      _telnetServerActive(false),
      _telnetPort(23),
      _otaInProgress(false),
      _otaAnimationIndex(0),
      _staSsid(),
      _staPassword(),
      _apSsid(kDefaultApSsid),
      _apPassword(kDefaultApPassword),
      _preferAccessPoint(false) {}

void Baratinha::beginSerial(uint32_t baud) {
  Serial.begin(baud);
}

bool Baratinha::setupAll(uint32_t serialBaud,
                         uint8_t ledBrightness,
                         uint32_t pwmFrequency,
                         uint8_t pwmResolutionBits,
                         uint8_t tofSda,
                         uint8_t tofScl,
                         uint16_t tofTimeoutMs) {
  beginSerial(serialBaud);
  Serial.println(F("[Baratinha] Iniciando setup completo."));

  setupLeds(ledBrightness);
  Serial.println(F("[Baratinha] LEDs configurados."));

  setupMotors(pwmFrequency, pwmResolutionBits);
  Serial.println(F("[Baratinha] Motores configurados."));

  setupButtons();
  Serial.println(F("[Baratinha] Botao pronto."));

  if (!setupTOF(tofSda, tofScl, tofTimeoutMs)) {
    Serial.println(F("[Baratinha] ERRO: Sensor ToF nao respondeu."));
    setAllLeds(0, 255, 40);
    FastLED.show();
    if (_otaEnabled) {
      autoConfigureOTA();
    }
    return false;
  }

  Serial.println(F("[Baratinha] Sensor ToF pronto."));
  setAllLeds(96, 255, 80);
  FastLED.show();
  Serial.println(F("[Baratinha] Setup concluido."));

  if (_otaEnabled) {
    autoConfigureOTA();
  }
  return true;
}

bool Baratinha::setupTOF(uint8_t sda, uint8_t scl, uint16_t timeoutMs) {
  Wire.begin(sda, scl);
  _tof.setTimeout(timeoutMs);
  if (!_tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    _tofReady = false;
    return false;
  }

  _tof.startContinuous();
  _tofReady = true;
  return true;
}

void Baratinha::setupLeds(uint8_t brightness) {
  FastLED.addLeds<WS2812B, kLedDataPin, RGB>(_leds, kNumLeds);
  FastLED.setBrightness(brightness);
  FastLED.clear();
  FastLED.show();
}

void Baratinha::setupMotors(uint32_t pwmFrequency, uint8_t pwmResolutionBits) {
  ledcSetup(kPwmChannelM1, pwmFrequency, pwmResolutionBits);
  ledcAttachPin(kPwmM1Pin, kPwmChannelM1);

  ledcSetup(kPwmChannelM2, pwmFrequency, pwmResolutionBits);
  ledcAttachPin(kPwmM2Pin, kPwmChannelM2);

  gpio_set_direction(kIn1, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn2, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn3, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction(kIn4, GPIO_MODE_INPUT_OUTPUT);
}

void Baratinha::setupButtons() {
  gpio_set_direction(kButtonPin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(kButtonPin, GPIO_PULLDOWN_ONLY);
  gpio_pulldown_en(kButtonPin);

  _buttonCurrent = readButtonRaw();
  _buttonLast = _buttonCurrent;
}

void Baratinha::awaitStart(uint16_t pollDelayMs) {
  applyBootAnimation();

  Serial.println(F("Aguardando inicio"));
  setAllLeds(0, 255, 30);
  FastLED.show();

  _buttonCurrent = readButtonRaw();
  _buttonLast = _buttonCurrent;

  while (!(_buttonLast < kButtonLowThreshold && _buttonCurrent > kButtonHighThreshold)) {
    delay(pollDelayMs);
    _buttonLast = _buttonCurrent;
    _buttonCurrent = readButtonRaw();
    Serial.println(_buttonCurrent);
    processOTA();
  }

  applyResumeAnimation();
  Serial.println(F("Iniciou"));

  setAllLeds(120, 255, 30);
  FastLED.show();
  Serial.println(F("Fim setup."));

  _running = true;
  resetControlTick();
}

void Baratinha::updateStartStop() {
  _buttonLast = _buttonCurrent;
  _buttonCurrent = readButtonRaw();
  updateStartStopState(_buttonCurrent);
  processOTA();
}

bool Baratinha::isRunning() const {
  return _running;
}

void Baratinha::setRunning(bool running) {
  if (_running == running) {
    return;
  }

  _running = running;
  if (_running) {
    applyResumeAnimation();
    Serial.println(F("Despausou"));
    resetControlTick();
  } else {
    applyPauseIndicator();
    stop();
    Serial.println(F("Pausou"));
  }
}

bool Baratinha::controlTickDue() {
  const uint32_t now = micros();

  if ((now - _lastControlMicros) >= _controlIntervalUs) {
    // Ajusta o contador interno para evitar drift excessivo.
    _lastControlMicros += _controlIntervalUs;
    if ((now - _lastControlMicros) >= _controlIntervalUs) {
      _lastControlMicros = now;
    }
    return true;
  }
  return false;
}

void Baratinha::setControlIntervalUs(uint32_t intervalUs) {
  if (intervalUs == 0) {
    intervalUs = kDefaultControlIntervalUs;
  }
  _controlIntervalUs = intervalUs;
  resetControlTick();
}

void Baratinha::setControlInterval(float seconds) {
  if (!(seconds > 0.0f)) {
    setControlIntervalUs(kDefaultControlIntervalUs);
    return;
  }
  const uint32_t intervalUs = static_cast<uint32_t>(seconds * 1000000.0f);
  setControlIntervalUs(intervalUs);
}

uint32_t Baratinha::controlIntervalUs() const {
  return _controlIntervalUs;
}

float Baratinha::controlPeriodSeconds() const {
  return static_cast<float>(_controlIntervalUs) / 1000000.0f;
}

float Baratinha::readDistance() {
  if (!_tofReady) {
    return 0;
  }
  float distance = static_cast<float>(_tof.readRangeContinuousMillimeters());
  return distance < 0.0f ? 0.0f : distance;
}

void Baratinha::move1D(int pwm, bool light) {
  const int magnitude = abs(pwm);
  const int bright = (magnitude < 20) ? 0 : magnitude;

  if (light) {
    if (pwm < 0) {
      _leds[0] = CHSV(200, 255, bright);
      _leds[3] = CHSV(200, 255, 0);
      _leds[1] = CHSV(200, 255, bright);
      _leds[2] = CHSV(200, 255, 0);
    } else {
      _leds[0] = CHSV(100, 255, 0);
      _leds[3] = CHSV(100, 255, bright);
      _leds[1] = CHSV(100, 255, 0);
      _leds[2] = CHSV(100, 255, bright);
    }
    FastLED.show();
  }

  motorE_PWM(pwm);
  motorD_PWM(pwm);
}

void Baratinha::stop() {
  motorE_PWM(0);
  motorD_PWM(0);
}

void Baratinha::setColor(char ledId, uint8_t h, uint8_t s, uint8_t v) {
  switch (ledId) {
    case '0':
    case 0:
      _leds[0] = CHSV(h, s, v);
      break;
    case '1':
    case 1:
      _leds[1] = CHSV(h, s, v);
      break;
    case '2':
    case 2:
      _leds[2] = CHSV(h, s, v);
      break;
    case '3':
    case 3:
      _leds[3] = CHSV(h, s, v);
      break;
    case 'a':
    default:
      setAllLeds(h, s, v);
      break;
  }

  FastLED.show();
}

void Baratinha::enableTelemetry(bool enable) {
  _telemetryEnabled = enable;
}

void Baratinha::setTelemetryDivider(uint8_t divider) {
  if (divider == 0) {
    divider = 1;
  }
  _telemetryDivider = divider;
  _telemetryCounter = 0;
}

bool Baratinha::setupOTAStation(const char* ssid, const char* password) {
  _otaConfigured = false;
  if (ssid == nullptr || password == nullptr) {
    Serial.println(F("Credenciais invalidas para OTA station."));
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print(F("Conectando OTA (station)"));
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Falha ao conectar a rede WiFi para OTA."));
    return false;
  }

  configureOTAHandlers();
  ArduinoOTA.begin();
  _otaConfigured = true;
  startTelnetServer(_telnetPort);

  Serial.print(F("OTA pronto. IP: "));
  Serial.println(WiFi.localIP());
  return true;
}

bool Baratinha::setupOTAAccessPoint(const char* ssid, const char* password) {
  _otaConfigured = false;
  const char* apSsid = (ssid != nullptr && ssid[0] != '\0') ? ssid : kDefaultApSsid;
  const char* apPassword = (password != nullptr && password[0] != '\0') ? password : kDefaultApPassword;

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(apSsid, apPassword)) {
    Serial.println(F("Falha ao iniciar access point para OTA."));
    return false;
  }

  configureOTAHandlers();
  ArduinoOTA.begin();
  _otaConfigured = true;
  startTelnetServer(_telnetPort);

  Serial.print(F("OTA AP pronto. IP: "));
  Serial.println(WiFi.softAPIP());
  return true;
}

void Baratinha::enableOTA(bool enable) {
  if (_otaEnabled == enable) {
    return;
  }
  _otaEnabled = enable;
  if (!enable) {
    _otaInProgress = false;
    _otaConfigured = false;
    if (_telnetClient) {
      _telnetClient.stop();
    }
    if (_telnetServerActive) {
      _telnetServer.stop();
      _telnetServerActive = false;
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    autoConfigureOTA();
  }
}

void Baratinha::enableTelnetTelemetry(bool enable, uint16_t port) {
  _telnetEnabled = enable;
  _telnetPort = port;

  if (!enable) {
    if (_telnetClient) {
      _telnetClient.stop();
    }
    if (_telnetServerActive) {
      _telnetServer.stop();
      _telnetServerActive = false;
    }
    return;
  }

  if (_otaConfigured) {
    startTelnetServer(_telnetPort);
  }
}

void Baratinha::setStationCredentials(const char* ssid, const char* password) {
  _staSsid = (ssid != nullptr) ? ssid : "";
  _staPassword = (password != nullptr) ? password : "";
}

void Baratinha::setAccessPointCredentials(const char* ssid, const char* password) {
  _apSsid = (ssid != nullptr && ssid[0] != '\0') ? ssid : kDefaultApSsid;
  _apPassword = (password != nullptr && password[0] != '\0') ? password : kDefaultApPassword;
}

void Baratinha::setPreferAccessPoint(bool prefer) {
  _preferAccessPoint = prefer;
}

void Baratinha::println() {
  Serial.println();
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    _telnetClient.println();
  }
}

void Baratinha::printf(const char* format, ...) {
  if (format == nullptr) {
    return;
  }

  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  broadcastRaw(buffer);
}

bool Baratinha::configureOTA(bool preferAccessPoint,
                             const char* staSsid,
                             const char* staPassword,
                             const char* apSsid,
                             const char* apPassword) {
  bool success = false;
  const bool hasStationCreds = (staSsid != nullptr && staSsid[0] != '\0' &&
                                staPassword != nullptr && staPassword[0] != '\0');

  if (preferAccessPoint) {
    println(F("Preferencia: Access Point."));
    success = setupOTAAccessPoint(apSsid, apPassword);
    if (!success && hasStationCreds) {
      println(F("Falha no modo AP. Tentando conectar em modo station..."));
      success = setupOTAStation(staSsid, staPassword);
    }
  } else {
    println(F("Preferencia: Station."));
    if (hasStationCreds) {
      success = setupOTAStation(staSsid, staPassword);
    } else {
      println(F("Credenciais de station ausentes."));
    }
    if (!success) {
      println(F("Alternando para Access Point."));
      success = setupOTAAccessPoint(apSsid, apPassword);
    }
  }

  if (!success) {
    println(F("Nao foi possivel iniciar OTA em nenhum modo."));
  }
  return success;
}

void Baratinha::configureOTAHandlers() {
  ArduinoOTA.onStart([this]() {
    Serial.println(F("OTA comecou"));
    _otaInProgress = true;
    _otaAnimationIndex = 0;
    showOTAStartAnimation();
  });
  ArduinoOTA.onEnd([this]() {
    Serial.println(F("OTA finalizado"));
    _otaInProgress = false;
    showOTAEndAnimation();
  });
  ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
    const unsigned int percent = (total == 0) ? 0 : (progress * 100U / total);
    Serial.print(F("OTA progresso: "));
    Serial.print(percent);
    Serial.println(F("%"));
    showOTAProgressAnimation(progress, total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print(F("Erro OTA: "));
    Serial.println(static_cast<int>(error));
  });
}

void Baratinha::emitTelemetry(float setpoint, float measurement, float error,
                              float controlRaw, float controlLimited,
                              float pTerm, float iTerm, float dTerm) {
  if (!_telemetryEnabled) {
    return;
  }

  if (++_telemetryCounter < _telemetryDivider) {
    return;
  }
  _telemetryCounter = 0;

  auto emitTo = [&](Print& out) {
    out.print(millis());
    out.print(',');
    out.print(setpoint);
    out.print(',');
    out.print(measurement);
    out.print(',');
    out.print(error);
    out.print(',');
    out.print(controlRaw);
    out.print(',');
    out.print(controlLimited);
    out.print(',');
    out.print(pTerm);
    out.print(',');
    out.print(iTerm);
    out.print(',');
    out.println(dTerm);
  };

  emitTo(Serial);
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    emitTo(_telnetClient);
  }
}

void Baratinha::broadcastRaw(const char* message) {
  if (message == nullptr) {
    return;
  }

  Serial.print(message);
  if (_telnetEnabled && _telnetClient && _telnetClient.connected()) {
    _telnetClient.print(message);
  }
}

bool Baratinha::autoConfigureOTA() {
  if (!_otaEnabled) {
    return false;
  }
  if (_otaConfigured) {
    return true;
  }

  const char* staSsid = _staSsid.length() ? _staSsid.c_str() : nullptr;
  const char* staPassword = _staPassword.length() ? _staPassword.c_str() : nullptr;
  const char* apSsid = _apSsid.length() ? _apSsid.c_str() : kDefaultApSsid;
  const char* apPassword = _apPassword.length() ? _apPassword.c_str() : kDefaultApPassword;

  return configureOTA(_preferAccessPoint, staSsid, staPassword, apSsid, apPassword);
}

void Baratinha::motorE_PWM(int vel) {
  int duty = vel;
  if (vel < 0) {
    gpio_set_pull_mode(kIn1, GPIO_PULLDOWN_ONLY);
    gpio_set_level(kIn2, 1);
    duty = -vel;
  } else {
    gpio_set_pull_mode(kIn1, GPIO_PULLUP_ONLY);
    gpio_set_level(kIn2, 0);
  }

  duty = constrain(duty, 0, 255);
  ledcWrite(kPwmChannelM1, duty);
}

void Baratinha::motorD_PWM(int vel) {
  int duty = vel;
  if (vel < 0) {
    gpio_set_pull_mode(kIn4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(kIn3, GPIO_PULLDOWN_ONLY);
    duty = -vel;
  } else {
    gpio_set_pull_mode(kIn4, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(kIn3, GPIO_PULLUP_ONLY);
  }

  duty = constrain(duty, 0, 255);
  ledcWrite(kPwmChannelM2, duty);
}

void Baratinha::applyBootAnimation() {
  setAllLeds(120, 255, 100);
  FastLED.show();
  delay(300);
  processOTA();

  const uint8_t sequence[][3] = {
      {60, 255, 0},
      {60, 255, 100},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {120, 255, 50},
  };

  const uint16_t delays[] = {170, 170, 170, 170, 170, 500};

  for (size_t i = 0; i < (sizeof(sequence) / sizeof(sequence[0])); ++i) {
    setAllLeds(sequence[i][0], sequence[i][1], sequence[i][2]);
    FastLED.show();
    delay(delays[i]);
    processOTA();
  }
}

void Baratinha::applyResumeAnimation() {
  const uint8_t resumeSequence[][3] = {
      {120, 255, 50},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {60, 255, 50},
      {60, 255, 0},
      {120, 255, 50},
  };

  const uint16_t resumeDelays[] = {150, 150, 150, 150, 150, 150, 500};

  for (size_t i = 0; i < (sizeof(resumeSequence) / sizeof(resumeSequence[0])); ++i) {
    setAllLeds(resumeSequence[i][0], resumeSequence[i][1], resumeSequence[i][2]);
    FastLED.show();
    delay(resumeDelays[i]);
    processOTA();
  }
}

void Baratinha::applyPauseIndicator() {
  setAllLeds(0, 255, 30);
  FastLED.show();
}

void Baratinha::updateStartStopState(int rawValue) {
  if (_buttonLast < kButtonLowThreshold && rawValue > kButtonHighThreshold) {
    setRunning(!_running);
  }
}

int Baratinha::readButtonRaw() const {
  return analogRead(kButtonPin);
}

void Baratinha::setAllLeds(uint8_t h, uint8_t s, uint8_t v) {
  for (uint8_t i = 0; i < kNumLeds; ++i) {
    _leds[i] = CHSV(h, s, v);
  }
}

void Baratinha::resetControlTick() {
  _lastControlMicros = micros();
}

void Baratinha::processOTA() {
  if (_otaEnabled && _otaConfigured) {
    ArduinoOTA.handle();
  }
  processTelnet();
}

void Baratinha::processTelnet() {
  if (!_telnetEnabled || !_telnetServerActive) {
    return;
  }

  if (!_telnetClient || !_telnetClient.connected()) {
    if (_telnetClient) {
      _telnetClient.stop();
    }
    WiFiClient incoming = _telnetServer.available();
    if (incoming && incoming.connected()) {
      _telnetClient = incoming;
      _telnetClient.println(F("Bem-vindo ao console Telnet da Baratinha!"));
      _telnetClient.println(F("Telemetria sera exibida aqui quando habilitada."));
    }
  } else {
    while (_telnetClient.available() > 0) {
      _telnetClient.read();  // descarta comandos por enquanto
    }
  }
}

void Baratinha::startTelnetServer(uint16_t port) {
  if (!_telnetEnabled) {
    return;
  }

  if (_telnetClient) {
    _telnetClient.stop();
  }

  _telnetServer.stop();
  _telnetServer = WiFiServer(port);
  _telnetServer.begin();
  _telnetServer.setNoDelay(true);
  _telnetServerActive = true;

  Serial.print(F("Telnet ativo na porta "));
  Serial.println(port);
}

void Baratinha::showOTAStartAnimation() {
  for (uint8_t wave = 0; wave < 12; ++wave) {
    for (uint8_t i = 0; i < kNumLeds; ++i) {
      const uint8_t brightness = (i == (wave % kNumLeds)) ? 200 : 15;
      _leds[i] = CHSV(160, 255, brightness);
    }
    FastLED.show();
    delay(60);
  }
}

void Baratinha::showOTAProgressAnimation(unsigned int progress, unsigned int total) {
  if (!_otaInProgress) {
    return;
  }
  _otaAnimationIndex = (_otaAnimationIndex + 1) % kNumLeds;
  // const uint8_t steps = kNumLeds * 4;
  // if (total == 0) {
  //   _otaAnimationIndex = (_otaAnimationIndex + 1) % kNumLeds;
  // } else {
  //   const uint32_t scaled = (static_cast<uint32_t>(progress) * steps) / total;
  //   _otaAnimationIndex = static_cast<uint8_t>(scaled % kNumLeds);
  // }

  for (uint8_t i = 0; i < kNumLeds; ++i) {
    const uint8_t offset = (i + kNumLeds - _otaAnimationIndex) % kNumLeds;
    uint8_t brightness = 30;
    if (offset == 0) {
      brightness = 200;
    } else if (offset == 1 || offset == (kNumLeds - 1)) {
      brightness = 120;
    }
    _leds[i] = CHSV(160, 255, brightness);
  }

  FastLED.show();
}

void Baratinha::showOTAEndAnimation() {
  for (uint8_t pulse = 0; pulse < 3; ++pulse) {
    setAllLeds(96, 255, 200);
    FastLED.show();
    delay(120);
    setAllLeds(96, 255, 20);
    FastLED.show();
    delay(120);
  }
}
