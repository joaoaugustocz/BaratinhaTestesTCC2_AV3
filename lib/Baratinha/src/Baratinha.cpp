#include "Baratinha.h"

#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

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
      _otaConfigured(false) {}

void Baratinha::beginSerial(uint32_t baud) {
  Serial.begin(baud);
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

void Baratinha::setupOTAStation(const char* ssid, const char* password) {
  _otaConfigured = false;
  if (ssid == nullptr || password == nullptr) {
    Serial.println(F("Credenciais invalidas para OTA station."));
    _otaConfigured = false;
    return;
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
    _otaConfigured = false;
    return;
  }

  ArduinoOTA.onStart([]() {
    Serial.println(F("OTA comecou"));
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("OTA finalizado"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = (total == 0) ? 0 : (progress * 100U / total);
    Serial.print(F("OTA progresso: "));
    Serial.print(percent);
    Serial.println(F("%"));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print(F("Erro OTA: "));
    Serial.println(static_cast<int>(error));
  });

  ArduinoOTA.begin();
  _otaConfigured = true;

  Serial.print(F("OTA pronto. IP: "));
  Serial.println(WiFi.localIP());
}

void Baratinha::setupOTAAccessPoint(const char* ssid, const char* password) {
  _otaConfigured = false;
  if (ssid == nullptr || password == nullptr) {
    Serial.println(F("Credenciais invalidas para OTA access point."));
    _otaConfigured = false;
    return;
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, password)) {
    Serial.println(F("Falha ao iniciar access point para OTA."));
    _otaConfigured = false;
    return;
  }

  ArduinoOTA.onStart([]() {
    Serial.println(F("OTA comecou"));
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("OTA finalizado"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = (total == 0) ? 0 : (progress * 100U / total);
    Serial.print(F("OTA progresso: "));
    Serial.print(percent);
    Serial.println(F("%"));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print(F("Erro OTA: "));
    Serial.println(static_cast<int>(error));
  });

  ArduinoOTA.begin();
  _otaConfigured = true;

  Serial.print(F("OTA AP pronto. IP: "));
  Serial.println(WiFi.softAPIP());
}

void Baratinha::enableOTA(bool enable) {
  _otaEnabled = enable;
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

  Serial.print(millis());
  Serial.print(',');
  Serial.print(setpoint);
  Serial.print(',');
  Serial.print(measurement);
  Serial.print(',');
  Serial.print(error);
  Serial.print(',');
  Serial.print(controlRaw);
  Serial.print(',');
  Serial.print(controlLimited);
  Serial.print(',');
  Serial.print(pTerm);
  Serial.print(',');
  Serial.print(iTerm);
  Serial.print(',');
  Serial.println(dTerm);
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
}
