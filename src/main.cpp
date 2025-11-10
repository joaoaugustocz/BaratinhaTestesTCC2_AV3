#include <Arduino.h>
#include <Baratinha.h>
#include <math.h>

Baratinha bra;

namespace {

// Ajuste aqui os parametros do PID e as restricoes do controle.
constexpr float Ts = 0.01f;

constexpr float Kp = 0.00585f;
constexpr float Ki = 0.00509f;
constexpr float Kd = 0.00196f;
constexpr float Tf = 0.199595f;

constexpr float U_MAX = 1.0f;
constexpr int MIN_DIST_STOP_MM = 30;
constexpr int CTRL_SIGN = -1;
constexpr float setPoint = 350.0f;

// Ajuste a estrategia de OTA: true => cria Access Point, false => conecta em uma rede existente.
constexpr bool USE_WIFI_AP = true;
constexpr char WIFI_STA_SSID[] = "Agility-IBITU";
constexpr char WIFI_STA_PASSWORD[] = "0203040506";
// Credenciais do ponto de acesso criado pelo robo para OTA.
constexpr char WIFI_AP_SSID[] = "Baratinha-OTA";
constexpr char WIFI_AP_PASSWORD[] = "12345678";

float I_acc = 0.0f;
float e_prev = 0.0f;
float d_state_simple = 0.0f;

}  // namespace

void setup() {
  const bool hardwareReady = bra.setupAll();

  if (USE_WIFI_AP) {
    bra.setupOTAAccessPoint(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  } else {
    bra.setupOTAStation(WIFI_STA_SSID, WIFI_STA_PASSWORD);
  }
  // Caso nao deseje OTA, basta descomentar a linha abaixo.
  // bra.enableOTA(false);

  if (!hardwareReady) {
    Serial.println(F("Hardware nao inicializou corretamente. Aguardando OTA..."));
    while (true) {
      bra.updateStartStop();
      delay(500);
    }
  }

  bra.setControlInterval(Ts);
  bra.enableTelemetry(true);
  //bra.setTelemetryDivider(5);

  bra.awaitStart();
  bra.println(F("Inicio do controle."));
  bra.printf("Kp: %.5f, Ki: %.5f, Kd: %.5f, Tf: %.5f, Ts: %.5f\r\n", Kp, Ki, Kd, Tf, Ts);
  
}

void loop() {
  bra.updateStartStop();
  if (!bra.isRunning()) return;
  if (!bra.controlTickDue()) return;

  //bra.move1D(70, false);
  // float measurement = bra.readDistance();

  // const float error = setPoint - measurement;

  // I_acc += (Ki * Ts) * error;

  // const float d_new = (Tf / (Ts + Tf)) * d_state_simple +
  //                     (Kd / (Ts + Tf)) * (error - e_prev);

  // const float controlRaw = (Kp * error) + I_acc + d_new;

  // float controlLimited = controlRaw;
  // controlLimited = constrain(controlLimited, -U_MAX, U_MAX);


  // if (static_cast<int>(measurement) <= MIN_DIST_STOP_MM) controlLimited = 0.0f;
    
  

  // int pwm = static_cast<int>(lroundf(255.0f * (CTRL_SIGN * controlLimited)));
  // pwm = constrain(pwm, -255, 255);
  // bra.move1D(pwm, false);

  // e_prev = error;
  // d_state_simple = d_new;


  // bra.emitTelemetry(setPoint, measurement, error,
  //                   controlRaw, controlLimited,
  //                   Kp * error, I_acc, d_new);
}
