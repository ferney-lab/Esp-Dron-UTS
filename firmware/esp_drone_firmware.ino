// ============================================================
// ESP-DRONE UTS - SOLO PARADA DE EMERGENCIA EN HTML
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

const char* ssid     = "ESP-DRONE-UTS";
const char* password = "12345678";

WebServer server(80);
MPU6050 mpu;

#define M1_PIN 5   // Delantero Derecho (CCW)
#define M2_PIN 6   // Trasero Derecho   (CW)
#define M3_PIN 3   // Trasero Izquierdo (CCW)
#define M4_PIN 4   // Delantero Izquierdo (CW)

#define SDA_PIN 11
#define SCL_PIN 10
#define MPU_ADDR 0x68

#define PWM_FREQ          25000
#define PWM_RESOLUTION    10
#define PWM_MAX           1023
#define THRUST_MIN        180
#define THRUST_SPIN_FLOOR 200

volatile int thrustLiftoff = 920;
volatile int thrustHover   = 880;
#define MOTOR_MAX_THRUST   1000

unsigned long routineTakeoffMs = 1800;
unsigned long routineHoverMs   = 5000;
unsigned long routineLandMs    = 2500;

#define LOOP_MS 4
#define ACC_LSB  16384.0f
#define GYR_LSB  65.5f
#define RAD2DEG  57.29578f
const float COMP_TAU = 0.5f;

const float TILT_CUT_FLIGHT    = 50.0f;
const float TILT_CUT_TOUCHDOWN = 22.0f;

float pitchOffset = 0.0f;
float rollOffset  = 0.0f;
float gxOff = 0.0f, gyOff = 0.0f, gzOff = 0.0f;

enum FlightState {
  IDLE,
  ARMED_IDLE,
  ROUTINE_TAKEOFF,
  ROUTINE_HOVER,
  ROUTINE_LANDING,
  EMERGENCY
};

volatile FlightState flightState = IDLE;
volatile bool emergencyStopFlag = false;
volatile bool routineStartFlag  = false;
volatile bool landFlag          = false;
unsigned long stateStartTime    = 0;

float bodyAx = 0, bodyAy = 0, bodyAz = 1.0f;
float bodyGx = 0, bodyGy = 0, bodyGz = 0;
float pitch = 0, roll = 0, yaw = 0;
float dt = LOOP_MS * 0.001f;
unsigned long lastTime = 0;
bool mpuReady = false;

struct PID { float kp, ki, kd, integral, output; };
PID pidPitch = {1.60f, 0.03f, 0.06f, 0, 0};
PID pidRoll  = {1.60f, 0.03f, 0.06f, 0, 0};
PID pidYaw   = {0.80f, 0.00f, 0.00f, 0, 0};
const float I_MAX = 35.0f;

float cmdThrottle = 0;

void writeMotor(int pin, int val) {
  val = constrain(val, 0, PWM_MAX);
  ledcWrite(pin, val);
}

void motorStop() {
  ledcWrite(M1_PIN, 0);
  ledcWrite(M2_PIN, 0);
  ledcWrite(M3_PIN, 0);
  ledcWrite(M4_PIN, 0);
}

void resetPID() {
  pidPitch.integral = pidPitch.output = 0;
  pidRoll.integral  = pidRoll.output  = 0;
  pidYaw.integral   = pidYaw.output   = 0;
}

bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true) != 14) return false;

  int16_t r[7];
  for (int i = 0; i < 7; i++) {
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    r[i] = (int16_t)((h << 8) | l);
  }

  float rawAx = (float)r[0] / ACC_LSB;
  float rawAy = (float)r[1] / ACC_LSB;
  float rawAz = (float)r[2] / ACC_LSB;
  float rawGx = (float)r[4] / GYR_LSB - gxOff;
  float rawGy = (float)r[5] / GYR_LSB - gyOff;
  float rawGz = (float)r[6] / GYR_LSB - gzOff;

  // Ejes respecto a FRONT (cinta flex)
  bodyAx = -rawAy; // Adelante (Front)
  bodyAy =  rawAx; // Izquierda (Left)
  bodyAz =  rawAz; // Arriba (Up)

  bodyGx = -rawGy; // Giro Roll
  bodyGy =  rawGx; // Giro Pitch
  bodyGz =  rawGz; // Giro Yaw

  return true;
}

void updateAttitude() {
  float denomP = sqrtf(bodyAy * bodyAy + bodyAz * bodyAz);
  float denomR = sqrtf(bodyAx * bodyAx + bodyAz * bodyAz);
  if (denomP < 0.05f) denomP = 0.05f;
  if (denomR < 0.05f) denomR = 0.05f;

  float pA = atan2f(bodyAx, denomP) * RAD2DEG - pitchOffset;
  float rA = atan2f(-bodyAy, denomR) * RAD2DEG - rollOffset;

  float a = COMP_TAU / (COMP_TAU + dt);
  pitch = a * (pitch + bodyGy * dt) + (1.0f - a) * pA;
  roll  = a * (roll  + bodyGx * dt) + (1.0f - a) * rA;
  yaw  += bodyGz * dt;
}

void calibrateFromZero() {
  Serial.println("\n>>> CALIBRANDO EN REPOSO SOBRE LA MESA (NO MOVER) <<<");
  delay(500);

  float sAx = 0, sAy = 0, sAz = 0;
  float sGx = 0, sGy = 0, sGz = 0;
  int validSamples = 0;

  for (int i = 0; i < 200; i++) {
    if (readMPU()) {
      sAx += bodyAx;
      sAy += bodyAy;
      sAz += bodyAz;
      sGx += bodyGx + gxOff;
      sGy += bodyGy + gyOff;
      sGz += bodyGz + gzOff;
      validSamples++;
    }
    delay(5);
  }

  if (validSamples > 50) {
    float mAx = sAx / validSamples;
    float mAy = sAy / validSamples;
    float mAz = sAz / validSamples;
    gxOff = sGx / validSamples;
    gyOff = sGy / validSamples;
    gzOff = sGz / validSamples;

    pitchOffset = atan2f(mAx, sqrtf(mAy * mAy + mAz * mAz)) * RAD2DEG;
    rollOffset  = atan2f(-mAy, sqrtf(mAx * mAx + mAz * mAz)) * RAD2DEG;
    mpuReady = true;

    Serial.printf("[OK] CALIBRACIÓN EXITOSA (%d muestras):\n", validSamples);
    Serial.printf("     Pitch Offset = %.2f° | Roll Offset = %.2f°\n", pitchOffset, rollOffset);
    Serial.printf("     Gyro Offsets = [%.3f, %.3f, %.3f]\n\n", gxOff, gyOff, gzOff);
  } else {
    mpuReady = false;
    Serial.println("[ERROR] No se pudo calibrar el sensor.");
  }
}

float calcPID(PID &p, float errorAngle, float rateGyro, float limit, bool allowI) {
  if (allowI && fabsf(p.output) < limit) {
    p.integral = constrain(p.integral + errorAngle * dt, -I_MAX, I_MAX);
  } else {
    p.integral = 0;
  }
  float dTerm = -p.kd * rateGyro;
  float o = (p.kp * errorAngle) + (p.ki * p.integral) + dTerm;
  p.output = constrain(o, -limit, limit);
  return p.output;
}

void applyMotors() {
  if (emergencyStopFlag || flightState == IDLE || flightState == EMERGENCY) {
    motorStop();
    cmdThrottle = 0;
    resetPID();
    return;
  }

  // En ralentí de suelo, motores giran parejos
  if (flightState == ARMED_IDLE) {
    writeMotor(M1_PIN, THRUST_SPIN_FLOOR);
    writeMotor(M2_PIN, THRUST_SPIN_FLOOR);
    writeMotor(M3_PIN, THRUST_SPIN_FLOOR);
    writeMotor(M4_PIN, THRUST_SPIN_FLOOR);

    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 350) {
      Serial.printf("[VUELO] Thr=%.0f | M1=%d M2=%d M3=%d M4=%d | P=%.1f R=%.1f\n",
                    cmdThrottle, THRUST_SPIN_FLOOR, THRUST_SPIN_FLOOR, THRUST_SPIN_FLOOR, THRUST_SPIN_FLOOR, pitch, roll);
      lastDbg = millis();
    }
    return;
  }

  if (flightState == ROUTINE_LANDING) {
    if (fabsf(pitch) > TILT_CUT_TOUCHDOWN || fabsf(roll) > TILT_CUT_TOUCHDOWN) {
      motorStop();
      flightState = IDLE;
      Serial.println("[ATERRIZAJE] Contacto con el suelo -> Motores cortados");
      return;
    }
  } else {
    if (fabsf(pitch) > TILT_CUT_FLIGHT || fabsf(roll) > TILT_CUT_FLIGHT) {
      emergencyStopFlag = true;
      flightState = EMERGENCY;
      motorStop();
      Serial.println("[EMERGENCIA] Corte por inclinacion en vuelo");
      return;
    }
  }

  int m1, m2, m3, m4;
  if (flightState == ROUTINE_HOVER || flightState == ROUTINE_TAKEOFF || flightState == ROUTINE_LANDING) {
    bool allowI = (flightState == ROUTINE_HOVER);

    float oP = calcPID(pidPitch, -pitch, bodyGy, 95.0f, allowI);
    // FIX: Roll usa +roll (no -roll) para corregir en la dirección correcta
    float oR = calcPID(pidRoll,   roll,  bodyGx, 95.0f, allowI);
    float oY = calcPID(pidYaw,  -bodyGz, 0.0f, 25.0f, false);

    int minV = (flightState == ROUTINE_LANDING) ? 0 : THRUST_MIN;

    m1 = constrain((int)(cmdThrottle - oP + oR + oY), minV, MOTOR_MAX_THRUST);
    m2 = constrain((int)(cmdThrottle + oP + oR - oY), minV, MOTOR_MAX_THRUST);
    m3 = constrain((int)(cmdThrottle + oP - oR + oY), minV, MOTOR_MAX_THRUST);
    m4 = constrain((int)(cmdThrottle - oP - oR - oY), minV, MOTOR_MAX_THRUST);
  } else {
    m1 = m2 = m3 = m4 = 0;
  }

  writeMotor(M1_PIN, m1);
  writeMotor(M2_PIN, m2);
  writeMotor(M3_PIN, m3);
  writeMotor(M4_PIN, m4);

  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 350 && flightState != IDLE) {
    Serial.printf("[VUELO] Thr=%.0f | M1=%d M2=%d M3=%d M4=%d | P=%.1f R=%.1f\n",
                  cmdThrottle, m1, m2, m3, m4, pitch, roll);
    lastDbg = millis();
  }
}

void controlLoopStep() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();
  dt = (nowUs - lastTime) * 1e-6f;
  lastTime = nowUs;
  if (dt <= 0 || dt > 0.05f) dt = LOOP_MS * 0.001f;

  if (readMPU()) {
    updateAttitude();
  }

  if (routineStartFlag) {
    routineStartFlag = false;
    if (mpuReady && (flightState == IDLE || flightState == EMERGENCY)) {
      emergencyStopFlag = false;
      resetPID();
      yaw = 0;
      stateStartTime = nowMs;
      flightState = ARMED_IDLE;
      Serial.println("[AUTO] Armado en ralenti...");
    }
  }

  if (landFlag) {
    landFlag = false;
    if (flightState == ROUTINE_HOVER || flightState == ROUTINE_TAKEOFF) {
      stateStartTime = nowMs;
      flightState = ROUTINE_LANDING;
    }
  }

  unsigned long elapsed = nowMs - stateStartTime;

  switch (flightState) {
    case IDLE:
    case EMERGENCY:
      cmdThrottle = 0;
      break;

    case ARMED_IDLE: {
      cmdThrottle = THRUST_SPIN_FLOOR;
      if (elapsed > 1000) {
        stateStartTime = nowMs;
        flightState = ROUTINE_TAKEOFF;
        Serial.println("[AUTO] Despegue firme!");
      }
      break;
    }

    case ROUTINE_TAKEOFF: {
      if (elapsed < routineTakeoffMs) {
        float p = (float)elapsed / (float)routineTakeoffMs;
        float ease = p * p * (3.0f - 2.0f * p);
        cmdThrottle = THRUST_SPIN_FLOOR + (thrustLiftoff - THRUST_SPIN_FLOOR) * ease;
      } else {
        cmdThrottle = thrustHover;
        stateStartTime = nowMs;
        flightState = ROUTINE_HOVER;
        Serial.printf("[VUELO] Hover sostenido a %d\n", thrustHover);
      }
      break;
    }

    case ROUTINE_HOVER:
      cmdThrottle = thrustHover;
      if (elapsed > routineHoverMs) {
        stateStartTime = nowMs;
        flightState = ROUTINE_LANDING;
        Serial.println("[VUELO] Iniciando descenso suave...");
      }
      break;

    case ROUTINE_LANDING: {
      if (elapsed < routineLandMs) {
        cmdThrottle = thrustHover * (1.0f - (float)elapsed / (float)routineLandMs);
      } else {
        cmdThrottle = 0;
        flightState = IDLE;
        motorStop();
        Serial.println("[FIN] Aterrizaje completado con exito");
      }
      break;
    }
  }

  applyMotors();
}

// ================================================================
// WEB HANDLERS — Solo parada de emergencia + telemetría
// ================================================================
void handleStop() {
  motorStop();
  emergencyStopFlag = true;
  flightState = EMERGENCY;
  cmdThrottle = 0;
  resetPID();
  server.send(200, "text/plain", "STOPPED");
}

void handleStartRoutine() {
  routineStartFlag = true;
  server.send(200, "text/plain", "OK");
}

void handleLand() {
  landFlag = true;
  server.send(200, "text/plain", "OK");
}

void handleData() {
  String e;
  switch (flightState) {
    case IDLE:            e = "IDLE (LISTO)"; break;
    case ARMED_IDLE:      e = "ARMADO (RALENTI)"; break;
    case ROUTINE_TAKEOFF: e = "DESPEGANDO"; break;
    case ROUTINE_HOVER:   e = "VOLANDO"; break;
    case ROUTINE_LANDING: e = "ATERRIZANDO"; break;
    case EMERGENCY:       e = "EMERGENCIA"; break;
  }
  String j = "{\"pitch\":" + String(pitch, 1) +
             ",\"roll\":" + String(roll, 1) +
             ",\"throttle\":" + String((int)cmdThrottle) +
             ",\"estado\":\"" + e + "\"}";
  server.send(200, "application/json", j);
}

// ================================================================
// HTML — Solo botón de PARADA DE EMERGENCIA + telemetría
// ================================================================
const char paginaHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP DRONE UTS</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#0d0d0d;color:#fff;padding:16px;display:flex;justify-content:center;min-height:100vh;align-items:center}
.container{max-width:440px;width:100%}
h1{color:#00e5ff;font-size:1.5em;text-align:center;margin-bottom:12px}
.card{background:#181818;border:1px solid #2a2a2a;border-radius:12px;padding:16px;margin-bottom:12px}
.status{font-size:1.3em;font-weight:bold;color:#00ff88;text-align:center}
.status.emergencia{color:#ff3333}
.status.rutina{color:#ffaa00}
.telemetry{display:flex;justify-content:space-around;margin:10px 0;text-align:center}
.tel-val{font-size:1.1em;color:#00e5ff;font-weight:bold}
.btn-stop{width:100%;background:#cc0000;color:#fff;border:none;border-radius:12px;padding:40px;font-size:1.5em;font-weight:900;cursor:pointer;box-shadow:0 0 25px rgba(255,0,0,0.5);text-transform:uppercase;letter-spacing:2px;transition:all .1s}
.btn-stop:active{background:#880000;transform:scale(0.97)}
</style></head><body>
<div class="container">
  <h1>ESP DRONE UTS</h1>
  <div class="card">
    <div class="status" id="stTxt">MODO: IDLE (LISTO)</div>
    <div class="telemetry">
      <div>PITCH<div class="tel-val" id="pVal">0.0&deg;</div></div>
      <div>ROLL<div class="tel-val" id="rVal">0.0&deg;</div></div>
      <div>THR<div class="tel-val" id="tVal">0</div></div>
    </div>
  </div>
  <button class="btn-stop" onclick="stopNow()">⛔ PARADA DE EMERGENCIA</button>
</div>
<script>
setInterval(()=>{
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('pVal').innerText=d.pitch.toFixed(1)+'°';
    document.getElementById('rVal').innerText=d.roll.toFixed(1)+'°';
    document.getElementById('tVal').innerText=d.throttle;
    var el=document.getElementById('stTxt');
    el.innerText='MODO: '+d.estado;
    el.className='status '+(d.estado=='EMERGENCIA'?'emergencia':d.estado!='IDLE (LISTO)'?'rutina':'');
  }).catch(()=>{});
},350);
function stopNow(){
  fetch('/stop');fetch('/stop');
  navigator.sendBeacon('/stop');
  document.getElementById('stTxt').innerText='MODO: EMERGENCIA (PARADO)';
  document.getElementById('stTxt').className='status emergencia';
}
</script></body></html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", paginaHTML); }

void setup() {
  const int mp[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
  for (int i = 0; i < 4; i++) {
    pinMode(mp[i], OUTPUT);
    digitalWrite(mp[i], LOW);
    ledcAttach(mp[i], PWM_FREQ, PWM_RESOLUTION);
  }
  motorStop();
  flightState = IDLE;

  Serial.begin(115200);
  delay(1000);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN, 50000);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  mpu.initialize();

  calibrateFromZero();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, 1, 0, 4);

  server.on("/",              handleRoot);
  server.on("/data",          handleData);
  server.on("/start_routine", handleStartRoutine);
  server.on("/land",          handleLand);
  server.on("/stop",          handleStop);
  server.begin();

  lastTime = micros();
  Serial.println("[OK] Sistema listo en 192.168.4.1");
}

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = micros();
  
  if (now - lastLoop >= LOOP_MS * 1000) {
    lastLoop = now;
    controlLoopStep();
  }

  server.handleClient();
}
