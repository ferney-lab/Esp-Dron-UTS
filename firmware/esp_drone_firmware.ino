// ============================================================================
// ESP_DRONE_ARDUINO  (version dia 28 + correccion de yaw + despegue progresivo)
//
// CAMBIOS SOBRE LA VERSION DEL DIA 28 (todo lo demas queda igual):
//   1. YAW CORREGIDO: el mezclador da prioridad a roll/pitch (no se va de lado),
//      le abre margen al yaw bajando un poco el thrust si hace falta, y los
//      integrales de yaw ya no se "cargan" cuando no hay margen (anti-windup).
//   2. POTENCIA DE MOTORES 104 %: cada motor recibe su mezcla x 1.04 (tope
//      fisico: 100 % de PWM).
//   3. DESPEGUE PROGRESIVO: el boton DESPEGAR sube el thrust de forma suave
//      (curva S) en 3 s hasta el HOVER y lo mantiene ahi.
//   4. HOVER 70 %: valor por defecto del despegue.
//   5. El monitor serie ya no puede bloquear el programa con el USB conectado.
//  10. Despegue parejo: el angulo del dron apoyado se toma como nivel (antes M1/M4
//      arrancaban mas rapido). Apoyado corrige angulo sin integrales.
//  12. NO SIGUE SUBIENDO: DESPEGAR detecta el despegue y mantiene (max 12 s).
//      Sin sensor de altura, un error del acelerometro de 0.01 g lo hacia subir
//      hasta 8 m. Ahora: la correccion automatica solo QUITA empuje, el hover se
//      estima 1.5 % bajo, frena 3 % al llegar arriba y baja 0.5 %/s. Peor caso
//      simulado: 37 cm. Para quedarse mas arriba: joystick izquierdo.
//      Compensacion de bateria apagada por defecto.
//  11. Rutina 1 corta: sube, llega al punto mas alto y aterriza ("Arriba s" = 0).
//      Rampa mas rapida en la zona de despegue y tolerancia de 1.5 s al Wi-Fi.
//   9. TRES RUTINAS (botones 1 VERTICAL, 2 DERECHA, 3 VUELTA IZQUIERDA). Hover maximo 70 %.
//  14. (10/11) La rutina 3 se llama VUELTA IZQUIERDA: gira 360 grados hacia la
//      izquierda (antihorario visto desde arriba). Solo cambia el nombre.
//   8. Cada RUTINA AUTO empieza recalibrando el cero del giroscopio (~1 s, sin
//      mover el dron): despues de un vuelo el MPU esta caliente y su cero cambia.
//  13. DESPEGUE VERTICAL (09/11):
//      a) Ganancias roll/pitch del firmware de fabrica (rate 250/500/2.5, angulo 5.9/2.9).
//      b) Los integrales ya NO se borran en el aire: antes, si el thrust bajaba
//         de 60 % (hover estimado bajo, freno arriba, bajada) se ponian en 0 en
//         pleno vuelo y el dron se ladeaba. Ahora solo se borran apoyado.
//      c) Vuelo MANUAL tambien toma el angulo apoyado como nivel (antes solo la rutina).
//      f) Al TOCAR SUELO (o en manual con thrust < 60 %) los integrales se
//         congelan: no se borran (no se ladea al bajar) ni se cargan apoyado.
//      g) Ya no se apaga en el aire por vibracion: el "golpe con el suelo" debe
//         durar 20 ms (antes 1 muestra) y en el aire pasa a aterrizaje suave en
//         vez de cortar motores. DESPEGAR se mantiene hasta ATERRIZAR (max 30 s).
//      e) Rutina 1 sube mas (Subida 1.2 s ~ 35 cm) y se mantiene 5 s arriba
//         estabilizandose antes de bajar (pagina web: Subida 1.2, Arriba 5).
//      d) Integrales de roll/pitch APRENDIDOS: en el aire, estable, se guarda
//         cuanto necesitaba el PID para mantenerse nivelado (desbalance de peso o
//         de motores) y se PRECARGA en el siguiente despegue. El primer vuelo
//         despues de encender arranca en 0; desde el segundo sale compensado.
//   7. En la rutina los integrales del PID no se acumulan mientras el dron sigue
//      apoyado (antes lo inclinaban al despegar). Bateria minima 3.8 V; si el
//      hover medido pasa de 80 % aterriza enseguida (bateria agotada).
//   6. RUTINA AUTOMATICA (boton RUTINA AUTO): sube la potencia de forma
//      progresiva, DETECTA el despegue con el acelerometro y deja de subir la
//      potencia; sube un poco ("Subida s"), SE MANTIENE en esa altura ("En el
//      aire s") y aterriza de forma progresiva, apagando al tocar el suelo.
// ----------------------------------------------------------------------------
// Port para Arduino IDE del lazo de vuelo de esp-drone (Espressif), placa
// ESP32-S2 Drone V1.2, motores 720.  Proyecto UTS.
// ----------------------------------------------------------------------------
// Copia la MISMA cadena de control que el firmware original:
//   sensors_mpu6050_hm5883L_ms5611.c : MPU6050 +-2000 dps / +-16 g, DLPF 42 Hz,
//        1000 Hz, bias del giroscopio por varianza, escala del acelerometro,
//        filtros lpf2p 80 Hz (gyro) y 30 Hz (acel), mismos ejes del ESP32-S2.
//   sensfusion6.c     : filtro Mahony (twoKp 0.8, twoKi 0.002) a 250 Hz.
//   controller_pid.c / attitude_pid_controller.c / pid.c / pid.h :
//        PID angulo -> PID velocidad a 500 Hz, ganancias ESP32-S2 V1.2.
//   power_distribution_stock.c : mezclador X (roll y pitch / 2).
//   motors.c          : PWM 15 kHz.
//   crtp_commander_rpyt.c : comandos roll/pitch/yaw/thrust de la app,
//        bloqueo de thrust hasta recibir 0, MIN 1000 / MAX 60000.
//   commander.c       : sin comandos 0.5 s -> nivela; 2 s -> apaga.
//   wifi_esp32.c      : AP "ESP-DRONE_<MAC>", clave 12345678, 192.168.43.42,
//        UDP 2390, paquetes CRTP con checksum.
//
// FORMAS DE CONTROLARLO
//   1) Navegador del celular: http://192.168.43.42  (joystick igual a la app)
//   2) App ESP-Drone / cfclient por UDP 2390 (mismo protocolo del original)
//   3) Monitor serie 115200, "Nueva linea": escribe  help
//
// Al encender: dejalo QUIETO y PLANO hasta ver "Ready to fly" (como el original).
// PRIMERA PRUEBA SIN HELICES.
// ============================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ============================================================================
// TIPOS (definidos antes de cualquier funcion: el IDE de Arduino genera
// prototipos automaticos y estos tipos deben existir antes)
// ============================================================================
struct PidObject {                 // pid.h
  float desired, error, prevError, integ, deriv;
  float kp, ki, kd;
  float outP, outI, outD;
  float iLimit, outputLimit, dt;
};

struct Lpf2p {                     // filter.h
  float a1, a2, b0, b1, b2;
  float d1, d2;
};

struct RpytCmd {                   // CommanderCrtpLegacyValues
  float roll, pitch, yaw;          // grados, grados, grados/s
  uint16_t thrust;                 // 0..65535
  uint32_t ms;                     // millis() de recepcion
};

// ============================================================================
// HARDWARE (Kconfig.projbuild, TARGET_ESP32_S2_DRONE_V1_2)
// ============================================================================
#define M1_PIN 5        // MOTOR01_PIN
#define M2_PIN 6        // MOTOR02_PIN
#define M3_PIN 3        // MOTOR03_PIN
#define M4_PIN 4        // MOTOR04_PIN
const int MOTOR_PIN[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};

#define SDA_PIN 11
#define SCL_PIN 10
#define MPU_ADDR 0x68
#define VBAT_PIN 2      // ADC1_PIN, divisor x2 (pm_esplane.c)
#define VBAT_MULT 2.0f
#define PM_BAT_LOW_VOLTAGE 3.2f

// ---- Datos de este dron (para referencia y ajustes) ----
// Peso 52 g con bateria 1S 800 mAh, motores coreless 720.
// Compensacion por bateria (idea de ENABLE_THRUST_BAT_COMPENSATED en motors.c):
// al bajar el voltaje en vuelo (4.0 -> 3.55 V en tus logs) los motores empujan
// menos con el mismo PWM; se sube el PWM en proporcion VBAT_REF / vbat.
#define VBAT_REF        3.9f    // voltaje al que el factor vale 1.0
#define BAT_COMP_MAX    1.20f   // nunca subir mas de 20 %

#define MOTOR_PWM_FREQ 15000   // motors.c
#define MOTOR_PWM_BITS 10      // el original usa 8; 10 da mas resolucion, misma escala

// Sentido de giro esperado (docs/_static/espdrone_s2_v1_2_diretion2.png):
//   M1 delantero der. ANTIHORARIO (helice B)   M4 delantero izq. HORARIO (helice A)
//   M3 trasero izq.   ANTIHORARIO (helice B)   M2 trasero der.   HORARIO (helice A)
// Si en TU dron los 4 giran al reves (M1/M3 horario, M2/M4 antihorario, con las
// helices tambien al reves), el yaw se invierte y se dispara. Pon -1 aqui o usa
// el comando serie "yawinv" para probar.
#define YAW_MIX_SIGN_DEFAULT (+1)   // YAW POSITIVO (+1), FIJO. Con -1 el yaw se disparo en vuelo (log del 26/10)

// Limite de la salida del yaw (el original no tiene). Asi, aunque el yaw
// falle, nunca puede apagar dos motores como paso en el log (y = 32767).
#define YAW_OUTPUT_LIMIT 12000.0f   // ~18 % del rango de thrust
#define YAW_MAX_SHIFT    9000       // cuanto puede bajar el thrust (max ~14 %) para darle margen al yaw
#define YAW_MAX_ERROR    25.0f      // grados: error de rumbo maximo que se acumula

// Potencia de motores: cada motor = mezcla x MOTOR_POWER_PCT / 100
#define MOTOR_POWER_PCT  104.0f
// Despegue progresivo (boton DESPEGAR de la pagina)
#define HOVER_PCT_DEFAULT 64        // thrust de la rampa manual (x1.04 en motores = ~67 %)
#define TAKEOFF_RAMP_MS   3000      // tiempo de la subida progresiva hasta el hover

// ---------------- RUTINA AUTOMATICA (solo MPU6050) ----------------
#define AUTO_SPIN_PCT     30.0f   // arranque de motores en el suelo
#define AUTO_RAMP_PCT_S   15.0f   // % por segundo cerca del despegue (progresivo, sin quedarse a medias)
#define AUTO_FAST_PCT_S   40.0f   // % por segundo hasta acercarse al hover del vuelo anterior
#define AUTO_FAST_MARGIN   0.12f  // la parte rapida para 12 % antes de ese hover
#define AUTO_MAX_PCT      95.0f   // si no despega aqui, aborta (motores x1.04 -> ~99 %)
#define HOVER_MAX_PCT     70.0f   // en el aire el thrust nunca pasa de 70 %

// ---------------- LAS 3 RUTINAS ----------------
// Todas: despegue progresivo -> sube "Subida s" -> SE MANTIENE (no sigue
// subiendo) -> accion -> se estabiliza -> aterrizaje bajando la potencia.
#define MISSION_STABLE_S   0.3f   // segundos ARRIBA antes de empezar la accion (rutinas 2 y 3)
#define MISSION_CLIMB_S    1.2f   // rutinas 2 y 3 suben mas (~40 cm) para tener tiempo de maniobrar
#define MISSION_AFTER_S    1.5f   // segundos estabilizandose despues de la accion
#define VERT_AIR_S         5.0f   // rutina 1: segundos ARRIBA estabilizandose antes de bajar (0 = baja apenas llega)
#define RIGHT_TILT_DEG     8.0f   // rutina 2: inclinacion para avanzar a la derecha (~55 cm)
#define RIGHT_PUSH_S       1.2f   // rutina 2: tiempo inclinado (avanza) = tiempo frenando
#define TURN_RATE_DPS     90.0f   // rutina 3: velocidad de giro a la IZQUIERDA (antihorario, 4 s la vuelta)
#define TURN_MAX_S        12.0f   // rutina 3: si no completa la vuelta en este tiempo, sigue
#define LIFT_ACC_G         0.04f  // aceleracion hacia arriba sostenida = despego
#define VZ_KP              0.25f  // lazo vertical (fraccion de thrust por m/s)
#define VZ_CORR_MAX        0.12f  // correccion maxima hacia ABAJO
#define VZ_UP_MAX          0.00f  // correccion automatica hacia ARRIBA: NINGUNA (ver abajo)
#define HOLD_DECAY_PCT_S   0.50f  // en el hover el empuje baja 0.5 %/s: tiende a bajar suave, nunca a subir
#define HOVER_SAFETY       0.985f // hover estimado x0.985
#define TOP_BRAKE          0.03f  // al llegar arriba: -3 % durante 1 s (frena la velocidad que traia)
#define HOLD_DECAY_MAX     0.06f  // como maximo baja 6 % (despues el piloto o el aterrizaje)
#define PROFILE_E          0.10f  // +-10 % sobre el hover para acelerar/frenar
#define LAND_START_DROP    0.03f  // aterrizaje: hover - 3 % ...
#define LAND_DROP_PCT_S    0.015f // ... y baja 1.5 % mas por segundo
#define LAND_MAX_VZ       -0.60f  // si baja mas rapido que esto, frena
#define AUTO_TILT_CUT     60.0f   // en la rutina: inclinacion mayor = apagar
#define AUTO_MIN_VBAT      3.80f  // no arranca con menos (en reposo). Con 3.25 V en vuelo el hover subio a 89 %
#define HOVER_ABORT_PCT   80.0f   // si el hover medido pasa de esto: no sube, aterriza (sin margen)
// (entre 70 y 80 % despega, pero el hover queda limitado a 70 % y baja solo)

enum { AUTO_OFF = 0, AUTO_SPIN, AUTO_RAMP, AUTO_CLIMB, AUTO_HOVER, AUTO_DESC, AUTO_LAND };
const char* AUTO_NAME[7] = {"OFF", "ARRANQUE", "BUSCANDO DESPEGUE", "SUBIENDO", "SE MANTIENE", "BAJANDO", "TOCANDO SUELO"};
const char* MISSION_NAME[5] = {"", "1 VERTICAL", "2 DERECHA", "3 VUELTA IZQUIERDA", "DESPEGAR Y MANTENER"};
#define HOLD_MAX_S 30.0f        // DESPEGAR: se mantiene hasta ATERRIZAR (maximo 30 s, luego aterriza solo)

// ============================================================================
// CONSTANTES DEL ORIGINAL
// ============================================================================
// Frecuencias (stabilizer_types.h, estimator_complementary.c)
#define RATE_MAIN_LOOP   1000
#define ATTITUDE_RATE    500
#define ESTIMATOR_RATE   250
#define ATTITUDE_DT      (1.0f / ATTITUDE_RATE)
#define ESTIMATOR_DT     (1.0f / ESTIMATOR_RATE)

// Sensores
#define DEG_PER_LSB   ((2.0f * 2000.0f) / 65536.0f)   // MPU6050_DEG_PER_LSB_2000
#define G_PER_LSB     ((2.0f * 16.0f) / 65536.0f)     // MPU6050_G_PER_LSB_16
#define GYRO_LPF_HZ   80
#define ACCEL_LPF_HZ  30
#define BIAS_SAMPLES  1024                            // SENSORS_NBR_OF_BIAS_SAMPLES
#define GYRO_VARIANCE_BASE 5000.0f                    // umbral de varianza (LSB^2)
#define ACC_SCALE_SAMPLES 200

// Mahony (sensfusion6.c)
#define TWO_KP_DEF (2.0f * 0.4f)
#define TWO_KI_DEF (2.0f * 0.001f)

// PID roll/pitch = ganancias del firmware esp-drone de fabrica (ESP32Drone_Firmware .bin)
// Valores anteriores: rate 190/440/2.6, angulo 5.3/2.5
#define PID_ROLL_RATE_KP  250.0f
#define PID_ROLL_RATE_KI  500.0f
#define PID_ROLL_RATE_KD  2.5f
#define PID_ROLL_RATE_INTEGRATION_LIMIT 33.3f
#define PID_PITCH_RATE_KP 250.0f
#define PID_PITCH_RATE_KI 500.0f
#define PID_PITCH_RATE_KD 2.5f
#define PID_PITCH_RATE_INTEGRATION_LIMIT 33.3f
#define PID_YAW_RATE_KP   120.0f
#define PID_YAW_RATE_KI   16.7f
#define PID_YAW_RATE_KD   0.0f
#define PID_YAW_RATE_INTEGRATION_LIMIT 166.7f
#define PID_ROLL_KP  5.9f
#define PID_ROLL_KI  2.9f
#define PID_ROLL_KD  0.0f
#define PID_ROLL_INTEGRATION_LIMIT 20.0f
#define PID_PITCH_KP 5.9f
#define PID_PITCH_KI 2.9f
#define PID_PITCH_KD 0.0f
#define PID_PITCH_INTEGRATION_LIMIT 20.0f
#define PID_YAW_KP   6.0f
#define PID_YAW_KI   1.0f
#define PID_YAW_KD   0.35f
#define PID_YAW_INTEGRATION_LIMIT 360.0f

// Comandos (crtp_commander_rpyt.c, commander.h)
#define MIN_THRUST 1000
// El original limita a 60000 (91.5 %). Con 52 g, motores 720 y bateria de
// 800 mAh el hover medido fue ~69 %: con 91.5 % casi no queda margen para
// corregir, asi que aqui se permite el 100 %.
#define MAX_THRUST 65535
#define WDT_TIMEOUT_STABILIZE_MS 500
#define WDT_TIMEOUT_SHUTDOWN_MS  2000

// Prueba de motores al arrancar (motors.h)
#define MOTORS_TEST_RATIO        ((uint16_t)(0.2f * 65536))
#define MOTORS_TEST_ON_TIME_MS   50
#define MOTORS_TEST_DELAY_TIME_MS 150

// Wi-Fi (Kconfig + wifi_esp32.c)
#define WIFI_BASE_SSID "ESP-DRONE"
#define WIFI_PASSWORD  "12345678"
#define WIFI_CHANNEL   6
#define UDP_PORT       2390

// ============================================================================
// ESTADO GLOBAL
// ============================================================================
WiFiUDP udp;
WebServer server(80);

// Sensores
float gyroX, gyroY, gyroZ;        // grados/s, ejes del cuerpo (Crazyflie)
float accX, accY, accZ;           // g
float gyroBiasX, gyroBiasY, gyroBiasZ;   // LSB, ejes del cuerpo
float accScale = 1.0f;
volatile bool gyroBiasFound = false;
bool accScaleFound = false;
Lpf2p gyroLpf[3], accLpf[3];
int mpuFailCount = 0;

// Estimador
float qw = 1, qx = 0, qy = 0, qz = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;
float gravX = 0, gravY = 0, gravZ = 1;
volatile float stateRoll = 0, statePitch = 0, stateYaw = 0;

// Controlador
PidObject pidRollRate, pidPitchRate, pidYawRate;
PidObject pidRoll, pidPitch, pidYaw;
float attDesRoll = 0, attDesPitch = 0, attDesYaw = 0;
float rateDesRoll = 0, rateDesPitch = 0, rateDesYaw = 0;
int16_t ctrlRoll = 0, ctrlPitch = 0, ctrlYaw = 0;
volatile uint16_t ctrlThrust = 0;
volatile uint16_t motorRatio[4] = {0, 0, 0, 0};
volatile int yawMixSign = YAW_MIX_SIGN_DEFAULT;   // siempre +1 (positivo)
volatile bool yawWrongWarn = false;   // detector de yaw invertido
volatile bool yawLimited = false;     // el mezclador no tuvo margen para todo el yaw
volatile bool motorsSaturated = false;

// Rutina automatica
volatile uint8_t autoPhase = AUTO_OFF;
volatile bool autoReq = false, autoLandReq = false, autoAbort = false;
volatile float autoHoverS = VERT_AIR_S;   // rutina 1: segundos totales en el aire
volatile float autoClimbS = 1.2f;     // segundos de subida despues de despegar (1.2 s ~ 35 cm; antes 0.8 s ~ 16 cm)
volatile float autoOffset = 0;        // ajuste de altura con el joystick (+-12 %)
volatile float liftoffPct = -1;       // hover medido al despegar
float savedHover = 0;                 // hover del vuelo anterior (rampa mas rapida)
float autoThr = 0, autoBase = 0, landBase = 0, satWin = 0, profE = PROFILE_E;
float aLiftF = 0, aBiasF = 0, climbBrakeS = 0, zHold = 0, hovAvg = 0;
bool biasLatched = false;
volatile bool autoNeedT0 = false;   // el arranque espera a terminar la recalibracion
volatile uint8_t autoMission = 1;   // 1 vertical, 2 derecha, 3 vuelta
// Nivel de referencia de la rutina: el angulo que tiene el dron APOYADO antes
// de arrancar es su "cero" (piso o MPU un poco inclinados). Sin esto el PID
// aceleraba los motores del lado "bajo" desde el primer instante.
float levelTrimR = 0, levelTrimP = 0;
#define LEVEL_TRIM_MAX 6.0f
#define AUTO_LINK_LOSS_MS 1500
uint8_t missionStep = 0;            // 0 estabilizando, 1 accion, 2 frenando, 3 estabilizando, 4 listo
uint32_t liftT0 = 0, stepT0 = 0;    // momento del despegue / del paso actual
float missionRoll = 0, missionYawRate = 0, turned = 0;
uint32_t autoT0 = 0;
int liftCnt = 0, landStill = 0, autoTiltCnt = 0;
volatile float vzEst = 0, aVertG = 0, zEst = 0, vzLeakTau = 1.5f;
float baseZacc = 1.0f, baseZcal = 1.0f;   // gravedad de referencia (se aprende en reposo)
volatile const char* uiMsg = "";
void setMsg(const char* m) { uiMsg = m; }

// Integradores congelados en el suelo (no esta en el original). Mientras el
// thrust este por debajo de este %, los integrales de los 6 PID se mantienen
// en 0: apoyado en el suelo el dron no puede corregir y se "cargan" (en el log
// llegaron a r=-17000 p=+22000 y dejaron M3 casi en 0 % antes de despegar).
volatile float integMinThrustPct = 60.0f;
int yawWrongCount = 0;
// Estado "en el aire": se activa al despegar y SOLO se desactiva con thrust 0
// (o rutina en SPIN/RAMP). Asi bajar el thrust en vuelo no borra los integrales.
bool airborne = false;
uint32_t airborneTicks = 0;           // ciclos de 500 Hz en el aire
uint16_t prevSpThrust = 0;            // para detectar el arranque en manual
// Integrales aprendidos del vuelo anterior (se precargan al despegar)
float learnIntegR = 0, learnIntegP = 0, learnIntegRR = 0, learnIntegPR = 0;
bool integLearned = false;
#define INTEG_LEARN_TAU_S   1.0f      // promedio de ~1 s
#define INTEG_LEARN_AFTER   500       // empieza a aprender 1 s despues de despegar
#define INTEG_LEARN_MAX_DEG 10.0f     // solo aprende con el dron casi nivelado

// Comandos
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
RpytCmd lastCmd = {0, 0, 0, 0, 0};
bool thrustLocked = true;
bool wdtTripped = false;
volatile bool emergencyStop = false;
volatile uint8_t lastSource = 0;   // 0 nada, 1 app UDP, 2 web

// Herramientas
volatile int  motorTestIdx = -1;          // prueba de motores por serie
volatile uint32_t motorTestUntil = 0;
volatile uint16_t motorTestRatio = 0;
uint8_t logMode = 1;
volatile float vbat = 0;
volatile bool batComp = false;    // APAGADA: sumaba empuje extra al bajar la bateria ("batcomp 1" para activarla)
char ssidName[33];
TaskHandle_t stabTask = NULL;

// ============================================================================
// FILTRO lpf2p (filter.c)
// ============================================================================
void lpf2pInit(Lpf2p &f, float sampleFreq, float cutoffFreq) {
  float fr = sampleFreq / cutoffFreq;
  float ohm = tanf(M_PI / fr);
  float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
  f.b0 = ohm * ohm / c;
  f.b1 = 2.0f * f.b0;
  f.b2 = f.b0;
  f.a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  f.a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
  f.d1 = 0; f.d2 = 0;
}

float lpf2pApply(Lpf2p &f, float sample) {
  float d0 = sample - f.d1 * f.a1 - f.d2 * f.a2;
  if (!isfinite(d0)) d0 = sample;
  float out = d0 * f.b0 + f.d1 * f.b1 + f.d2 * f.b2;
  f.d2 = f.d1;
  f.d1 = d0;
  return out;
}

// ============================================================================
// PID (pid.c)  -  filtro D desactivado, igual que attitude_pid_controller.c
// ============================================================================
void pidInit(PidObject &p, float kp, float ki, float kd, float dt, float iLimit) {
  memset(&p, 0, sizeof(p));
  p.kp = kp; p.ki = ki; p.kd = kd; p.dt = dt;
  p.iLimit = iLimit; p.outputLimit = 0;
}

void pidReset(PidObject &p) {
  p.error = 0; p.prevError = 0; p.integ = 0; p.deriv = 0;
}

float pidUpdate(PidObject &p, float measured, bool updateError) {
  float output = 0.0f;
  if (updateError) p.error = p.desired - measured;

  p.outP = p.kp * p.error;
  output += p.outP;

  float deriv = (p.error - p.prevError) / p.dt;
  p.deriv = isnan(deriv) ? 0 : deriv;
  p.outD = p.kd * p.deriv;
  output += p.outD;

  p.integ += p.error * p.dt;
  if (p.iLimit != 0) p.integ = constrain(p.integ, -p.iLimit, p.iLimit);
  p.outI = p.ki * p.integ;
  output += p.outI;

  if (p.outputLimit != 0) output = constrain(output, -p.outputLimit, p.outputLimit);
  p.prevError = p.error;
  return output;
}

void attitudeControllerInit() {
  pidInit(pidRollRate,  PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,  ATTITUDE_DT, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidInit(pidPitchRate, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, ATTITUDE_DT, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidInit(pidYawRate,   PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,   ATTITUDE_DT, PID_YAW_RATE_INTEGRATION_LIMIT);
  pidYawRate.outputLimit = YAW_OUTPUT_LIMIT;
  pidInit(pidRoll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ATTITUDE_DT, PID_ROLL_INTEGRATION_LIMIT);
  pidInit(pidPitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, ATTITUDE_DT, PID_PITCH_INTEGRATION_LIMIT);
  pidInit(pidYaw,   PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   ATTITUDE_DT, PID_YAW_INTEGRATION_LIMIT);
}

void attitudeControllerResetAllPID() {
  pidReset(pidRoll); pidReset(pidPitch); pidReset(pidYaw);
  pidReset(pidRollRate); pidReset(pidPitchRate); pidReset(pidYawRate);
}

int16_t saturateSignedInt16(float in) {
  if (in > INT16_MAX) return INT16_MAX;
  if (in < -INT16_MAX) return -INT16_MAX;
  return (int16_t)in;
}

float capAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ============================================================================
// MPU6050
// ============================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  mpuWrite(0x6B, 0x80);          // reset
  delay(100);
  mpuWrite(0x6B, 0x01);          // despierta, reloj PLL con gyro X
  delay(50);
  mpuWrite(0x19, 0x00);          // mpu6050SetRate(0)  -> 1 kHz
  mpuWrite(0x1A, 0x03);          // mpu6050SetDLPFMode(MPU6050_DLPF_BW_42)
  mpuWrite(0x1B, 0x18);          // gyro +-2000 dps
  mpuWrite(0x1C, 0x18);          // acel +-16 g
  delay(50);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);              // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)1, true) != 1) return false;
  uint8_t who = Wire.read();
  Serial.printf("[MPU] WHO_AM_I = 0x%02X\n", who);
  return true;
}

bool mpuReadRaw(int16_t r[7]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t h = Wire.read(), l = Wire.read();
    r[i] = (int16_t)((h << 8) | l);
  }
  return true;
}

// ---- Bias del giroscopio por varianza (processGyroBias) ----
int32_t biasN = 0;
double biasSum[3], biasSq[3];

void biasAddSample(float x, float y, float z) {
  float v[3] = {x, y, z};
  for (int i = 0; i < 3; i++) { biasSum[i] += v[i]; biasSq[i] += (double)v[i] * v[i]; }
  biasN++;
  if (biasN >= BIAS_SAMPLES) {
    bool ok = true;
    float mean[3];
    for (int i = 0; i < 3; i++) {
      mean[i] = biasSum[i] / biasN;
      double var = biasSq[i] / biasN - (double)mean[i] * mean[i];
      if (var > GYRO_VARIANCE_BASE) ok = false;
    }
    if (ok) {
      gyroBiasX = mean[0]; gyroBiasY = mean[1]; gyroBiasZ = mean[2];
      gyroBiasFound = true;
    }
    biasN = 0;
    for (int i = 0; i < 3; i++) { biasSum[i] = 0; biasSq[i] = 0; }
  }
}

// Recalibra el cero del giroscopio (se llama al iniciar cada RUTINA AUTO).
// El MPU6050 cambia su cero con la temperatura: despues del primer vuelo,
// con la placa y los motores calientes, el cero de la calibracion de
// encendido ya no sirve y el PID corrige un giro que no existe.
void gyroRecalibrate() {
  biasN = 0;
  for (int i = 0; i < 3; i++) { biasSum[i] = 0; biasSq[i] = 0; }
  integralFBx = integralFBy = integralFBz = 0;   // tambien el integral del filtro Mahony
  gyroBiasFound = false;                         // sensorsAcquire vuelve a medir ~1 s
}

// ---- Escala del acelerometro (processAccScale) ----
double accScaleSum = 0;
int accScaleCount = 0;

// sensorsAcquire: lee, aplica ejes del ESP32-S2, bias, escala y filtros
bool sensorsAcquire() {
  int16_t r[7];
  if (!mpuReadRaw(r)) return false;

  // Ejes del ESP32-S2 Drone (rama #else de sensors_mpu6050_hm5883L_ms5611.c):
  // accelRaw.y = sensor X, accelRaw.x = sensor Y (lo mismo para el gyro)
  float accRawX = r[1], accRawY = r[0], accRawZ = r[2];
  float gyrRawX = r[5], gyrRawY = r[4], gyrRawZ = r[6];

  if (!gyroBiasFound) {
    biasAddSample(gyrRawX, gyrRawY, gyrRawZ);
    return true;
  }
  if (!accScaleFound) {
    accScaleSum += sqrtf(powf(accRawX * G_PER_LSB, 2) + powf(accRawY * G_PER_LSB, 2) + powf(accRawZ * G_PER_LSB, 2));
    if (++accScaleCount >= ACC_SCALE_SAMPLES) {
      accScale = accScaleSum / ACC_SCALE_SAMPLES;
      accScaleFound = true;
    }
  }

  float gx = -(gyrRawX - gyroBiasX) * DEG_PER_LSB;
  float gy =  (gyrRawY - gyroBiasY) * DEG_PER_LSB;
  float gz =  (gyrRawZ - gyroBiasZ) * DEG_PER_LSB;
  float ax = -(accRawX) * G_PER_LSB / accScale;
  float ay =  (accRawY) * G_PER_LSB / accScale;
  float az =  (accRawZ) * G_PER_LSB / accScale;

  gyroX = lpf2pApply(gyroLpf[0], gx);
  gyroY = lpf2pApply(gyroLpf[1], gy);
  gyroZ = lpf2pApply(gyroLpf[2], gz);
  accX  = lpf2pApply(accLpf[0], ax);
  accY  = lpf2pApply(accLpf[1], ay);
  accZ  = lpf2pApply(accLpf[2], az);
  return true;
}

// ============================================================================
// FUSION MAHONY (sensfusion6.c)
// ============================================================================
float invSqrt(float x) { return 1.0f / sqrtf(x); }

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;
  gx = gx * M_PI / 180; gy = gy * M_PI / 180; gz = gz * M_PI / 180;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    halfvx = qx * qz - qw * qy;
    halfvy = qw * qx + qy * qz;
    halfvz = qw * qw - 0.5f + qz * qz;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    integralFBx += TWO_KI_DEF * halfex * dt;
    integralFBy += TWO_KI_DEF * halfey * dt;
    integralFBz += TWO_KI_DEF * halfez * dt;
    gx += integralFBx; gy += integralFBy; gz += integralFBz;

    gx += TWO_KP_DEF * halfex;
    gy += TWO_KP_DEF * halfey;
    gz += TWO_KP_DEF * halfez;
  }

  gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
  qa = qw; qb = qx; qc = qy;
  qw += (-qb * gx - qc * gy - qz * gz);
  qx += (qa * gx + qc * gz - qz * gy);
  qy += (qa * gy - qb * gz + qz * gx);
  qz += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw *= recipNorm; qx *= recipNorm; qy *= recipNorm; qz *= recipNorm;

  // estimatedGravityDirection
  gravX = 2 * (qx * qz - qw * qy);
  gravY = 2 * (qw * qx + qy * qz);
  gravZ = qw * qw - qx * qx - qy * qy + qz * qz;
}

void sensfusion6GetEulerRPY(float &roll, float &pitch, float &yaw) {
  float gx = constrain(gravX, -1.0f, 1.0f);
  yaw   = atan2f(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / M_PI;
  pitch = asinf(gx) * 180 / M_PI;      // "Pitch seems to be inverted" (original)
  roll  = atan2f(gravY, gravZ) * 180 / M_PI;
}

// Velocidad vertical estimada con el acelerometro (250 Hz)
void verticalEstimatorUpdate(float dt) {
  float a = gravX * accX + gravY * accY + gravZ * accZ - baseZacc;   // g, sin gravedad
  aVertG = a;
  if (vzLeakTau < 10.0f) { if (fabsf(a) < 0.01f) a = 0; else a -= copysignf(0.01f, a); }
  float v = vzEst + a * 9.81f * dt;
  v -= v * dt / vzLeakTau;
  vzEst = v;
  zEst = zEst + v * dt;
}

// ============================================================================
// MOTORES (motors.c + power_distribution_stock.c)
// ============================================================================
void motorsSetRatio(int id, uint16_t ratio) {
  motorRatio[id] = ratio;
  ledcWrite(MOTOR_PIN[id], ratio >> (16 - MOTOR_PWM_BITS));
}

void powerStop() { for (int i = 0; i < 4; i++) motorsSetRatio(i, 0); }

uint16_t limitThrust(int32_t v) { return (uint16_t)constrain(v, (int32_t)0, (int32_t)UINT16_MAX); }

// Mezclador X con PRIORIDADES (correccion del yaw):
//   1) roll y pitch: mantener el dron nivelado
//   2) thrust: si algun motor pasa del tope, baja el de los 4 por igual
//   3) yaw: usa el margen que sobra; si falta, el thrust baja hasta YAW_MAX_SHIFT
void powerDistribution(uint16_t thrust, int16_t roll, int16_t pitch, int16_t yaw) {
  int32_t r = roll / 2;
  int32_t p = pitch / 2;
  int32_t ys = (int32_t)yaw * yawMixSign;
  const int sgn[4] = {+1, -1, +1, -1};          // M1 y M3 suben con yaw +
  const float k = MOTOR_POWER_PCT / 100.0f;     // potencia 104 %
  int32_t m[4];
  m[0] = (int32_t)(((int32_t)thrust - r + p) * k);   // M1 delantero derecho
  m[1] = (int32_t)(((int32_t)thrust - r - p) * k);   // M2 trasero derecho
  m[2] = (int32_t)(((int32_t)thrust + r - p) * k);   // M3 trasero izquierdo
  m[3] = (int32_t)(((int32_t)thrust + r + p) * k);   // M4 delantero izquierdo
  ys = (int32_t)(ys * k);

  bool sat = false;
  int32_t mx = max(max(m[0], m[1]), max(m[2], m[3]));
  if (mx > 65535) { for (int i = 0; i < 4; i++) m[i] -= (mx - 65535); sat = true; }

  // Margen para el yaw: si los motores que deben SUBIR para frenar el giro
  // estan cerca del tope, se baja el thrust de los 4 lo justo (maximo
  // YAW_MAX_SHIFT) para que el yaw tenga todo lo que pide.
  if (ys != 0) {
    int d = ys > 0 ? 1 : -1;
    int32_t want = abs(ys);
    int32_t ceilRoom = 65535, floorRoom = 65535;
    for (int i = 0; i < 4; i++) {
      if (sgn[i] * d > 0) ceilRoom = min(ceilRoom, 65535 - m[i]);
      else                floorRoom = min(floorRoom, m[i]);
    }
    if (ceilRoom < want && floorRoom > want) {
      int32_t shift = min(min(want - ceilRoom, floorRoom - want), (int32_t)YAW_MAX_SHIFT);
      for (int i = 0; i < 4; i++) m[i] -= shift;
    }
  }

  // Yaw: el mayor valor que cabe sin sacar ningun motor de 0..65535
  int32_t yHi = 65535, yLo = -65535;
  for (int i = 0; i < 4; i++) {
    int32_t mi = constrain(m[i], (int32_t)0, (int32_t)65535);
    if (sgn[i] > 0) { yHi = min(yHi, 65535 - mi); yLo = max(yLo, -mi); }
    else            { yHi = min(yHi, mi);         yLo = max(yLo, mi - 65535); }
  }
  yHi = max(yHi, (int32_t)0); yLo = min(yLo, (int32_t)0);
  int32_t yc = constrain(ys, yLo, yHi);
  yawLimited = (abs(ys) > 2000 && abs(yc) < abs(ys) / 2);

  for (int i = 0; i < 4; i++) {
    m[i] += sgn[i] * yc;
    if (m[i] < 0) sat = true;
    motorsSetRatio(i, limitThrust(m[i]));
  }
  motorsSaturated = sat;
}

void motorsTestAtBoot() {      // motorsTest(): cada motor 20 % durante 50 ms
  for (int i = 0; i < 4; i++) {
    motorsSetRatio(i, MOTORS_TEST_RATIO);
    delay(MOTORS_TEST_ON_TIME_MS);
    motorsSetRatio(i, 0);
    delay(MOTORS_TEST_DELAY_TIME_MS);
  }
}

// ============================================================================
// COMANDOS (crtp_commander_rpyt.c + commander.c)
// ============================================================================
void commanderSet(float roll, float pitch, float yaw, uint16_t thrust, uint8_t source) {
  portENTER_CRITICAL(&cmdMux);
  lastCmd.roll = roll; lastCmd.pitch = pitch; lastCmd.yaw = yaw;
  lastCmd.thrust = thrust;
  lastCmd.ms = millis();
  lastSource = source;
  portEXIT_CRITICAL(&cmdMux);
}

// Devuelve el setpoint para el controlador, con watchdog y bloqueo de thrust
void commanderGetSetpoint(float &roll, float &pitch, float &yawRate, uint16_t &thrust) {
  RpytCmd c;
  portENTER_CRITICAL(&cmdMux);
  c = lastCmd;
  portEXIT_CRITICAL(&cmdMux);
  uint32_t age = millis() - c.ms;

  if (c.ms == 0 || age > WDT_TIMEOUT_SHUTDOWN_MS) {         // nullSetpoint: apaga
    roll = 0; pitch = 0; yawRate = 0; thrust = 0;
    wdtTripped = true;                                       // al volver: thrust bloqueado
    return;
  }
  if (age > WDT_TIMEOUT_STABILIZE_MS) {                      // nivelar, mantener Z
    c.roll = 0; c.pitch = 0; c.yaw = 0;
  }
  if (wdtTripped && age <= WDT_TIMEOUT_STABILIZE_MS) {       // comandos de nuevo tras apagado
    wdtTripped = false;
    thrustLocked = true;                                     // exige thrust 0 para rearmar
  }

  if (c.thrust == 0) thrustLocked = false;
  if (emergencyStop) thrustLocked = true;
  if (emergencyStop && c.thrust == 0) emergencyStop = false;

  if (thrustLocked || c.thrust < MIN_THRUST) thrust = 0;
  else thrust = min((uint16_t)c.thrust, (uint16_t)MAX_THRUST);

  roll = c.roll;
  pitch = c.pitch;
  yawRate = -c.yaw;          // "legacy rate input is inverted"
}

// ============================================================================
// RUTINA AUTOMATICA: arranque -> busca el despegue -> sube -> se mantiene ->
// baja -> toca suelo. Solo con el MPU6050 (sin sensor de altura).
// ============================================================================
// Golpe con el suelo: aceleracion vertical > 0.30 g SOSTENIDA 20 ms (10 ciclos).
// Antes bastaba UNA muestra: con los motores desparejos la vibracion daba picos
// de 0.3 g en el aire y la rutina apagaba los motores volando.
#define GROUND_HIT_G      0.30f
#define GROUND_HIT_CYCLES 10
int groundHitCnt = 0;
bool groundHit() {
  if (aVertG > GROUND_HIT_G) groundHitCnt++; else groundHitCnt = 0;
  return groundHitCnt >= GROUND_HIT_CYCLES;
}

void autoGo(uint8_t ph) {
  autoPhase = ph;
  autoT0 = millis();
  groundHitCnt = 0;
  Serial.printf("[AUTO] -> %s\n", AUTO_NAME[ph]);
}

void autoEnd(const char* why) {
  autoPhase = AUTO_OFF;
  missionRoll = 0; missionYawRate = 0;
  vzLeakTau = 1.5f; baseZacc = baseZcal;
  autoThr = 0;
  thrustLocked = true;
  setMsg(why);
  Serial.printf("[AUTO] %s\n", why);
}

void autoLand(const char* why) {
  if (autoPhase == AUTO_SPIN || autoPhase == AUTO_RAMP) { autoEnd(why); return; }   // aun en el suelo
  if (autoPhase == AUTO_HOVER) {                    // bajada normal: espejo de la subida
    if (millis() - autoT0 > 1000) autoBase = hovAvg;
    autoGo(AUTO_DESC);
  } else if (autoPhase == AUTO_CLIMB) {
    landBase = autoBase; landStill = 0; vzLeakTau = 1.5f;   // estimador que se asienta en 0 al apoyar
    autoGo(AUTO_LAND);
  } else return;
  setMsg(why);
  Serial.printf("[AUTO] %s\n", why);
}

uint32_t cmdAgeMs() {
  uint32_t ms;
  portENTER_CRITICAL(&cmdMux);
  ms = lastCmd.ms;
  portEXIT_CRITICAL(&cmdMux);
  return ms == 0 ? 0xFFFFFFFFUL : millis() - ms;
}

void autoStart() {
  if (autoPhase != AUTO_OFF) return;
  const char* err = NULL;
  if (!gyroBiasFound || !accScaleFound) err = "Esperando calibracion (no mover)";
  else if (ctrlThrust > 0) err = "La rutina se inicia desde el suelo";
  else if (cmdAgeMs() > 300) err = "Sin enlace con la pagina web";
  else if (vbat > 0.5f && vbat < AUTO_MIN_VBAT) err = "Bateria baja: cargala a 4.2 V";
  if (err) { setMsg(err); Serial.printf("[AUTO] No arranca: %s\n", err); return; }
  emergencyStop = false;
  autoThr = 0; satWin = 0; liftCnt = 0; vzEst = 0; zEst = 0; aLiftF = 0; autoTiltCnt = 0;
  baseZacc = baseZcal; aBiasF = 0; vzLeakTau = 1.5f; biasLatched = false;   // aBiasF se mide en el ARRANQUE
  liftoffPct = -1;
  gyroRecalibrate();                 // cero del giroscopio nuevo para cada vuelo
  autoNeedT0 = true;
  setMsg("Calibrando giroscopio (no mover)...");
  Serial.printf("[RUTINA] %s | subida %.1f s | hover max %.0f %%\n", MISSION_NAME[autoMission], (float)autoClimbS, HOVER_MAX_PCT);
  Serial.println("[AUTO] Recalibrando giroscopio: no mover el dron...");
  autoGo(AUTO_SPIN);
}

// Devuelve el thrust (0..65535) de la rutina. 500 Hz.
uint16_t autoStep() {
  const float dt = ATTITUDE_DT;
  float t = (millis() - autoT0) / 1000.0f;

  if (autoLandReq) { autoLandReq = false; autoLand("Aterrizaje pedido"); t = 0; }
  // La rutina es automatica: tolera cortes cortos de Wi-Fi (con los motores
  // altos el voltaje cae y la senal se debilita). Sin senal 1.5 s -> aterriza.
  if (cmdAgeMs() > AUTO_LINK_LOSS_MS && autoPhase != AUTO_LAND && autoPhase != AUTO_DESC) { autoLand("Sin senal: aterrizando"); t = 0; }
  if (autoPhase == AUTO_OFF) return 0;

  switch (autoPhase) {
    case AUTO_SPIN:                                   // motores al 30 % en el suelo
      if (autoNeedT0) {                               // primera vez despues de recalibrar
        autoNeedT0 = false; autoT0 = millis(); t = 0;
        levelTrimR = stateRoll; levelTrimP = statePitch;   // el dron apoyado = nivel
        Serial.printf("[AUTO] Giroscopio recalibrado (bias LSB: %.0f %.0f %.0f). Nivel apoyado: R=%.1f P=%.1f (se toma como cero)\n",
                      gyroBiasX, gyroBiasY, gyroBiasZ, levelTrimR, levelTrimP);
        if (fabsf(levelTrimR) > LEVEL_TRIM_MAX || fabsf(levelTrimP) > LEVEL_TRIM_MAX) {
          levelTrimR = levelTrimP = 0;
          autoEnd("Superficie muy inclinada: ponlo en piso plano");
          return 0;
        }
        setMsg("Rutina automatica");
      }
      autoThr = AUTO_SPIN_PCT / 100.0f * fminf(1.0f, t / 0.4f);
      vzEst = 0; zEst = 0;
      // Motores al 30 % y el dron apoyado: se mide el error de cero del
      // acelerometro con vibracion (1.2 s). Con poco tiempo de medicion el error
      // quedaba en ~0.002 g y el dron bajaba ~9 cm/s en el hover sin notarlo.
      if (t > 0.4f) aBiasF += (aVertG - aBiasF) * (dt / 0.5f);
      if (t >= 1.6f) { autoGo(AUTO_RAMP); liftCnt = 0; }
      break;

    case AUTO_RAMP: {                                 // sube PROGRESIVO hasta que despega
      float fastTo = savedHover > 0.3f ? savedHover - AUTO_FAST_MARGIN : 0.50f;   // primer vuelo: rapido hasta 50 %
      float rate = (autoThr < fastTo) ? AUTO_FAST_PCT_S : AUTO_RAMP_PCT_S;
      autoThr = fminf(autoThr + rate / 100.0f * dt, AUTO_MAX_PCT / 100.0f);
      // Apoyado, lo que mide el acelerometro es sesgo por vibracion: se descuenta
      aLiftF += (aVertG - aBiasF - aLiftF) * (dt / 0.03f);
      float learnTo = (savedHover > 0.3f ? savedHover : 0.55f) - 0.12f;
      if (t > 0.3f && aLiftF > LIFT_ACC_G * 0.3f) biasLatched = true;
      if (autoThr < learnTo && !biasLatched) {
        aBiasF += (aVertG - aBiasF) * (dt / 0.5f);
        vzEst = 0; zEst = 0;
      }
      if (vzEst < 0) { vzEst = 0; zEst = 0; }
      if (aLiftF > LIFT_ACC_G) {
        if (++liftCnt > 15) {                         // 30 ms acelerando hacia arriba = DESPEGO
          // Con thrust T acelera a g: el hover es T/(1+a). AQUI DEJA DE SUBIR LA POTENCIA.
          // Hover estimado 1.5 % mas bajo a proposito: si hay que equivocarse,
          // mejor que tienda a bajar y no a subir y subir.
          autoBase = autoThr / (1.0f + fmaxf(aLiftF, 0.0f)) * HOVER_SAFETY;
          savedHover = autoBase;
          liftoffPct = autoBase * 100.0f;
          liftT0 = millis(); missionStep = 0; turned = 0; missionRoll = 0; missionYawRate = 0;
          baseZacc += aBiasF;
          vzLeakTau = 30.0f;
          satWin = 0;
          Serial.printf("[AUTO] DESPEGO con T=%.0f %%. Hover estimado %.0f %%\n", autoThr * 100, (float)liftoffPct);
          if (liftoffPct > HOVER_MAX_PCT) {
            setMsg("Hover mayor a 70 %: carga la bateria");
            Serial.printf("[AUTO] Hover %.0f %% > %.0f %%: no podra sostenerse y bajara solo.\n", (float)liftoffPct, HOVER_MAX_PCT);
          }
          if (liftoffPct > HOVER_ABORT_PCT) {           // sin margen: no seguir, apoyar suave
            Serial.printf("[AUTO] Hover %.0f %% > %.0f %%: sin margen para estabilizar. Aterrizando. CARGA LA BATERIA.\n",
                          (float)liftoffPct, HOVER_ABORT_PCT);
            landBase = autoBase; landStill = 0; vzLeakTau = 1.5f;
            autoGo(AUTO_LAND);
            setMsg("Bateria baja: hover muy alto, aterrizando");
            break;
          }
          if ((autoMission == 2 || autoMission == 3) && autoClimbS < MISSION_CLIMB_S) autoClimbS = MISSION_CLIMB_S;
          float e = fminf(PROFILE_E, 0.92f / autoBase - 1.0f);
          profE = fmaxf(e, 0.03f);
          Serial.printf("[AUTO] Subida %.1f s -> altura aprox. %.0f cm, luego se mantiene %.0f s\n",
                        (float)autoClimbS, 9.81f * profE * powf(autoClimbS / 2, 2) * 100, (float)autoHoverS);
          climbBrakeS = fmaxf(0.0f, vzEst) / (9.81f * profE);
          autoGo(AUTO_CLIMB);
          setMsg("Subiendo");
        }
      } else liftCnt = 0;
      if (autoThr >= AUTO_MAX_PCT / 100.0f && t > (AUTO_MAX_PCT - AUTO_SPIN_PCT) / AUTO_RAMP_PCT_S + 1.0f) {
        autoEnd("No despego: falta empuje (bateria?)");
        return 0;
      }
      break;
    }

    case AUTO_CLIMB: {                                // acelera arriba y luego FRENA
      float half = autoClimbS / 2.0f;
      autoThr = autoBase * (t < half ? 1.0f + profE : 1.0f - profE);
      satWin += ((motorsSaturated ? 1.0f : 0.0f) - satWin) * (dt / 0.5f);
      if (t > 0.3f && satWin > 0.6f) { autoLand("Motores al tope: aterrizando"); t = 0; break; }
      if (t >= autoClimbS + climbBrakeS || (t > half && vzEst < 0.0f)) {
        zHold = zEst; hovAvg = autoBase;
        autoGo(AUTO_HOVER); setMsg("Se mantiene");
      }
      break;
    }

    case AUTO_HOVER: {                                // SE MANTIENE: no sigue subiendo
      // SIN SENSOR DE ALTURA el acelerometro puede creer que baja cuando en
      // realidad sube (vibracion distinta en el aire): si se le dejara AGREGAR
      // empuje, el dron sube y sube. Por eso aqui:
      //  - la correccion automatica solo QUITA empuje (frena si sube),
      //  - el empuje de hover baja muy despacio con el tiempo (tiende a bajar),
      //  - el piloto puede subir/bajar con el joystick izquierdo (autoOffset).
      float vzDes = constrain(1.0f * (zHold - zEst), -0.25f, 0.25f);
      float corr = constrain(VZ_KP * (vzDes - vzEst), -VZ_CORR_MAX, VZ_UP_MAX);
      // (se probo NO bajar el empuje durante las maniobras 2 y 3: con error del
      //  acelerometro volvian a subir hasta 2-8 m. Se mantiene igual para todas.)
      float decay = fminf(HOLD_DECAY_PCT_S / 100.0f * t, HOLD_DECAY_MAX) + (t < 1.0f ? TOP_BRAKE : 0.0f);
      autoThr = constrain(autoBase * (1.0f - decay) + corr + autoOffset, 0.2f, HOVER_MAX_PCT / 100.0f);   // maximo 70 %
      hovAvg += (autoThr - hovAvg) * (dt / 1.0f);
      satWin += ((motorsSaturated ? 1.0f : 0.0f) - satWin) * (dt / 0.5f);
      if (t > 0.5f && satWin > 0.6f) { missionRoll = 0; missionYawRate = 0; autoLand("Motores al tope: aterrizando"); t = 0; break; }
      if (vbat > 0.5f && vbat < 3.3f) { missionRoll = 0; missionYawRate = 0; autoLand("Bateria baja: aterrizando"); t = 0; break; }

      // En el aire NO se apagan los motores por un golpe: se pasa a aterrizaje
      // suave (si de verdad toco el suelo, el aterrizaje termina enseguida).
      if (t > 1.0f && groundHit()) {
        missionRoll = 0; missionYawRate = 0;
        landBase = autoBase; landStill = 0; vzLeakTau = 1.5f;
        autoGo(AUTO_LAND); setMsg("Golpe detectado: aterrizando suave");
        Serial.println("[AUTO] Golpe/vibracion fuerte en el aire: aterrizaje suave (no apaga)");
        break;
      }
      float tAir = (millis() - liftT0) / 1000.0f;   // segundos desde el despegue
      float ts = (millis() - stepT0) / 1000.0f;     // segundos en el paso actual
      bool done = false;
      if (autoMission == 1) {                       // ---- 1 VERTICAL: sube, llega arriba y (tras 'Arriba s') baja
        done = t >= autoHoverS;
      } else if (autoMission == 2) {                // ---- 2 DERECHA
        if (missionStep == 0 && t >= MISSION_STABLE_S) { missionStep = 1; stepT0 = millis(); setMsg("Avanzando a la derecha"); Serial.println("[RUTINA] Avanza a la derecha"); }
        else if (missionStep == 1) { missionRoll = RIGHT_TILT_DEG;  if (ts >= RIGHT_PUSH_S) { missionStep = 2; stepT0 = millis(); } }
        else if (missionStep == 2) { missionRoll = -RIGHT_TILT_DEG; if (ts >= RIGHT_PUSH_S) { missionStep = 3; stepT0 = millis(); missionRoll = 0; setMsg("Estabilizando"); } }
        else if (missionStep == 3) { missionRoll = 0; if (ts >= MISSION_AFTER_S) done = true; }
      } else if (autoMission == 4) {                // ---- DESPEGAR: se mantiene hasta ATERRIZAR
        done = t >= HOLD_MAX_S;
      } else if (autoMission == 3) {                // ---- 3 VUELTA IZQUIERDA (360 grados antihorario)
        if (missionStep == 0 && t >= MISSION_STABLE_S) { missionStep = 1; stepT0 = millis(); turned = 0; setMsg("Dando la vuelta a la izquierda"); Serial.println("[RUTINA] Vuelta de 360 grados a la izquierda"); }
        else if (missionStep == 1) {
          missionYawRate = TURN_RATE_DPS;
          turned += gyroZ * dt;                       // grados girados de verdad (giroscopio)
          if (fabsf(turned) >= 360.0f || ts >= TURN_MAX_S) {
            Serial.printf("[RUTINA] Giro completado: %.0f grados en %.1f s\n", turned, ts);
            missionYawRate = 0; missionStep = 3; stepT0 = millis(); setMsg("Estabilizando");
          }
        }
        else if (missionStep == 3) { missionYawRate = 0; if (ts >= MISSION_AFTER_S) done = true; }
      }
      if (done || (autoMission != 4 && t >= 20.0f)) {
        missionRoll = 0; missionYawRate = 0;
        Serial.printf("[RUTINA] %s: %.1f s en el aire. Aterrizando.\n", MISSION_NAME[autoMission], tAir);
        autoLand("Rutina completa: aterrizando"); t = 0;
      }
      break;
    }

    case AUTO_DESC: {                                 // bajada progresiva (espejo de la subida)
      float half = autoClimbS / 2.0f;
      autoThr = autoBase * (t < half ? 1.0f - profE : 1.0f + profE);
      if (t > 0.3f && groundHit()) {               // toco el suelo bajando: termina suave, no apaga en seco
        landBase = autoBase; landStill = 0; vzLeakTau = 1.5f;
        autoGo(AUTO_LAND);
        break;
      }
      if (t >= autoClimbS) { landBase = autoBase; landStill = 0; vzLeakTau = 1.5f; autoGo(AUTO_LAND); }
      break;
    }

    case AUTO_LAND: {                                 // apoya suave y apaga
      float drop = LAND_START_DROP + LAND_DROP_PCT_S * t;
      float brake = (vzEst < LAND_MAX_VZ) ? constrain(VZ_KP * (LAND_MAX_VZ - vzEst), 0.0f, 0.05f) : 0.0f;
      autoThr = constrain(autoBase - drop + brake, 0.0f, autoBase);
      if (t > 1.0f && vzEst > -0.05f && aVertG < 0.05f) landStill++; else landStill = 0;
      bool touch = (t > 0.4f && groundHit())          // golpe con el suelo (sostenido 20 ms)
                || (drop > 0.08f && landStill > 250)   // quieto con -8 %: esta apoyado
                || (drop > 0.20f);
      if (touch) { autoEnd("Aterrizado, motores apagados"); return 0; }
      break;
    }
  }
  return (uint16_t)(constrain(autoThr, 0.0f, 1.0f) * 65535.0f);
}

// ============================================================================
// CONTROLADOR PID (controller_pid.c) - se llama a 500 Hz
// ============================================================================
void controllerPid() {
  float spRoll, spPitch, spYawRate;
  uint16_t spThrust;
  commanderGetSetpoint(spRoll, spPitch, spYawRate, spThrust);

  // Rutina automatica: ella pone el thrust (roll/pitch/yaw del joystick siguen activos)
  if (autoAbort) { autoAbort = false; autoReq = false; if (autoPhase != AUTO_OFF) autoEnd("Rutina cancelada por STOP"); spThrust = 0; }
  if (autoReq) { autoReq = false; autoStart(); }
  if (autoPhase != AUTO_OFF) {
    spThrust = autoStep();
    if (autoPhase == AUTO_HOVER) {                 // acciones de las rutinas 2 y 3
      spRoll += missionRoll;                       // + = lado derecho abajo = avanza a la derecha
      spYawRate += missionYawRate;                 // + = giro antihorario
    } else { missionRoll = 0; missionYawRate = 0; }
    if (spThrust > 0 && (fabsf(stateRoll - levelTrimR) > AUTO_TILT_CUT || fabsf(statePitch - levelTrimP) > AUTO_TILT_CUT)) {
      if (++autoTiltCnt > 50) { autoEnd("Volcado detectado: motores apagados"); emergencyStop = true; spThrust = 0; }
    } else autoTiltCnt = 0;
  }

  attDesYaw = capAngle(attDesYaw + spYawRate * ATTITUDE_DT);
  attDesRoll = spRoll;
  attDesPitch = spPitch;

  // Yaw: si el mezclador no tuvo margen, los integrales de yaw se congelan
  // (no se cargan) pero NO se borran: siguen compensando el desbalance.
  float yawIntegPrev = pidYaw.integ, yawRateIntegPrev = pidYawRate.integ;
  bool yawFreeze = yawLimited;
  // Cerca del suelo (TOCANDO SUELO, o manual con thrust bajo) los integrales de
  // roll/pitch/yaw se CONGELAN: ni se borran ni crecen. Apoyado el dron no puede
  // corregir el angulo y antes se cargaban hasta el tope (r=32767 en el log del
  // 09/11), descompensando los motores en el suelo.
  bool lowZone = (autoPhase == AUTO_LAND)
              || (autoPhase == AUTO_OFF && spThrust > 0 && spThrust < integMinThrustPct / 100.0f * 65535.0f);
  bool integFreeze = airborne && lowZone;
  float rIntegPrev = pidRoll.integ, pIntegPrev = pidPitch.integ;
  float rrIntegPrev = pidRollRate.integ, prIntegPrev = pidPitchRate.integ;
  if (integFreeze) yawFreeze = true;

  // MANUAL: al arrancar los motores (thrust 0 -> >0) el angulo del dron
  // apoyado se toma como nivel, igual que en la rutina. Si esta muy inclinado
  // (mas de LEVEL_TRIM_MAX) no se usa: se vuela con el cero del acelerometro.
  if (autoPhase == AUTO_OFF && prevSpThrust == 0 && spThrust > 0) {
    if (fabsf(stateRoll) < LEVEL_TRIM_MAX && fabsf(statePitch) < LEVEL_TRIM_MAX) {
      levelTrimR = stateRoll; levelTrimP = statePitch;
    } else {
      levelTrimR = levelTrimP = 0;
    }
  }
  prevSpThrust = spThrust;

  // attitudeControllerCorrectAttitudePID
  // Angulos medidos respecto al nivel del dron apoyado (rutina y manual).
  float measRoll  = stateRoll  - levelTrimR;
  float measPitch = statePitch - levelTrimP;
  // APOYADO: con el nivel ya calibrado el PID SI corrige el angulo (si un lado
  // se levanta antes, lo baja) pero sin integrales (no se acumula nada; ver
  // mas abajo). Probado en vuelo: sin esta correccion el dron pivoteo sobre
  // las patas izquierdas hasta -16 grados antes de despegar.
  bool groundHold = false;
  pidRoll.desired = groundHold ? measRoll : attDesRoll;
  rateDesRoll = pidUpdate(pidRoll, measRoll, true);
  pidPitch.desired = groundHold ? measPitch : attDesPitch;
  rateDesPitch = pidUpdate(pidPitch, measPitch, true);
  float yawError = attDesYaw - stateYaw;
  if (yawError > 180.0f) yawError -= 360.0f;
  else if (yawError < -180.0f) yawError += 360.0f;
  // El rumbo deseado nunca se aleja mas de YAW_MAX_ERROR del real
  if (yawError > YAW_MAX_ERROR)  { yawError = YAW_MAX_ERROR;  attDesYaw = capAngle(stateYaw + YAW_MAX_ERROR); }
  if (yawError < -YAW_MAX_ERROR) { yawError = -YAW_MAX_ERROR; attDesYaw = capAngle(stateYaw - YAW_MAX_ERROR); }
  pidYaw.error = yawError;
  rateDesYaw = pidUpdate(pidYaw, stateYaw, false);

  // attitudeControllerCorrectRatePID(gyro.x, -gyro.y, gyro.z, ...)
  pidRollRate.desired = rateDesRoll;
  ctrlRoll = saturateSignedInt16(pidUpdate(pidRollRate, gyroX, true));
  pidPitchRate.desired = rateDesPitch;
  ctrlPitch = saturateSignedInt16(pidUpdate(pidPitchRate, -gyroY, true));
  pidYawRate.desired = rateDesYaw;
  ctrlYaw = saturateSignedInt16(pidUpdate(pidYawRate, gyroZ, true));
  ctrlYaw = -ctrlYaw;
  if (yawFreeze) { pidYaw.integ = yawIntegPrev; pidYawRate.integ = yawRateIntegPrev; }
  if (integFreeze) {
    pidRoll.integ = rIntegPrev; pidPitch.integ = pIntegPrev;
    pidRollRate.integ = rrIntegPrev; pidPitchRate.integ = prIntegPrev;
  }

  // Detector: si el yaw empuja fuerte y el dron gira HACIA EL MISMO lado que
  // empuja, el sentido esta invertido. ctrlYaw*signo > 0 deberia dar gyroZ < 0.
  float push = (float)ctrlYaw * yawMixSign;
  if (spThrust > 0 && !motorsSaturated && !yawLimited && fabsf(push) > 0.8f * YAW_OUTPUT_LIMIT && push * gyroZ > 0 && fabsf(gyroZ) > 30.0f) {
    if (++yawWrongCount > 300) { yawWrongWarn = true; yawWrongCount = 0; }   // 0.6 s
  } else if (yawWrongCount > 0) {
    yawWrongCount--;
  }

  // Integrales en 0 mientras el dron sigue APOYADO: en el suelo no puede
  // corregir y se "cargan"; al despegar lo tiraban hacia un lado. En la rutina
  // automatica se mantienen en 0 hasta que el acelerometro detecta el despegue.
  // "En el aire" se ENGANCHA: se activa al despegar y solo se suelta con thrust 0.
  // (Antes se usaba thrust < 60 % en cada ciclo: con el hover estimado bajo, el
  //  freno de arriba o la bajada, los integrales se borraban EN PLENO VUELO.)
  bool wasAirborne = airborne;
  if (spThrust == 0 || autoPhase == AUTO_SPIN || autoPhase == AUTO_RAMP) airborne = false;
  else if (autoPhase != AUTO_OFF) airborne = true;   // CLIMB, HOVER, DESC, LAND: ya despego
  else if (spThrust >= integMinThrustPct / 100.0f * 65535.0f) airborne = true;   // manual
  airborneTicks = airborne ? airborneTicks + 1 : 0;

  if (!airborne) {
    pidRoll.integ = 0; pidPitch.integ = 0; pidYaw.integ = 0;
    pidRollRate.integ = 0; pidPitchRate.integ = 0; pidYawRate.integ = 0;
    attDesYaw = stateYaw;            // apoyado no puede girar: no acumular error de rumbo
  } else if (!wasAirborne) {
    // DESPEGUE: se precarga lo que el PID necesito en el vuelo anterior para
    // mantenerse nivelado. Asi compensa el desbalance desde el primer instante
    // en vez de ladearse mientras los integrales crecen desde 0.
    if (integLearned) {
      pidRoll.integ = learnIntegR;   pidPitch.integ = learnIntegP;
      pidRollRate.integ = learnIntegRR; pidPitchRate.integ = learnIntegPR;
      Serial.printf("[PID] Despegue con integrales precargados: ang R=%.2f P=%.2f | rate R=%.2f P=%.2f\n",
                    learnIntegR, learnIntegP, learnIntegRR, learnIntegPR);
    }
  } else if (airborneTicks > INTEG_LEARN_AFTER && !motorsSaturated
             && missionRoll == 0 && missionYawRate == 0
             && autoPhase != AUTO_DESC && autoPhase != AUTO_LAND
             && fabsf(measRoll) < INTEG_LEARN_MAX_DEG && fabsf(measPitch) < INTEG_LEARN_MAX_DEG) {
    // En el aire y estable: aprender (promedio lento) los integrales de roll/pitch
    float k = ATTITUDE_DT / INTEG_LEARN_TAU_S;
    if (!integLearned) {
      learnIntegR = pidRoll.integ; learnIntegP = pidPitch.integ;
      learnIntegRR = pidRollRate.integ; learnIntegPR = pidPitchRate.integ;
      integLearned = true;
    } else {
      learnIntegR  += (pidRoll.integ      - learnIntegR)  * k;
      learnIntegP  += (pidPitch.integ     - learnIntegP)  * k;
      learnIntegRR += (pidRollRate.integ  - learnIntegRR) * k;
      learnIntegPR += (pidPitchRate.integ - learnIntegPR) * k;
    }
  }

  ctrlThrust = spThrust;
  if (ctrlThrust == 0) {
    ctrlRoll = 0; ctrlPitch = 0; ctrlYaw = 0;
    attitudeControllerResetAllPID();
    attDesYaw = stateYaw;
  }
}

// ============================================================================
// TAREA DEL ESTABILIZADOR (stabilizer.c) - 1000 Hz
// ============================================================================
void stabilizerTask(void *arg) {
  TickType_t last = xTaskGetTickCount();
  uint32_t tick = 0;
  bool readyAnnounced = false;

  for (;;) {
    vTaskDelayUntil(&last, 1);       // 1 tick = 1 ms
    tick++;

    if (sensorsAcquire()) mpuFailCount = 0;
    else if (++mpuFailCount > 20) { powerStop(); continue; }

    if (!gyroBiasFound || !accScaleFound) { powerStop(); continue; }
    if (!readyAnnounced) {
      readyAnnounced = true;
      Serial.println("[SENSORS] isBiasValueFound!");
      Serial.println("[STAB] Ready to fly.");
    }

    if (tick % (RATE_MAIN_LOOP / ESTIMATOR_RATE) == 0) {
      sensfusion6UpdateQ(gyroX, gyroY, gyroZ, accX, accY, accZ, ESTIMATOR_DT);
      float r, p, y;
      sensfusion6GetEulerRPY(r, p, y);
      stateRoll = r; statePitch = p; stateYaw = y;
      // Gravedad de referencia: se aprende con motores apagados y el dron quieto
      if (ctrlThrust == 0 && autoPhase == AUTO_OFF && fabsf(gyroX) < 5 && fabsf(gyroY) < 5 && fabsf(gyroZ) < 5) {
        float g = gravX * accX + gravY * accY + gravZ * accZ;
        if (g > 0.9f && g < 1.1f) baseZcal += (g - baseZcal) * 0.004f;   // ~1 s
        if (autoPhase == AUTO_OFF) baseZacc = baseZcal;
      }
      verticalEstimatorUpdate(ESTIMATOR_DT);
    }

    // Prueba de motores pedida por serie (solo con thrust 0)
    if (motorTestIdx >= 0) {
      if (millis() < motorTestUntil && ctrlThrust == 0) {
        for (int i = 0; i < 4; i++) motorsSetRatio(i, i == motorTestIdx ? motorTestRatio : 0);
        continue;
      }
      motorTestIdx = -1;
      powerStop();
    }

    if (tick % (RATE_MAIN_LOOP / ATTITUDE_RATE) == 0) {
      controllerPid();
      if (emergencyStop || ctrlThrust == 0) { powerStop(); motorsSaturated = false; yawLimited = false; }
      else {
        float v = vbat;
        float k = (batComp && v > 3.0f && v < 4.4f) ? constrain(VBAT_REF / v, 1.0f, BAT_COMP_MAX) : 1.0f;
        uint16_t t = (uint16_t)min(65535.0f, ctrlThrust * k);
        powerDistribution(t, ctrlRoll, ctrlPitch, ctrlYaw);
      }
    }
  }
}

// ============================================================================
// UDP: app ESP-Drone / cfclient (wifi_esp32.c + crtp)
// ============================================================================
void handleUdp() {
  for (int k = 0; k < 8; k++) {
    int len = udp.parsePacket();
    if (len <= 0) return;
    uint8_t buf[64];
    int n = udp.read(buf, sizeof(buf));
    if (n < 2) continue;
    uint8_t sum = 0;
    for (int i = 0; i < n - 1; i++) sum += buf[i];
    if (sum != buf[n - 1]) continue;              // checksum
    uint8_t port = (buf[0] >> 4) & 0x0F;
    uint8_t chan = buf[0] & 0x03;
    if (port == 3 && chan == 0 && n - 1 >= 15) {  // CRTP_PORT_SETPOINT, rpyt
      float r, p, y; uint16_t t;
      memcpy(&r, &buf[1], 4); memcpy(&p, &buf[5], 4);
      memcpy(&y, &buf[9], 4); memcpy(&t, &buf[13], 2);
      commanderSet(r, p, y, t, 1);
    }
  }
}

// ============================================================================
// PAGINA WEB: joystick (reemplaza la app)
// ============================================================================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP-Drone</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-user-select:none;user-select:none;touch-action:none}
body{background:#111;color:#eee;font-family:Arial,sans-serif;height:100vh;display:flex;flex-direction:column;overflow:hidden}
#top{display:flex;flex-wrap:wrap;gap:6px 14px;justify-content:center;align-items:center;padding:6px;font-size:13px}
#top b{color:#00e5ff}
#st{font-weight:bold}
#main{flex:1;display:flex;gap:10px;padding:8px;min-height:0}
.pad{flex:1;position:relative;border:2px solid #333;border-radius:14px;background:#1a1a1a}
.lbl{position:absolute;top:6px;left:0;right:0;text-align:center;font-size:12px;color:#888}
.knob{position:absolute;width:56px;height:56px;margin:-28px;border-radius:50%;background:#00a5c8;opacity:.85}
#bar{position:absolute;left:8px;bottom:8px;width:10px;border-radius:5px;background:#00e5ff}
#mid{display:flex;flex-direction:column;gap:6px;width:124px;overflow-y:auto;touch-action:pan-y}
#mid *{touch-action:pan-y}
button{border:none;border-radius:10px;padding:11px 6px;font-weight:bold;color:#fff;font-size:14px;flex-shrink:0}
#stop{background:#c00;font-size:18px;padding:18px 6px}
#trim{background:#333;font-size:12px}
.opt{font-size:11px;color:#aaa;display:flex;gap:4px;align-items:center}
</style></head><body>
<div id="top"><span id="st">--</span>
<span>R <b id="r">0</b></span><span>P <b id="p">0</b></span><span>Y <b id="y">0</b></span>
<span>T <b id="t">0%</b></span><span>M <b id="m">0 0 0 0</b></span><span>BAT <b id="v">--</b></span>
<span>Hover <b id="lo">--</b></span><span id="msg" style="width:100%;text-align:center;color:#fc6;font-size:12px"></span></div>
<div id="main">
 <div class="pad" id="L"><div class="lbl">THRUST (arriba/abajo) · YAW (lados)</div><div id="bar"></div><div class="knob" id="kL"></div></div>
 <div id="mid">
  <button id="stop">STOP</button>
  <button class="rt" data-m="4" style="background:#0a7">DESPEGAR</button>
  <button class="rt" data-m="1" style="background:#06c">1 VERTICAL</button>
  <button class="rt" data-m="2" style="background:#0789a8">2 DERECHA</button>
  <button class="rt" data-m="3" style="background:#5a3fb0">3 VUELTA IZQ.</button>
  <button id="ld" style="background:#c70">ATERRIZAR</button>
  <div class="opt">Subida s<input id="cs" type="number" min="0" max="2" step="0.05" value="1.2" style="width:50px"></div>
  <div class="opt">Arriba s<input id="hs" type="number" min="0" max="20" step="0.5" value="5" style="width:50px"></div>
  <div class="opt">Hover %<input id="hv" type="number" min="30" max="95" value="%HOVER%" style="width:50px"></div>
  <div class="opt">Max %<input id="mx" type="number" min="30" max="100" value="100" style="width:50px"></div>
  <div class="opt">Ang max<input id="am" type="number" min="5" max="30" value="15" style="width:50px"></div>
  <label class="opt"><input type="checkbox" id="ip">Invertir pitch</label>
  <label class="opt"><input type="checkbox" id="ir">Invertir roll</label>
 </div>
 <div class="pad" id="R"><div class="lbl">ROLL / PITCH</div><div class="knob" id="kR"></div></div>
</div>
<script>
var thr=0,yaw=0,roll=0,pitch=0,busy=false,stopped=false,autoOn=false;
function $(i){return document.getElementById(i)}
function ls(k,d){try{var v=localStorage.getItem(k);return v===null?d:v}catch(e){return d}}
function ss(k,v){try{localStorage.setItem(k,v)}catch(e){}}
$('mx').value=ls('mx',100);$('hv').value=ls('hv70',%HOVER%);$('hv').onchange=function(){ss('hv70',$('hv').value)};$('am').value=ls('am',15);$('ip').checked=ls('ip','0')=='1';$('ir').checked=ls('ir','0')=='1';
['mx','am','cs','hs'].forEach(function(k){$(k).value=ls(k+'4',$(k).value);$(k).onchange=function(){ss(k+'4',$(k).value)}});
['ip','ir'].forEach(function(k){$(k).onchange=function(){ss(k,$(k).checked?'1':'0')}});
function place(){
 var L=$('L'),R=$('R');
 $('kL').style.left=(L.clientWidth/2+yaw*L.clientWidth/2*0.9)+'px';
 $('kL').style.top=(L.clientHeight*(1-thr)*0.9+L.clientHeight*0.05)+'px';
 $('bar').style.height=(thr*(L.clientHeight-16))+'px';
 $('kR').style.left=(R.clientWidth/2+roll*R.clientWidth/2*0.9)+'px';
 $('kR').style.top=(R.clientHeight/2-pitch*R.clientHeight/2*0.9)+'px';
}
function pad(id,fn,end){
 var el=$(id),pid=null;
 el.addEventListener('pointerdown',function(e){pid=e.pointerId;el.setPointerCapture(pid);fn(e,el);place()});
 el.addEventListener('pointermove',function(e){if(e.pointerId===pid){fn(e,el);place()}});
 function up(e){if(e.pointerId===pid){pid=null;end();place()}}
 el.addEventListener('pointerup',up);el.addEventListener('pointercancel',up);
}
function rel(e,el){var b=el.getBoundingClientRect();return{x:(e.clientX-b.left)/b.width,y:(e.clientY-b.top)/b.height}}
function cl(v){return Math.max(-1,Math.min(1,v))}
pad('L',function(e,el){var p=rel(e,el);if(stopped)return;if(anim){clearInterval(anim);anim=null}thr=Math.max(0,Math.min(1,(0.95-p.y)/0.9));yaw=cl((p.x-0.5)*2)},function(){yaw=0});
pad('R',function(e,el){var p=rel(e,el);roll=cl((p.x-0.5)*2);pitch=cl((0.5-p.y)*2)},function(){roll=0;pitch=0});
var anim=null;
document.querySelectorAll('.rt').forEach(function(b){b.onclick=function(){
 if(stopped||autoOn)return;if(anim){clearInterval(anim);anim=null}thr=0.5;autoOn=true;place();
 var c=+$('cs').value;if(!(c>=0))c=0.8;
 fetch('/auto?m='+b.dataset.m+'&c='+Math.max(0,Math.min(2,c))+'&d='+Math.max(0,Math.min(20,+$('hs').value||0)))}});
$('ld').onclick=function(){fetch('/land')};
$('stop').onclick=function(){autoOn=false;if(anim){clearInterval(anim);anim=null}thr=0;yaw=0;roll=0;pitch=0;stopped=true;fetch('/kill');place();setTimeout(function(){stopped=false},800)};
// DESPEGAR: ahora es automatico. Sube progresivo, DETECTA el despegue, deja de
// subir la potencia y SE MANTIENE hasta que pulses ATERRIZAR (max 30 s).
// El joystick izquierdo corrige la altura (centro = mantener).
function send(){
 if(busy)return;busy=true;
 var mx=Math.max(30,Math.min(100,+$('mx').value||90))/100,am=+$('am').value||15;
 var t=(!autoOn&&thr>0.02)?Math.round((0.25+thr*(mx-0.25))*65535):0;
 // en RUTINA AUTO el joystick izquierdo solo corrige la altura (centro = mantener)
 var ao=autoOn?'&ao='+((thr-0.5)*0.24).toFixed(3):'';
 // Convencion del original: pitch + = nariz ARRIBA, roll + = lado derecho ABAJO.
 // Joystick arriba = avanzar = nariz abajo -> pitch negativo.
 var r=roll*am*($('ir').checked?-1:1),p=-pitch*am*($('ip').checked?-1:1);
 // zona muerta 0.2 como la app: mover el thrust no debe girar el dron
 var yd=Math.abs(yaw)<0.2?0:(yaw-Math.sign(yaw)*0.2)/0.8,y=yd*90;
 fetch('/c?r='+r.toFixed(2)+'&p='+p.toFixed(2)+'&y='+y.toFixed(1)+'&t='+t+ao).then(function(){busy=false}).catch(function(){busy=false});
}
setInterval(send,50);
setInterval(function(){fetch('/t').then(function(x){return x.json()}).then(function(d){
 var was=autoOn;autoOn=d.a>0;if(was&&!autoOn){thr=0;place()}if(!was&&autoOn){thr=0.5;place()}
 $('st').innerText=d.s;$('st').style.color=(d.s=='VOLANDO'||d.s.indexOf('AUTO')==0)?'#0f8':(d.s=='LISTO'?'#0cf':'#fa0');
 $('lo').innerText=d.lo>0?d.lo+' %':'--';$('msg').innerText=d.msg;
 $('r').innerText=d.r;$('p').innerText=d.p;$('y').innerText=d.y;$('t').innerText=d.t+'%';$('m').innerText=d.m.join(' ');
 $('v').innerText=d.v>0.5?d.v.toFixed(2)+' V':'--';$('v').style.color=d.v>=3.8?'#0f8':(d.v>=3.5?'#fa0':'#f33');
}).catch(function(){})},300);
window.onresize=place;place();
</script></body></html>)HTML";

void handleRoot() {
  String html = FPSTR(PAGE);
  html.replace("%HOVER%", String(HOVER_PCT_DEFAULT));
  html.replace("%RAMP%", String(TAKEOFF_RAMP_MS));
  server.send(200, "text/html", html);
}

void handleCmd() {
  float r = server.arg("r").toFloat();
  float p = server.arg("p").toFloat();
  float y = server.arg("y").toFloat();
  long t = server.arg("t").toInt();
  autoOffset = server.hasArg("ao") ? constrain(server.arg("ao").toFloat(), -0.12f, 0.12f) : 0.0f;
  commanderSet(constrain(r, -30.0f, 30.0f), constrain(p, -30.0f, 30.0f),
               constrain(y, -200.0f, 200.0f), (uint16_t)constrain(t, 0L, 65535L), 2);
  server.send(200, "text/plain", "ok");
}

void handleAuto() {
  if (server.hasArg("c")) autoClimbS = constrain(server.arg("c").toFloat(), 0.0f, 2.0f);
  if (server.hasArg("d")) autoHoverS = constrain(server.arg("d").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("m")) autoMission = constrain((int)server.arg("m").toInt(), 1, 4);
  autoReq = true;
  server.send(200, "text/plain", "ok");
}
void handleLand() { autoLandReq = true; server.send(200, "text/plain", "ok"); }

void handleKill() {
  emergencyStop = true;
  autoAbort = true;
  powerStop();
  commanderSet(0, 0, 0, 0, 2);
  Serial.println("[STOP] Parada desde la pagina web");
  server.send(200, "text/plain", "ok");
}

const char* stateName() {
  if (!gyroBiasFound || !accScaleFound) return "CALIBRANDO (no mover)";
  if (emergencyStop) return "STOP";
  if (autoPhase == AUTO_SPIN && autoNeedT0) return "AUTO: CALIBRANDO (no mover)";
  if (autoPhase == AUTO_SPIN || autoPhase == AUTO_RAMP) return "AUTO: DESPEGANDO";
  if (autoPhase == AUTO_CLIMB) return "AUTO: SUBIENDO";
  if (autoPhase == AUTO_HOVER) {
    if (autoMission == 2 && (missionStep == 1 || missionStep == 2)) return "AUTO: AVANZANDO DERECHA";
    if (autoMission == 3 && missionStep == 1) return "AUTO: VUELTA A LA IZQUIERDA";
    return "AUTO: SE MANTIENE";
  }
  if (autoPhase == AUTO_DESC || autoPhase == AUTO_LAND) return "AUTO: ATERRIZANDO";
  if (ctrlThrust > 0) return "VOLANDO";
  if (thrustLocked) return "BLOQUEADO: baja thrust a 0";
  return "LISTO";
}

void handleTelemetry() {
  char buf[360];
  snprintf(buf, sizeof(buf),
           "{\"s\":\"%s\",\"r\":%.1f,\"p\":%.1f,\"y\":%.0f,\"t\":%d,\"m\":[%d,%d,%d,%d],\"v\":%.2f,\"a\":%d,\"lo\":%.0f,\"msg\":\"%s\"}",
           stateName(), stateRoll, statePitch, stateYaw, (int)(ctrlThrust * 100L / 65535),
           (int)(motorRatio[0] * 100L / 65535), (int)(motorRatio[1] * 100L / 65535),
           (int)(motorRatio[2] * 100L / 65535), (int)(motorRatio[3] * 100L / 65535), vbat,
           (int)autoPhase, (float)(liftoffPct > 0 ? liftoffPct : savedHover * 100.0f), (const char*)uiMsg);
  server.send(200, "application/json", buf);
}

// ============================================================================
// MONITOR SERIE
// ============================================================================
void printHelp() {
  Serial.println(F("\n=========== COMANDOS ==========="));
  Serial.println(F("help             esta ayuda"));
  Serial.println(F("s                PARADA (motores a 0)"));
  Serial.println(F("auto <1|2|3> [sub] rutina: 1 vertical 7 s, 2 derecha, 3 vuelta izquierda (ej: auto 2 0.8)"));
  Serial.println(F("land             aterrizar ya (rutina automatica)"));
  Serial.println(F("seq              gira M1, M2, M3, M4 por turnos 2 s a 20 % (SIN HELICES)"));
  Serial.println(F("m <1-4> <pct>    gira un motor 2 s al pct indicado"));
  Serial.println(F("bat              voltaje de bateria"));
  Serial.println(F("pid              ver ganancias actuales"));
  Serial.println(F("set <nombre> <v> cambia una ganancia en vivo (se pierde al apagar, como en cfclient)"));
  Serial.println(F("                 nombres: rr_kp rr_ki rr_kd pr_kp pr_ki pr_kd yr_kp yr_ki"));
  Serial.println(F("                          r_kp r_ki p_kp p_ki y_kp y_ki y_kd"));
  Serial.println(F("log <0|1>        telemetria en vuelo apagada/encendida"));
  Serial.println(F("ilim <pct>       thrust desde el que empiezan a integrar los PID (por defecto 60)"));
  Serial.println(F("batcomp <0|1>    compensacion de thrust por voltaje de bateria (por defecto 1)"));
  Serial.println(F("================================\n"));
}

void printPid() {
  Serial.printf("Velocidad roll : kp=%.1f ki=%.1f kd=%.2f\n", pidRollRate.kp, pidRollRate.ki, pidRollRate.kd);
  Serial.printf("Velocidad pitch: kp=%.1f ki=%.1f kd=%.2f\n", pidPitchRate.kp, pidPitchRate.ki, pidPitchRate.kd);
  Serial.printf("Velocidad yaw  : kp=%.1f ki=%.1f\n", pidYawRate.kp, pidYawRate.ki);
  Serial.printf("Angulo roll    : kp=%.2f ki=%.2f\n", pidRoll.kp, pidRoll.ki);
  Serial.printf("Angulo pitch   : kp=%.2f ki=%.2f\n", pidPitch.kp, pidPitch.ki);
  Serial.printf("Angulo yaw     : kp=%.2f ki=%.2f kd=%.2f\n", pidYaw.kp, pidYaw.ki, pidYaw.kd);
}

float* pidParam(const String &n) {
  if (n == "rr_kp") return &pidRollRate.kp;  if (n == "rr_ki") return &pidRollRate.ki;  if (n == "rr_kd") return &pidRollRate.kd;
  if (n == "pr_kp") return &pidPitchRate.kp; if (n == "pr_ki") return &pidPitchRate.ki; if (n == "pr_kd") return &pidPitchRate.kd;
  if (n == "yr_kp") return &pidYawRate.kp;   if (n == "yr_ki") return &pidYawRate.ki;
  if (n == "r_kp")  return &pidRoll.kp;      if (n == "r_ki")  return &pidRoll.ki;
  if (n == "p_kp")  return &pidPitch.kp;     if (n == "p_ki")  return &pidPitch.ki;
  if (n == "y_kp")  return &pidYaw.kp;       if (n == "y_ki")  return &pidYaw.ki;  if (n == "y_kd") return &pidYaw.kd;
  return NULL;
}

void startMotorTest(int idx, float pct) {
  if (ctrlThrust > 0) { Serial.println("[TEST] Solo con thrust 0"); return; }
  motorTestRatio = (uint16_t)(constrain(pct, 0.0f, 60.0f) / 100.0f * 65535);
  motorTestUntil = millis() + 2000;
  motorTestIdx = idx;
}

int seqStep = -1;
uint32_t seqNext = 0;

void runSeq() {
  if (seqStep < 0 || millis() < seqNext) return;
  if (seqStep >= 4) { seqStep = -1; Serial.println("[SEQ] Fin."); return; }
  const char* donde[4] = {"DELANTERO DERECHO, antihorario", "TRASERO DERECHO, horario",
                          "TRASERO IZQUIERDO, antihorario", "DELANTERO IZQUIERDO, horario"};
  Serial.printf("[SEQ] Gira M%d (pin %d): debe ser %s visto desde arriba\n",
                seqStep + 1, MOTOR_PIN[seqStep], donde[seqStep]);
  startMotorTest(seqStep, 20);
  seqStep++;
  seqNext = millis() + 3000;
}

void handleSerial() {
  static char line[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (n < sizeof(line) - 1) line[n++] = ch; continue; }
    line[n] = 0; n = 0;
    char c[16] = {0}; float a = 0, b = 0;
    int k = sscanf(line, "%15s %f %f", c, &a, &b);
    if (k < 1) continue;
    String cmd(c); cmd.toLowerCase();

    if (cmd == "help") printHelp();
    else if (cmd == "s") { emergencyStop = true; autoAbort = true; powerStop(); seqStep = -1; motorTestIdx = -1; Serial.println("[STOP]"); }
    else if (cmd == "auto") {                 // auto <1|2|3> [subida s]
      if (k >= 2) autoMission = constrain((int)a, 1, 4);
      if (k >= 3) autoClimbS = constrain(b, 0.0f, 2.0f);
      autoReq = true;
    }
    else if (cmd == "land") autoLandReq = true;
    else if (cmd == "seq") { seqStep = 0; seqNext = 0; Serial.println("[SEQ] SIN HELICES. Mira desde arriba con el frente hacia adelante."); }
    else if (cmd == "m" && k >= 3) startMotorTest(constrain((int)a, 1, 4) - 1, b);
    else if (cmd == "bat") Serial.printf("[BAT] %.2f V\n", vbat);
    else if (cmd == "pid") printPid();
    else if (cmd == "log" && k >= 2) logMode = (uint8_t)a;
    else if (cmd == "batcomp" && k >= 2) {
      batComp = (a != 0);
      Serial.printf("[BAT] Compensacion por voltaje %s\n", batComp ? "ACTIVADA" : "desactivada");
    }
    else if (cmd == "ilim" && k >= 2) {
      integMinThrustPct = constrain(a, 0.0f, 90.0f);
      Serial.printf("[ILIM] Los integrales empiezan a %.0f %% de thrust\n", integMinThrustPct);
    }
    else if (cmd == "yawinv") {
      Serial.println("[YAW] El signo del yaw queda FIJO en +1 (positivo). Comando desactivado.");
    }
    else if (cmd == "yawinv_desactivado") {
      if (ctrlThrust > 0) Serial.println("[YAW] Solo con thrust 0");
      else { yawMixSign = -yawMixSign; Serial.printf("[YAW] signo del yaw en el mezclador = %+d\n", yawMixSign); }
    }
    else if (cmd == "set") {
      char name[16] = {0}; float v = 0;
      if (sscanf(line, "%*s %15s %f", name, &v) == 2) {
        float* p = pidParam(String(name));
        if (p) { *p = v; Serial.printf("[SET] %s = %.3f\n", name, v); }
        else Serial.println("[SET] Nombre desconocido (escribe help)");
      }
    }
    else Serial.println("[?] Escribe help");
  }
}

void telemetry() {
  static uint32_t last = 0, lastBat = 0;
  uint32_t now = millis();
  if (yawWrongWarn) {
    yawWrongWarn = false;
    Serial.println("[YAW] AVISO: el dron gira hacia el lado que el yaw empuja -> sentido invertido. "
                   "Revisa helices A/B y sentido de giro de los motores (seq). El signo del yaw queda en +1.");
  }
  if (now - lastBat >= 200) {
    lastBat = now;
    float v = analogReadMilliVolts(VBAT_PIN) * VBAT_MULT / 1000.0f;
    vbat = (vbat < 0.5f) ? v : 0.8f * vbat + 0.2f * v;
  }
  if (logMode && ctrlThrust > 0 && now - last >= 250) {
    last = now;
    if (autoPhase != AUTO_OFF) Serial.printf("[AUTO %s] vz=%5.2f ", AUTO_NAME[autoPhase], (float)vzEst);
    Serial.printf("[VUELO] T=%3d%% | R=%6.1f P=%6.1f Y=%6.1f | M=[%3d %3d %3d %3d]%% | out r=%6d p=%6d y=%6d | %s | V=%.2f%s%s\n",
                  (int)(ctrlThrust * 100L / 65535), stateRoll, statePitch, stateYaw,
                  (int)(motorRatio[0] * 100L / 65535), (int)(motorRatio[1] * 100L / 65535),
                  (int)(motorRatio[2] * 100L / 65535), (int)(motorRatio[3] * 100L / 65535),
                  ctrlRoll, ctrlPitch, ctrlYaw, lastSource == 1 ? "app" : "web",
                  vbat, vbat > 0.5f && vbat < PM_BAT_LOW_VOLTAGE ? " BATERIA BAJA" : "",
                  motorsSaturated ? " SATURADO" : (yawLimited ? " YAW-SIN-MARGEN" : ""));
  }
}

// ============================================================================
void setup() {
  for (int i = 0; i < 4; i++) { pinMode(MOTOR_PIN[i], OUTPUT); digitalWrite(MOTOR_PIN[i], LOW); }
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);          // USB: nunca esperar al PC (antes bloqueaba con el cable conectado)
#endif
  delay(800);
  Serial.println("\n=== ESP_DRONE_ARDUINO dia 28 + yaw corregido + rutina automatica ===");
  Serial.printf("    Potencia motores %.0f %% | hover %d %% | rampa de despegue %d ms\n", MOTOR_POWER_PCT, HOVER_PCT_DEFAULT, TAKEOFF_RAMP_MS);
  Serial.println("    Configurado para: 52 g, bateria 1S 800 mAh, motores 720, thrust max 100 %");

  for (int i = 0; i < 4; i++) ledcAttach(MOTOR_PIN[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  powerStop();
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  if (!mpuInit()) Serial.println("[MPU] ERROR: no responde (SDA 11, SCL 10)");
  for (int i = 0; i < 3; i++) {
    lpf2pInit(gyroLpf[i], 1000, GYRO_LPF_HZ);
    lpf2pInit(accLpf[i], 1000, ACCEL_LPF_HZ);
  }
  attitudeControllerInit();

  // Wi-Fi como el original: ESP-DRONE_<MAC>, 192.168.43.42
  uint8_t mac[6];
  WiFi.mode(WIFI_AP);
  WiFi.softAPmacAddress(mac);
  snprintf(ssidName, sizeof(ssidName), "%s_%02X%02X%02X%02X%02X%02X",
           WIFI_BASE_SSID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  WiFi.softAPConfig(IPAddress(192, 168, 43, 42), IPAddress(192, 168, 43, 42), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssidName, WIFI_PASSWORD, WIFI_CHANNEL, 0, 4);
  udp.begin(UDP_PORT);

  server.on("/", handleRoot);
  server.on("/c", handleCmd);
  server.on("/kill", handleKill);
  server.on("/auto", handleAuto);
  server.on("/land", handleLand);
  server.on("/t", handleTelemetry);
  server.begin();
  Serial.printf("[WIFI] Red: %s  clave: %s  -> http://192.168.43.42  (UDP %d)\n", ssidName, WIFI_PASSWORD, UDP_PORT);

  motorsTestAtBoot();
  Serial.println("[SENSORS] Calibrando giroscopio: deja el dron QUIETO y PLANO...");
  xTaskCreate(stabilizerTask, "STABILIZER", 6144, NULL, 7, &stabTask);
  printHelp();
}

void loop() {
  handleUdp();
  server.handleClient();
  handleSerial();
  runSeq();
  telemetry();
  delay(1);
}