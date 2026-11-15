// lidar_torreta.ino — torreta 3D con TF-Luna en un ESP32 dedicado (2º ESP32 del rover).
// Port del MK2 de Charles Grassin (MIT, ../3d_lidar_CGrassin): espejo girado por un NEMA17
// (DRV8825) e inclinado por 2 servos. Cambios: ESP32, motor a pasos por timer (sin bloquear),
// lectura del TF-Luna sin while(1) (si se desconecta avisa, no se cuelga), sin desbordar buffer.
//
// Salida USB 115200, una línea por punto: "x\ty\tz\tfuerza\n" en cm (formato del LidarViewer).
// Mensajes con '#' (el visor los ignora). Comandos: p = parar motor, c = continuar.
#include <ESP32Servo.h>

// ---------- Pines (ESP32 DevKit 30 pines) ----------
const int LUNA_RX = 16;   // RX del ESP32 <- TX del TF-Luna (pin 2 del Luna)
const int LUNA_TX = 17;   // TX del ESP32 -> RX del TF-Luna (pin 3)
const int PIN_STEP = 25, PIN_DIR = 26;
const int PIN_EN   = 27;  // EN del DRV8825 (activo en BAJO). Libre = siempre habilitado
const int PIN_SERVO_I = 32, PIN_SERVO_D = 33;

// ---------- Mecánica (ajustar a la impresión) ----------
const int   PASOS_VUELTA = 200 * 3 / 2 * 16;  // pasos motor × reducción del engrane × micropasos
const float MS_VUELTA    = 1000.0f;           // [ms] por vuelta del espejo (1 vuelta/s = 60 rpm)
const float THETA_OFS    = 0.0f;              // [°] ajusta el "frente" del escaneo al frente del rover
const int   SERVO_NEUTRO = 90, SERVO_RANGO = 40;      // barrido de servos: 90 ± 40
const float PHI_A = -0.3f, PHI_B = 71.0f;     // ángulo del espejo = A·servo + B (geometría del varillaje del MK2)
const bool  BARRIDO_3D   = true;              // false = espejo fijo (escaneo 2D, plano horizontal)
const float RHO_OFS_CM   = 5.5f;              // distancia del Luna al espejo [cm]

// ---------- TF-Luna ----------
const uint32_t LUNA_BAUD = 115200;
const int FUERZA_MIN = 100;                   // hoja de datos: < 100 o = 65535 => lectura no confiable
const uint32_t LUNA_TIMEOUT_MS = 500;

HardwareSerial Luna(2);
Servo servoI, servoD;
hw_timer_t *timer;
volatile uint32_t pasos = 0;   // pasos totales desde el arranque
volatile bool nivel = false;
bool girando = true;
int servo = SERVO_NEUTRO - SERVO_RANGO, sentidoServo = 1;

// Media onda del STEP. ponytail: velocidad fija sin rampa; si el motor pierde pasos al arrancar, sube MS_VUELTA.
void IRAM_ATTR alTimer() {
  nivel = !nivel;
  digitalWrite(PIN_STEP, nivel);
  if (nivel) pasos++;
}

void moverServos(int a) {
  servoI.write(180 - a);   // los 2 servos están espejados
  servoD.write(a);
}

// Parser de tramas del TF-Luna: 59 59 DistL DistH FuerzaL FuerzaH TempL TempH Checksum
struct ParserLuna {
  uint8_t b[9];
  int n = 0;
  // Mete un byte. true cuando hay trama completa y válida.
  bool meter(uint8_t c, int &dist, int &fuerza) {
    if (n < 2 && c != 0x59) { n = 0; return false; }
    b[n++] = c;
    if (n < 9) return false;
    n = 0;
    uint8_t suma = 0;
    for (int i = 0; i < 8; i++) suma += b[i];
    if (suma != b[8]) return false;
    dist = b[2] | b[3] << 8;     // [cm]
    fuerza = b[4] | b[5] << 8;
    return true;
  }
} parser;

bool autoprueba() {
  ParserLuna p;
  int d = 0, f = 0, ok = 0;
  uint8_t t[] = {0x00, 0x59, 0x59, 0x2C, 0x01, 0xE8, 0x03, 0x00, 0x09, 0};  // basura + 300 cm, fuerza 1000
  for (int i = 1; i < 9; i++) t[9] += t[i];                                  // checksum
  for (uint8_t c : t) ok += p.meter(c, d, f);
  bool mala = false;
  t[9] ^= 1;   // checksum mal => se descarta
  for (int i = 1; i < 10; i++) mala |= p.meter(t[i], d, f);
  return ok == 1 && d == 300 && f == 1000 && !mala;
}

void setup() {
  Serial.begin(115200);
  Luna.begin(LUNA_BAUD, SERIAL_8N1, LUNA_RX, LUNA_TX);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_EN, LOW);
  servoI.attach(PIN_SERVO_I);
  servoD.attach(PIN_SERVO_D);
  moverServos(BARRIDO_3D ? servo : SERVO_NEUTRO);

  timer = timerBegin(1000000);   // 1 MHz => alarma en µs (API core 3.x)
  timerAttachInterrupt(timer, &alTimer);
  timerAlarm(timer, (uint64_t)(MS_VUELTA * 1000 / (2 * PASOS_VUELTA)), true, 0);

  Serial.printf("# lidar_torreta: %d pasos/vuelta, %.0f ms/vuelta, %s. Autoprueba parser: %s\n",
                PASOS_VUELTA, MS_VUELTA, BARRIDO_3D ? "3D" : "2D", autoprueba() ? "ok" : "FALLA");
}

void loop() {
  static uint32_t tUltimo = millis(), vueltaAnt = 0;
  static bool avisado = false;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'p' || c == 'c') {
      girando = c == 'c';
      digitalWrite(PIN_EN, !girando);   // EN alto = driver apagado
      girando ? timerStart(timer) : timerStop(timer);
      Serial.println(girando ? "# girando" : "# parado");
    }
  }

  // Una vuelta completa => mover el espejo 1° (barrido en zig-zag)
  uint32_t vuelta = pasos / PASOS_VUELTA;
  if (BARRIDO_3D && vuelta != vueltaAnt) {
    vueltaAnt = vuelta;
    servo += sentidoServo;
    if (abs(servo - SERVO_NEUTRO) >= SERVO_RANGO) sentidoServo = -sentidoServo;
    moverServos(servo);
  }

  int dist, fuerza;
  while (Luna.available()) {
    if (!parser.meter(Luna.read(), dist, fuerza)) continue;
    tUltimo = millis();
    avisado = false;
    if (dist == 0 || fuerza < FUERZA_MIN || fuerza == 65535) continue;
    // ponytail: la trama llega ~1 periodo después de medir; si la nube sale "torcida", corrige con THETA_OFS
    float theta = ((pasos % PASOS_VUELTA) * 360.0f / PASOS_VUELTA + THETA_OFS) * DEG_TO_RAD;
    int sv = BARRIDO_3D ? servo : SERVO_NEUTRO;
    float phi = (2 * (PHI_A * sv + PHI_B) - 90.0f) * DEG_TO_RAD;   // rebote en el espejo: 2× su ángulo
    float rho = dist - RHO_OFS_CM;
    Serial.printf("%d\t%d\t%d\t%d\n", (int)(rho * cosf(phi) * cosf(theta)),
                  (int)(rho * cosf(phi) * sinf(theta)), (int)(rho * sinf(phi)), fuerza);
  }

  if (!avisado && millis() - tUltimo > LUNA_TIMEOUT_MS) {
    Serial.printf("# TF-Luna sin datos: TX del Luna -> GPIO%d? 5 V? pin 5 del Luna a GND = I2C (debe ir libre)\n", LUNA_RX);
    avisado = true;
  }
}
