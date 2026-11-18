// config.h — TODOS los parámetros del rover. Nada de números mágicos en la lógica.
#pragma once

// ---------- Geometría ----------
const float L = 0.30f;   // distancia entre ejes [m] (placeholder)
const float B = 0.26f;   // ancho de vía [m] (placeholder)
const float R = 0.06f;   // radio de llanta [m] (placeholder)

// ---------- Encoder (GM25-370, 1 solo canal) ----------
// Hoja de datos: 493.9 PPR en la flecha de salida (11 PPR x 45:1), ±10%.
// Contamos AMBOS flancos (CHANGE) del canal A => CPR = 2 * 493.9.
// Si la distancia medida no cuadra, dale 10 vueltas a mano a la llanta y ajusta.
const float CPR = 2 * 493.9f;  // cuentas por vuelta de LLANTA

// ---------- Calibración de χ (se usará en la rutina de calibración) ----------
const float SPEEDS[] = {0.15f, 0.25f, 0.35f};  // [m/s]
const float T_TURN = 3.0f;                     // [s]
const int   N_REPS = 3;

// ---------- Control ----------
const float V_MAX     = 0.72f;  // [m/s] medido con PWM 255 y ruedas al aire (barrido 2026-11-16)
// Valores de fábrica. Las ganancias por rueda se ajustan desde panel.py y se guardan en flash (W);
// "D" vuelve a estos valores. Sintonizados 2026-11-16 (ruedas al aire): sobrepaso <= 12 %, error < 1.3 %.
// KFF NO es 255/V_MAX: la curva PWM->v no es lineal (zona muerta ~50-75 PWM); 230 es el mejor valor único.
const float KFF       = 230.0f;  // feedforward PWM por m/s
const float KP        = 200.0f;
const float KI        = 600.0f;
const uint32_t DT_MS  = 50;      // periodo de control (20 Hz; con 1 canal más rápido = poca resolución)

// ---------- Seguridad ----------
const int      PWM_MOVER       = 120;   // PWM con el que la rueda YA debería girar
const uint32_t ENC_TIMEOUT_MS  = 500;   // encoder sin cuentas con PWM >= PWM_MOVER => paro
const uint32_t SAT_TIMEOUT_MS  = 300;   // PWM al tope y sin llegar al setpoint => saturada
const float    SAT_TOL         = 0.05f; // [m/s] tolerancia para "alcanzó setpoint"
const uint32_t CMD_TIMEOUT_MS  = 1000;  // sin comandos (Serial o ESP-NOW) => paro (se pierde el enlace)
const int      IMU_FALLOS_MAX  = 5;     // lecturas I2C fallidas seguidas => paro

// ---------- IMU GY-87 (MPU6050: giro Z. Magnetómetro: mag.h, calibración desde panel.py) ----------
const uint32_t IMU_DT_US = 5000;   // lectura del giroscopio a 200 Hz (DLPF a 44 Hz)
const uint32_t BIAS_MS   = 1000;   // tiempo en reposo para medir el bias antes de cada prueba
const float GYRO_SIGNO   = 1.0f;   // +1 si Z de la IMU apunta hacia ARRIBA; -1 si está de cabeza
                                   // (antihorario visto desde arriba = θ positivo)

// ---------- Lidar TF-Luna en torreta (espejo FIJO a 45° => escaneo 2D horizontal) ----------
// A4988: MS1=H, MS2=H, MS3=L => 1/8 de micropaso. SLEEP puenteado con RESET. Vref según tu placa.
// El motor solo gira con el flujo L activo (F L); en los demás flujos el driver queda apagado.
const int      PASOS_VUELTA    = 200 * 3 / 2 * 8;  // pasos motor × reducción del engrane × micropasos
const float    MS_VUELTA       = 1000.0f;          // [ms] por vuelta del espejo (1 vuelta/s)
const uint32_t RAMPA_MS        = 1000;             // arranque suave del stepper (si pierde pasos, súbelo)
const float    THETA_OFS_LIDAR = 0.0f;             // [°] alinea el 0 del escaneo con el frente del rover
const float    RHO_OFS_CM      = 5.5f;             // [cm] distancia del Luna al espejo
const int      FUERZA_MIN      = 100;              // hoja de datos: < 100 o 65535 => lectura no confiable
const uint32_t LUNA_TIMEOUT_MS = 500;              // sin tramas => aviso (no detiene el rover)
const int      LIDAR_PTS_LINEA = 10;               // puntos por línea "L,..." (menos líneas por el enlace)

// ---------- Enlace ----------
const bool USAR_ESPNOW = true;    // true = USB y ESP-NOW a la vez. false = solo USB.

// ---------- ESP-NOW ----------
// MAC del maestro (el ESP conectado a la PC). FF:FF... = broadcast: funciona sin configurar nada.
// Con la MAC real (la imprime el maestro al arrancar) ESP-NOW confirma la entrega y reintenta.
uint8_t MAC_MAESTRO[6] = {0x68, 0x25, 0xDD, 0x32, 0x08, 0x78};

// ---------- Pines ESP32 DevKit 30 pines ----------
// Un L298N por lado. IN1+IN3 puenteados a un pin, IN2+IN4 a otro, ENA y ENB separados (PWM por rueda).
// Quitar los jumpers de ENA/ENB del L298N. GND común ESP32 <-> L298N.
const int PIN_IZQ_A  = 25;  // IN1+IN3 lado izquierdo
const int PIN_IZQ_B  = 26;  // IN2+IN4 lado izquierdo
const int PIN_DER_A  = 32;  // IN1+IN3 lado derecho
const int PIN_DER_B  = 33;  // IN2+IN4 lado derecho
// Orden de ruedas en todos los arreglos: 0=DI (del. izq), 1=TI (tras. izq), 2=DD (del. der), 3=TD (tras. der)
const int PIN_EN[4]  = {27, 14, 13, 4};
// 34/35/36/39 son solo-entrada y SIN pull-up interno: si el encoder no trae pull-up, poner 10k a 3.3V.
const int PIN_ENC[4] = {34, 35, 36, 39};
const int PIN_SDA = 21, PIN_SCL = 22;  // GY-87 (MPU6050 + magnetómetro por bypass)
const int PIN_LUNA_RX = 16, PIN_LUNA_TX = 17;  // UART2: RX del ESP <- TX del Luna (pin 2); pin 5 del Luna LIBRE (= UART)
// A4988. 5 y 15 son pines de arranque: DIR sin pulsos no mueve nada; EN en alto al bootear = driver apagado.
const int PIN_STEP = 23, PIN_DIR = 5, PIN_STEP_EN = 15;
// Libres: 18/19 (GPS, UART1), 2 (LED de la placa; servo o sensor de cero). NO usar 12 (si está en alto no bootea).
