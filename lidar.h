// lidar.h — TF-Luna por UART2 + torreta con stepper (A4988) y espejo FIJO => escaneo 2D.
// El ángulo sale de contar pasos (sin sensor de cero): si el motor pierde pasos, el 0 se corre.
// Salida: "L,a,d,a,d,..." con LIDAR_PTS_LINEA puntos; a = ángulo en décimas de grado, d = distancia [cm]
// ya corregida por RHO_OFS_CM. Si el Luna no responde solo avisa (no detiene el rover).
#pragma once
#include "config.h"
#include "enlace.h"

HardwareSerial Luna(2);
hw_timer_t *timerPasos;
volatile uint32_t pasos = 0;  // pasos desde el arranque (el ángulo es pasos % PASOS_VUELTA)
volatile bool nivelStep = false;
bool lidarActivo = false;
uint32_t tArranque, tRampa, tLuna;
bool avisoLuna = false;
char lineaLidar[251];
int nPts = 0, largo = 0;

const float PERIODO_US = MS_VUELTA * 1000.0f / (2 * PASOS_VUELTA);  // media onda de STEP a velocidad final

// Media onda del STEP. ponytail: timer + digitalWrite (opción simple); STEP por LEDC/RMT si da tirones.
void IRAM_ATTR alTimerPasos() {
  nivelStep = !nivelStep;
  digitalWrite(PIN_STEP, nivelStep);
  if (nivelStep) pasos++;
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
} parserLuna;

// Autoprueba del parser (se corre al arrancar): trama buena => 300 cm / 1000; checksum malo => se descarta.
bool autopruebaLuna() {
  ParserLuna p;
  int d = 0, f = 0, ok = 0;
  uint8_t t[] = {0x00, 0x59, 0x59, 0x2C, 0x01, 0xE8, 0x03, 0x00, 0x09, 0};
  for (int i = 1; i < 9; i++) t[9] += t[i];
  for (uint8_t c : t) ok += p.meter(c, d, f);
  bool mala = false;
  t[9] ^= 1;
  for (int i = 1; i < 10; i++) mala |= p.meter(t[i], d, f);
  return ok == 1 && d == 300 && f == 1000 && !mala;
}

void iniciarLidar() {
  Luna.begin(115200, SERIAL_8N1, PIN_LUNA_RX, PIN_LUNA_TX);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);
  digitalWrite(PIN_STEP_EN, HIGH);   // EN activo en BAJO: apagado hasta pedir el flujo L
  timerPasos = timerBegin(1000000);  // 1 MHz => alarma en µs (API core 3.x)
  timerAttachInterrupt(timerPasos, &alTimerPasos);
  timerStop(timerPasos);
  enviar("# lidar: %d pasos/vuelta, %.0f ms/vuelta, autoprueba parser %s\n",
         PASOS_VUELTA, MS_VUELTA, autopruebaLuna() ? "ok" : "FALLA");
}

// Periodo de media onda según la rampa de arranque (velocidad lineal de 10 % a 100 % en RAMPA_MS).
void ponerPeriodo(float frac) {
  timerWrite(timerPasos, 0);   // si el contador ya pasó la nueva alarma, no dispararía nunca
  timerAlarm(timerPasos, (uint64_t)(PERIODO_US / frac), true, 0);
}

void encenderLidar(bool on) {
  if (on == lidarActivo) return;
  lidarActivo = on;
  if (on) {
    digitalWrite(PIN_STEP_EN, LOW);
    tArranque = tRampa = tLuna = millis();
    avisoLuna = false;
    nPts = 0;
    ponerPeriodo(0.1f);
    timerStart(timerPasos);
  } else {
    timerStop(timerPasos);
    digitalWrite(PIN_STEP_EN, HIGH);
  }
}

// Llamar en cada loop: rampa del motor, lee tramas y manda los puntos en grupos.
void actualizarLidar() {
  if (!lidarActivo) { while (Luna.available()) Luna.read(); return; }   // descarta lo que llegue

  uint32_t ahora = millis(), t = ahora - tArranque;
  if (t <= RAMPA_MS + 50 && ahora - tRampa >= 50) {   // ajusta la velocidad cada 50 ms durante la rampa
    tRampa = ahora;
    ponerPeriodo(constrain((float)t / RAMPA_MS, 0.1f, 1.0f));
  }

  int dist, fuerza;
  while (Luna.available()) {
    if (!parserLuna.meter(Luna.read(), dist, fuerza)) continue;
    tLuna = ahora;
    avisoLuna = false;
    if (dist == 0 || fuerza < FUERZA_MIN || fuerza == 65535) continue;
    // ponytail: la trama llega ~1 periodo después de medir; si el escaneo sale "torcido", corrige con THETA_OFS_LIDAR
    float ang = fmodf((pasos % PASOS_VUELTA) * 360.0f / PASOS_VUELTA + THETA_OFS_LIDAR + 360.0f, 360.0f);
    if (nPts == 0) largo = snprintf(lineaLidar, sizeof(lineaLidar), "L");
    largo += snprintf(lineaLidar + largo, sizeof(lineaLidar) - largo, ",%d,%d",
                      (int)(ang * 10), (int)(dist - RHO_OFS_CM));
    if (++nPts == LIDAR_PTS_LINEA) { enviar("%s\n", lineaLidar); nPts = 0; }
  }

  if (!avisoLuna && ahora - tLuna > LUNA_TIMEOUT_MS) {
    enviar("# lidar: TF-Luna sin datos (TX del Luna -> GPIO%d? 5 V? pin 5 del Luna libre = UART)\n", PIN_LUNA_RX);
    avisoLuna = true;
  }
}
