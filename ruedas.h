// ruedas.h — motores (L298N), encoders de 1 canal y PID por rueda (ganancias por rueda, ajustables en vivo).
#pragma once
#include <Arduino.h>
#include "config.h"
#include "enlace.h"

volatile uint32_t cuentas[4];
void IRAM_ATTR isr0() { cuentas[0]++; }
void IRAM_ATTR isr1() { cuentas[1]++; }
void IRAM_ATTR isr2() { cuentas[2]++; }
void IRAM_ATTR isr3() { cuentas[3]++; }

// Ganancias por rueda (orden DI, TI, DD, TD). Arrancan con config.h; rover.ino las carga de flash.
float kp[4]  = {KP, KP, KP, KP};
float ki[4]  = {KI, KI, KI, KI};
float kff[4] = {KFF, KFF, KFF, KFF};

float sp[2];             // setpoint por lado [m/s]: 0=izq, 1=der (las 2 ruedas del lado comparten)
int   pwmLado[2];        // modo PWM directo (lazo abierto), con signo, por lado
bool  lazoAbierto = false;
int8_t dir[2] = {1, 1};  // último sentido ordenado por lado (sirve para el signo mientras frena)
float vMed[4];           // velocidad medida [m/s] con signo (el signo se toma del comando: 1 canal no da sentido)
uint32_t tRuedas = 0;
int   pwm[4];
float integ[4];
bool  saturada[4];
uint32_t tSat[4], tSinCuentas[4], cuentasAnt[4];
bool  paro = false;
String motivoParo;
bool  depuracion = false;  // comando X: los fallos de encoder/IMU solo avisan (para medir con osciloscopio)
uint32_t tAviso = 0;

int lado(int i) { return i < 2 ? 0 : 1; }

bool moviendo() { return sp[0] != 0 || sp[1] != 0 || pwmLado[0] != 0 || pwmLado[1] != 0; }

void ponerDireccion(int pinA, int pinB, float v) {
  digitalWrite(pinA, v > 0);
  digitalWrite(pinB, v < 0);   // v == 0 => ambos LOW => rueda libre
}

void pararMotores() {
  sp[0] = sp[1] = 0;
  pwmLado[0] = pwmLado[1] = 0;
  lazoAbierto = false;
  for (int i = 0; i < 4; i++) { analogWrite(PIN_EN[i], 0); pwm[i] = 0; integ[i] = 0; }
  ponerDireccion(PIN_IZQ_A, PIN_IZQ_B, 0);
  ponerDireccion(PIN_DER_A, PIN_DER_B, 0);
}

// siempre = true: para aun en modo depuración (pérdida del enlace: único freno si la PC se cae).
void activarParo(const String& motivo, bool siempre = false) {
  if (depuracion && !siempre) {
    if (millis() - tAviso > 1000) { tAviso = millis(); enviar("# aviso (depuración, sin paro): %s\n", motivo.c_str()); }
    return;
  }
  pararMotores();
  if (!paro) { paro = true; motivoParo = motivo; enviar("# PARO: %s\n", motivo.c_str()); }
}

void iniciarRuedas() {
  for (int p : {PIN_IZQ_A, PIN_IZQ_B, PIN_DER_A, PIN_DER_B}) pinMode(p, OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(PIN_ENC[i], INPUT);
  attachInterrupt(PIN_ENC[0], isr0, CHANGE);
  attachInterrupt(PIN_ENC[1], isr1, CHANGE);
  attachInterrupt(PIN_ENC[2], isr2, CHANGE);
  attachInterrupt(PIN_ENC[3], isr3, CHANGE);
  pararMotores();  // ponytail: analogWrite (1 kHz, 8 bits) sirve en core 2.x y 3.x; LEDC directo si el zumbido molesta
}

void fijarSentidos(float l, float r) {
  if (l != 0) dir[0] = l > 0 ? 1 : -1;
  if (r != 0) dir[1] = r > 0 ? 1 : -1;
  ponerDireccion(PIN_IZQ_A, PIN_IZQ_B, l);
  ponerDireccion(PIN_DER_A, PIN_DER_B, r);
}

// Velocidades por lado [m/s] con PID. Satura a ±V_MAX.
void fijarVelocidades(float vL, float vR) {
  if (paro) return;
  lazoAbierto = false;
  pwmLado[0] = pwmLado[1] = 0;
  sp[0] = constrain(vL, -V_MAX, V_MAX);
  sp[1] = constrain(vR, -V_MAX, V_MAX);
  fijarSentidos(sp[0], sp[1]);
}

// PWM directo por lado (-255..255), sin PID: para medir zona muerta y KFF real.
void fijarPwm(int pL, int pR) {
  if (paro) return;
  lazoAbierto = true;
  sp[0] = sp[1] = 0;
  pwmLado[0] = constrain(pL, -255, 255);
  pwmLado[1] = constrain(pR, -255, 255);
  fijarSentidos(pwmLado[0], pwmLado[1]);
}

// Llamar cada DT_MS. Mide, corre el PID (o aplica el PWM directo) y vigila encoders/saturación.
void actualizarRuedas(uint32_t ahora) {
  float dt = (ahora - tRuedas) / 1000.0f;  // dt real: el loop puede atrasarse (p. ej. al medir bias)
  tRuedas = ahora;
  if (dt <= 0) return;
  for (int i = 0; i < 4; i++) {
    uint32_t c = cuentas[i];                 // lectura de 32 bits: atómica en ESP32
    uint32_t dc = c - cuentasAnt[i];
    cuentasAnt[i] = c;

    float ref = fabsf(sp[lado(i)]);
    float v = dc / CPR * TWO_PI * R / dt;    // magnitud [m/s]
    vMed[i] = dir[lado(i)] * v;
    // ponytail: al invertir sentido la rueda aún gira hacia atrás y aquí se lee como positiva;
    // pasa en ~1 periodo y no afecta los giros de calibración (sentido fijo durante la prueba).

    bool quieta = lazoAbierto ? pwmLado[lado(i)] == 0 : ref == 0;
    if (quieta || paro) { pwm[i] = 0; integ[i] = 0; saturada[i] = false; tSat[i] = tSinCuentas[i] = ahora; analogWrite(PIN_EN[i], 0); continue; }

    if (lazoAbierto) {
      pwm[i] = abs(pwmLado[lado(i)]);
      saturada[i] = false;
    } else {
      float e = ref - v;
      float u = kff[i] * ref + kp[i] * e + ki[i] * integ[i];
      if (u > 0 && u < 255) integ[i] += e * dt;  // anti-windup: solo integra fuera de saturación
      pwm[i] = constrain((int)u, 0, 255);
      // Saturación: PWM al tope y aún lejos del setpoint por SAT_TIMEOUT_MS
      if (pwm[i] < 255 || e < SAT_TOL) tSat[i] = ahora;
      saturada[i] = ahora - tSat[i] > SAT_TIMEOUT_MS;
    }
    analogWrite(PIN_EN[i], pwm[i]);

    // Encoder muerto: PWM suficiente para mover y cero cuentas por ENC_TIMEOUT_MS
    if (dc > 0 || pwm[i] < PWM_MOVER) tSinCuentas[i] = ahora;
    if (ahora - tSinCuentas[i] > ENC_TIMEOUT_MS) activarParo("encoder " + String(i) + " sin cuentas");
  }
}
