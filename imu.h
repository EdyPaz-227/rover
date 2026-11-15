// imu.h — giroscopio Z del MPU6050 (GY-87) por I2C directo, sin librería. Integra θ.
#pragma once
#include <Wire.h>
#include "config.h"
#include "ruedas.h"

const uint8_t MPU = 0x68;
float biasZ = 0, wz = 0, theta = 0;  // [rad/s], [rad/s], [rad]
uint32_t tImu = 0;
int fallosImu = 0;

bool escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Giro Z crudo en rad/s (con bias). false si falla el I2C.
bool leerGiroZ(float &w) {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU, (uint8_t)2) != 2) return false;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  w = GYRO_SIGNO * raw / 65.5f * DEG_TO_RAD;  // ±500 °/s => 65.5 LSB por °/s
  return true;
}

bool iniciarIMU() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  return escribirReg(0x6B, 0x01)    // despertar, reloj del giroscopio X (más estable)
      && escribirReg(0x1A, 0x03)    // filtro pasa-bajas ~44 Hz
      && escribirReg(0x1B, 0x08);   // ±500 °/s (giro en su lugar a 0.85 m/s ≈ 375 °/s)
}

// Promedia el giro Z en reposo durante BIAS_MS y pone θ = 0. Bloquea; solo con motores parados.
bool medirBias() {
  float suma = 0, w;
  int n = 0;
  for (uint32_t t0 = millis(); millis() - t0 < BIAS_MS; delay(2))
    if (leerGiroZ(w)) { suma += w; n++; }
  if (n < 10) return false;
  biasZ = suma / n;
  theta = 0;
  tImu = micros();
  return true;
}

// Llamar en cada loop: lee a IMU_DT_US e integra θ. Paro si la IMU deja de responder.
void actualizarIMU() {
  uint32_t ahora = micros();
  if (ahora - tImu < IMU_DT_US) return;
  float dt = (ahora - tImu) * 1e-6f, w;
  tImu = ahora;
  if (!leerGiroZ(w)) {
    if (++fallosImu >= IMU_FALLOS_MAX) activarParo("IMU no responde");
    return;
  }
  fallosImu = 0;
  wz = w - biasZ;
  theta += wz * dt;  // ponytail: Euler a 200 Hz; basta para giros de pocos segundos
}
