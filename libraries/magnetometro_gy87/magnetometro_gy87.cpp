#include "magnetometro_gy87.h"

// --- Registros HMC5883L (Honeywell, big-endian, orden X-Z-Y) --------------
static const uint8_t HMC_CFG_A   = 0x00;
static const uint8_t HMC_CFG_B   = 0x01;
static const uint8_t HMC_MODO    = 0x02;
static const uint8_t HMC_DATOS   = 0x03;
static const uint8_t HMC_ID_A    = 0x0A;   // debe leer 'H' '4' '3'

// --- Registros QMC5883L (QST, little-endian, orden X-Y-Z) -----------------
static const uint8_t QMC_DATOS   = 0x00;
static const uint8_t QMC_ESTADO  = 0x06;
static const uint8_t QMC_CTRL_1  = 0x09;
static const uint8_t QMC_CTRL_2  = 0x0A;
static const uint8_t QMC_PERIODO = 0x0B;
static const uint8_t QMC_ID      = 0x0D;   // debe leer 0xFF

MagnetometroGY87::MagnetometroGY87()
    : _bus(&Wire), _tipo(MAG_NO_DETECTADO), _dir(0), _declinacion(0.0f) {
  for (uint8_t i = 0; i < 3; i++) {
    _cal.desplazamiento[i] = 0.0f;
    _cal.escala[i]         = 1.0f;
  }
}

bool MagnetometroGY87::_escribirReg(uint8_t dir, uint8_t reg, uint8_t valor) {
  _bus->beginTransmission(dir);
  _bus->write(reg);
  _bus->write(valor);
  return (_bus->endTransmission() == 0);
}

bool MagnetometroGY87::_leerRegs(uint8_t dir, uint8_t reg, uint8_t n, uint8_t *destino) {
  _bus->beginTransmission(dir);
  _bus->write(reg);
  if (_bus->endTransmission(false) != 0) return false;
  uint8_t recibidos = _bus->requestFrom(dir, n);
  if (recibidos != n) return false;
  for (uint8_t i = 0; i < n; i++) destino[i] = _bus->read();
  return true;
}

bool MagnetometroGY87::_detectarDispositivo(uint8_t dir) {
  _bus->beginTransmission(dir);
  return (_bus->endTransmission() == 0);
}

bool MagnetometroGY87::habilitarBypassMPU6050(TwoWire *bus) {
  // 1) Despertar el MPU6050 SIN alterar su fuente de reloj: se lee
  //    PWR_MGMT_1 y solo se limpia el bit SLEEP (bit 6). Asi esta funcion se
  //    puede llamar despues de configurar el MPU6050 sin deshacer su ajuste.
  bus->beginTransmission(DIR_MPU6050);
  bus->write(REG_MPU_PWR_MGMT_1);
  if (bus->endTransmission(false) != 0) return false;
  if (bus->requestFrom(DIR_MPU6050, (uint8_t)1) != 1) return false;
  uint8_t pwr = bus->read();
  pwr &= ~(1 << 6);
  bus->beginTransmission(DIR_MPU6050);
  bus->write(REG_MPU_PWR_MGMT_1);
  bus->write(pwr);
  if (bus->endTransmission() != 0) return false;
  delay(10);

  // 2) Desactivar el maestro I2C interno (si esta activo, bloquea el bus aux)
  bus->beginTransmission(DIR_MPU6050);
  bus->write(REG_MPU_USER_CTRL);
  bus->write(0x00);
  if (bus->endTransmission() != 0) return false;

  // 3) Habilitar bypass: I2C_BYPASS_EN = bit 1 de INT_PIN_CFG
  bus->beginTransmission(DIR_MPU6050);
  bus->write(REG_MPU_INT_PIN_CFG);
  bus->write(0x02);
  if (bus->endTransmission() != 0) return false;
  delay(10);
  return true;
}

bool MagnetometroGY87::_iniciarHMC() {
  uint8_t id[3] = {0, 0, 0};
  if (!_leerRegs(DIR_HMC5883L, HMC_ID_A, 3, id)) return false;
  if (!(id[0] == 'H' && id[1] == '4' && id[2] == '3')) return false;

  // CRA: 8 muestras promediadas, 15 Hz, medicion normal
  if (!_escribirReg(DIR_HMC5883L, HMC_CFG_A, 0x70)) return false;
  // CRB: ganancia +-1.3 Ga (1090 LSB/Gauss)
  if (!_escribirReg(DIR_HMC5883L, HMC_CFG_B, 0x20)) return false;
  // Modo continuo
  if (!_escribirReg(DIR_HMC5883L, HMC_MODO, 0x00)) return false;
  delay(10);
  _tipo = MAG_HMC5883L;
  _dir  = DIR_HMC5883L;
  return true;
}

bool MagnetometroGY87::_iniciarQMC() {
  uint8_t id = 0;
  if (!_leerRegs(DIR_QMC5883L, QMC_ID, 1, &id)) return false;
  if (id != 0xFF) return false;

  // Reset por software
  if (!_escribirReg(DIR_QMC5883L, QMC_CTRL_2, 0x80)) return false;
  delay(10);
  // El periodo SET/RESET debe valer 0x01 (obligatorio segun hoja de datos)
  if (!_escribirReg(DIR_QMC5883L, QMC_PERIODO, 0x01)) return false;
  // CTRL1 = 0x1D -> OSR=512, RNG=+-8G, ODR=200 Hz, MODO=continuo
  if (!_escribirReg(DIR_QMC5883L, QMC_CTRL_1, 0x1D)) return false;
  delay(10);
  _tipo = MAG_QMC5883L;
  _dir  = DIR_QMC5883L;
  return true;
}

bool MagnetometroGY87::iniciar(TwoWire *bus) {
  _bus = bus;
  _tipo = MAG_NO_DETECTADO;

  if (!habilitarBypassMPU6050(_bus)) {
    // Puede no haber MPU6050 (magnetometro en placa suelta). Se sigue igual.
  }

  if (_detectarDispositivo(DIR_QMC5883L) && _iniciarQMC()) return true;
  if (_detectarDispositivo(DIR_HMC5883L) && _iniciarHMC()) return true;
  return false;
}

const char* MagnetometroGY87::nombreTipo() const {
  switch (_tipo) {
    case MAG_HMC5883L: return "HMC5883L";
    case MAG_QMC5883L: return "QMC5883L";
    default:           return "NO_DETECTADO";
  }
}

bool MagnetometroGY87::leerCrudo(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];
  if (_tipo == MAG_HMC5883L) {
    // HMC5883L: big-endian y orden de ejes X, Z, Y (asi lo entrega el chip)
    if (!_leerRegs(DIR_HMC5883L, HMC_DATOS, 6, b)) return false;
    x = (int16_t)((b[0] << 8) | b[1]);
    z = (int16_t)((b[2] << 8) | b[3]);
    y = (int16_t)((b[4] << 8) | b[5]);
    // -4096 indica saturacion del ADC
    if (x == -4096 || y == -4096 || z == -4096) return false;
    return true;
  }
  if (_tipo == MAG_QMC5883L) {
    // QMC5883L: little-endian y orden X, Y, Z
    if (!_leerRegs(DIR_QMC5883L, QMC_DATOS, 6, b)) return false;
    x = (int16_t)(b[0] | (b[1] << 8));
    y = (int16_t)(b[2] | (b[3] << 8));
    z = (int16_t)(b[4] | (b[5] << 8));
    return true;
  }
  return false;
}

void MagnetometroGY87::calibrarValores(int16_t cx, int16_t cy, int16_t cz,
                                       float &x, float &y, float &z) const {
  float fx = ((float)cx - _cal.desplazamiento[0]) * _cal.escala[0];
  float fy = ((float)cy - _cal.desplazamiento[1]) * _cal.escala[1];
  float fz = ((float)cz - _cal.desplazamiento[2]) * _cal.escala[2];
  x = MAPEO_MAG_X(fx, fy, fz);
  y = MAPEO_MAG_Y(fx, fy, fz);
  z = MAPEO_MAG_Z(fx, fy, fz);
}

bool MagnetometroGY87::leerCalibrado(float &x, float &y, float &z) {
  int16_t cx, cy, cz;
  if (!leerCrudo(cx, cy, cz)) return false;
  calibrarValores(cx, cy, cz, x, y, z);
  return true;
}

float MagnetometroGY87::rumboPlanoDe(float mx, float my) const {
  float rumbo = atan2f(my, mx) * 180.0f / PI + _declinacion;
  while (rumbo <   0.0f) rumbo += 360.0f;
  while (rumbo >= 360.0f) rumbo -= 360.0f;
  return rumbo;
}

float MagnetometroGY87::rumboPlano() {
  float x, y, z;
  if (!leerCalibrado(x, y, z)) return NAN;
  return rumboPlanoDe(x, y);
}

float MagnetometroGY87::rumboCompensado(float acel_x, float acel_y, float acel_z) {
  float mx, my, mz;
  if (!leerCalibrado(mx, my, mz)) return NAN;
  return rumboCompensadoDe(mx, my, mz, acel_x, acel_y, acel_z);
}

float MagnetometroGY87::rumboCompensadoDe(float mx, float my, float mz,
                                          float acel_x, float acel_y,
                                          float acel_z) const {
  // Alabeo (roll) y cabeceo (pitch) a partir del vector de gravedad
  float roll  = atan2f(acel_y, acel_z);
  float pitch = atan2f(-acel_x, sqrtf(acel_y * acel_y + acel_z * acel_z));

  float sr = sinf(roll),  cr = cosf(roll);
  float sp = sinf(pitch), cp = cosf(pitch);

  // Proyeccion del campo magnetico al plano horizontal
  float xh = mx * cp + my * sr * sp + mz * cr * sp;
  float yh = my * cr - mz * sr;

  float rumbo = atan2f(-yh, xh) * 180.0f / PI + _declinacion;
  while (rumbo <   0.0f) rumbo += 360.0f;
  while (rumbo >= 360.0f) rumbo -= 360.0f;
  return rumbo;
}
