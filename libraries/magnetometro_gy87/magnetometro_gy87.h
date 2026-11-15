/*
 * magnetometro_gy87.h
 * ---------------------------------------------------------------------------
 * Driver del magnetometro del modulo GY-87 con AUTODETECCION del chip.
 *
 * MOTIVO DE ESTE MODULO:
 *   La serigrafia del GY-87 dice HMC5883L, pero Honeywell descontinuo ese chip
 *   y la mayoria de los modulos que se venden hoy traen un QMC5883L (QST), que
 *   tiene OTRA direccion I2C y OTRO mapa de registros. Este driver detecta cual
 *   de los dos esta presente y expone la misma interfaz para ambos.
 *
 *   Ademas, en el GY-87 el magnetometro cuelga del bus I2C auxiliar del
 *   MPU6050 y NO es visible hasta habilitar el modo bypass.
 *
 * Referencia de pruebas: P-00 (deteccion), P-02 (calibracion), P-05/P-06 (rumbo).
 * ---------------------------------------------------------------------------
 */
#ifndef MAGNETOMETRO_GY87_H
#define MAGNETOMETRO_GY87_H

#include <Arduino.h>
#include <Wire.h>

// --- Direcciones I2C del modulo GY-87 -------------------------------------
static const uint8_t DIR_MPU6050        = 0x68;
static const uint8_t DIR_BMP180         = 0x77;
static const uint8_t DIR_HMC5883L       = 0x1E;
static const uint8_t DIR_QMC5883L       = 0x0D;

// --- Registros del MPU6050 necesarios para el bypass ----------------------
static const uint8_t REG_MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_MPU_USER_CTRL  = 0x6A;
static const uint8_t REG_MPU_INT_PIN_CFG= 0x37;

enum TipoMagnetometro {
  MAG_NO_DETECTADO = 0,
  MAG_HMC5883L     = 1,
  MAG_QMC5883L     = 2
};

/*
 * Calibracion hard-iron (desplazamiento) y soft-iron diagonal (escala).
 * Se obtiene con la prueba P-02 y el script analisis_calibracion_magnetometro.py
 */
struct CalibracionMagnetica {
  float desplazamiento[3];  // en cuentas crudas
  float escala[3];          // adimensional, ~1.0
};

class MagnetometroGY87 {
 public:
  MagnetometroGY87();

  /* Habilita el bypass del MPU6050 y detecta e inicializa el magnetometro.
   * Devuelve false si no encuentra ningun magnetometro conocido. */
  bool iniciar(TwoWire *bus = &Wire);

  /* Solo habilita el bypass, sin tocar el magnetometro. Util para P-00. */
  static bool habilitarBypassMPU6050(TwoWire *bus = &Wire);

  TipoMagnetometro tipo() const { return _tipo; }
  const char*      nombreTipo() const;
  uint8_t          direccion() const { return _dir; }

  bool leerCrudo(int16_t &x, int16_t &y, int16_t &z);
  bool leerCalibrado(float &x, float &y, float &z);

  /* Aplica calibracion y mapeo de ejes a valores YA leidos.
   * Se usa para no repetir la lectura I2C cuando ya se tienen los crudos. */
  void calibrarValores(int16_t cx, int16_t cy, int16_t cz,
                       float &x, float &y, float &z) const;

  /* Versiones que operan sobre valores ya calibrados (sin tocar el bus). */
  float rumboPlanoDe(float mx, float my) const;
  float rumboCompensadoDe(float mx, float my, float mz,
                          float acel_x, float acel_y, float acel_z) const;

  void  aplicarCalibracion(const CalibracionMagnetica &c) { _cal = c; }
  const CalibracionMagnetica& calibracion() const { return _cal; }

  /* Declinacion magnetica del sitio, en grados (positiva al este).
   * Para Puebla se determina en P-05 comparando contra rumbos conocidos. */
  void fijarDeclinacion(float grados) { _declinacion = grados; }

  /* Rumbo SIN compensacion de inclinacion. Solo valido con el rover nivelado.
   * Se conserva para poder comparar contra el compensado en P-06. */
  float rumboPlano();

  /* Rumbo CON compensacion de inclinacion usando el acelerometro.
   *
   * ATENCION: la convencion de signos y el mapeo de ejes entre el acelerometro
   * (MPU6050) y el magnetometro depende de la orientacion fisica de los chips
   * en la placa. NO se asume correcta: se VALIDA en P-05/P-06 comparando contra
   * rumbos conocidos. Si el rumbo gira al reves o esta desfasado 90 grados,
   * ajusta MAPEO_* mas abajo, no la calibracion.
   *
   * acel_* en g (o en cualquier unidad consistente; solo importa la relacion).
   */
  float rumboCompensado(float acel_x, float acel_y, float acel_z);

 private:
  bool _escribirReg(uint8_t dir, uint8_t reg, uint8_t valor);
  bool _leerRegs(uint8_t dir, uint8_t reg, uint8_t n, uint8_t *destino);
  bool _detectarDispositivo(uint8_t dir);
  bool _iniciarHMC();
  bool _iniciarQMC();

  TwoWire            *_bus;
  TipoMagnetometro    _tipo;
  uint8_t             _dir;
  CalibracionMagnetica _cal;
  float               _declinacion;
};

/* --------------------------------------------------------------------------
 * MAPEO DE EJES  (ajustar solo si P-05 lo exige)
 * Permite reordenar/invertir los ejes del magnetometro para que coincidan con
 * el marco del acelerometro y con el frente fisico del rover.
 * Valor por defecto: identidad.
 * -------------------------------------------------------------------------- */
#ifndef MAPEO_MAG_X
  #define MAPEO_MAG_X( x, y, z )  ( x)
#endif
#ifndef MAPEO_MAG_Y
  #define MAPEO_MAG_Y( x, y, z )  ( y)
#endif
#ifndef MAPEO_MAG_Z
  #define MAPEO_MAG_Z( x, y, z )  ( z)
#endif

#endif  // MAGNETOMETRO_GY87_H
