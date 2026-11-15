// sensores.ino — prueba en banco de los sensores del Rover de Horak (sin motores).
// GY-87 (MPU6050 + magnetómetro con la librería magnetometro_gy87 + BMP180) y GPS GT-U7.
// Al arrancar: escaneo I2C, estado de cada sensor y bias del giroscopio. Luego CSV a 10 Hz.
// Comandos por el monitor serie (115200, "Nueva línea"):
//   i   escaneo I2C        b   recalcular bias del giroscopio (no mover)
//
// Calibración del magnetómetro (P-02): con el rover ARMADO completo, girarlo lento en todas
// las orientaciones ~90 s mientras se captura:   python ..\..\capturar.py mag_p02
// y luego:  analisis_calibracion_magnetometro.py datos\mag_p02_....csv  (pruebas_navegacion\analisis)
// Pegar el resultado en CAL_MAG. Las columnas mag_x,mag_y,mag_z son CRUDAS justo para eso.
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <mpu6050_simple.h>
#include <magnetometro_gy87.h>

// ---- Configuración (pines iguales a ../../config.h) ----
const int PIN_SDA = 21, PIN_SCL = 22;
// GPS en UART1: el UART2 (16/17) queda para el lidar. CONFIRMAR al cablear.
const int GPS_RX = 19;          // RX del ESP32 <- TX del GPS
const int GPS_TX = 18;          // TX del ESP32 -> RX del GPS (opcional)
const uint32_t GPS_BAUD = 9600;   // fábrica del GT-U7
const uint32_t PERIODO_MS = 100;  // 10 Hz
const float DECLINACION = 0.0f;   // [grados, + al este]
// De P-02. Identidad = sin calibrar. (La del CanSat Delphinus NO sirve: otro vehículo.)
const CalibracionMagnetica CAL_MAG = {{0, 0, 0}, {1, 1, 1}};

MPU6050Simple mpu;
MagnetometroGY87 mag;
Adafruit_BMP085 bmp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
bool okMPU, okMAG, okBMP, avisoGPS;

void escanearI2C() {
  Serial.print("# I2C:");
  int n = 0;
  for (uint8_t d = 1; d < 127; d++) {
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", d); n++; }
  }
  Serial.printf("  (%d)  esperados: 0x68 MPU, 0x77 BMP, 0x0D QMC o 0x1E HMC\n", n);
}

void calibrarBias() {
  Serial.println("# bias del giroscopio: NO MUEVAS el rover (3 s)");
  mpu.calibrarBiasGiroscopio(600, 5);
  const float *b = mpu.biasGiroscopio();
  Serial.printf("# bias_giro [deg/s] = %.3f, %.3f, %.3f\n", b[0], b[1], b[2]);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  Wire.setTimeOut(20);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  escanearI2C();                   // antes del bypass: el magnetómetro todavía no se ve
  okMPU = mpu.iniciar(&Wire);      // PRIMERO el MPU (su reset borra el bypass)...
  okMAG = mag.iniciar(&Wire);      // ...DESPUÉS el magnetómetro (activa el bypass)
  okBMP = bmp.begin(BMP085_STANDARD, &Wire);
  escanearI2C();                   // ahora debe aparecer 0x0D o 0x1E
  mag.aplicarCalibracion(CAL_MAG);
  mag.fijarDeclinacion(DECLINACION);
  Serial.printf("# MPU:%d  MAG:%d (%s)  BMP:%d  GPS: RX=%d a %lu bps\n",
                okMPU, okMAG, mag.nombreTipo(), okBMP, GPS_RX, GPS_BAUD);
  if (okMPU) calibrarBias();
  Serial.println("t_ms,acel_x,acel_y,acel_z,giro_x,giro_y,giro_z,mag_x,mag_y,mag_z,"
                 "rumbo_plano,rumbo_comp,presion_pa,temp_c,gps_fix,lat,lon,sats,hdop");
}

void loop() {
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') escanearI2C();
    if (c == 'b' && okMPU) calibrarBias();
  }

  if (!avisoGPS && millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.printf("# GPS sin datos: TX del GPS -> GPIO%d? baudios? (prueba ../../gps_diag2)\n", GPS_RX);
    avisoGPS = true;
  }

  static uint32_t tAnt = 0;
  if (millis() - tAnt < PERIODO_MS) return;
  tAnt = millis();

  MedidaInercial m = okMPU ? mpu.leer() : MedidaInercial{{NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, false};
  int16_t cx = 0, cy = 0, cz = 0;
  bool okLect = okMAG && mag.leerCrudo(cx, cy, cz);
  float rPlano = NAN, rComp = NAN;
  if (okLect) {
    float mx, my, mz;
    mag.calibrarValores(cx, cy, cz, mx, my, mz);
    rPlano = mag.rumboPlanoDe(mx, my);
    if (m.valida) rComp = mag.rumboCompensadoDe(mx, my, mz, m.acel[0], m.acel[1], m.acel[2]);
  }
  // ponytail: BMP bloquea ~13 ms por lectura; a 10 Hz sobra. Leerlo menos seguido si estorba.
  long pres = okBMP ? bmp.readPressure() : 0;
  float temp = okBMP ? bmp.readTemperature() : NAN;

  Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,", millis(),
                m.acel[0], m.acel[1], m.acel[2], m.giro[0], m.giro[1], m.giro[2]);
  if (okLect) Serial.printf("%d,%d,%d,", cx, cy, cz); else Serial.print(",,,");
  Serial.printf("%.1f,%.1f,%ld,%.1f,%d,%.7f,%.7f,%lu,%.2f\n", rPlano, rComp, pres, temp,
                gps.location.isValid(), gps.location.lat(), gps.location.lng(),
                gps.satellites.value(), gps.hdop.hdop());
}
