// mag.h — magnetómetro del GY-87 con la librería propia magnetometro_gy87 (autodetecta QMC/HMC).
// Iniciar DESPUÉS del MPU6050 (iniciarIMU): la librería activa el bypass del MPU.
// La calibración (hard-iron = desplazamiento, soft-iron diagonal = escala) se calcula en panel.py,
// llega con "C ..." y se guarda en flash con "W".
#pragma once
#include <magnetometro_gy87.h>

MagnetometroGY87 mag;
bool okMag = false;
CalibracionMagnetica calMag = {{0, 0, 0}, {1, 1, 1}};

bool iniciarMag() {
  okMag = mag.iniciar(&Wire);
  mag.aplicarCalibracion(calMag);
  return okMag;
}

void fijarCalMag(const CalibracionMagnetica &c) {
  calMag = c;
  mag.aplicarCalibracion(calMag);
}
