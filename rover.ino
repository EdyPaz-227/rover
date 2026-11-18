// rover.ino — Rover de Horak (esclavo), skid-steer 4WD en ESP32 DevKit 30 pines.
// Recibe comandos por Serial (USB) o ESP-NOW (si USAR_ESPNOW); responde por ambos.
//
// Comandos:
//   V <vL> <vR>     velocidades por lado [m/s] con PID, ej. "V 0.2 -0.2". Reenviar < 1 s.
//   P <pL> <pR>     PWM directo por lado (-255..255), sin PID. Reenviar < 1 s.
//   S               parar
//   B               medir bias del giroscopio en reposo (1 s) y poner θ = 0
//   R               rearmar después de un paro de seguridad
//   F <M|I|G|L|N>   flujo de datos: M motores, I IMU, G magnetómetro, L lidar, N nada
//   K               reporta ganancias y calibración del magnetómetro ("K,..." y "C,...")
//   K <i> <kp> <ki> <kff>          ganancias de la rueda i (0=DI 1=TI 2=DD 3=TD)
//   C <ox> <oy> <oz> <sx> <sy> <sz> calibración del magnetómetro (desplazamiento y escala)
//   W               guarda ganancias + calibración del mag en flash
//   D               vuelve a los valores de config.h (y borra la flash)
//   X <1|0>         modo depuración: fallos de encoder/IMU solo avisan, no paran (ruedas AL AIRE).
//                   El paro por "sin comandos" sigue activo. Arranca en 0; no se guarda en flash.
//
// Flujos (uno a la vez, para no saturar el enlace):
//   M  t_ms,spL,spR,vDI,vTI,vDD,vTD,pwmDI,pwmTI,pwmDD,pwmTD,wz,theta,sat,paro   (sin prefijo: lo usan
//      calibrar_chi.py y banco.py; es el flujo por defecto al arrancar)
//   I  I,t_ms,wz,theta
//   G  G,t_ms,mag_x,mag_y,mag_z,rumbo     (crudos, para calibrar; rumbo plano ya calibrado)
//   L  L,a,d,a,d,...                      (ver lidar.h; también enciende el stepper)
// Mensajes: líneas que empiezan con '#'.
#include <Preferences.h>
#include "config.h"
#include "enlace.h"
#include "ruedas.h"
#include "imu.h"
#include "mag.h"
#include "lidar.h"

uint32_t tCtrl = 0, tCmd = 0;
char flujo = 'M';
Preferences prefs;

void reportarAjustes() {
  for (int i = 0; i < 4; i++) enviar("K,%d,%.4f,%.4f,%.4f\n", i, kp[i], ki[i], kff[i]);
  enviar("C,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f\n", calMag.desplazamiento[0], calMag.desplazamiento[1],
         calMag.desplazamiento[2], calMag.escala[0], calMag.escala[1], calMag.escala[2]);
}

void cargarAjustes() {
  if (!prefs.begin("rover", true)) return;   // no existe todavía: se queda con config.h
  if (prefs.getBytesLength("kp") == sizeof(kp)) {
    prefs.getBytes("kp", kp, sizeof(kp));
    prefs.getBytes("ki", ki, sizeof(ki));
    prefs.getBytes("kff", kff, sizeof(kff));
  }
  CalibracionMagnetica c;
  if (prefs.getBytes("mag", &c, sizeof(c)) == sizeof(c)) fijarCalMag(c);
  prefs.end();
}

void guardarAjustes() {
  prefs.begin("rover", false);
  prefs.putBytes("kp", kp, sizeof(kp));
  prefs.putBytes("ki", ki, sizeof(ki));
  prefs.putBytes("kff", kff, sizeof(kff));
  prefs.putBytes("mag", &calMag, sizeof(calMag));
  prefs.end();
  enviar("# ajustes guardados en flash\n");
}

void ajustesDeFabrica() {
  prefs.begin("rover", false);
  prefs.clear();
  prefs.end();
  for (int i = 0; i < 4; i++) { kp[i] = KP; ki[i] = KI; kff[i] = KFF; }
  fijarCalMag({{0, 0, 0}, {1, 1, 1}});
  enviar("# valores de config.h restaurados (flash borrada)\n");
  reportarAjustes();
}

void procesar(const String &l) {
  tCmd = millis();
  const char *a = l.c_str() + 1;
  char c = toupper(l[0]);
  if (c == 'V') {
    float vL = 0, vR = 0;
    if (sscanf(a, "%f %f", &vL, &vR) == 2) fijarVelocidades(vL, vR);
    else enviar("# error: usa V <vL> <vR>\n");
  } else if (c == 'P') {
    int pL = 0, pR = 0;
    if (sscanf(a, "%d %d", &pL, &pR) == 2) fijarPwm(pL, pR);
    else enviar("# error: usa P <pwmL> <pwmR>\n");
  } else if (c == 'S') {
    pararMotores();
  } else if (c == 'B') {
    if (moviendo()) enviar("# error: B solo con el rover parado\n");
    else if (medirBias()) enviar("# bias %.5f rad/s\n", biasZ);
    else activarParo("IMU no responde al medir bias");
  } else if (c == 'R') {
    paro = false;
    pararMotores();
    enviar("# rearmado\n");
  } else if (c == 'F') {
    char f = ' ';
    sscanf(a, " %c", &f);
    f = toupper(f);
    if (!strchr("MIGLN", f)) { enviar("# error: usa F <M|I|G|L|N>\n"); return; }
    flujo = f;
    encenderLidar(flujo == 'L');
    if (flujo == 'G' && !okMag) enviar("# magnetómetro no detectado\n");
    enviar("# flujo %c\n", flujo);
  } else if (c == 'K') {
    int i;
    float p, in, ff;
    if (sscanf(a, "%d %f %f %f", &i, &p, &in, &ff) == 4 && i >= 0 && i < 4) {
      kp[i] = p; ki[i] = in; kff[i] = ff; integ[i] = 0;
    }
    reportarAjustes();
  } else if (c == 'C') {
    CalibracionMagnetica m;
    if (sscanf(a, "%f %f %f %f %f %f", &m.desplazamiento[0], &m.desplazamiento[1], &m.desplazamiento[2],
               &m.escala[0], &m.escala[1], &m.escala[2]) == 6) fijarCalMag(m);
    else enviar("# error: usa C <ox> <oy> <oz> <sx> <sy> <sz>\n");
    reportarAjustes();
  } else if (c == 'X') {
    int on = 0;
    sscanf(a, "%d", &on);
    depuracion = on;
    if (depuracion && paro) { paro = false; pararMotores(); }   // rearma para poder mover
    enviar(depuracion ? "# DEPURACION ON: fallos de encoder/IMU NO paran (el paro por enlace sigue)\n"
                      : "# depuracion OFF: paros de seguridad activos\n");
  } else if (c == 'W') {
    guardarAjustes();
  } else if (c == 'D') {
    ajustesDeFabrica();
  } else {
    enviar("# comando desconocido: %s\n", l.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  iniciarEnlace();
  iniciarRuedas();
  if (!iniciarIMU() || !medirBias()) activarParo("IMU no encontrada (GY-87 en 21/22?)");
  if (!iniciarMag()) enviar("# magnetómetro no detectado (sigue sin él)\n");
  else enviar("# magnetómetro %s\n", mag.nombreTipo());
  cargarAjustes();
  iniciarLidar();
  enviar("# Rover listo. L/B = %.2f (mayor L/B => mayor chi esperado)\n", L / B);
  reportarAjustes();
  enviar("t_ms,spL,spR,vDI,vTI,vDD,vTD,pwmDI,pwmTI,pwmDD,pwmTD,wz,theta,sat,paro\n");
}

void loop() {
  String l = recibir();
  if (l.length()) procesar(l);

  actualizarIMU();
  actualizarLidar();
  uint32_t ahora = millis();

  // Paro si se pierde el enlace con la PC mientras el rover se mueve
  if (moviendo() && ahora - tCmd > CMD_TIMEOUT_MS) activarParo("sin comandos", true);

  if (ahora - tCtrl < DT_MS) return;
  tCtrl = ahora;
  actualizarRuedas(ahora);

  if (flujo == 'M') {
    int sat = 0;
    for (int i = 0; i < 4; i++) sat |= saturada[i] << i;
    enviar("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.4f,%.4f,%d,%d\n",
           ahora, sp[0], sp[1], vMed[0], vMed[1], vMed[2], vMed[3],
           pwm[0], pwm[1], pwm[2], pwm[3], wz, theta, sat, paro);
  } else if (flujo == 'I') {
    enviar("I,%lu,%.4f,%.4f\n", ahora, wz, theta);
  } else if (flujo == 'G' && okMag) {
    int16_t x, y, z;
    float mx, my, mz;
    if (mag.leerCrudo(x, y, z)) {
      mag.calibrarValores(x, y, z, mx, my, mz);
      enviar("G,%lu,%d,%d,%d,%.1f\n", ahora, x, y, z, mag.rumboPlanoDe(mx, my));
    }
  }
}
