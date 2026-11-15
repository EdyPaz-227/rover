// enlace.h — comunicación: Serial (USB) y, si USAR_ESPNOW, ESP-NOW al maestro. Lo que se envía sale por ambos.
#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <stdarg.h>
#include "config.h"

char rxBuf[251];
volatile bool rxListo = false;

// Corre en la tarea de WiFi: solo copia el comando; loop() lo procesa.
void alRecibir(const esp_now_recv_info_t *info, const uint8_t *d, int n) {
  if (rxListo) return;  // ponytail: 1 comando en espera; si llegan 2 juntos se pierde uno (Python reenvía V cada 200 ms)
  n = min(n, (int)sizeof(rxBuf) - 1);
  memcpy(rxBuf, d, n);
  rxBuf[n] = 0;
  rxListo = true;
}

void iniciarEnlace() {
  WiFi.mode(WIFI_STA);
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_STA);  // del eFuse; WiFi.macAddress() da 00:00... tan temprano en core 3.x
  Serial.printf("# MAC del rover: %02X:%02X:%02X:%02X:%02X:%02X\n", m[0], m[1], m[2], m[3], m[4], m[5]);
  if (!USAR_ESPNOW) { Serial.println("# ESP-NOW desactivado (USAR_ESPNOW = false): solo USB"); return; }
  if (esp_now_init() != ESP_OK) { Serial.println("# error iniciando ESP-NOW (sigue por USB)"); return; }
  esp_now_register_recv_cb(alRecibir);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, MAC_MAESTRO, 6);
  esp_now_add_peer(&p);
  Serial.println("# ESP-NOW listo (acepta USB y ESP-NOW)");
}

// Como printf, pero manda la línea por Serial y por ESP-NOW (máx. 250 bytes).
void enviar(const char *fmt, ...) {
  char buf[251];
  va_list a;
  va_start(a, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, a);
  va_end(a);
  Serial.print(buf);
  if (USAR_ESPNOW) esp_now_send(MAC_MAESTRO, (uint8_t *)buf, min(n, 250));
}

// Línea recibida por Serial o ESP-NOW ("" si no hay nada).
String recibir() {
  String l;
  if (rxListo) { l = rxBuf; rxListo = false; }
  else if (Serial.available()) l = Serial.readStringUntil('\n');
  l.trim();
  return l;
}
