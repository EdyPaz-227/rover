// maestro.ino — puente USB <-> ESP-NOW. Va conectado a la PC.
// Cada línea que llega por Serial se manda al rover; lo que manda el rover se imprime tal cual.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>

// MAC del rover (la imprime al arrancar). Con la MAC real ESP-NOW confirma la entrega y reintenta.
// FF:FF:FF:FF:FF:FF = broadcast, por si cambias de placa.
uint8_t MAC_ROVER[6] = {0xEC, 0x64, 0xC9, 0x85, 0x8F, 0xB0};

void alRecibir(const esp_now_recv_info_t *info, const uint8_t *d, int n) {
  Serial.write(d, n);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  WiFi.mode(WIFI_STA);
  // ponytail: potencia baja para evitar brownout con USB débil. Si falta alcance: WIFI_POWER_19_5dBm (máx.)
  WiFi.setTxPower(WIFI_POWER_11dBm);
  uint8_t m[6];
  esp_read_mac(m, ESP_MAC_WIFI_STA);  // del eFuse; WiFi.macAddress() da 00:00... tan temprano en core 3.x
  Serial.printf("# MAC del maestro: %02X:%02X:%02X:%02X:%02X:%02X\n", m[0], m[1], m[2], m[3], m[4], m[5]);
  if (esp_now_init() != ESP_OK) { Serial.println("# error iniciando ESP-NOW"); return; }
  esp_now_register_recv_cb(alRecibir);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, MAC_ROVER, 6);
  esp_now_add_peer(&p);
  Serial.println("# Maestro listo");
}

void loop() {
  if (!Serial.available()) return;
  String l = Serial.readStringUntil('\n') + "\n";
  esp_now_send(MAC_ROVER, (const uint8_t *)l.c_str(), min((int)l.length(), 250));
}
