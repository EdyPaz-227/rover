# Para seguir en otra compu (notas para Claude)

Leer primero `CLAUDE.md` (contexto completo del rover). Memorias de la sesión anterior en `_memoria/`:
cópialas a la carpeta de memoria de Claude de este proyecto o tómalas como contexto.

## Estado al 2026-11-15
- ESP-NOW activado (`USAR_ESPNOW = true`): el rover acepta USB y ESP-NOW a la vez.
- MACs: rover `EC:64:C9:85:8F:B0`, maestro `68:25:DD:32:08:78`. Ambos flasheados con su firmware.
- Todo sketch con ESP-NOW imprime su MAC al arrancar con `esp_read_mac` (en core 3.x `WiFi.macAddress()` da 00:00 al inicio).
- El maestro tenía brownout: se cambió el cable USB y se bajó TX a 11 dBm (`maestro.ino`). Si falta alcance, subir a 19.5 dBm.
- Panel probado por USB directo (llega telemetría). **Pendiente: probar el panel por el maestro** (rover a batería).

## Setup en la compu nueva
1. Descomprimir para que quede `Documentos\Arduino\rover\` (las rutas relativas usan `..\`).
2. Librería: copiar `_libraries/magnetometro_gy87` a `Documentos\Arduino\libraries\`. Otras: TinyGPSPlus (Library Manager).
3. Arduino IDE con core esp32 3.3.x. Placa: ESP32 Dev Module. Si no entra a modo descarga, mantener BOOT.
4. Python: crear venv en `Documentos\Arduino\.venv` e instalar `pip install -r requirements.txt` (viene en el zip).
   Probar: `..\.venv\Scripts\python panel.py --test`.
5. Abrir el panel: `..\.venv\Scripts\python panel.py` (no usar el Python global).

## Usuario
- Prefiere comentarios/mensajes en español, código simple de prototipo.
- Cerrar el Monitor Serie del IDE antes de usar el puerto (lo bloquea).
- Antes de flashear, confirmar qué placa está conectada (por la MAC) para no cruzar firmwares.
