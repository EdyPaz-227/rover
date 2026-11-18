# Rover de Horak — calibración de χ (skid-steer 4WD)

## Contexto
Rover lunar de 4 ruedas, 4 motores DC con encoder, dirección **skid-steer** (los 2 motores de cada lado reciben el mismo setpoint). Suspensión: eje delantero pivotante sobre perno longitudinal (no afecta la cinemática en el plano). IMU montada en el **chasis**.

Modelo cinemático:
- v = (vR + vL) / 2
- ω = (vR − vL) / (χ · B)
- Inversa: vL = v − ω·χ·B/2, vR = v + ω·χ·B/2

χ ≥ 1 absorbe el derrape lateral al girar. Se calibra experimentalmente.

## Objetivo del código
Programa que calibre χ con giros en su lugar y guarde los resultados.

1. Ordenar vL = −vR durante t segundos (giro en su lugar).
2. Medir el ángulo real θ integrando el giroscopio Z de la IMU (restar bias medido en reposo antes de cada prueba).
3. Calcular χ = (vR − vL)·t / (B·θ), usando la velocidad **medida** por encoders, no la comandada.
4. Repetir N veces, en ambos sentidos (horario/antihorario) y a varias velocidades.
5. Reportar χ promedio, desviación estándar y χ por velocidad.

## Parámetros (todos variables en un solo archivo de config, nunca hardcodeados en la lógica)
- `L` — distancia entre ejes [m] (placeholder: 0.30)
- `B` — ancho de vía [m] (placeholder: 0.26)
- `r` — radio de llanta [m] (placeholder: 0.06)
- `CPR` — cuentas por revolución del encoder (incluye reducción)
- `SPEEDS` — lista de velocidades de prueba [m/s]
- `T_TURN` — duración de cada giro [s]
- `N_REPS` — repeticiones por velocidad y sentido

Además calcular y mostrar la relación L/B, porque entre mayor sea L/B, mayor es el χ esperado.

## Hardware confirmado
- MCU: **ESP32 DevKit 30 pines**, core Arduino esp32 3.3.x (compila con `arduino-cli compile --fqbn esp32:esp32:esp32 .`).
- Motores: 4× GM25-370 12V 140 rpm, reducción 45:1, encoder 493.9 PPR en la salida, stall ≤ 2 A. Se usa **solo 1 canal** del encoder (sin sentido → el signo sale del comando). Se cuentan ambos flancos (CHANGE) → CPR = 987.8.
- IMU: **GY-87** en el chasis; solo el MPU6050 (giro Z, ±500 °/s, 200 Hz). Magnetómetro: **siempre con la librería `..\libraries\magnetometro_gy87`** (autodetecta QMC/HMC; iniciar el MPU antes que el mag por el bypass). Pendiente de calibrar.
- Lidar: **TF-Luna** (UART2 16/17) en torreta giratoria impresa, basada en el MK2 de Charles Grassin (MIT): copia en `lidar/3d_lidar_CGrassin/` (firmware STM32, visor Processing, STL). **Ahora integrado en el ESP32 del rover** (`lidar.h`, espejo fijo, A4988) por falta de espacio. Versión previa en 2º ESP32: `lidar/lidar_torreta/` (TF-Luna UART2 16/17, DRV8825 STEP 25/DIR 26/EN 27, servos 32/33; salida USB `x\ty\tz\tfuerza` en cm). Visor: `lidar/ver_nube.py` o el LidarViewer original. Pendiente: probar en hardware y enlazarlo con el rover.
- GPS: GT-U7 a 9600 (TinyGPSPlus), probado antes en el CanSat Delphinus.
- Enlace: **ESP-NOW maestro/esclavo**. Esclavo = rover (`rover.ino`); maestro = puente USB↔ESP-NOW (`maestro/maestro.ino`) conectado a la PC. MACs reales configuradas (FF:FF… = broadcast si se cambia de placa). Todo sketch con ESP-NOW imprime su MAC al arrancar. El rover también acepta los mismos comandos por USB directo.
- PC: Python con pyserial en `..\.venv`. Patrón de referencia: `..\control_rover.py`.
- Drivers: 2× L298N, **uno por lado**. IN1+IN3 a un pin, IN2+IN4 a otro (dirección compartida por lado); ENA/ENB separados → PWM y PID por rueda. Jumpers de ENA/ENB quitados.
- Pines: ver `config.h`. Reservados para el futuro: 21/22 (I2C, IMU), 16/17 (UART2, lidar).

## Estado del código
- [x] `config.h` — parámetros y pines.
- [x] `ruedas.h` — L298N, encoders, PID por rueda (feedforward + PI), detección de saturación, paro si un encoder no cuenta.
- [x] `enlace.h` — Serial + ESP-NOW; `enviar()` sale por ambos.
- [x] `imu.h` — MPU6050 por I2C sin librería, bias (`B`), integración de θ, paro si la IMU no responde.
- [x] `rover.ino` — comandos `V vL vR`, `P pL pR` (PWM directo), `S`, `B`, `R`, `F M|I|G|L|N` (flujo de datos a pedido), `K` (ganancias PID por rueda), `C` (cal. magnetómetro), `W` (guardar en flash), `D` (config.h), `X 1|0` (depuración: fallos de encoder/IMU solo avisan; el paro por enlace sigue). Flujo M = telemetría CSV `t_ms,spL,spR,vDI,vTI,vDD,vTD,pwm×4,wz,theta,sat,paro` (por defecto); paro si no llegan comandos en 1 s. `USAR_ESPNOW=false` en config.h = solo USB.
- [x] `mag.h` — magnetómetro con `magnetometro_gy87`; calibración desde el panel, guardada en flash.
- [x] `lidar.h` — TF-Luna UART2 16/17 + A4988 (STEP 23, DIR 5, EN 15, 1/8 micropaso, timer con rampa), espejo fijo a 45° → 2D. El stepper solo gira con `F L`. Reemplaza al 2º ESP32 (`lidar/lidar_torreta/` queda como referencia).
- [x] `panel.py` + `interfaz/` — GUI PySide6/pyqtgraph por pestañas (motores/PID, IMU, magnetómetro con hierro duro/suave en `magcal.py`, lidar). Cada pestaña pide solo su flujo. Graba en `datos/panel_<flujo>_YYYYMMDD_HHMMSS.csv`. Autoprueba: `--test`.
- [x] `maestro/maestro.ino` — puente USB↔ESP-NOW.
- [x] `calibrar_chi.py` — corre las pruebas, guarda `datos/calib_chi_YYYYMMDD_HHMMSS.csv` (+ `_resumen.csv`), descarta saturadas, reporta χ global/por velocidad/por sentido. Lee parámetros de `config.h`. Autoprueba: `--test`.
- [x] `banco.py` — escalones de velocidad con ruedas al aire; métricas por rueda (subida, sobrepaso, error, SAT) + gráfica. Autoprueba: `--test`.
- [x] `pruebas/sensores/sensores.ino` — escaneo I2C + MPU6050 + magnetómetro + BMP180 + GPS, CSV a 10 Hz. `capturar.py` guarda el serial en `datos/`.
- [ ] Calibrar magnetómetro (P-02) con el rover armado → `CAL_MAG`; análisis con `..\pruebas_navegacion\analisis\analisis_calibracion_magnetometro.py`.
- [ ] Confirmar pines del GPS (propuesto UART1 RX=19/TX=18; 16/17 son del lidar).
- [ ] Probar en hardware: sentido de motores, GYRO_SIGNO, pull-ups de encoders, sintonizar KP/KI.
- [x] MACs reales: rover EC:64:C9:85:8F:B0, maestro 68:25:DD:32:08:78. Ambos flasheados; maestro con TX 11 dBm por brownout.

## Reglas
- Plataforma: firmware en ESP32 (core 3.3.x); registro y análisis en PC con Python. Pregunta antes de asumir lo demás.
- Separar en módulos: config, control de motores (PID por rueda), lectura de IMU/encoders, rutina de calibración y análisis.
- Guardar cada corrida en CSV con **fecha y hora en el nombre del archivo** (`calib_chi_YYYYMMDD_HHMMSS.csv`). Columnas: t, setpoints, velocidades medidas por rueda, ω de la IMU y θ acumulado.
- Descartar pruebas donde alguna rueda no alcanzó su setpoint (saturación) y marcarlas en el log.
- Hacer un paro seguro si se pierde la IMU o si un encoder deja de contar.
- Comentarios y mensajes en español.
- Es un prototipo: prioriza código simple y fácil de probar.
