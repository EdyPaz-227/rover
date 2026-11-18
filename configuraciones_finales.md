# Configuraciones finales — Rover de Horak

Registro de las configuraciones que se van fijando en las pruebas. Agregar una sección por cada prueba nueva.

---

## Reglas de operación (decisión del equipo)

| Regla | Valor |
|---|---|
| PWM mínimo para que el rover avance | **150** |
| PWM para que se detenga | **0** (bajar directo a 0, sin valores intermedios) |

> Los umbrales medidos con las ruedas al aire son menores (55–75, ver abajo). El 150 deja margen para
> cuando el rover cargue su peso en el suelo. **Ojo:** esta regla todavía NO está en el firmware; hoy el PID
> puede mandar PWM menores a 150 por sí solo.

---

## Prueba 2026-11-16 — Calibración de motores (ruedas al aire, por ESP-NOW desde el maestro en COM5)

### PID por rueda (guardado en la flash del rover y como valores de fábrica en `config.h`)

| Rueda | KP | KI | KFF |
|---|---|---|---|
| DI | 200 | 600 | 230 |
| TI | 200 | 600 | 230 |
| DD | 200 | 600 | 230 |
| TD | 200 | 600 | 230 |

Antes: KP 150, KI 300, KFF 300. Resultado medido a 0.15–0.5 m/s: sobrepaso 2–12 % (antes 10–20 %),
error final < 1.3 % (antes 1.3–2.8 %), establecimiento 0.15–0.8 s en la mayoría de los casos (antes ~2–2.7 s).
Gráfica: `datos/pid_fabrica_vs_nueva.png`.

### `config.h`

| Parámetro | Antes | Ahora | Por qué |
|---|---|---|---|
| `V_MAX` | 0.85 m/s | **0.72 m/s** | velocidad real con PWM 255 |
| `KFF` | 255 / V_MAX | **230** (fijo) | la curva PWM→v no es lineal; 230 es el mejor valor único |
| `KP` | 150 | **200** | |
| `KI` | 300 | **600** | |

### Umbral de PWM medido (lazo abierto, escalones de 5, ruedas al aire)

| Rueda | Arranca adelante | Se detiene (bajando) | Arranca en reversa | v con PWM 255 |
|---|---|---|---|---|
| DI | 60 | 45 | 65 | 0.76 m/s |
| TI | 70 | 50 | **no gira** | 0.73 m/s |
| DD | 55 | 45 | 60 | 0.72 m/s |
| TD | 75 | 50 | 70 | 0.73 m/s |

Velocidad típica (adelante, subiendo): PWM 75 ≈ 0.23 m/s · 100 ≈ 0.39 · 150 ≈ 0.56 · 200 ≈ 0.64 · 255 ≈ 0.73.

### Encoder

- 1 cuenta en 50 ms = **0.0076 m/s**. La "escalera" en la gráfica de velocidad es esta resolución, no el PID.
- Con PID a velocidad constante: ±0.5–0.7 cuentas, PWM ±1. Sin oscilación.
- DI es más ruidosa incluso sin PID → revisar algo mecánico en esa rueda.

### Firmware (flasheado al rover 2026-11-16)

- Comando nuevo `X 1|0`: modo depuración. Los fallos de encoder/IMU solo avisan y no paran los motores.
  El paro por "sin comandos" (1 s) sigue activo. Arranca apagado y no se guarda en flash.
  En el panel: casilla "Depuración" en la caja "Modo" de Motores/PID.

### Pendientes detectados

- [ ] **TI no gira en reversa**: no le llega voltaje. Revisar el puente IN2↔IN4 del L298N izquierdo o el canal B del driver.
- [ ] **La ESP32 del rover se reinicia** al arrancar los 4 motores de golpe (brownout). Revisar la alimentación de la ESP.
- [ ] Opcional: offset de feedforward por rueda + integración condicional (en simulación baja el establecimiento a ~0.45 s).
- [ ] Llevar la regla de PWM mínimo 150 al firmware, si se decide.

### Datos de estas pruebas

- `datos/barrido_pwm_20261116_182948.csv` — barrido de PWM (umbrales y curva PWM→v)
- `datos/lazo_fabrica_20261116_183541.csv` — escalones con ganancias de fábrica
- `datos/lazo_A_kp200_ki600_kff230_20261116_183853.csv` — escalones con las ganancias nuevas
