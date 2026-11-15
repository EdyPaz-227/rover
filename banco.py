"""Prueba de banco del PID por rueda: escalones de velocidad con las ruedas AL AIRE.

Usa el firmware normal (rover.ino). Por cada escalón manda V durante T_ESC y luego 0, graba la
telemetría y calcula por rueda: tiempo de subida (10–90 %), sobrepaso, error en estado estable
y si se saturó. Guarda datos/banco_YYYYMMDD_HHMMSS.csv y la gráfica .png (si hay matplotlib).

Uso:  ..\\.venv\\Scripts\\python banco.py [COM5] [--lado izq|der|ambos]
      ..\\.venv\\Scripts\\python banco.py --test
Para la gráfica:  ..\\.venv\\Scripts\\python -m pip install matplotlib
"""
import csv
import re
import statistics
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports

from calibrar_chi import AQUI, COLS, PERIODO_V, Paro, abrir, esperar, fila, leer_config

T_ESC = 2.0     # [s] duración de cada escalón
T_CERO = 1.0    # [s] en cero entre escalones
RUEDAS = [("vDI", "spL"), ("vTI", "spL"), ("vDD", "spR"), ("vTD", "spR")]


def escalones(cfg_txt):
    """Velocidades de config.h + una cerca de V_MAX (para ver dónde satura) + una en reversa."""
    v_max = float(re.search(r"\bV_MAX\s*=\s*([\d.]+)", cfg_txt).group(1))
    vs = leer_config(cfg_txt)["SPEEDS"]
    return vs + [round(0.9 * v_max, 3), -vs[-1]]


def metricas(ts, vs, sp):
    """Respuesta a un escalón 0 -> sp. ts en s desde el inicio del escalón."""
    y = [v / sp for v in vs]                 # normalizada: 1 = setpoint
    cola = y[int(0.6 * len(y)):] or y        # último 40 % = estado estable
    cruce = lambda nivel: next((t for t, yi in zip(ts, y) if yi >= nivel), None)
    t10, t90 = cruce(0.1), cruce(0.9)
    return dict(subida=t90 - t10 if t10 is not None and t90 is not None else None,
                sobrepaso=max(0.0, max(y) - 1) * 100,
                error=(1 - statistics.mean(cola)) * 100)


def correr(ser, v, lado):
    """Un escalón de T_ESC + T_CERO. Devuelve las filas de telemetría."""
    vL = v if lado in ("izq", "ambos") else 0.0
    vR = v if lado in ("der", "ambos") else 0.0
    filas, t0, t_env = [], time.time(), -1.0
    while (t := time.time() - t0) < T_ESC + T_CERO:
        if t - t_env >= PERIODO_V:
            ser.write((f"V {vL:.3f} {vR:.3f}\n" if t < T_ESC else "V 0 0\n").encode())
            t_env = t
        linea = ser.readline().decode(errors="replace")
        if (f := fila(linea)) is not None:
            filas.append(f)
            if f["paro"]:
                raise Paro("el rover reportó paro (¿encoder desconectado?)")
        elif "PARO" in linea:
            raise Paro(linea.strip())
    return filas


def analizar(filas, lado):
    """Métricas por rueda del tramo con setpoint != 0."""
    activas = [r for r in RUEDAS if lado == "ambos" or r[1] == ("spL" if lado == "izq" else "spR")]
    res = []
    for vcol, spcol in activas:
        tramo = [f for f in filas if f[spcol] != 0]
        if len(tramo) < 5:
            continue
        t0 = tramo[0]["t_ms"]
        m = metricas([(f["t_ms"] - t0) / 1000 for f in tramo], [f[vcol] for f in tramo], tramo[0][spcol])
        bit = RUEDAS.index((vcol, spcol))
        m.update(rueda=vcol[1:], sat=any(int(f["sat"]) >> bit & 1 for f in tramo),
                 pwm=max(abs(f["pwm" + vcol[1:]]) for f in tramo))
        res.append(m)
    return res


def graficar(todas, ruta_png):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("(sin matplotlib: no hay gráfica. ..\\.venv\\Scripts\\python -m pip install matplotlib)")
        return
    t0 = todas[0]["t_ms"]
    t = [(f["t_ms"] - t0) / 1000 for f in todas]
    fig, ejes = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    for ax, (vcol, spcol) in zip(ejes, RUEDAS):
        ax.plot(t, [f[spcol] for f in todas], "k--", label="setpoint")
        ax.plot(t, [f[vcol] for f in todas], label="medida")
        ax2 = ax.twinx()
        ax2.plot(t, [f["pwm" + vcol[1:]] for f in todas], color="tab:orange", alpha=0.4)
        ax2.set_ylim(-260, 260)
        ax.set_ylabel(f"{vcol[1:]} [m/s]")
        ax.legend(loc="upper left", fontsize=8)
    ejes[-1].set_xlabel("t [s]   (naranja = PWM)")
    fig.tight_layout()
    fig.savefig(ruta_png, dpi=120)
    print(f"Gráfica: {ruta_png}")
    plt.show()


def main():
    lado = sys.argv[sys.argv.index("--lado") + 1] if "--lado" in sys.argv else "ambos"
    args = [a for a in sys.argv[1:] if not a.startswith("--") and a not in ("izq", "der", "ambos")]
    cfg_txt = (AQUI / "config.h").read_text(encoding="utf-8")
    puertos = [p.device for p in list_ports.comports()]
    puerto = args[0] if args else (puertos[0] if puertos else None)
    if not puerto:
        sys.exit("No hay puertos seriales.")
    ser = abrir(puerto)
    print(f"Conectado a {puerto}. Lado: {lado}. Escalones: {escalones(cfg_txt)} m/s")
    input("RUEDAS AL AIRE. Enter para empezar (Ctrl+C aborta)... ")
    ser.write(b"R\n")
    esperar(ser, 0.5)

    sello = datetime.now().strftime("%Y%m%d_%H%M%S")
    (AQUI / "datos").mkdir(exist_ok=True)
    ruta = AQUI / "datos" / f"banco_{sello}.csv"
    todas = []
    print(f"\n{'v':>7} {'rueda':>5} {'subida[s]':>9} {'sobrep[%]':>9} {'error[%]':>8} {'PWMmax':>6}  sat")
    with open(ruta, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(["escalon", "v_obj"] + COLS)
        try:
            for i, v in enumerate(escalones(cfg_txt), 1):
                filas = correr(ser, v, lado)
                todas += filas
                for f in filas:
                    w.writerow([i, v] + [f[c] for c in COLS])
                for m in analizar(filas, lado):
                    sub = f"{m['subida']:.2f}" if m["subida"] is not None else "nunca"
                    print(f"{v:7.3f} {m['rueda']:>5} {sub:>9} {m['sobrepaso']:9.1f} "
                          f"{m['error']:8.1f} {m['pwm']:6.0f}  {'SAT' if m['sat'] else ''}")
        except (Paro, KeyboardInterrupt, serial.SerialException) as e:
            print(f"\n*** ABORTADO: {e or 'Ctrl+C'}")
        finally:
            try:
                ser.write(b"S\n")
            except serial.SerialException:
                pass
    print(f"\nDatos: {ruta}")
    print("Guía: sobrepaso alto/oscila -> baja KP; error estable grande o lento -> sube KI;\n"
          "      SAT en el escalón alto -> V_MAX real es menor que en config.h.")
    if todas:
        graficar(todas, ruta.with_suffix(".png"))


def test():
    cfg_txt = (AQUI / "config.h").read_text(encoding="utf-8")
    e = escalones(cfg_txt)
    assert len(e) >= 3 and e[-1] < 0 and max(e) <= 0.85, e
    # primer orden, tau = 0.2 s, sp = 0.3: subida 10–90 % = tau·ln 9 ≈ 0.44 s, sin sobrepaso
    import math
    ts = [i * 0.05 for i in range(41)]
    m = metricas(ts, [0.3 * (1 - math.exp(-t / 0.2)) for t in ts], 0.3)
    assert abs(m["subida"] - 0.2 * math.log(9)) < 0.06 and m["sobrepaso"] == 0 and abs(m["error"]) < 1, m
    m = metricas(ts, [-0.3 * min(1.2, t / 0.5) for t in ts], -0.3)   # reversa con 20 % de sobrepaso
    assert abs(m["sobrepaso"] - 20) < 1e-6 and m["error"] < 0, m
    assert metricas(ts, [0.0] * len(ts), 0.3)["subida"] is None       # rueda que no gira
    base = dict(zip(COLS, [0.0] * len(COLS)))
    filas = [dict(base, t_ms=t * 50, spL=0.3, vDI=0.3, vTI=0.3, sat=2, pwmTI=255) for t in range(10)]
    r = analizar(filas, "izq")
    assert [x["rueda"] for x in r] == ["DI", "TI"] and not r[0]["sat"] and r[1]["sat"], r
    print("ok")


if __name__ == "__main__":
    test() if "--test" in sys.argv else main()
