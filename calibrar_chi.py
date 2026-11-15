"""Calibración de χ del rover (skid-steer) con giros en su lugar.

Habla con el rover por un puerto serial: el maestro ESP-NOW o el rover directo por USB
(el protocolo es el mismo). Los parámetros (B, L, SPEEDS, T_TURN, N_REPS) se leen de config.h.

Por cada velocidad, sentido y repetición:
  S -> B (bias del giroscopio, θ = 0) -> V vL vR durante T_TURN -> S y graba el frenado.
  χ = ∫(vR − vL) dt / (B·θ), con las velocidades MEDIDAS por los encoders y θ de la IMU.
Se descarta la prueba si alguna rueda se saturó. Si el rover reporta paro, se aborta todo.

Uso:  ..\\.venv\\Scripts\\python calibrar_chi.py [COM5]
      ..\\.venv\\Scripts\\python calibrar_chi.py --test
"""
import csv
import re
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

AQUI = Path(__file__).parent
PERIODO_V = 0.2   # reenviar V cada tanto (menor que CMD_TIMEOUT_MS del rover)
T_FRENADO = 1.5   # seguir grabando tras S: el derrape al frenar también cuenta en θ
COLS = "t_ms,spL,spR,vDI,vTI,vDD,vTD,pwmDI,pwmTI,pwmDD,pwmTD,wz,theta,sat,paro".split(",")


class Paro(Exception):
    pass


def leer_config(texto):
    """Saca los parámetros de config.h (fuente única)."""
    num = lambda n: float(re.search(rf"\b{n}\s*=\s*([-\d.]+)", texto).group(1))
    speeds = re.search(r"SPEEDS\[\]\s*=\s*\{([^}]*)\}", texto).group(1)
    return dict(L=num("L"), B=num("B"), T_TURN=num("T_TURN"), N_REPS=int(num("N_REPS")),
                SPEEDS=[float(x.strip().rstrip("f")) for x in speeds.split(",")])


def fila(linea):
    """Línea de telemetría -> dict, o None si es otra cosa (mensajes '#', basura)."""
    p = linea.strip().split(",")
    if len(p) != len(COLS) or not p[0].isdigit():
        return None
    try:
        return dict(zip(COLS, map(float, p)))
    except ValueError:
        return None


def calcular_chi(filas, B):
    """χ = ∫(vR − vL) dt / (B·θ). Trapecios con el t real (aguanta paquetes perdidos)."""
    dv = lambda f: (f["vDD"] + f["vTD"]) / 2 - (f["vDI"] + f["vTI"]) / 2
    integral = sum((b["t_ms"] - a["t_ms"]) / 1000 * (dv(a) + dv(b)) / 2
                   for a, b in zip(filas, filas[1:]))
    theta = filas[-1]["theta"] - filas[0]["theta"]
    return integral / (B * theta) if abs(theta) > 0.1 else float("nan"), theta


def esperar(ser, segundos, buscar=None):
    """Lee e imprime mensajes '#' hasta 'segundos' o hasta ver 'buscar'. Aborta si hay paro."""
    fin = time.time() + segundos
    while time.time() < fin:
        linea = ser.readline().decode(errors="replace")
        if "PARO" in linea:
            raise Paro(linea.strip())
        if linea.startswith("#"):
            print("   ", linea.strip())
            if buscar and linea.startswith(buscar):
                return True
    return False


def prueba(ser, v, sentido, T):
    """Un giro en su lugar. sentido +1 = antihorario (vR > 0). Devuelve las filas grabadas."""
    vR = sentido * v
    vL = -vR
    ser.write(b"S\n")
    esperar(ser, 0.8)
    ser.reset_input_buffer()
    ser.write(b"B\n")
    if not esperar(ser, 4, "# bias"):
        raise Paro("el rover no respondió a B (¿enlace o IMU?)")

    filas, t0, t_env, s_enviado = [], time.time(), -1.0, False
    while (t := time.time() - t0) < T + T_FRENADO:
        if t < T and t - t_env >= PERIODO_V:
            ser.write(f"V {vL:.3f} {vR:.3f}\n".encode())
            t_env = t
        elif t >= T and not s_enviado:
            ser.write(b"S\n")
            s_enviado = True
        linea = ser.readline().decode(errors="replace")
        if (f := fila(linea)) is not None:
            filas.append(f)
            if f["paro"]:
                raise Paro("el rover reportó paro")
        elif "PARO" in linea:
            raise Paro(linea.strip())
    return filas


def abrir(puerto):
    s = serial.Serial(baudrate=115200, timeout=0.05)
    s.port = puerto
    s.dtr = s.rts = False  # que abrir el puerto no reinicie al ESP
    s.open()
    return s


def main():
    cfg = leer_config((AQUI / "config.h").read_text(encoding="utf-8"))
    B = cfg["B"]
    print(f"B = {B} m, L = {cfg['L']} m, L/B = {cfg['L'] / B:.2f} (mayor L/B => mayor χ esperado)")
    print(f"Velocidades {cfg['SPEEDS']} m/s, T_TURN = {cfg['T_TURN']} s, N_REPS = {cfg['N_REPS']}")

    puertos = [p.device for p in list_ports.comports()]
    puerto = sys.argv[1] if len(sys.argv) > 1 else (puertos[0] if puertos else None)
    if not puerto:
        sys.exit("No hay puertos seriales. Conecta el maestro (o el rover por USB).")
    ser = abrir(puerto)
    print(f"Conectado a {puerto}")
    input("Rover en el piso, con espacio para girar. Enter para empezar (Ctrl+C aborta)... ")
    ser.write(b"R\n")

    sello = datetime.now().strftime("%Y%m%d_%H%M%S")
    (AQUI / "datos").mkdir(exist_ok=True)
    ruta = AQUI / "datos" / f"calib_chi_{sello}.csv"
    ruta_res = AQUI / "datos" / f"calib_chi_{sello}_resumen.csv"
    resultados = []
    with open(ruta, "w", newline="") as fcsv:
        crudo = csv.writer(fcsv)
        crudo.writerow(["prueba", "v", "sentido", "rep"] + COLS)
        n = 0
        try:
            for v in cfg["SPEEDS"]:
                for sentido in (1, -1):
                    for rep in range(1, cfg["N_REPS"] + 1):
                        n += 1
                        nombre = "antihorario" if sentido > 0 else "horario"
                        print(f"[{n}] v = {v} m/s {nombre} rep {rep}")
                        filas = prueba(ser, v, sentido, cfg["T_TURN"])
                        for f in filas:
                            crudo.writerow([n, v, sentido, rep] + [f[c] for c in COLS])
                        r = dict(prueba=n, v=v, sentido=sentido, rep=rep, chi=float("nan"),
                                 theta=0.0, valida=False, motivo="")
                        if len(filas) < 10:
                            r["motivo"] = "casi sin telemetría (¿enlace?)"
                        elif any(f["sat"] for f in filas):
                            r["motivo"] = "SATURADA: alguna rueda no alcanzó su setpoint"
                        else:
                            r["chi"], r["theta"] = calcular_chi(filas, B)
                            if r["chi"] != r["chi"]:  # NaN
                                r["motivo"] = "θ casi cero (¿IMU?)"
                            elif r["chi"] < 0:
                                r["motivo"] = "θ con signo contrario a vR−vL: cambia GYRO_SIGNO"
                            else:
                                r["valida"] = True
                        print(f"     χ = {r['chi']:.3f}  θ = {r['theta']:.2f} rad  {r['motivo'] or 'ok'}")
                        resultados.append(r)
        except (Paro, KeyboardInterrupt, serial.SerialException) as e:
            print(f"\n*** ABORTADO: {e or 'Ctrl+C'}")
        finally:
            try:
                ser.write(b"S\n")
            except serial.SerialException:
                pass

    with open(ruta_res, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["prueba", "v", "sentido", "rep", "chi", "theta", "valida", "motivo"])
        w.writeheader()
        w.writerows(resultados)
    print(f"\nDatos: {ruta}\nResumen: {ruta_res}")
    reporte(resultados)


def reporte(resultados):
    val = [r for r in resultados if r["valida"]]
    print(f"Pruebas válidas: {len(val)} de {len(resultados)}")
    if not val:
        return
    def linea(etiqueta, rs):
        chis = [r["chi"] for r in rs]
        sd = statistics.stdev(chis) if len(chis) > 1 else 0.0
        print(f"  {etiqueta:<22} χ = {statistics.mean(chis):.3f} ± {sd:.3f}  (n={len(chis)})")
    linea("GLOBAL", val)
    for v in sorted({r["v"] for r in val}):
        linea(f"v = {v} m/s", [r for r in val if r["v"] == v])
    for s, nombre in ((1, "antihorario"), (-1, "horario")):
        if rs := [r for r in val if r["sentido"] == s]:
            linea(nombre, rs)


def test():
    cfg = leer_config((AQUI / "config.h").read_text(encoding="utf-8"))
    assert cfg["B"] == 0.26 and cfg["L"] == 0.30 and cfg["N_REPS"] >= 1 and cfg["SPEEDS"], cfg
    assert fila("# bias 0.001") is None and fila("1,2") is None
    linea = "100,-0.2,0.2,-0.2,-0.2,0.2,0.2,90,90,90,90,1.5,0.3,0,0"
    assert fila(linea)["theta"] == 0.3
    # Δv = 0.5 m/s durante 2 s, B = 0.26, χ = 1.2  =>  θ = 0.5·2 / (0.26·1.2)
    th = 0.5 * 2 / (0.26 * 1.2)
    filas = [dict(t_ms=t, vDI=-0.25, vTI=-0.25, vDD=0.25, vTD=0.25, theta=th * t / 2000)
             for t in range(0, 2001, 50)]
    chi, theta = calcular_chi(filas, 0.26)
    assert abs(chi - 1.2) < 1e-9 and abs(theta - th) < 1e-9, chi
    nan = calcular_chi([dict(filas[0]), dict(filas[0], t_ms=50)], 0.26)[0]
    assert nan != nan  # θ = 0 -> NaN, no división entre cero
    print("ok")


if __name__ == "__main__":
    test() if "--test" in sys.argv else main()
