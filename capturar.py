"""Guarda todo lo que llega por serial en datos/<nombre>_YYYYMMDD_HHMMSS.csv hasta Ctrl+C.
Sirve para pruebas/sensores (calibración del magnetómetro, deriva del GPS, ruido de la IMU).

Uso:  ..\\.venv\\Scripts\\python capturar.py mag_p02 [COM5] [--seg 90]
"""
import sys
import time
from datetime import datetime

from serial.tools import list_ports

from calibrar_chi import AQUI, abrir

args = [a for a in sys.argv[1:] if not a.startswith("--")]
seg = float(sys.argv[sys.argv.index("--seg") + 1]) if "--seg" in sys.argv else float("inf")
if "--seg" in sys.argv:
    args.remove(sys.argv[sys.argv.index("--seg") + 1])
if not args:
    sys.exit(__doc__)
puertos = [p.device for p in list_ports.comports()]
puerto = args[1] if len(args) > 1 else (puertos[0] if puertos else sys.exit("No hay puertos."))

(AQUI / "datos").mkdir(exist_ok=True)
ruta = AQUI / "datos" / f"{args[0]}_{datetime.now():%Y%m%d_%H%M%S}.csv"
ser = abrir(puerto)
n, t0 = 0, time.time()
print(f"{puerto} -> {ruta}  (Ctrl+C para terminar)")
with open(ruta, "w", encoding="utf-8", newline="") as f:
    try:
        while time.time() - t0 < seg:
            linea = ser.readline().decode(errors="replace")
            if not linea:
                continue
            f.write(linea.rstrip("\r\n") + "\n")
            if linea.startswith("#"):
                print(linea.rstrip())
            else:
                n += 1
                print(f"\r{n} filas, {time.time() - t0:.0f} s", end="")
    except KeyboardInterrupt:
        pass
print(f"\nGuardado: {ruta}")
