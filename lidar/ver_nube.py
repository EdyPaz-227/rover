"""Muestra una nube de puntos del lidar (líneas "x\ty\tz\tfuerza", las de '#' se ignoran).
Alternativa al LidarViewer de Processing.  Captura:  ..\\..\\.venv\\Scripts\\python ..\\capturar.py lidar
Uso:  python ver_nube.py ..\\datos\\lidar_YYYYMMDD_HHMMSS.csv     (necesita matplotlib)
"""
import sys

import matplotlib.pyplot as plt

pts = []
for linea in open(sys.argv[1], encoding="utf-8", errors="replace"):
    p = linea.strip().split("\t")
    if len(p) == 4 and not linea.startswith("#"):
        pts.append([float(v) for v in p])
if not pts:
    sys.exit("Sin puntos en el archivo.")
x, y, z, f = zip(*pts)
ax = plt.figure(figsize=(9, 8)).add_subplot(projection="3d")
ax.scatter(x, y, z, c=f, s=2, cmap="viridis")
ax.scatter([0], [0], [0], c="red", s=40)   # el lidar
ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]"); ax.set_zlabel("z [cm]")
ax.set_box_aspect((max(x) - min(x) or 1, max(y) - min(y) or 1, max(z) - min(z) or 1))
ax.set_title(f"{len(pts)} puntos")
plt.show()
