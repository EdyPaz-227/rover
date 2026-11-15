"""Pestaña IMU: giro Z y θ integrado. Sirve para verificar GYRO_SIGNO y la escala del giroscopio. Flujo I."""
import math
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

VENTANA_S = 20

AYUDA = ("<b>Prueba de signo:</b> rover parado → «Medir bias». Gíralo a mano en sentido ANTIHORARIO "
         "visto desde arriba: θ debe SUBIR. Si baja, pon GYRO_SIGNO = -1.0 en config.h.<br>"
         "<b>Prueba de escala:</b> da 1 vuelta completa a mano y regresa a la marca: θ ≈ 360°.")


class Imu(QtWidgets.QWidget):
    TITULO = "IMU"
    FLUJO = "I"

    def __init__(self, enlace):
        super().__init__()
        self.enlace = enlace
        self.datos = deque(maxlen=VENTANA_S * 25)
        self.nuevo = False

        b_bias = QtWidgets.QPushButton("Medir bias (θ = 0)")
        b_bias.clicked.connect(lambda: self.enlace.enviar("B"))
        self.lectura = QtWidgets.QLabel("θ = —")
        self.lectura.setStyleSheet("font-size:22px;font-weight:bold")
        ayuda = QtWidgets.QLabel(AYUDA)
        ayuda.setWordWrap(True)

        g = pg.GraphicsLayoutWidget()
        p1 = g.addPlot(row=0, col=0, title="ω z [°/s]")
        p2 = g.addPlot(row=1, col=0, title="θ [°]")
        for p in (p1, p2):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setXRange(-VENTANA_S, 0)
        self.c_w = p1.plot(pen=pg.mkPen("#1f77b4", width=2))
        self.c_t = p2.plot(pen=pg.mkPen("#d62728", width=2))

        arriba = QtWidgets.QHBoxLayout()
        arriba.addWidget(b_bias)
        arriba.addWidget(self.lectura)
        arriba.addWidget(ayuda, 1)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(arriba)
        lay.addWidget(g, 1)

    def recibir(self, tipo, d):
        if tipo == "I" and len(d) == 3:
            self.datos.append(d)
            self.nuevo = True

    def refrescar(self):
        if not self.nuevo:
            return
        self.nuevo = False
        a = np.array(self.datos)
        t = (a[:, 0] - a[-1, 0]) / 1000
        self.c_w.setData(t, np.degrees(a[:, 1]))
        self.c_t.setData(t, np.degrees(a[:, 2]))
        self.lectura.setText(f"θ = {math.degrees(a[-1, 2]):+.1f}°   ω = {math.degrees(a[-1, 1]):+.1f} °/s")
