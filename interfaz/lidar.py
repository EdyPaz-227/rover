"""Pestaña lidar: escaneo 2D del TF-Luna (espejo fijo). Flujo L (también enciende el stepper)."""
import time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

ESCALA_MAX_CM = 800   # rango inicial de la vista


def xy(pts):
    """[(ángulo°, dist cm)] -> x, y [cm]. x = frente del rover."""
    if not pts:
        return np.empty(0), np.empty(0)
    a = np.array(pts, float)
    r = np.radians(a[:, 0])
    return a[:, 1] * np.cos(r), a[:, 1] * np.sin(r)


class Barrido:
    """Junta puntos y separa vueltas cuando el ángulo da la vuelta (360 -> 0)."""

    def __init__(self):
        self.actual, self.anterior = [], []
        self.ang_ant = None
        self.vueltas = 0

    def meter(self, ang, dist):
        if self.ang_ant is not None and ang < self.ang_ant - 180:
            self.anterior, self.actual = self.actual, []
            self.vueltas += 1
        self.ang_ant = ang
        self.actual.append((ang, dist))


class Lidar(QtWidgets.QWidget):
    TITULO = "Lidar"
    FLUJO = "L"

    def __init__(self, enlace):
        super().__init__()
        self.enlace = enlace
        self.barrido = Barrido()
        self.n, self.t0, self.tasa = 0, time.time(), 0.0
        self.nuevo = False

        info = QtWidgets.QLabel("Al abrir esta pestaña el stepper arranca (rampa de 1 s); al salir se apaga. "
                                "0° = frente del rover (eje X, línea verde); ajústalo con THETA_OFS_LIDAR en config.h.")
        info.setWordWrap(True)
        self.estado = QtWidgets.QLabel("sin datos")
        self.estado.setStyleSheet("font-size:16px;font-weight:bold")
        self.plot = pg.PlotWidget(title="Escaneo 2D [cm]  (gris = vuelta anterior, rojo = actual)")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setRange(xRange=(-ESCALA_MAX_CM, ESCALA_MAX_CM), yRange=(-ESCALA_MAX_CM, ESCALA_MAX_CM))
        for r in (100, 200, 400, 800):
            c = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
            c.setPen(pg.mkPen("#bbb", style=QtCore.Qt.DotLine))
            self.plot.addItem(c)
        self.plot.plot([0, 30], [0, 0], pen=pg.mkPen("#2ca02c", width=3))
        self.sc_ant = pg.ScatterPlotItem(size=4, pen=None, brush=pg.mkBrush(140, 140, 140, 160))
        self.sc_act = pg.ScatterPlotItem(size=5, pen=None, brush=pg.mkBrush(214, 39, 40, 220))
        self.plot.addItem(self.sc_ant)
        self.plot.addItem(self.sc_act)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(info)
        lay.addWidget(self.estado)
        lay.addWidget(self.plot, 1)

    def recibir(self, tipo, d):
        if tipo != "L":
            return
        for k in range(0, len(d) - 1, 2):
            self.barrido.meter(d[k] / 10, d[k + 1])
        self.n += len(d) // 2
        self.nuevo = True

    def refrescar(self):
        if (dt := time.time() - self.t0) >= 1:
            self.tasa, self.n, self.t0 = self.n / dt, 0, time.time()
        if not self.nuevo:
            return
        self.nuevo = False
        b = self.barrido
        self.estado.setText(f"{self.tasa:.0f} puntos/s   vueltas: {b.vueltas}   puntos por vuelta: {len(b.anterior)}")
        self.sc_ant.setData(*xy(b.anterior))
        self.sc_act.setData(*xy(b.actual))


if __name__ == "__main__":
    b = Barrido()
    for _ in range(2):
        for ang in range(0, 360, 10):
            b.meter(ang, 100)
    assert b.vueltas == 1 and len(b.anterior) == 36 and len(b.actual) == 36
    x, y = xy(b.anterior)
    assert np.allclose(np.hypot(x, y), 100) and np.isclose(x[0], 100) and np.isclose(y[9], 100)
    print("lidar: ok")
