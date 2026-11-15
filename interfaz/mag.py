"""Pestaña magnetómetro: captura la nube cruda, calcula hierro duro/suave (magcal) y la manda al rover. Flujo G."""
import numpy as np
import pyqtgraph as pg
from PySide6 import QtWidgets

from interfaz import magcal

AYUDA = ("Con el rover <b>ARMADO</b> (motores, baterías, cables en su lugar) y lejos de metales: "
         "«Capturar» y gira el rover LENTO en todas las orientaciones ~90 s (los tres planos). "
         "Luego «Calcular». Si σ ≤ 0.05 cumple (criterio P-02) → «Enviar al rover» → «Guardar en flash».")
PLANOS = [("X", "Y", 0, 1), ("X", "Z", 0, 2), ("Y", "Z", 1, 2)]


class Mag(QtWidgets.QWidget):
    TITULO = "Magnetómetro"
    FLUJO = "G"

    def __init__(self, enlace):
        super().__init__()
        self.enlace = enlace
        self.crudos = []          # [x, y, z]
        self.cal = None           # (desplazamiento, escala)
        self.capturando = False
        self.nuevo = False
        self.rumbo = None

        self.b_cap = QtWidgets.QPushButton("Capturar")
        self.b_cap.setCheckable(True)
        self.b_cap.toggled.connect(self._capturar)
        botones = [("Borrar", self._borrar), ("Calcular", self._calcular),
                   ("Enviar al rover", self._enviar), ("Guardar en flash", lambda: self.enlace.enviar("W")),
                   ("Quitar calibración", lambda: self.enlace.enviar("C 0 0 0 1 1 1"))]
        fila = QtWidgets.QHBoxLayout()
        fila.addWidget(self.b_cap)
        for txt, fn in botones:
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(fn)
            fila.addWidget(b)
        fila.addStretch()

        ayuda = QtWidgets.QLabel(AYUDA)
        ayuda.setWordWrap(True)
        self.resultado = QtWidgets.QLabel("Sin calcular.")
        self.resultado.setStyleSheet("font-family:Consolas,monospace")
        self.en_rover = QtWidgets.QLabel("En el rover: —")
        self.estado = QtWidgets.QLabel("0 puntos")
        self.estado.setStyleSheet("font-size:16px;font-weight:bold")

        g = pg.GraphicsLayoutWidget()
        self.sc_crudo, self.sc_cal = [], []
        for k, (a, b, _, _) in enumerate(PLANOS):
            p = g.addPlot(row=0, col=k, title=f"{a}–{b}  (gris = crudo, azul = calibrado)")
            p.setAspectLocked(True)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", a)
            p.setLabel("left", b)
            self.sc_crudo.append(pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(150, 150, 150, 120)))
            self.sc_cal.append(pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(31, 119, 180, 160)))
            p.addItem(self.sc_crudo[-1])
            p.addItem(self.sc_cal[-1])

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(ayuda)
        lay.addLayout(fila)
        lay.addWidget(self.estado)
        lay.addWidget(g, 1)
        lay.addWidget(self.resultado)
        lay.addWidget(self.en_rover)

    def _capturar(self, on):
        self.capturando = on
        self.b_cap.setText("Detener captura" if on else "Capturar")

    def _borrar(self):
        self.crudos.clear()
        self.cal = None
        for s in self.sc_crudo + self.sc_cal:
            s.clear()
        self.resultado.setText("Sin calcular.")
        self.nuevo = True

    def _calcular(self):
        try:
            off, esc, semi = magcal.ajustar_elipsoide(*np.array(self.crudos).T)
        except ValueError as e:
            self.resultado.setText(f"No se pudo calcular: {e}")
            return
        self.cal = (off, esc)
        sigma = magcal.sigma_radio(magcal.aplicar(self.crudos, off, esc))
        cumple = "CUMPLE" if sigma <= magcal.CRITERIO_SIGMA else "NO cumple: captura más orientaciones"
        self.resultado.setText(
            f"Hierro duro (desplazamiento): {off[0]:9.2f} {off[1]:9.2f} {off[2]:9.2f}  cuentas\n"
            f"Hierro suave (escala):        {esc[0]:9.5f} {esc[1]:9.5f} {esc[2]:9.5f}\n"
            f"Semiejes: {semi[0]:.1f} {semi[1]:.1f} {semi[2]:.1f}   n = {len(self.crudos)}   "
            f"σ radio normalizado = {sigma:.4f}  ({cumple}, criterio {magcal.CRITERIO_SIGMA})")
        self.nuevo = True

    def _enviar(self):
        if self.cal is None:
            self.resultado.setText("Primero «Calcular».")
            return
        off, esc = self.cal
        self.enlace.enviar("C " + " ".join(f"{v:.2f}" for v in off) + " " + " ".join(f"{v:.5f}" for v in esc))

    def recibir(self, tipo, d):
        if tipo == "G" and len(d) == 5:
            self.rumbo = d[4]
            if self.capturando:
                self.crudos.append(d[1:4])
            self.nuevo = True
        elif tipo == "C" and len(d) == 6:
            self.en_rover.setText("En el rover: desplazamiento " + " ".join(f"{v:.2f}" for v in d[:3])
                                  + "   escala " + " ".join(f"{v:.5f}" for v in d[3:]))

    def refrescar(self):
        if not self.nuevo:
            return
        self.nuevo = False
        rumbo = f"   rumbo (plano, con la calibración del rover) = {self.rumbo:.1f}°" if self.rumbo is not None else ""
        self.estado.setText(f"{len(self.crudos)} puntos" + (" — capturando…" if self.capturando else "") + rumbo)
        if not self.crudos:
            return
        a = np.array(self.crudos)
        c = magcal.aplicar(a, *self.cal) if self.cal else None
        for k, (_, _, i, j) in enumerate(PLANOS):
            self.sc_crudo[k].setData(a[:, i], a[:, j])
            if c is not None:
                self.sc_cal[k].setData(c[:, i], c[:, j])
