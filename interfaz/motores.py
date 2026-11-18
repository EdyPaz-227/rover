"""Pestaña de motores: mando por lado (PID o PWM directo), escalones, teclado WASD,
ganancias PID por rueda y gráficas en tiempo real de velocidad y PWM.

Flujo M. La dirección es compartida por lado (L298N), así que se manda por lado; cada rueda
tiene su propio PID y se grafica por separado.
"""
import re
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

from calibrar_chi import AQUI, PERIODO_V

RUEDAS = ["DI (del. izq)", "TI (tras. izq)", "DD (del. der)", "TD (tras. der)"]
COLORES = ["#1f77b4", "#2ca02c", "#d62728", "#ff7f0e"]
VENTANA_S = 15   # segundos visibles en las gráficas


def v_max():
    texto = (AQUI / "config.h").read_text(encoding="utf-8")
    return float(re.search(r"\bV_MAX\s*=\s*([\d.]+)", texto).group(1))


class Motores(QtWidgets.QWidget):
    TITULO = "Motores / PID"
    FLUJO = "M"

    def __init__(self, enlace):
        super().__init__()
        self.enlace = enlace
        self.vmax = v_max()
        self.datos = deque(maxlen=VENTANA_S * 25)   # filas M (20 Hz)
        self.teclas = set()
        self.escalon = None      # (l, r) mientras dura un escalón
        self.enviando = False
        self.nuevo = False

        izq = QtWidgets.QVBoxLayout()
        izq.addWidget(self._caja_modo())
        izq.addWidget(self._caja_mando())
        izq.addWidget(self._caja_escalon())
        izq.addWidget(self._caja_teclado())
        izq.addWidget(self._caja_pid())
        izq.addStretch()
        panel_izq = QtWidgets.QWidget()
        panel_izq.setLayout(izq)
        panel_izq.setFixedWidth(370)

        lay = QtWidgets.QHBoxLayout(self)
        lay.addWidget(panel_izq)
        lay.addWidget(self._graficas(), 1)
        self._cambio_modo()

        self.latido = QtCore.QTimer(self, interval=int(PERIODO_V * 1000), timeout=self._latido)
        self.latido.start()
        QtWidgets.QApplication.instance().installEventFilter(self)

    # ---------- construcción ----------
    def _caja_modo(self):
        caja = QtWidgets.QGroupBox("Modo")
        self.r_vel = QtWidgets.QRadioButton("Velocidad con PID [m/s]", checked=True)
        self.r_pwm = QtWidgets.QRadioButton("PWM directo, sin PID [-255..255]")
        self.r_vel.toggled.connect(self._cambio_modo)
        self.depurar = QtWidgets.QCheckBox("Depuración: fallos de encoder/IMU NO paran")
        self.depurar.setToolTip("Para medir con osciloscopio. Solo con las ruedas al aire.\n"
                                "El paro por pérdida de enlace (1 s sin comandos) sigue activo.\n"
                                "Se apaga solo si el rover se reinicia.")
        self.depurar.clicked.connect(self._depuracion)
        lay = QtWidgets.QVBoxLayout(caja)
        lay.addWidget(self.r_vel)
        lay.addWidget(self.r_pwm)
        lay.addWidget(self.depurar)
        return caja

    def _depuracion(self, on):
        if on and QtWidgets.QMessageBox.question(
                self, "Modo depuración",
                "Los fallos de encoder e IMU ya no detendrán los motores (solo aparecerá un aviso).\n"
                "¿Las ruedas están al aire?") != QtWidgets.QMessageBox.Yes:
            self.depurar.setChecked(False)
            return
        self.enlace.enviar(f"X {int(on)}")
        self.depurar.setStyleSheet("color:#c62828; font-weight:bold" if on else "")

    def _caja_mando(self):
        caja = QtWidgets.QGroupBox("Mando por lado")
        self.sliders, self.etiquetas = [], []
        lay = QtWidgets.QGridLayout(caja)
        for fila, nombre in enumerate(("Izq", "Der")):
            s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            s.valueChanged.connect(lambda _, k=fila: self._slider(k))
            e = QtWidgets.QLabel("0")
            e.setFixedWidth(60)
            lay.addWidget(QtWidgets.QLabel(nombre), fila, 0)
            lay.addWidget(s, fila, 1)
            lay.addWidget(e, fila, 2)
            self.sliders.append(s)
            self.etiquetas.append(e)
        self.unir = QtWidgets.QCheckBox("Mover ambos lados juntos")
        b_cero = QtWidgets.QPushButton("Parar (Esc)")
        b_cero.clicked.connect(self.detener)
        lay.addWidget(self.unir, 2, 0, 1, 2)
        lay.addWidget(b_cero, 2, 2)
        return caja

    def _caja_escalon(self):
        caja = QtWidgets.QGroupBox("Escalón (para sintonizar)")
        self.esc_lado = QtWidgets.QComboBox()
        self.esc_lado.addItems(["ambos", "izq", "der", "giro en su lugar"])
        self.esc_amp = QtWidgets.QDoubleSpinBox(decimals=2, value=0.25, singleStep=0.05)
        self.esc_dur = QtWidgets.QDoubleSpinBox(decimals=1, value=2.0, minimum=0.2, maximum=10, suffix=" s")
        b = QtWidgets.QPushButton("Dar escalón")
        b.clicked.connect(self._dar_escalon)
        lay = QtWidgets.QFormLayout(caja)
        lay.addRow("Lado", self.esc_lado)
        lay.addRow("Amplitud", self.esc_amp)
        lay.addRow("Duración", self.esc_dur)
        lay.addRow(b)
        return caja

    def _caja_teclado(self):
        caja = QtWidgets.QGroupBox("Teclado")
        self.tec_on = QtWidgets.QCheckBox("WASD activo (W/S avanza/retrocede, A/D gira)")
        self.tec_v = QtWidgets.QDoubleSpinBox(decimals=2, value=0.2, singleStep=0.05)
        lay = QtWidgets.QFormLayout(caja)
        lay.addRow(self.tec_on)
        lay.addRow("Magnitud", self.tec_v)
        return caja

    def _caja_pid(self):
        caja = QtWidgets.QGroupBox("PID por rueda   u = KFF·ref + KP·e + KI·∫e")
        self.tabla = QtWidgets.QTableWidget(4, 3)
        self.tabla.setHorizontalHeaderLabels(["KP", "KI", "KFF"])
        self.tabla.setVerticalHeaderLabels([r.split()[0] for r in RUEDAS])
        self.tabla.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tabla.setFixedHeight(150)
        botones = [("Aplicar", self._aplicar_pid), ("DI → todas", self._copiar_fila),
                   ("Leer del rover", lambda: self.enlace.enviar("K")),
                   ("Guardar en flash", lambda: self.enlace.enviar("W")),
                   ("Valores de config.h", self._fabrica)]
        lay = QtWidgets.QGridLayout(caja)
        lay.addWidget(self.tabla, 0, 0, 1, 2)
        for k, (txt, fn) in enumerate(botones):
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(fn)
            lay.addWidget(b, 1 + k // 2, k % 2)
        self.info = QtWidgets.QLabel("")
        self.info.setWordWrap(True)
        lay.addWidget(self.info, 4, 0, 1, 2)
        return caja

    def _graficas(self):
        g = pg.GraphicsLayoutWidget()
        self.curvas_v, self.curvas_sp, self.plots_v = [], [], []
        for i, nombre in enumerate(RUEDAS):
            p = g.addPlot(row=i // 2, col=i % 2, title=nombre)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", "v [m/s]")
            p.getAxis("left").enableAutoSIPrefix(False)
            p.setXRange(-VENTANA_S, 0)
            self.curvas_sp.append(p.plot(pen=pg.mkPen("#888", width=2, style=QtCore.Qt.DashLine), name="setpoint"))
            self.curvas_v.append(p.plot(pen=pg.mkPen(COLORES[i], width=2), name="medida"))
            self.plots_v.append(p)
        p = g.addPlot(row=2, col=0, colspan=2, title="PWM aplicado (0–255)")
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setYRange(0, 260)
        p.setXRange(-VENTANA_S, 0)
        p.addLegend(offset=(5, 5))
        self.curvas_pwm = [p.plot(pen=pg.mkPen(COLORES[i], width=2), name=RUEDAS[i].split()[0]) for i in range(4)]
        return g

    # ---------- mando ----------
    def _es_pwm(self):
        return self.r_pwm.isChecked()

    def _cambio_modo(self, *_):
        self.detener()
        tope = 255 if self._es_pwm() else 100   # en velocidad: % de V_MAX
        for s in self.sliders:
            s.setRange(-tope, tope)
        maximo = 255 if self._es_pwm() else self.vmax
        paso, dec = (10, 0) if self._es_pwm() else (0.05, 2)
        for sp in (self.esc_amp, self.tec_v):
            sp.setDecimals(dec)
            sp.setRange(0, maximo)
            sp.setSingleStep(paso)
            sp.setValue(120 if self._es_pwm() else 0.2)

    def _slider(self, k):
        if self.unir.isChecked():
            otro = self.sliders[1 - k]
            otro.blockSignals(True)
            otro.setValue(self.sliders[k].value())
            otro.blockSignals(False)
        for s, e in zip(self.sliders, self.etiquetas):
            e.setText(f"{self._valor_slider(s):.0f}" if self._es_pwm() else f"{self._valor_slider(s):.2f} m/s")

    def _valor_slider(self, s):
        return s.value() if self._es_pwm() else s.value() / 100 * self.vmax

    def _dar_escalon(self):
        a = self.esc_amp.value()
        self.escalon = {"ambos": (a, a), "izq": (a, 0), "der": (0, a), "giro en su lugar": (-a, a)}[self.esc_lado.currentText()]
        self._latido()
        QtCore.QTimer.singleShot(int(self.esc_dur.value() * 1000), self._fin_escalon)

    def _fin_escalon(self):
        self.escalon = None
        self._latido()

    def mando(self):
        """(izq, der) según prioridad: teclado > escalón > sliders."""
        if self.teclas:
            avance = ("W" in self.teclas) - ("S" in self.teclas)
            giro = ("A" in self.teclas) - ("D" in self.teclas)
            v = self.tec_v.value()
            return (max(-v, min(v, v * (avance - giro))), max(-v, min(v, v * (avance + giro))))
        if self.escalon:
            return self.escalon
        return tuple(self._valor_slider(s) for s in self.sliders)

    def _latido(self):
        """Reenvía el mando cada PERIODO_V (el rover para solo si no recibe nada en 1 s)."""
        if not self.isVisible():
            return
        l, r = self.mando()
        if l or r:
            self.enlace.enviar(f"P {round(l)} {round(r)}" if self._es_pwm() else f"V {l:.3f} {r:.3f}")
            self.enviando = True
        elif self.enviando:
            self.enlace.enviar("S")
            self.enviando = False

    def detener(self):
        self.teclas.clear()
        self.escalon = None
        for s in getattr(self, "sliders", []):
            s.setValue(0)
        if getattr(self, "enviando", False):
            self.enlace.enviar("S")
            self.enviando = False

    def eventFilter(self, obj, ev):
        if ev.type() in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease) and self.tec_on.isChecked() \
                and self.isVisible() and not ev.isAutoRepeat():
            letra = ev.text().upper()
            if letra in ("W", "A", "S", "D"):
                (self.teclas.add if ev.type() == QtCore.QEvent.KeyPress else self.teclas.discard)(letra)
                self._latido()
                return True
        return False

    # ---------- PID ----------
    def _aplicar_pid(self):
        for i in range(4):
            try:
                kp, ki, kff = (float(self.tabla.item(i, c).text()) for c in range(3))
            except (AttributeError, ValueError):
                self.info.setText(f"Fila {RUEDAS[i]}: valores inválidos")
                return
            self.enlace.enviar(f"K {i} {kp:g} {ki:g} {kff:g}")
        self.info.setText("Aplicado (solo RAM). 'Guardar en flash' para que sobreviva al reinicio.")

    def _copiar_fila(self):
        for c in range(3):
            txt = self.tabla.item(0, c).text() if self.tabla.item(0, c) else ""
            for i in range(1, 4):
                self.tabla.setItem(i, c, QtWidgets.QTableWidgetItem(txt))

    def _fabrica(self):
        if QtWidgets.QMessageBox.question(self, "Valores de fábrica",
                                          "¿Volver a config.h y borrar la flash (ganancias y calibración del mag)?") \
                == QtWidgets.QMessageBox.Yes:
            self.enlace.enviar("D")

    # ---------- datos ----------
    def recibir(self, tipo, d):
        if tipo == "M":
            self.datos.append(d)
            self.nuevo = True
        elif tipo == "K" and len(d) == 4 and 0 <= d[0] < 4:
            for c in range(3):
                self.tabla.setItem(int(d[0]), c, QtWidgets.QTableWidgetItem(f"{d[1 + c]:g}"))

    def refrescar(self):
        if not self.nuevo or not self.datos:
            return
        self.nuevo = False
        a = np.array(self.datos)
        t = (a[:, 0] - a[-1, 0]) / 1000
        for i in range(4):
            self.curvas_sp[i].setData(t, a[:, 1 + (i >= 2)])
            self.curvas_v[i].setData(t, a[:, 3 + i])
            self.curvas_pwm[i].setData(t, a[:, 7 + i])
            sat = int(a[-1, 13]) >> i & 1
            self.plots_v[i].setTitle(f"{RUEDAS[i]}   v = {a[-1, 3 + i]:+.3f} m/s   PWM {a[-1, 7 + i]:.0f}"
                                     + ("   <b style='color:#c62828'>SATURADA</b>" if sat else ""))
