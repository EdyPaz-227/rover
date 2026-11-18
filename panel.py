"""Panel de pruebas y calibración del rover, conectado por USB (o al maestro ESP-NOW, mismo protocolo).

Cada pestaña es un módulo de interfaz/ y le pide al rover SOLO su flujo de datos (F M/I/G/L).
Para agregar una sección: una clase con TITULO, FLUJO, recibir(tipo, datos) y refrescar(), y
ponerla en PESTANAS.

Uso:  ..\\.venv\\Scripts\\python panel.py
      ..\\.venv\\Scripts\\python panel.py --test     (autopruebas sin rover)
Teclas: Esc = PARO (en cualquier pestaña).
"""
import sys
from datetime import datetime

from calibrar_chi import AQUI, COLS

ENCABEZADOS = {
    "M": ",".join(COLS),
    "I": "tipo,t_ms,wz,theta",
    "G": "tipo,t_ms,mag_x,mag_y,mag_z,rumbo",
    "L": "tipo,pares (ang_decimas_grado,dist_cm) repetidos",
}

if "--test" in sys.argv:
    import runpy
    for m in ("interfaz.enlace", "interfaz.magcal", "interfaz.lidar"):
        runpy.run_module(m, run_name="__main__")
    sys.exit()

import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

from interfaz.enlace import Enlace
from interfaz.imu import Imu
from interfaz.lidar import Lidar
from interfaz.mag import Mag
from interfaz.motores import Motores

PESTANAS = [Motores, Imu, Mag, Lidar]


class Panel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover de Horak — panel de pruebas")
        self.enlace = Enlace()
        self.enlace.al_enviar = lambda c: self.log(f"> {c}") if c[0] not in "VP" else None
        self.archivo = None

        # --- barra de conexión ---
        self.puertos = QtWidgets.QComboBox()
        b_act = QtWidgets.QPushButton("↻")
        b_act.setToolTip("Actualizar puertos")
        b_act.clicked.connect(self.listar_puertos)
        self.b_con = QtWidgets.QPushButton("Conectar")
        self.b_con.clicked.connect(self.conectar)
        b_paro = QtWidgets.QPushButton("PARO (Esc)")
        b_paro.setStyleSheet("background:#c62828;color:white;font-weight:bold;padding:4px 14px")
        b_paro.clicked.connect(self.paro)
        b_rearmar = QtWidgets.QPushButton("Rearmar")
        b_rearmar.clicked.connect(lambda: self.enlace.enviar("R"))
        self.b_grabar = QtWidgets.QPushButton("● Grabar")
        self.b_grabar.setCheckable(True)
        self.b_grabar.setToolTip("Guarda el flujo de la pestaña actual en datos/panel_<flujo>_YYYYMMDD_HHMMSS.csv")
        self.b_grabar.toggled.connect(self.grabar)
        self.estado = QtWidgets.QLabel("desconectado")
        barra = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("Puerto:"), self.puertos, b_act, self.b_con, b_paro, b_rearmar, self.b_grabar):
            barra.addWidget(w)
        barra.addWidget(self.estado, 1)

        # --- pestañas ---
        self.tabs = QtWidgets.QTabWidget()
        self.pestanas = [P(self.enlace) for P in PESTANAS]
        for p in self.pestanas:
            caja = QtWidgets.QScrollArea(widgetResizable=True)   # barra de scroll en pantallas chicas
            caja.setWidget(p)
            self.tabs.addTab(caja, p.TITULO)
        self.tabs.currentChanged.connect(self.cambio_pestana)

        # --- bitácora + comando manual ---
        self.bitacora = QtWidgets.QPlainTextEdit(readOnly=True)
        self.bitacora.setMaximumBlockCount(500)
        self.bitacora.setFixedHeight(110)
        self.manual = QtWidgets.QLineEdit(placeholderText="comando manual (ej. K, F N, V 0.2 0.2) + Enter")
        self.manual.returnPressed.connect(self.enviar_manual)

        cuerpo = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(cuerpo)
        lay.addLayout(barra)
        lay.addWidget(self.tabs, 1)
        lay.addWidget(self.bitacora)
        lay.addWidget(self.manual)
        self.setCentralWidget(cuerpo)

        esc = QtGui.QShortcut(QtGui.QKeySequence("Esc"), self)
        esc.setContext(QtCore.Qt.ApplicationShortcut)
        esc.activated.connect(self.paro)
        self.reloj = QtCore.QTimer(self, interval=40, timeout=self.tick)
        self.reloj.start()
        self.listar_puertos()

    # ---------- conexión ----------
    def listar_puertos(self):
        self.puertos.clear()
        for p in list_ports.comports():
            self.puertos.addItem(f"{p.device} — {p.description}", p.device)

    def conectar(self):
        if self.enlace.conectado:
            self.b_grabar.setChecked(False)
            self.enlace.desconectar()
            self.b_con.setText("Conectar")
            self.estado.setText("desconectado")
            return
        puerto = self.puertos.currentData()
        if not puerto:
            return self.log("# no hay puerto seleccionado")
        try:
            self.enlace.conectar(puerto)
        except Exception as e:
            return self.log(f"# no se pudo abrir {puerto}: {e}")
        self.b_con.setText("Desconectar")
        self.estado.setText(f"conectado a {puerto}")
        self.enlace.enviar("S")
        self.enlace.enviar("K")
        self.cambio_pestana()

    def paro(self):
        for p in self.pestanas:
            if hasattr(p, "detener"):
                p.detener()
        self.enlace.enviar("S")

    def actual(self):
        """Pestaña visible (cada una va dentro de un QScrollArea)."""
        return self.pestanas[self.tabs.currentIndex()]

    def cambio_pestana(self, *_):
        self.b_grabar.setChecked(False)
        self.paro()   # al cambiar de sección, el rover se detiene
        self.enlace.enviar(f"F {self.actual().FLUJO}")

    def enviar_manual(self):
        if t := self.manual.text().strip():
            self.enlace.enviar(t)
        self.manual.clear()

    # ---------- grabación ----------
    def grabar(self, on):
        if self.archivo:
            self.archivo.close()
            self.log(f"# guardado {self.archivo.name}")
            self.archivo = None
        if on:
            flujo = self.actual().FLUJO
            (AQUI / "datos").mkdir(exist_ok=True)
            ruta = AQUI / "datos" / f"panel_{flujo}_{datetime.now():%Y%m%d_%H%M%S}.csv"
            self.archivo = open(ruta, "w", encoding="utf-8", newline="")
            self.archivo.write(ENCABEZADOS.get(flujo, "linea") + "\n")
            self.log(f"# grabando en {ruta.name}")

    # ---------- ciclo ----------
    def log(self, texto):
        self.bitacora.appendPlainText(texto)

    def tick(self):
        flujo = self.actual().FLUJO
        for tipo, datos, cruda in self.enlace.pendientes():
            if tipo == "#":
                self.log(datos)
                if "PARO" in datos:
                    self.estado.setText("⚠ " + datos.lstrip("# "))
                    self.estado.setStyleSheet("color:#c62828;font-weight:bold")
                elif "rearmado" in datos:
                    self.estado.setText("rearmado")
                    self.estado.setStyleSheet("")
                continue
            if self.archivo and tipo == flujo:
                self.archivo.write(cruda + "\n")
            for p in self.pestanas:
                p.recibir(tipo, datos)
        self.actual().refrescar()

    def closeEvent(self, e):
        self.b_grabar.setChecked(False)
        self.enlace.desconectar()
        super().closeEvent(e)


if __name__ == "__main__":
    pg.setConfigOptions(antialias=True)
    app = QtWidgets.QApplication(sys.argv)
    w = Panel()
    zona = app.primaryScreen().availableGeometry()   # que no se salga en pantallas chicas
    w.resize(min(1300, zona.width()), min(850, zona.height() - 40))
    w.show()
    sys.exit(app.exec())
