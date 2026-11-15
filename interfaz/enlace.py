"""Enlace serial con el rover: un hilo lee líneas y las deja en una cola; la interfaz las saca.

Protocolo (ver rover.ino):
  '#...'            mensaje        -> ("#", "texto")
  15 números        flujo motores  -> ("M", [floats])     (sin prefijo, compatible con calibrar_chi.py)
  'I,..' 'G,..' 'L,..' 'K,..' 'C,..' -> (letra, [floats])
"""
import queue
import threading

from calibrar_chi import COLS, abrir


def parsear(linea):
    """Línea -> (tipo, datos) o None si es basura."""
    linea = linea.strip()
    if not linea:
        return None
    if linea.startswith("#"):
        return "#", linea
    p = linea.split(",")
    try:
        if p[0] in ("I", "G", "L", "K", "C"):
            return p[0], [float(x) for x in p[1:]]
        if len(p) == len(COLS) and p[0].isdigit():
            return "M", [float(x) for x in p]
    except ValueError:
        pass
    return None


class Enlace:
    def __init__(self):
        self.ser = None
        self.cola = queue.Queue()
        self.al_enviar = None   # callback opcional para mostrar en la bitácora lo que se manda

    @property
    def conectado(self):
        return self.ser is not None

    def conectar(self, puerto):
        self.ser = abrir(puerto)
        threading.Thread(target=self._leer, args=(self.ser,), daemon=True).start()

    def desconectar(self):
        if self.ser:
            try:
                self.ser.write(b"S\n")
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def enviar(self, cmd):
        if not self.ser:
            return
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as e:
            self.cola.put(("#", f"# error al enviar: {e}"))
            return
        if self.al_enviar:
            self.al_enviar(cmd)

    def _leer(self, ser):
        while self.ser is ser:
            try:
                linea = ser.readline().decode(errors="replace")
            except Exception as e:
                self.cola.put(("#", f"# puerto cerrado: {e}"))
                self.ser = None
                return
            if (r := parsear(linea)) is not None:
                self.cola.put(r + (linea.strip(),))   # (tipo, datos, linea_cruda)

    def pendientes(self, maximo=2000):
        """Saca lo que haya en la cola sin bloquear."""
        out = []
        while len(out) < maximo:
            try:
                out.append(self.cola.get_nowait())
            except queue.Empty:
                break
        return out


if __name__ == "__main__":
    assert parsear("# hola") == ("#", "# hola")
    assert parsear("I,100,0.5,1.2") == ("I", [100.0, 0.5, 1.2])
    m = parsear("1000,0.2,0.2,0.1,0.1,0.1,0.1,100,100,100,100,0.01,0.5,0,0")
    assert m[0] == "M" and len(m[1]) == 15
    assert parsear("1000,0.2") is None and parsear("basura,,") is None and parsear("I,x") is None
    print("enlace: ok")
