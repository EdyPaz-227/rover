"""Calibración del magnetómetro: hierro duro (desplazamiento) y hierro suave diagonal (escala).

Mismo ajuste que pruebas_navegacion/analisis/analisis_calibracion_magnetometro.py (P-02), para que
los resultados sean comparables. Es la corrección que aplica la librería magnetometro_gy87:
    calibrado = (crudo - desplazamiento) * escala
"""
import numpy as np

CRITERIO_SIGMA = 0.05   # P-02: desviación del radio normalizado aceptable


def ajustar_elipsoide(x, y, z):
    """Elipsoide alineado a los ejes por mínimos cuadrados: a x² + b y² + c z² + d x + e y + f z = 1.

    Devuelve (desplazamiento[3], escala[3], semiejes[3]). ValueError si no hay suficientes orientaciones.
    ponytail: soft-iron diagonal (sin términos cruzados), igual que el firmware; matriz 3x3 si no alcanza.
    """
    x, y, z = (np.asarray(v, float) for v in (x, y, z))
    if len(x) < 20:
        raise ValueError("muy pocos puntos (mínimo 20)")
    a_mat = np.column_stack([x**2, y**2, z**2, x, y, z])
    coef, *_ = np.linalg.lstsq(a_mat, np.ones_like(x), rcond=None)
    a, b, c, d, e, f = coef
    if min(a, b, c) <= 0:
        raise ValueError("no converge a un elipsoide: gira el rover en los TRES planos")
    centro = np.array([-d / (2 * a), -e / (2 * b), -f / (2 * c)])
    k = 1.0 + a * centro[0] ** 2 + b * centro[1] ** 2 + c * centro[2] ** 2
    semiejes = np.sqrt(k / np.array([a, b, c]))
    escala = semiejes.mean() / semiejes   # normaliza al semieje medio: no cambia la magnitud global
    return centro, escala, semiejes


def aplicar(xyz, desplazamiento, escala):
    """xyz: arreglo (n, 3) crudo -> calibrado."""
    return (np.asarray(xyz, float) - desplazamiento) * escala


def sigma_radio(xyz_cal):
    """Desviación estándar del radio normalizado: 0 = esfera perfecta."""
    r = np.linalg.norm(xyz_cal, axis=1)
    return float(np.std(r / r.mean()))


if __name__ == "__main__":
    # Esfera de radio 450 deformada (escala 1.1, 1.0, 0.9) y corrida (-160, 220, 140) => debe recuperarlo.
    rng = np.random.default_rng(1)
    u = rng.normal(size=(800, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    real_off, real_def = np.array([-160, 220, 140]), np.array([1.1, 1.0, 0.9])
    crudo = u * 450 * real_def + real_off + rng.normal(0, 2, size=u.shape)
    off, esc, _ = ajustar_elipsoide(*crudo.T)
    assert np.allclose(off, real_off, atol=3), off
    assert np.allclose(esc * real_def, (esc * real_def).mean(), rtol=0.01), esc
    assert sigma_radio(aplicar(crudo, off, esc)) < 0.01
    try:
        ajustar_elipsoide([1, 2], [1, 2], [1, 2])
        raise AssertionError("debió fallar con pocos puntos")
    except ValueError:
        pass
    print("magcal: ok")
