"""Double pendulum on the cart, built from the real parts.

Masses and inertias come from ZIPY's STLs (tools/mass_props.py) plus the bought
parts, placed as in the model artifact (ZIPY elbow + bearing cartridge). The
controller is a lag-aware 6-state LQR: the cart is a first-order velocity lag
(tau) behind a speed command, as measured on the rig.

    python tools/sim/dp.py
"""
import math, sys, warnings
import numpy as np
import scipy.linalg as la
sys.path.insert(0, __file__.replace('\\', '/').rsplit('/', 2)[0])
from stl_holes import load_stl
from mass_props import mass_props

warnings.filterwarnings('ignore')
G = 9.81
PLA = 1.24e-3            # g/mm^3
STEEL = 7.9e-3
L1 = 250.0               # base pivot to elbow axis, mm
STL = 'STL Files/'


def printed(name, fill, to_plane):
    """(mass g, centroid (x, y) mm, Izz about centroid g.mm^2) in the swing plane.
    to_plane maps STL (x, y, z) -> plane (x, y, z); z is the joint axis."""
    tri = to_plane(load_stl(STL + name))
    V, cen, I = mass_props(tri)
    rho = PLA * fill
    return V * rho, cen[:2], I[2, 2] * rho


def point(m, x, y, k2=0.0):
    return m, np.array([x, y], float), m * k2


def ring_z(m, ro, ri, x, y):
    """A tube on an axis parallel to the joint axis: Izz = m(ro^2 + ri^2)/2."""
    return m, np.array([x, y], float), m * (ro * ro + ri * ri) / 2


def combine(parts, about):
    m = sum(p[0] for p in parts)
    c = sum(p[0] * p[1] for p in parts) / m
    I = sum(p[2] + p[0] * np.sum((p[1] - about) ** 2) for p in parts)
    return m, c, I


def bodies(fill=0.85, shaft='steel', elbow_extra=0.0, link2_tip=0.0, cable=True):
    """Body 1 = everything that turns with link 1 (about the base pivot).
    Body 2 = everything that turns with link 2 (about the elbow axis)."""
    ident = lambda t: t
    link_y = lambda t: t + np.array([0, -16.0, 0])                     # holes 36/54 -> 20/38
    elbow_map = lambda t: t @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float).T + np.array([-22, L1, -66.0])
    hub = printed('zipy_dp_hub.stl', fill, ident)
    b1 = [hub,                                                           # base hub (as now)
          printed('zipy_dp_link1.stl', fill, link_y),                    # link 1 (as now)
          point(2.7, 0, 29),                                             # 2 x M3 hub bolts + nuts
          point(1.4, 0, -10),                                            # hub set screw + nut
          printed('zipy_dp_elbow.stl', fill, elbow_map),                 # ZIPY elbow
          point(2 * 0.95 + 2 * 0.25, 0, 201),                            # 2 x M3x12 + nuts, elbow to link 1
          ring_z(12.0 * 1.24 * fill, 19, 8.5, 0, L1),                    # cartridge + spacers, ~12 cm3 PLA
          ring_z(2 * 8.8, 11, 6.5, 0, L1),                               # 2 x 608: outer rings + balls + cage
          point(6 * 0.55, 0, L1, 15.5 ** 2),                             # 6 x M3x8 cartridge screws
          point(4.5, 0, L1 + 2, 10 ** 2),                                # AS5048A board, bracket, header
          point(elbow_extra, 0, L1)]
    if cable:
        b1.append(point(3.0, 0, 125, 250 ** 2 / 12))                     # ~0.25 m of thin cable on link 1
    hub2 = printed('zipy_dp_hub.stl', fill, ident)
    bore = PLA * fill * math.pi * (4.0 ** 2 - 3.15 ** 2) * 15            # 6.3 -> 8 mm bore
    hub2 = (hub2[0] - bore, hub2[1], hub2[2])
    sm = {'steel': STEEL, 'alu': 2.7e-3, 'carbon': 1.6e-3}[shaft] * math.pi * 16 * 50
    b2 = [hub2,
          printed('zipy_dp_link2.stl', fill, link_y),
          point(2.7, 0, 29),                                             # 2 x M3x16 + nuts
          point(1.4, 0, -10),                                            # set screw + nut
          ring_z(sm, 4, 0, 0, 0),                                        # 8 x 50 shaft
          ring_z(2 * 3.2, 6.05, 4, 0, 0),                                # 2 x 608 inner rings
          point(0.55 + 1.6, 0, 0, 5 ** 2),                               # magnet + cap + sleeve + spacer
          point(link2_tip, 0, 225)]
    m1, c1, I1 = combine(b1, np.array([0.0, 0.0]))
    m2, c2v, I2e = combine(b2, np.array([0.0, 0.0]))
    return dict(m1=m1 / 1e3, c1=c1[1] / 1e3, I1=I1 / 1e9, m2=m2 / 1e3, c2=c2v[1] / 1e3, I2=I2e / 1e9,
                l1=L1 / 1e3, b1=b1, b2=b2)


def single_now(fill=0.85):
    """The pendulum on the rig today: base hub + link 1 + its screws."""
    ident = lambda t: t
    link_y = lambda t: t + np.array([0, -16.0, 0])
    parts = [printed('zipy_dp_hub.stl', fill, ident), printed('zipy_dp_link1.stl', fill, link_y),
             point(2.7, 0, 29), point(1.4, 0, -10)]
    m, c, I = combine(parts, np.array([0.0, 0.0]))
    return m / 1e3, c[1] / 1e3, I / 1e9


def linear(P):
    """M q'' = Gm q - Bt a   (q = absolute link angles from upright, a = cart accel)."""
    m1, c1, I1, m2, c2, I2, l1 = (P[k] for k in ('m1', 'c1', 'I1', 'm2', 'c2', 'I2', 'l1'))
    M = np.array([[I1 + m2 * l1 * l1, m2 * l1 * c2], [m2 * l1 * c2, I2]])
    Gm = G * np.diag([m1 * c1 + m2 * l1, m2 * c2])
    Bt = np.array([m1 * c1 + m2 * l1, m2 * c2])
    return M, Gm, Bt


def lqr(P, tau, Q=(10, 1, 300, 300, 10, 10)):
    M, Gm, Bt = linear(P)
    Mi = np.linalg.inv(M); MG, MB = Mi @ Gm, Mi @ Bt
    A = np.zeros((6, 6)); A[0, 1] = 1; A[1, 1] = -1 / tau; A[2, 4] = 1; A[3, 5] = 1
    A[4:6, 2:4] = MG; A[4:6, 1] = MB / tau
    B = np.zeros((6, 1)); B[1, 0] = 1 / tau; B[4:6, 0] = -MB / tau
    K = (B.T @ la.solve_continuous_are(A, B, np.diag(Q), np.array([[1.0]]))).ravel()
    rate = max(np.linalg.eigvals(np.block([[np.zeros((2, 2)), np.eye(2)], [MG, np.zeros((2, 2))]])).real)
    return K, rate


def run(P, tau=0.06, vmax=0.317, th0=(2.0, 2.0), push=0.0, t_push=3.0, T=8.0, dt=0.0005,
        fr_base=0.0007, fr_elbow=0.00015, cpr1=2400, cpr2=4096, upd2=0.001, travel=0.268):
    """Full nonlinear plant. Elbow sensor: AS5048A PWM (12 bit, ~1 kHz), reads link 2 RELATIVE to link 1."""
    K, _ = lqr(P, tau)
    m1, c1, I1, m2, c2, I2, l1 = (P[k] for k in ('m1', 'c1', 'I1', 'm2', 'c2', 'I2', 'l1'))
    x = v = 0.0; th = np.radians(th0).astype(float); w = np.zeros(2)
    q1, q2 = 2 * math.pi / cpr1, 2 * math.pi / cpr2
    phi_held = round((th[1] - th[0]) / q2) * q2; t_next2 = 0.0
    prev = None; wf = np.zeros(2); vf = 0.0; a_f = dt / (0.01 + dt)
    peak_v = peak_x = 0.0
    for k in range(int(T / dt)):
        t = k * dt
        if t >= t_next2:
            phi_held = round((th[1] - th[0]) / q2) * q2; t_next2 += upd2
        th1m = round(th[0] / q1) * q1
        meas = np.array([th1m, th1m + phi_held])
        if prev is None: prev = meas.copy()
        wf += a_f * ((meas - prev) / dt - wf); prev = meas; vf += a_f * (v - vf)
        u = float(np.clip(-K @ np.r_[x, vf, meas, wf], -vmax, vmax))
        a = (u - v) / tau; v += a * dt; x += v * dt
        s12, c12 = math.sin(th[0] - th[1]), math.cos(th[0] - th[1])
        M = np.array([[I1 + m2 * l1 * l1, m2 * l1 * c2 * c12], [m2 * l1 * c2 * c12, I2]])
        rel = w[1] - w[0]
        f = np.array([
            G * (m1 * c1 + m2 * l1) * math.sin(th[0]) - (m1 * c1 + m2 * l1) * math.cos(th[0]) * a
            - m2 * l1 * c2 * s12 * w[1] ** 2 - fr_base * math.tanh(w[0] / 0.01) + fr_elbow * math.tanh(rel / 0.01),
            G * m2 * c2 * math.sin(th[1]) - m2 * c2 * math.cos(th[1]) * a
            + m2 * l1 * c2 * s12 * w[0] ** 2 - fr_elbow * math.tanh(rel / 0.01)])
        w += np.linalg.solve(M, f) * dt; th += w * dt
        if push and abs(t - t_push) < dt / 2: w[1] += push
        peak_v = max(peak_v, abs(v)); peak_x = max(peak_x, abs(x))
        if max(abs(th)) > math.radians(30) or abs(x) > travel:
            return False, peak_v, peak_x
    return True, peak_v, peak_x


def max_tilt(P, both=True, **kw):
    """Largest starting tilt (deg) it recovers from: both links leaning together, or link 2 alone."""
    lo, hi = 0.0, 12.0
    for _ in range(9):
        mid = (lo + hi) / 2
        ok = run(P, th0=(mid, mid) if both else (0.0, mid), T=5.0, **kw)[0]
        lo, hi = (mid, hi) if ok else (lo, mid)
    return lo


if __name__ == '__main__':
    m, c, I = single_now()
    print('pendulum today (hub + link 1): %.0f g, CoM %.0f mm, equivalent simple length %.1f mm '
          '-> period %.3f s  (rig measured 0.765 s, L = 218 mm uniform-rod equivalent = 145.4 mm simple)'
          % (m * 1e3, c * 1e3, I / (m * c) * 1e3, 2 * math.pi * math.sqrt(I / (m * G * c))))
    print('  fastest fall rate today: %.1f /s' % math.sqrt(m * G * c / I))
    P = bodies()
    print('\nas designed (printed parts at 85% of solid PLA, steel 8x50 shaft):')
    print('  body 1 (link 1 + elbow + cartridge + sensor): %.0f g, CoM %.0f mm up, I about pivot %.0f g.cm2'
          % (P['m1'] * 1e3, P['c1'] * 1e3, P['I1'] * 1e7))
    print('  body 2 (link 2 + hub + shaft + magnet):        %.0f g, CoM %.0f mm past the elbow, I about elbow %.0f g.cm2'
          % (P['m2'] * 1e3, P['c2'] * 1e3, P['I2'] * 1e7))
