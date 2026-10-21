"""Balance-only study: LQR check, then catch-region and drift for each duty mapping."""
import math, os, sys, warnings
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
warnings.filterwarnings('ignore')
import numpy as np, scipy.linalg as la
from sim import Plant, Ctrl, energy, wrap, B, LAM, X_PHYS

# --- 1. LQR gains for an ACCELERATION input -----------------------------------
A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, LAM**2, 0]])
Bm = np.array([[0], [1], [0], [-B]])
Q = np.diag([12, 1, 140, 12]); R = np.array([[1.0]])
P = la.solve_continuous_are(A, Bm, Q, R)
K = (np.linalg.inv(R) @ Bm.T @ P).ravel()
print('LQR K (u = -K s):', np.round(K, 2), '-> firmware uses u = +[3.39 4.46 37.17 6.75] s')
print('closed-loop poles:', np.round(np.linalg.eigvals(A - Bm @ K.reshape(1, -1)), 2))


def balance_run(law, tau, th0, w0, x0=0.0, v0=0.0, T=8.0, fr_r=0.20, fr_l=0.10, **kw):
    pl = Plant(tau=tau, fr_r=fr_r, fr_l=fr_l, x0=x0)
    pl.thu = th0; pl.w = w0; pl.v = v0
    c = Ctrl('v45', BAL_LAW=law, **kw)
    c.mode = 'BALANCE'; c.signProven = True
    # pre-load filters with the true state
    c.thF = c.thPrev = th0; c.wF = w0; c.xF = c.xPrev = x0; c.vF = v0; c.vCmd = v0
    c.balStart = 0.0
    c.p['MAX_DROPS'] = 1
    dtp, dtc = 0.0005, 0.002
    t = 0; maxx = 0; xs = []; ths = []
    while t < T:
        cart, pole = pl.counts()
        d = c.tick(cart, pole, dtc)
        for _ in range(4):
            pl.step(d, dtp)
        t += dtc
        maxx = max(maxx, abs(pl.x)); xs.append(pl.x); ths.append(wrap(pl.thu))
        if c.mode != 'BALANCE' or c.halted or pl.crash:
            return False, t, maxx, None
    xs = np.array(xs[-1000:]); ths = np.array(ths[-1000:])
    return True, t, maxx, (np.std(ths) * 180 / math.pi, np.mean(xs) * 1000, np.std(xs) * 1000)


if __name__ == '__main__':
    print()
    for tau in (0.006, 0.03, 0.08):
        for law, extra in (('acc', {}), ('int', {}), ('int', {'TAU_FF': tau})):
            name = law + ('+ff' if extra else '')
            # hold from upright, small nudge, cart off centre by 60 mm
            ok, t, mx, st = balance_run(law, tau, 0.03, 0.0, x0=0.06, **extra)
            hold = 'held 8 s, wobble %.2f deg, cart mean %+.0f mm sd %.1f mm, max %.0f mm' % (st[0], st[1], st[2], mx * 1000) if ok else 'FELL at %.2f s (max cart %.0f mm)' % (t, mx * 1000)
            # catch region: th0 grid with w0 on the energy-zero path (coasting up)
            caught = []
            for deg in (5, 8, 11, 14, 17, 20, 25):
                th = deg * math.pi / 180
                w = -LAM * th * math.sqrt(max(0.0, (1 - math.cos(th)) / (th * th / 2)))  # E=0 approach
                ok2, *_ = balance_run(law, tau, th, w, T=4.0, **extra)
                caught.append(deg if ok2 else None)
            ok_deg = [d for d in caught if d is not None]
            print('tau %3.0f ms  %-7s %s | catch on E=0 path up to %s deg' % (tau * 1000, name, hold, max(ok_deg) if ok_deg else 'none'))
