"""Can the cart swing the double pendulum up? A feasibility study.

Searches for ANY cart motion that takes both links from hanging still to
upright and still, inside the cart's limits: speed command |u| <= vmax, the
measured first-order motor lag tau, and the travel. Direct collocation
(Hermite-Simpson) solved with SLSQP, stepping vmax down from an easy value so
each solve starts from the last one. A solution found is then checked in a
fine nonlinear simulation, tracked by a time-varying LQR and handed to the
balance LQR at the top, with encoder quantisation.

    python tools/sim/dp_swingup.py

Pendulum: tools/sim/dp.py bodies() - ZIPY links, elbow, cartridge, 608s,
8 x 50 shaft, link-2 hub, as designed. Angles from upright; hanging = pi.
"""
import math, sys, time, warnings
import numpy as np
import scipy.linalg as la
from scipy.optimize import minimize
sys.path.insert(0, __file__.replace('\\', '/').rsplit('/', 1)[0])
sys.path.insert(0, __file__.replace('\\', '/').rsplit('/', 2)[0])
import dp

warnings.filterwarnings('ignore')
G = 9.81
P = dp.bodies()
M1C1, M2, C2, I1, I2, L1 = P['m1'] * P['c1'], P['m2'], P['c2'], P['I1'], P['I2'], P['l1']
FR1, FR2 = 0.0007, 0.00015            # base and elbow friction, N.m


def f(S, U, tau, prm=None):
    """State derivative, vectorised. S (...,6) = x v th1 th2 w1 w2, U (...) speed command."""
    m1c1, m2, c2, i1, i2 = prm if prm else (M1C1, M2, C2, I1, I2)
    v, t1, t2, w1, w2 = S[..., 1], S[..., 2], S[..., 3], S[..., 4], S[..., 5]
    a = (U - v) / tau
    A1 = m1c1 + m2 * L1
    d = t1 - t2
    m11, m12, m22 = i1 + m2 * L1 * L1, m2 * L1 * c2 * np.cos(d), i2
    rel = w2 - w1
    r1 = G * A1 * np.sin(t1) - A1 * np.cos(t1) * a - m2 * L1 * c2 * np.sin(d) * w2 ** 2 \
        - FR1 * np.tanh(w1 / 0.05) + FR2 * np.tanh(rel / 0.05)
    r2 = G * m2 * c2 * np.sin(t2) - m2 * c2 * np.cos(t2) * a + m2 * L1 * c2 * np.sin(d) * w1 ** 2 \
        - FR2 * np.tanh(rel / 0.05)
    det = m11 * m22 - m12 * m12
    return np.stack([v, a, w1, w2, (m22 * r1 - m12 * r2) / det, (m11 * r2 - m12 * r1) / det], -1)


class Problem:
    def __init__(self, T, N, vmax, tau, xlim, target):
        self.T, self.N, self.vmax, self.tau, self.xlim = T, N, vmax, tau, xlim
        self.h = T / N
        self.s0 = np.array([0, 0, math.pi, math.pi, 0, 0.0])
        self.sf = np.array([0, 0, target[0], target[1], 0, 0.0])

    def split(self, z):
        n = self.N + 1
        return z[:6 * n].reshape(n, 6), z[6 * n:]

    def defects(self, z):
        S, U = self.split(z)
        h = self.h
        F = f(S, U, self.tau)
        Sm = 0.5 * (S[:-1] + S[1:]) + h / 8 * (F[:-1] - F[1:])
        Um = 0.5 * (U[:-1] + U[1:])
        Fm = f(Sm, Um, self.tau)
        D = S[1:] - S[:-1] - h / 6 * (F[:-1] + 4 * Fm + F[1:])
        ends = np.r_[S[0] - self.s0, S[-1, 1:] - self.sf[1:]]      # final x free inside the bounds
        return np.r_[D.ravel(), ends]

    def jac(self, z):
        """Finite differences, perturbing every third node together (defects are banded)."""
        n = self.N + 1
        base = self.defects(z)
        J = np.zeros((base.size, z.size))
        eps = 1e-6
        for var in range(7):
            for c in range(3):
                nodes = np.arange(c, n, 3)
                idx = nodes * 6 + var if var < 6 else 6 * n + nodes
                zp = z.copy(); zp[idx] += eps
                dd = (self.defects(zp) - base) / eps
                for k, j in zip(nodes, idx):
                    rows = list(range(max(k - 1, 0) * 6, min(k + 1, self.N) * 6))
                    if k == 0: rows += list(range(self.N * 6, self.N * 6 + 6))
                    if k == self.N: rows += list(range(self.N * 6 + 6, self.N * 6 + 11))
                    J[rows, j] = dd[rows]
        return J

    def cost(self, z):
        S, U = self.split(z)
        return 1e-2 * self.h * np.sum(U ** 2)

    def cost_grad(self, z):
        g = np.zeros_like(z); S, U = self.split(z)
        g[6 * (self.N + 1):] = 2e-2 * self.h * U
        return g

    def bounds(self):
        n = self.N + 1
        b = []
        for _ in range(n):
            b += [(-self.xlim, self.xlim), (-self.vmax, self.vmax), (None, None), (None, None), (-40, 40), (-40, 40)]
        b += [(-self.vmax, self.vmax)] * n
        return b

    def solve(self, z0, iters=400):
        z0 = np.clip(z0, [lo if lo is not None else -1e9 for lo, hi in self.bounds()],
                     [hi if hi is not None else 1e9 for lo, hi in self.bounds()])
        r = minimize(self.cost, z0, jac=self.cost_grad, method='SLSQP', bounds=self.bounds(),
                     constraints=[{'type': 'eq', 'fun': self.defects, 'jac': self.jac}],
                     options={'maxiter': iters, 'ftol': 1e-9})
        viol = np.max(np.abs(self.defects(r.x)))
        return r.x, viol


def guess(pb, seed=0):
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 1, pb.N + 1)
    s = t * t * (3 - 2 * t)
    S = np.zeros((pb.N + 1, 6))
    S[:, 2] = math.pi + (pb.sf[2] - math.pi) * s
    S[:, 3] = math.pi + (pb.sf[3] - math.pi) * s
    S[:, 4] = np.gradient(S[:, 2], pb.h); S[:, 5] = np.gradient(S[:, 3], pb.h)
    w = 2 * math.pi / 1.0
    S[:, 0] = 0.6 * pb.xlim * np.sin(w * t * pb.T) * t * (1 - t) * 4 + 0.02 * rng.standard_normal(pb.N + 1)
    S[:, 1] = np.gradient(S[:, 0], pb.h)
    U = np.clip(S[:, 1] + pb.tau * np.gradient(S[:, 1], pb.h), -pb.vmax, pb.vmax)
    return np.r_[S.ravel(), U]


def resample(z, pb_from, pb_to):
    S, U = pb_from.split(z)
    t0 = np.linspace(0, pb_from.T, pb_from.N + 1); t1 = np.linspace(0, pb_to.T, pb_to.N + 1) * pb_from.T / pb_to.T
    S2 = np.stack([np.interp(t1, t0, S[:, i]) for i in range(6)], 1)
    S2[:, 1] *= pb_from.T / pb_to.T; S2[:, 4:] *= pb_from.T / pb_to.T
    return np.r_[S2.ravel(), np.interp(t1, t0, U)]


# ---------------- verification: fine simulation, TVLQR tracking, LQR catch ----------------
def lin(s, u, tau, prm=None, eps=1e-6):
    A = np.zeros((6, 6)); B = np.zeros(6)
    f0 = f(s, np.float64(u), tau, prm)
    for i in range(6):
        sp = s.copy(); sp[i] += eps; A[:, i] = (f(sp, np.float64(u), tau, prm) - f0) / eps
    B[:] = (f(s, np.float64(u + eps), tau, prm) - f0) / eps
    return A, B


def verify(pb, z, dt=0.002, tau_true=None, prm_true=None, s0_err=None, cpr1=2400, cpr2=4096, hold=3.0):
    """Track the planned path with a time-varying LQR (redesigned every 2 ms), then
    hand over to the balance LQR. Plant may differ from the plan (tau, masses)."""
    S, U = pb.split(z)
    tn = np.linspace(0, pb.T, pb.N + 1)
    ts = np.arange(0, pb.T + 1e-9, dt)
    Sr = hermite(pb, S, U, ts)          # straight lines between nodes are NOT a usable reference
    Ur = np.interp(ts, tn, U)
    Q = np.diag([10, 1, 200, 200, 5, 5]); R = np.array([[1.0]])
    # backward discrete Riccati along the path
    n = len(ts)
    Ks = np.zeros((n, 6))
    Ad, Bd = _disc(*lin(Sr[-1], Ur[-1], pb.tau), dt)
    Pm = la.solve_discrete_are(Ad, Bd.reshape(6, 1), Q * dt, R * dt)
    for k in range(n - 1, -1, -1):
        A, B = lin(Sr[k], Ur[k], pb.tau)
        Ad, Bd = _disc(A, B, dt)
        Bd = Bd.reshape(6, 1)
        K = np.linalg.solve(R * dt + Bd.T @ Pm @ Bd, Bd.T @ Pm @ Ad)
        Pm = Q * dt + Ad.T @ Pm @ (Ad - Bd @ K)
        Ks[k] = K.ravel()
    Kbal, _ = dp.lqr(P, pb.tau)
    tau = tau_true or pb.tau
    s = pb.s0.copy() + (s0_err if s0_err is not None else 0)
    q1, q2 = 2 * math.pi / cpr1, 2 * math.pi / cpr2
    sub = 10; h = dt / sub
    peak_x = 0.0
    for k in range(n + int(hold / dt)):
        meas = s.copy()
        meas[2] = round(s[2] / q1) * q1
        meas[3] = meas[2] + round((s[3] - s[2]) / q2) * q2
        if k < n:
            e = meas - Sr[k]; e[2:4] = (e[2:4] + math.pi) % (2 * math.pi) - math.pi
            u = Ur[k] - Ks[k] @ e
        else:
            e = meas - pb.sf; e[0] = meas[0]
            e[2:4] = (e[2:4] + math.pi) % (2 * math.pi) - math.pi
            u = -Kbal @ e
        u = float(np.clip(u, -pb.vmax, pb.vmax))
        for _ in range(sub):                                   # RK4 on the true plant
            k1 = f(s, u, tau, prm_true); k2 = f(s + h / 2 * k1, u, tau, prm_true)
            k3 = f(s + h / 2 * k2, u, tau, prm_true); k4 = f(s + h * k3, u, tau, prm_true)
            s = s + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        peak_x = max(peak_x, abs(s[0]))
        if abs(s[0]) > 0.268:
            return False, 'hit the end of the rail at %.2f s' % (k * dt)
    err = (s[2:4] - pb.sf[2:4] + math.pi) % (2 * math.pi) - math.pi
    ok = np.max(np.abs(err)) < math.radians(5)
    return ok, ('balanced %.0f s after arriving (links %.1f / %.1f deg, peak travel %.0f mm)' if ok else
                'did not hold at the top (links %.0f / %.0f deg, peak travel %.0f mm)') % (
        *((hold,) if ok else ()), math.degrees(err[0]), math.degrees(err[1]), peak_x * 1e3)


def _disc(A, B, dt):
    """Zero-order-hold discretisation."""
    M = np.zeros((7, 7)); M[:6, :6] = A * dt; M[:6, 6] = B * dt
    E = la.expm(M)
    return E[:6, :6], E[:6, 6]


def hermite(pb, S, U, ts):
    """The cubic Hermite-Simpson collocation itself assumes between nodes."""
    F = f(S, U, pb.tau)
    idx = np.minimum((ts / pb.h).astype(int), pb.N - 1)
    s = (ts - idx * pb.h) / pb.h
    h00, h10 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s
    h01, h11 = -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    return (h00[:, None] * S[idx] + h10[:, None] * pb.h * F[idx]
            + h01[:, None] * S[idx + 1] + h11[:, None] * pb.h * F[idx + 1])


def checks(pb, z):
    """Nominal, then the ways the real rig will differ from the plan."""
    out = []
    for label, kw in (('as planned', {}),
                      ('motor 25% slower (tau 75 ms)', dict(tau_true=pb.tau * 1.25)),
                      ('starts 2 deg off', dict(s0_err=np.array([0, 0, 0.035, -0.035, 0, 0]))),
                      ('link 2 10% heavier', dict(prm_true=(M1C1, M2 * 1.1, C2, I1, I2 * 1.1)))):
        ok, msg = verify(pb, z, **kw)
        out.append((label, ok, msg))
    return out


def log(*a):
    print(*a, flush=True)


if __name__ == '__main__':
    TAU, XLIM = 0.06, 0.25
    levels = [1.5, 1.2, 1.0, 0.85, 0.7, 0.6, 0.5, 0.45, 0.4, 0.35, 0.317]
    log('double pendulum swing-up search: m1 %.0f g, m2 %.0f g, travel +-%.0f mm, motor lag %.0f ms'
        % (P['m1'] * 1e3, P['m2'] * 1e3, XLIM * 1e3, TAU * 1e3))
    best, fails = None, 0
    for vmax in levels:
        found = None
        tries = []
        for T in (4.0, 6.0, 8.0):
            pb = Problem(T, int(round(15 * T)), vmax, TAU, XLIM, (0.0, 0.0))
            if best is not None:
                tries.append((pb, resample(best[1], best[0], pb), 'warm'))
            tries += [(pb, guess(pb, seed), 'seed %d' % seed) for seed in (0, 1)]
        for pb, z0, how in tries:
            t0 = time.time()
            Sx0, U0 = pb.split(z0)
            z0 = np.r_[Sx0.ravel(), np.clip(U0, -vmax, vmax)]
            z, viol = pb.solve(z0, iters=500)
            log('  vmax %.3f  T %.0f s  %-6s  violation %.1e  (%.0f s)' % (vmax, pb.T, how, viol, time.time() - t0))
            if viol < 1e-4:
                found = (pb, z); break
        if found:
            best, fails = found, 0
            pb, z = found
            Sx, U = pb.split(z)
            np.save(__file__.replace('dp_swingup.py', 'swingup_v%03d.npy' % round(vmax * 100)), z)
            log('FOUND at %.3f m/s: %.0f s of motion, peak travel %.0f mm, peak command %.2f m/s'
                % (vmax, pb.T, np.abs(Sx[:, 0]).max() * 1e3, np.abs(U).max()))
            for label, ok, msg in checks(pb, z):
                log('     check %-30s %s  %s' % (label, 'OK  ' if ok else 'FAIL', msg))
        else:
            fails += 1
            log('none found at %.3f m/s' % vmax)
            if fails >= 2 and vmax > 0.317:
                log('two levels in a row with nothing found - jumping to 0.317 m/s for a last direct try')
                levels_left = [0.317]
                for pb_T in (6.0, 8.0):
                    pb = Problem(pb_T, int(round(15 * pb_T)), 0.317, TAU, XLIM, (0.0, 0.0))
                    for z0, how in ([(resample(best[1], best[0], pb), 'warm')] if best else []) + \
                                   [(guess(pb, s), 'seed %d' % s) for s in (2, 3)]:
                        Sx0, U0 = pb.split(z0)
                        z, viol = pb.solve(np.r_[Sx0.ravel(), np.clip(U0, -0.317, 0.317)], iters=500)
                        log('  vmax 0.317  T %.0f s  %-6s  violation %.1e' % (pb_T, how, viol))
                        if viol < 1e-4:
                            log('FOUND at 0.317 m/s on the last try')
                            for label, ok, msg in checks(pb, z):
                                log('     check %-30s %s  %s' % (label, 'OK  ' if ok else 'FAIL', msg))
                            break
                break
    log('done')

