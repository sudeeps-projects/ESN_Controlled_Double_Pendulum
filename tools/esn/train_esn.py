"""Echo state network balance controller for the ZIPY cart-pole.

A fixed random reservoir (N leaky-tanh neurons, updated every 2 ms on what
the Teensy measures) with two trained linear readouts:

  predict : the pole's TRUE angle and rate 20 ms ahead (the measurements lag
            behind the truth by the 20 ms filter, and the motor by ~60 ms)
  control : the cart acceleration command, trained by DAgger to imitate an
            expert LQR that sees the true state the measurements lag behind

Trained in tools/sim on randomised rigs around the measured one, then
exported as a C header for the Teensy sketch (swing_pendulum_esn_1.0).

    python train_esn.py            # train, evaluate, write esn_weights.h
"""
import math, os, sys, time, warnings
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
warnings.filterwarnings('ignore')
from sim import Plant, Ctrl, energy, wrap, lqr_gains, G   # noqa: E402

# ---------------------------------------------------------------- the rig
L_NOM = 0.218                     # measured from the 2026-10-15 decay log
# FRICTION_FF here is the speed->duty friction step: the firmware's fixed BAL_FF = 0.08
CTRL_KW = dict(V_MAX=0.317, FRICTION_FF=0.08, X_MAX=0.268, Lc=L_NOM)
DT = 0.002

# ---------------------------------------------------------------- the ESN
N, U = 64, 6                      # reservoir size, inputs
LEAK, RHO, IN_SCALE, DENSITY = 0.25, 0.9, 0.6, 0.2
DELAY = 10                        # predict 10 ticks = 20 ms ahead
PUSH, PUSH_GAP = 1.2, 1.0         # training shoves (rad/s), spacing factor
IN_NORM = np.array([0.4, 0.4, 4.0, 0.2, 0.3, 0.3])   # sin th, 1-cos th, w, x, v, vCmd


def make_reservoir(seed=7):
    rng = np.random.default_rng(seed)
    W = rng.uniform(-1, 1, (N, N)) * (rng.random((N, N)) < DENSITY)
    W *= RHO / max(abs(np.linalg.eigvals(W)))
    Win = rng.uniform(-1, 1, (N, U)) * IN_SCALE
    b = rng.uniform(-0.2, 0.2, N)
    return W.astype(np.float32), Win.astype(np.float32), b.astype(np.float32)


W_RES, W_IN, B_RES = make_reservoir()


def esn_inputs(c):
    """Exactly what the Teensy has: filtered angle/rate/position/speed, speed command."""
    return np.array([math.sin(c.thF), 1.0 - math.cos(c.thF), c.wF, c.xF, c.vF, c.vCmd],
                    dtype=np.float32) / IN_NORM.astype(np.float32)


class ESN:
    def __init__(s):
        s.r = np.zeros(N, dtype=np.float32)
        s.u = np.zeros(U, dtype=np.float32)

    def step(s, u):
        s.u = u
        s.r = (1 - LEAK) * s.r + LEAK * np.tanh(W_IN @ u + W_RES @ s.r + B_RES)

    def features(s):
        return np.concatenate([s.r, s.u, [1.0]]).astype(np.float32)


# ---------------------------------------------------------------- rigs
def random_rig(rng, nominal=False):
    if nominal:
        return dict(tau=0.06, vfull=0.317 / (0.95 - 0.20), fr_r=0.20, fr_l=0.12, cp=0.06, Lp=L_NOM)
    fr_r = rng.uniform(0.15, 0.28)
    return dict(tau=rng.uniform(0.03, 0.09),
                vfull=rng.uniform(0.28, 0.35) / (0.95 - fr_r),
                fr_r=fr_r, fr_l=rng.uniform(0.08, 0.15),
                cp=rng.uniform(0.02, 0.15),
                Lp=L_NOM * rng.uniform(0.92, 1.08))


def approach_state(rng, Lp):
    """Pole coming up toward the top with about the right energy, as after swing-up."""
    th = math.radians(rng.uniform(22, 38)) * (1 if rng.random() < 0.5 else -1)
    E = rng.uniform(-0.03, 0.03) * G * Lp
    w2 = max(0.0, (E - (G * Lp / 2) * (math.cos(th) - 1)) * 6 / (Lp * Lp))
    return th, -math.copysign(math.sqrt(w2), th)


# ---------------------------------------------------------------- episodes
def episode(rng, policy, beta=1.0, Wc=None, Wp=None, T_bal=6.0, pushes=True, rig=None,
            record=True, x0=None):
    """One run: approach -> (1.5 catch logic) -> balance by `policy`.
    policy: 'lqr' (measured state, the 1.5 law), 'esn' (control readout),
            'esn_pred' (LQR on the ESN-predicted state), 'expert' (true state)."""
    rig = rig or random_rig(rng)
    pl = Plant(**rig, x0=x0 if x0 is not None else rng.uniform(-0.08, 0.08))
    th0, w0 = approach_state(rng, rig['Lp'])
    pl.thu, pl.w = th0, w0
    K_true = np.array(lqr_gains(rig['Lp']))
    K_nom = np.array(lqr_gains(L_NOM))
    esn = ESN()
    rec_phi, rec_a, rec_true = [], [], []
    truth = []                              # true (th, w) per tick, for the predictor
    phis_all = []
    state = dict(bal_t=0.0, drops=0, was_bal=False)

    def tick_fn(c):
        esn.step(esn_inputs(c))
        if record:
            phis_all.append(esn.features())
            truth.append((wrap(pl.thu), pl.w, c.thF, c.wF))

    def acc_fn(c):
        phi = esn.features()
        s_true = np.array([pl.x, pl.v, wrap(pl.thu), pl.w])
        a_exp = float(K_true @ s_true)
        if policy == 'expert':
            a = a_exp
        elif policy == 'lqr':
            a = float(K_nom @ np.array([c.xF, c.vF, c.thF, c.wF]))
        elif policy == 'esn_pred':
            dth, dw = Wp @ phi
            a = float(K_nom @ np.array([c.xF, c.vF, c.thF + dth, c.wF + dw]))
        else:
            a_esn = float(Wc @ phi)
            a = a_exp if rng.random() < beta else a_esn
        if record:
            rec_phi.append(phi); rec_a.append(a_exp)
        return a

    c = Ctrl('v45', **CTRL_KW, TICK_FN=tick_fn, ACC_FN=acc_fn)
    c.thF = c.thPrev = th0; c.wF = w0; c.xF = c.xPrev = pl.x
    c.kickUntil = 0.0; c.signProven = True; c.ecOn = True; c.lastTurn = 0.0
    t, next_push, first_catch = 0.0, None, None
    T_max = 4.0 + T_bal
    while t < T_max:
        cart, pole = pl.counts()
        d = c.tick(cart, pole, DT)
        for _ in range(4):
            pl.step(d, DT / 4)
        t += DT
        bal = c.mode == 'BALANCE'
        if bal:
            if first_catch is None:
                first_catch = t
                next_push = t + rng.uniform(1.0, 2.5)
            state['bal_t'] += DT
            if pushes and t >= next_push:
                pl.w += rng.uniform(-PUSH, PUSH)
                next_push = t + rng.uniform(1.0, 2.5) * PUSH_GAP
        if state['was_bal'] and not bal:
            state['drops'] += 1
        state['was_bal'] = bal
        if c.halted or pl.crash:
            break
        if first_catch is not None and t > first_catch + T_bal:
            break
    out = dict(caught=first_catch is not None, drops=state['drops'], bal_t=state['bal_t'],
               halted=c.halted, crash=pl.crash,
               held=first_catch is not None and state['drops'] == 0 and not c.halted and not pl.crash
                    and t >= first_catch + T_bal - 1e-6)
    if record:
        out['phi'], out['a'] = rec_phi, rec_a
        # predictor pairs: features at t, true state at t + DELAY
        P = np.array(phis_all); Tr = np.array(truth)
        if len(P) > DELAY:
            fut, now = Tr[DELAY:, :2], Tr[:-DELAY, 2:]
            near = np.abs(now[:, 0]) < 0.5
            out['pphi'], out['ptrue'] = P[:-DELAY][near], (fut - now)[near]
    return out


def ridge(X, Y, lam):
    X = np.asarray(X, dtype=np.float64); Y = np.asarray(Y, dtype=np.float64)
    A = X.T @ X
    A += lam * np.mean(np.diag(A)) * np.eye(A.shape[0])
    return np.linalg.solve(A, X.T @ Y).T


def evaluate(policy, Wc=None, Wp=None, n=40, seed=1000, T_bal=15.0, nominal=False, pushes=True):
    rng = np.random.default_rng(seed)
    held = drops = 0
    for i in range(n):
        r = episode(rng, policy, beta=0.0, Wc=Wc, Wp=Wp, T_bal=T_bal, record=False,
                    rig=random_rig(rng, nominal=nominal), pushes=pushes)
        held += r['held']; drops += r['drops']
    return held, n, drops


def train(iters=5, eps_per_iter=60, lam=1e-4, seed=0):
    rng = np.random.default_rng(seed)
    X, Y, PX, PY = [], [], [], []
    Wc = Wp = None
    for k in range(iters):
        beta = 1.0 if k == 0 else 0.5 ** k
        t0 = time.time(); n_ok = 0
        for e in range(eps_per_iter):
            r = episode(rng, 'esn' if Wc is not None else 'expert', beta=beta, Wc=Wc, T_bal=6.0)
            X += r['phi']; Y += r['a']; n_ok += r['held']
            if 'pphi' in r:
                PX.append(r['pphi']); PY.append(r['ptrue'])
        Wc = ridge(X, np.array(Y)[:, None], lam).ravel()
        Wp = ridge(np.vstack(PX), np.vstack(PY), lam)
        print('  DAgger iter %d (beta %.2f): %d samples, %d/%d episodes held, %.0f s' % (
            k, beta, len(X), n_ok, eps_per_iter, time.time() - t0), flush=True)
    # predictor quality
    PXa, PYa = np.vstack(PX), np.vstack(PY)
    pred = PXa @ Wp.T
    err = np.sqrt(np.mean((pred - PYa) ** 2, axis=0))
    base = np.sqrt(np.mean(PYa ** 2, axis=0))      # error of just using the measurement
    print('  predictor near the top, 20 ms ahead: RMS error angle %.2f deg (measurement alone %.2f), rate %.1f deg/s (alone %.1f)' % (
        math.degrees(err[0]), math.degrees(base[0]), math.degrees(err[1]), math.degrees(base[1])))
    return Wc, Wp


def export(Wc, Wp, path):
    def arr(name, a, fmt='%.7ef'):
        a = np.asarray(a, dtype=np.float32).ravel()
        body = ',\n  '.join(', '.join(fmt % v for v in a[i:i + 6]) for i in range(0, len(a), 6))
        return 'const float %s[%d] = {\n  %s\n};\n' % (name, len(a), body)
    with open(path, 'w') as f:
        f.write('// Generated by tools/esn/train_esn.py - do not edit by hand.\n')
        f.write('// Echo state network: %d leaky-tanh neurons, %d inputs, leak %.2f.\n' % (N, U, LEAK))
        f.write('// Trained (DAgger) on randomised rigs around L = %.3f m.\n' % L_NOM)
        f.write('#pragma once\n')
        f.write('const int ESN_N = %d, ESN_U = %d, ESN_F = %d;\n' % (N, U, N + U + 1))
        f.write('const float ESN_LEAK = %.6ff;\n' % LEAK)
        f.write(arr('ESN_IN_NORM', IN_NORM))
        f.write(arr('ESN_W', W_RES))          # row-major N x N
        f.write(arr('ESN_WIN', W_IN))         # row-major N x U
        f.write(arr('ESN_B', B_RES))
        f.write(arr('ESN_WC', Wc))            # control readout, F
        f.write(arr('ESN_WP', Wp))            # predictor readout, 2 x F: CORRECTION to measured angle, rate (20 ms ahead)
    print('  wrote', path)


if __name__ == '__main__':
    print('training ESN (N=%d) by DAgger on randomised rigs...' % N)
    Wc, Wp = train()
    for label, push, gap, pushes in (('no shoves', 0.0, 1.0, False), ('moderate shoves (0.8 rad/s every 2-5 s)', 0.8, 2.0, True),
                                     ('hard shoves (1.2 rad/s every 1-2.5 s)', 1.2, 1.0, True)):
        PUSH, PUSH_GAP = push, gap
        print('evaluating, 40 unseen randomised rigs, 15 s balance, %s' % label)
        for name, pol in (('expert (true state, not possible on the rig)', 'expert'),
                          ('LQR on measured state (firmware 1.5)', 'lqr'),
                          ('ESN control readout', 'esn'),
                          ('LQR on ESN-predicted state', 'esn_pred')):
            h, n, d = evaluate(pol, Wc, Wp, pushes=pushes)
            print('  %-46s held %2d/%d, drops %d' % (name, h, n, d), flush=True)
    export(Wc, Wp, os.path.join(HERE, 'esn_weights.h'))
    np.savez(os.path.join(HERE, 'esn_readouts.npz'), Wc=Wc, Wp=Wp)
