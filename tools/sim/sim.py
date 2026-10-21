"""ZIPY cart-pole simulator: plant + a port of the firmware's 500 Hz loop.

Defaults match firmware v4.6 (swing_pendulum_claude_1.4): pass Lc= the measured pole length,
Plant(Lp=) the real one. lqr_gains(L) regenerates the firmware's K_TABLE. variant='v44' replays 1.2.
Run:  python full.py   (v44 vs v45 over 18 rigs)   python robust.py   (unseen cases)

Plant: DC gear motor on a belt (first-order speed response, time constant tau),
Coulomb friction as a duty offset (different left/right), uniform rod pole.
Controller: v4.4 logic (variant 'v44') or the candidate v4.5 logic ('v45').
"""
import math, sys
import numpy as np

L, G = 0.402, 9.81
B = 3.0 / (2 * L)                 # pole response to cart accel
LAM = math.sqrt(3 * G / (2 * L))  # unstable pole, 6.05 rad/s
M_PER_COUNT = 0.040 / 2400
RAD_PER_COUNT = 2 * math.pi / 2400
X_PHYS = 0.230                    # mechanical end


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def energy(th, w, Lx=None):
    Lx = Lx or L
    return (Lx * Lx / 6) * w * w + (G * Lx / 2) * (math.cos(th) - 1)


def lqr_gains(Lx, q=(12, 1, 140, 12), r=1.0):
    import numpy as np, scipy.linalg as la
    lam2 = 1.5 * G / Lx; b = 1.5 / Lx
    A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, lam2, 0]])
    Bm = np.array([[0], [1], [0], [-b]])
    P = la.solve_continuous_are(A, Bm, np.diag(q), np.array([[r]]))
    return tuple(-(Bm.T @ P / r).ravel())


class Plant:
    def __init__(s, tau=0.03, vfull=0.40, fr_r=0.20, fr_l=0.10, cp=0.06, x0=0.0, Lp=None, stick=0.0, vs=0.02):
        s.tau, s.vfull, s.fr_r, s.fr_l, s.cp = tau, vfull, fr_r, fr_l, cp
        s.stick, s.vs = stick, vs      # stiction: extra duty to break away, fading over vs m/s (Stribeck)
        s.Bp = 3.0 / (2 * (Lp or L))     # the REAL pole; the controller still assumes L
        s.x, s.v, s.thu, s.w = x0, 0.0, math.pi, 0.0   # thu: unwrapped, from upright
        s.crash = False

    def step(s, duty, dt):
        if abs(s.v) < 1e-6:
            thr = (s.fr_r if duty > 0 else s.fr_l) + s.stick
            if abs(duty) <= thr:
                a = 0.0
                vn = 0.0
            else:
                deff = duty - math.copysign(thr, duty)
                a = (s.vfull * deff) / s.tau
                vn = a * dt
        else:
            thr = (s.fr_r if s.v > 0 else s.fr_l) + s.stick * math.exp(-abs(s.v) / s.vs)
            deff = duty - math.copysign(thr, s.v)
            a = (s.vfull * deff - s.v) / s.tau
            vn = s.v + a * dt
            if vn * s.v < 0:
                vn = 0.0
        if abs(s.x) >= X_PHYS and s.x * vn > 0:      # hit the end
            vn = 0.0
            s.crash = True
        a_eff = (vn - s.v) / dt
        s.x += 0.5 * (s.v + vn) * dt
        s.x = max(-X_PHYS, min(X_PHYS, s.x))
        s.v = vn
        thdd = s.Bp * (G * math.sin(s.thu) - a_eff * math.cos(s.thu)) - s.cp * s.w
        s.w += thdd * dt
        s.thu += s.w * dt

    def counts(s):
        cart = round(s.x / M_PER_COUNT)
        pole = round((s.thu - math.pi) / RAD_PER_COUNT)
        return cart, pole


class Ctrl:
    """Port of loop() in swing_pendulum_claude_1.2 (v44) / candidate v45."""

    def __init__(s, variant='v44', **kw):
        s.variant = variant
        s.p = dict(
            # shared
            DUTY_SWING=0.95, PHI_DEAD=0.035, SENS_GATE=0.35, KX_R=3.2, KV_R=1.1,
            KX=3.39, KV=4.46, KTH=37.17, KW=6.75, DUTY_MAX=0.98,
            A_MAX=2.2, V_MAX=0.367, FRICTION_FF=0.08,
            X_MAX=0.190, X_BRAKE=0.030, FENCE_PUSH=0.45,
            # v44
            E_HANDOVER=-0.80, TH_HANDOVER=35.0, GIVEUP_DEG=40.0,
            # v45
            E_TARGET=0.0, E_SCALE=1.2, E_DEAD=0.04,
            TH_CATCH=15.0, Z_CATCH=0.12, GIVEUP45=30.0, BAL_LAW='int',
            TAU_FF=0.0, HOLD_S=3.0, MAX_DROPS=5, E_SPIN_HALT=6.0, KI=0.0, ZO=0.09, ZU=0.06, EC=True, E_SWITCH=-0.8, KE=4.0, A_LIM=6.0, KXE=6.0, KVE=4.0, KIE=0.2, EINT_MAX=0.3, LEARN=0.5, E_T_LO=-0.3, E_T_HI=0.3, GUARD=False, TH_EC_IN=45.0, TH_EC_OUT=60.0, LIVE_CAL=False, CAL_AMP=35.0, CAL_TURNS=4,
        )
        s.p.update(kw)
        s.Lc = s.p.get('Lc', L)
        s.lam = math.sqrt(1.5 * G / s.Lc); s.bp = 1.5 / s.Lc
        s.f = s.Lc / L
        if s.p.get('Lc') and s.p.get('AUTO_K', True):
            s.p['KX'], s.p['KV'], s.p['KTH'], s.p['KW'] = lqr_gains(s.Lc)
        s.mode = 'SWINGUP'
        s.xF = s.vF = s.thF = s.wF = s.xPrev = s.thPrev = 0.0
        s.thPrev = math.pi
        s.thF = math.pi
        s.dirHold = 1
        s.sensPk = 0.5
        s.t = 0.0
        s.kickUntil = 0.2
        s.retries = 0
        s.drops = 0
        s.catches = []
        s.vCmd = 0.0
        s.balStart = None
        s.halted = None
        s.lastDuty = 0.0
        s.signCheckAt = 3.0
        s.eAtCheck = -3.94
        s.eBest = -9
        s.bestAt = 0.0
        s.signProven = False
        s.SIGN = 1.0
        s.log = []
        s.eInt = 0.0
        s.ecOn = False
        s.phiPrev = 0.0
        s.eBias = 0.0
        s.lastTurn = 180.0
        s.cal = 'pump'; s.calTurns = []; s.calCrossed = False; s.calPeak = 0.0; s.calPhiPrev = 0.0; s.calAmpLast = 0.0; s.Lmeas = None
        s.thPrevSign = 0.0
        s.wPrevSign = 0.0

    def set_length(s, Lx):
        s.Lc = Lx; s.lam = math.sqrt(1.5 * G / Lx); s.bp = 1.5 / Lx; s.f = Lx / L
        s.p['KX'], s.p['KV'], s.p['KTH'], s.p['KW'] = lqr_gains(Lx)

    def fence(s, duty, x):
        p = s.p
        d = p['X_MAX'] - abs(x)
        if d <= 0:
            return -p['FENCE_PUSH'] if x > 0 else p['FENCE_PUSH']
        if d < p['X_BRAKE']:
            if (x > 0 and duty > 0) or (x < 0 and duty < 0):
                duty *= d / p['X_BRAKE']
        return duty

    def start_swing(s, kick=True):
        s.mode = 'SWINGUP'
        s.dirHold = 1
        s.sensPk = 0.5
        s.kickUntil = s.t + (0.2 if kick else 0.0)
        s.signCheckAt = s.t + 3.0
        s.eAtCheck = energy(s.thF, 0, s.Lc)
        s.eBest = s.eAtCheck
        s.bestAt = s.t
        s.ecOn = False
        s.lastTurn = 180.0

    def tick(s, cart, pole, dt=0.002):
        p = s.p
        s.t += dt
        x = cart * M_PER_COUNT
        th = s.SIGN * wrap(pole * RAD_PER_COUNT - math.pi)
        alpha = dt / (0.020 + dt)
        s.vF += alpha * ((x - s.xPrev) / dt - s.vF)
        s.wF += alpha * (wrap(th - s.thPrev) / dt - s.wF)
        s.xPrev, s.thPrev, s.xF, s.thF = x, th, x, th
        E = energy(s.thF, s.wF, s.Lc)
        phi = wrap(s.thF - math.pi)
        if p.get('TICK_FN'): p['TICK_FN'](s)
        duty = 0.0
        if s.halted:
            return 0.0
        if E > p['E_SPIN_HALT'] * s.f and s.variant == 'v45':
            s.halted = 'spinning'
            return 0.0

        if s.mode == 'SWINGUP' and s.variant == 'v45' and p['LIVE_CAL'] and s.cal != 'done':
            # turning points (zero-safe bottom crossing, as the decay test)
            if (phi >= 0) != (s.calPhiPrev >= 0) and abs(phi) < 1.0:
                s.calCrossed = True; s.calPeak = 0.0
            s.calPhiPrev = phi
            s.calPeak = max(s.calPeak, abs(phi))
            turned = (s.wF >= 0) != (s.wPrevSign >= 0)
            if turned and s.calCrossed and s.calPeak > 0.12:
                s.calCrossed = False
                if s.cal == 'pump':
                    s.calAmpLast = s.calPeak
                    if s.calPeak * 180 / math.pi >= p['CAL_AMP']:
                        s.cal = 'free'; s.calTurns = []; s.calSkip = 1   # skip the first: cart still stopping
                else:
                    if s.calSkip > 0: s.calSkip -= 1
                    else: s.calTurns.append((s.t, s.calPeak))
                s.calPeak = 0.0
            if s.cal == 'free':
                if len(s.calTurns) >= p['CAL_TURNS']:
                    ests = []
                    for (t1, a1), (t2, a2) in zip(s.calTurns, s.calTurns[1:]):
                        a, b = 1.0, math.cos((a1 + a2) / 4)
                        for _ in range(6): a, b = (a + b) / 2, math.sqrt(a * b)
                        T0 = 2 * (t2 - t1) * a
                        ests.append(3 * G * T0 * T0 / (8 * math.pi ** 2))
                    Lm = sum(ests) / len(ests)
                    if 0.12 <= Lm <= 0.50:
                        s.set_length(Lm); s.Lmeas = Lm
                    s.cal = 'done'
                    s.bestAt = s.t; s.signCheckAt = s.t + 3.0; s.eAtCheck = energy(s.thF, 0, s.Lc); s.eBest = s.eAtCheck
                else:
                    s.wPrevSign = s.wF; s.thPrevSign = s.thF
                    s.lastDuty = 0.0
                    return 0.0                                    # cart stopped: let it swing freely
        if s.mode == 'SWINGUP':
            sens = abs(s.wF * math.cos(s.thF))
            s.sensPk = max(s.sensPk, sens) * 0.9994
            s.sensPk = max(s.sensPk, 0.05)
            conserve = E > -3.2 * s.f
            if s.variant == 'v45' and p['GUARD'] and (s.wF >= 0) != (s.wPrevSign >= 0):
                s.lastTurn = abs(s.thF) * 180 / math.pi          # angle from upright at a turning point
                if s.ecOn and s.lastTurn > p['TH_EC_OUT']:
                    s.ecOn = False                                # fell back far short: pump hard again
            ec_allowed = (not p['GUARD']) or s.lastTurn < p['TH_EC_IN']
            if s.variant == 'v45' and p['EC']:
                if not s.ecOn and E > p['E_SWITCH'] * s.f and ec_allowed:
                    s.ecOn = True
                    s.vCmd = s.vF
                elif s.ecOn and E < (p['E_SWITCH'] - 0.8) * s.f:
                    s.ecOn = False
            if s.variant == 'v45' and p['EC'] and s.ecOn:
                sg = 1.0 if s.wF * math.cos(s.thF) > 0 else -1.0
                if p['LEARN'] > 0:
                    # outcome learning near the top: fell back short -> raise target by the
                    # shortfall; went over the top -> lower it by the excess
                    gl2 = G * s.Lc / 2
                    if s.wPrevSign * s.wF < 0 and abs(s.thF) < math.pi / 2:          # turned back, upper half
                        short = gl2 * (1 - math.cos(s.thF))
                        s.eBias += p['LEARN'] * short
                    if s.thPrevSign * s.thF < 0 and abs(s.thF) < 0.5:               # crossed the top uncaught
                        excess = (s.Lc * s.Lc / 6) * s.wF * s.wF
                        s.eBias -= p['LEARN'] * excess
                    s.eBias = max(p['E_T_LO'] * s.f, min(p['E_T_HI'] * s.f, s.eBias))
                    s.eInt = s.eBias
                else:
                    s.eInt = max(-p['EINT_MAX'] * s.f, min(p['EINT_MAX'] * s.f, s.eInt + p['KIE'] * (p['E_TARGET'] * s.f - E) * dt))
                a = (p['KE'] / s.f) * (E - p['E_TARGET'] * s.f - s.eInt) * sg - p['KXE'] * s.xF - p['KVE'] * s.vF
                a = max(-p['A_LIM'], min(p['A_LIM'], a))
                vlim = 0.95 * p['V_MAX']
                s.vCmd = max(-vlim, min(vlim, s.vCmd + a * dt))
                duty = s.vCmd * (0.95 - p['FRICTION_FF']) / p['V_MAX'] + p['FRICTION_FF'] * max(-1, min(1, s.vCmd / 0.02))
            elif not conserve or sens > p['SENS_GATE'] * s.sensPk:
                if abs(phi) > p['PHI_DEAD']:
                    s.dirHold = 1 if phi > 0 else -1
                if s.variant == 'v44' or p['EC']:
                    duty = p['DUTY_SWING'] * s.dirHold     # firmware 4.4-4.7: full-power pumps
                else:
                    eErr = p['E_TARGET'] - E
                    if abs(eErr) < p['E_DEAD']:
                        duty = -(p['KX_R'] * s.xF + p['KV_R'] * s.vF)
                    else:
                        k = max(-1.0, min(1.0, (eErr + p['KI'] * s.eInt) / p['E_SCALE']))
                        mag = p['FRICTION_FF'] + (p['DUTY_SWING'] - p['FRICTION_FF']) * abs(k)
                        duty = s.dirHold * math.copysign(mag, k)
            else:
                duty = -(p['KX_R'] * s.xF + p['KV_R'] * s.vF)
            if phi * s.phiPrev < 0 and abs(s.wF) > 0.4 and abs(phi) < 1.0 and E > -1.5:
                s.eInt = max(-1.0, min(1.0, s.eInt + (p['E_TARGET'] - E)))
            s.phiPrev = phi
            if s.t < s.kickUntil:
                duty = p['DUTY_SWING']
            fd = s.fence(duty, s.xF)
            if s.variant == 'v45' and p['EC'] and s.ecOn and fd != duty:
                s.vCmd = (fd - math.copysign(p['FRICTION_FF'], fd)) * p['V_MAX'] / (0.95 - p['FRICTION_FF']) if abs(fd) > p['FRICTION_FF'] else 0.0
            duty = fd
            # sign check: v44 always; v45 only while low and unproven
            if s.t > s.signCheckAt:
                ok_to_check = (s.variant == 'v44') or (not s.signProven and E < -2.0 * s.f)
                if ok_to_check and E < s.eAtCheck + 0.10:
                    s.SIGN = -s.SIGN
                    s.thPrev = -s.thPrev
                    s.wF = -s.wF
                    s.log.append((s.t, 'flip'))
                s.eAtCheck = E
                s.signCheckAt = s.t + 3.0
            near = abs(E - p['E_TARGET'] * s.f) < 0.3 * s.f
            if E > s.eBest + 0.05 or (s.variant == 'v45' and near):
                s.eBest = max(E, s.eBest) if not near else s.eBest
                s.bestAt = s.t
            if s.t - s.bestAt > 12.0:
                s.halted = 'stalled'
            # handover
            if s.variant == 'v44':
                if E > p['E_HANDOVER'] and abs(s.thF) * 180 / math.pi < p['TH_HANDOVER']:
                    s.mode = 'BALANCE'
                    s.retries = 0
                    s.catches.append((s.t, s.thF, s.wF, E))
                    s.vCmd = s.vF
            else:
                z = s.thF + s.wF / s.lam + (s.bp / s.lam) * s.vF
                if p['ZO'] is not None:
                    zn = z * (1 if s.thF > 0 else -1)
                    zok = -p['ZO'] < zn < p['ZU']
                else:
                    zok = abs(z) < p['Z_CATCH']
                if abs(s.thF) * 180 / math.pi < p['TH_CATCH'] and zok:
                    s.mode = 'BALANCE'
                    s.signProven = True
                    s.catches.append((s.t, s.thF, s.wF, E))
                    s.vCmd = s.vF
                    s.balStart = s.t

        elif s.mode == 'BALANCE':
            aCmd = p['ACC_FN'](s) if p.get('ACC_FN') else (p['KX'] * s.xF + p['KV'] * s.vF + p['KTH'] * s.thF + p['KW'] * s.wF)
            law = p['BAL_LAW'] if s.variant == 'v45' else 'acc'
            if law == 'acc':
                duty = aCmd / p['A_MAX']
                bb = max(-1, min(1, aCmd))
                duty += p['FRICTION_FF'] * bb
            elif law == 'int':
                vlim = 0.95 * p['V_MAX']
                s.vCmd = max(-vlim, min(vlim, s.vCmd + aCmd * dt))
                vc = s.vCmd + p['TAU_FF'] * aCmd
                duty = vc * (0.95 - p['FRICTION_FF']) / p['V_MAX']
                duty += p['FRICTION_FF'] * max(-1, min(1, vc / 0.02))
            fd = s.fence(duty, s.xF)
            if law == 'int' and fd != duty:       # anti-windup at the fence
                s.vCmd = (fd - math.copysign(p['FRICTION_FF'], fd)) * p['V_MAX'] / (0.95 - p['FRICTION_FF']) if abs(fd) > p['FRICTION_FF'] else 0.0
            duty = fd
            give = p['GIVEUP_DEG'] if s.variant == 'v44' else p['GIVEUP45']
            if abs(s.xF) > X_PHYS:
                s.halted = 'past end'
            elif abs(s.thF) * 180 / math.pi > give:
                if s.variant == 'v44':
                    if s.retries < 4:
                        s.retries += 1
                        s.drops += 1
                        s.start_swing(kick=True)
                    else:
                        s.halted = 'gave up'
                else:
                    s.drops += 1
                    if s.drops >= p['MAX_DROPS']:
                        s.halted = 'gave up'
                    else:
                        s.start_swing(kick=False)
            elif s.variant == 'v45' and s.balStart is not None and s.t - s.balStart > p['HOLD_S']:
                s.drops = 0                       # held it properly: start counting afresh
                s.balStart = None
        s.wPrevSign = s.wF
        s.thPrevSign = s.thF
        duty = max(-p['DUTY_MAX'], min(p['DUTY_MAX'], duty))
        s.lastDuty = duty
        return duty


def run(variant='v44', T=30.0, plant_kw=None, ctrl_kw=None, wrong_sign=False, trace=False):
    plant = Plant(**(plant_kw or {}))
    ctrl = Ctrl(variant, **(ctrl_kw or {}))
    if wrong_sign:
        ctrl.SIGN = -1.0
    dtp, dtc = 0.0005, 0.002
    n_sub = int(round(dtc / dtp))
    duty = 0.0
    t = 0.0
    maxE, maxX = -9, 0.0
    bal_time = 0.0
    longest = 0.0
    cur = 0.0
    tr = []
    while t < T:
        cart, pole = plant.counts()
        duty = ctrl.tick(cart, pole, dtc)
        for _ in range(n_sub):
            plant.step(duty, dtp)
        t += dtc
        E = energy(wrap(plant.thu), plant.w)
        maxE = max(maxE, E)
        maxX = max(maxX, abs(plant.x))
        if ctrl.mode == 'BALANCE' and not ctrl.halted:
            bal_time += dtc
            cur += dtc
            longest = max(longest, cur)
        else:
            cur = 0.0
        if trace and int(t / dtc) % 25 == 0:
            tr.append((t, ctrl.mode, plant.x, wrap(plant.thu) * 180 / math.pi, plant.w * 180 / math.pi, E, duty))
        if ctrl.halted:
            break
    return dict(halted=ctrl.halted, catches=len(ctrl.catches), drops_total=None,
                first=ctrl.catches[0] if ctrl.catches else None,
                cat=ctrl.catches, bal=bal_time, longest=longest, maxE=maxE, maxX=maxX,
                crash=plant.crash, t_end=t, flips=[l for l in ctrl.log if l[1] == 'flip'], trace=tr)


if __name__ == '__main__':
    r = run('v44', T=40, plant_kw=dict(tau=0.03), wrong_sign=False)
    print('v44: halted', r['halted'], 'catches', r['catches'], 'longest bal %.2f s' % r['longest'],
          'maxE %.2f' % r['maxE'], 'maxX %.0f mm' % (r['maxX'] * 1000), 'crash', r['crash'])
    for c in r['cat'][:8]:
        print('  caught t=%.2f th=%.1f w=%.0f dps E=%.2f' % (c[0], c[1] * 180 / math.pi, c[2] * 180 / math.pi, c[3]))
