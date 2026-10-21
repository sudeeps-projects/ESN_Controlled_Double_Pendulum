import math, os, sys, warnings, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
warnings.filterwarnings('ignore')
import sim
from sim import run, Plant, Ctrl, energy, wrap

CK = dict(EC=True, E_SWITCH=-0.8, KE=4.0, E_TARGET=0.05, KIE=0.2, EINT_MAX=0.3, KXE=6.0, KVE=4.0,
          TH_CATCH=20.0, ZO=0.09, ZU=0.06, BAL_LAW='int', GIVEUP45=30.0, MAX_DROPS=5, HOLD_S=3.0)

# 1. unseen plants
ok = 0; n = 0; bad = []
for tau, fr, cp, vf, x0 in itertools.product((0.004, 0.015, 0.05, 0.12), ((0.15, 0.15), (0.35, 0.15)),
                                             (0.02, 0.12), (0.33, 0.45), (-0.08, 0.05)):
    r = run('v45', T=40, plant_kw=dict(tau=tau, fr_r=fr[0], fr_l=fr[1], cp=cp, vfull=vf, x0=x0), ctrl_kw=CK)
    n += 1; g = r['longest'] >= 10; ok += g
    if not g or r['crash']:
        bad.append((tau, fr, cp, vf, x0, r['halted'], r['catches'], round(r['longest'], 1), r['crash'], round(r['maxE'], 2)))
print('1. unseen plants: balanced >=10 s in %d/%d' % (ok, n))
for b in bad[:12]:
    print('   fail tau %.3f fr %s cp %.2f vfull %.2f x0 %+.2f -> %s catches %d longest %.1f crash %s maxE %.2f' % b)

# 2. wrong starting sign (the sign check must fix it during the low-energy phase)
ok = 0
for tau in (0.006, 0.03, 0.08):
    r = run('v45', T=45, plant_kw=dict(tau=tau, fr_r=0.2, fr_l=0.1), ctrl_kw=CK, wrong_sign=True)
    ok += r['longest'] >= 10
    print('2. wrong sign tau %2.0f ms: flips %d, first catch %s, longest %.1f s, %s' % (
        tau * 1000, len(r['flips']), ('%.1f s' % r['first'][0]) if r['first'] else '-', r['longest'], r['halted']))


# 3. spinning pole at the start (like after the old runaway) and 4. shoves during balance
def custom(tau, w0=0.0, shoves=(), T=40):
    pl = Plant(tau=tau, fr_r=0.2, fr_l=0.1)
    c = Ctrl('v45', **CK)
    if w0:
        pl.thu = 0.0; pl.w = w0
        c.thF = c.thPrev = 0.0; c.wF = w0
        c.kickUntil = 0.0
    t = 0; maxE = -9; bal = 0; longest = 0; cur = 0; si = 0
    while t < T:
        cart, pole = pl.counts()
        d = c.tick(cart, pole, 0.002)
        for _ in range(4): pl.step(d, 0.0005)
        t += 0.002
        if si < len(shoves) and t >= shoves[si][0]:
            pl.w += shoves[si][1]; si += 1
        E = energy(wrap(pl.thu), pl.w); maxE = max(maxE, E)
        if c.mode == 'BALANCE': cur += 0.002; longest = max(longest, cur)
        else: cur = 0
        if c.halted: break
    return c, longest, maxE, t

for tau in (0.006, 0.03, 0.08):
    c, lg, mE, t = custom(tau, w0=12.0)
    print('3. spinning start (690 dps) tau %2.0f ms: catches %d, longest %.1f s, halted %s, t %.1f' % (tau * 1000, len(c.catches), lg, c.halted, t))
for tau in (0.006, 0.03, 0.08):
    # three hard shoves (2.5 rad/s = 143 dps) after it has caught, 8 s apart
    c, lg, mE, t = custom(tau, shoves=((14, 2.5), (22, -2.5), (30, 2.5)), T=45)
    print('4. shoved 3x  tau %2.0f ms: catches %d, longest %.1f s, maxE %.2f, halted %s' % (tau * 1000, len(c.catches), lg, mE, c.halted))
# 5. impossible case must END, not loop: a pole so stiff it can never be caught
c, lg, mE, t = custom(0.03, shoves=tuple((10 + 2 * i, 6.0) for i in range(40)), T=120)
print('5. shoved every 2 s for ever: catches %d, ended: %s at %.0f s' % (len(c.catches), c.halted, t))
