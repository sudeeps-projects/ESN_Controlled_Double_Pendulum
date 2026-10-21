"""Full runs: v44 vs candidate v45 across motor speed, friction, start offset."""
import math, os, sys, warnings
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
warnings.filterwarnings('ignore')
from sim import run, Plant, Ctrl, energy, wrap

cases = []
for tau in (0.006, 0.03, 0.08):
    for fr in ((0.10, 0.10), (0.20, 0.10), (0.30, 0.12)):
        for x0 in (0.0, 0.08):
            cases.append((tau, fr, x0))


def summary(variant, ckw=None, T=40.0, label=None):
    ok = 0; rows = []
    for tau, (fr_r, fr_l), x0 in cases:
        r = run(variant, T=T, plant_kw=dict(tau=tau, fr_r=fr_r, fr_l=fr_l, x0=x0), ctrl_kw=ckw or {})
        good = r['longest'] >= 10.0
        ok += good
        rows.append((tau, fr_r, fr_l, x0, r))
    print('%-28s balanced >=10 s in %d/%d runs' % (label or variant, ok, len(cases)))
    return rows


def detail(rows, n=6):
    for tau, fr_r, fr_l, x0, r in rows[:n]:
        f = r['first']
        print('   tau %2.0fms fr %.2f/%.2f x0 %3.0fmm: %-8s catches %2d longest %5.1fs maxE %5.2f maxX %3.0fmm crash %s%s' % (
            tau * 1000, fr_r, fr_l, x0 * 1000, r['halted'] or '-', r['catches'], r['longest'], r['maxE'],
            r['maxX'] * 1000, r['crash'],
            ('  first catch %.1fdeg %.0fdps E%.2f' % (f[1] * 180 / math.pi, f[2] * 180 / math.pi, f[3])) if f else ''))


if __name__ == '__main__':
    rows = summary('v44', label='v44 (current 1.2)')
    detail(rows, 4)
    rows = summary('v45', label='v45 candidate')
    detail(rows, 18)
