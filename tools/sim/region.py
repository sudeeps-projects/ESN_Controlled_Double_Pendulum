"""Which (theta, z) states can the integrator balance law recover from?"""
import math, os, sys, warnings
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
warnings.filterwarnings('ignore')
from bal import balance_run
from sim import LAM, B

zs = [-0.24, -0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18, 0.24]
for tau in (0.006, 0.03, 0.08):
    print('tau %d ms   rows: theta (deg, approaching from + side); cols: z (rad)' % (tau * 1000))
    print('        ' + ' '.join('%5.2f' % z for z in zs))
    for deg in (5, 10, 15, 20, 25, 30):
        th = deg * math.pi / 180
        row = []
        for z in zs:
            w = (z - th) * LAM            # z = th + w/LAM  (cart at rest)
            ok, *_ = balance_run('int', tau, th, w, T=4.0, fr_r=0.25, fr_l=0.12)
            row.append('  ok ' if ok else '  -- ')
        print('  %2d deg ' % deg + ' '.join(row))
    print()
