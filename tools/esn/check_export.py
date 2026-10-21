"""Check esn_weights.h reproduces the trained network: parse the C arrays, run
them with the same loop order as esnStep()/esnDot() in swing_pendulum_esn_1.0,
and compare with the Python ESN on a recorded stretch of simulated balancing."""
import os, re, sys, math, warnings
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
warnings.filterwarnings('ignore')
import train_esn as T


def parse(path):
    txt = open(path).read()
    arrs = {}
    for m in re.finditer(r'const float (\w+)\[(\d+)\] = \{(.*?)\};', txt, re.S):
        vals = np.array([float(v.rstrip('f')) for v in re.findall(r'[-+0-9.eE]+f', m.group(3))], dtype=np.float32)
        assert len(vals) == int(m.group(2)), m.group(1)
        arrs[m.group(1)] = vals
    arrs['LEAK'] = np.float32(float(re.search(r'ESN_LEAK = ([0-9.]+)f', txt).group(1)))
    return arrs


def c_style(H, inputs_raw):
    N, U = 64, 6; F = N + U + 1
    r = np.zeros(N, np.float32); outs = []
    for raw in inputs_raw:
        u = (raw / H['ESN_IN_NORM']).astype(np.float32)
        pre = np.empty(N, np.float32)
        for i in range(N):
            s = np.float32(H['ESN_B'][i])
            s += np.dot(H['ESN_W'][i*N:(i+1)*N], r) + np.dot(H['ESN_WIN'][i*U:(i+1)*U], u)
            pre[i] = s
        r = ((1 - H['LEAK']) * r + H['LEAK'] * np.tanh(pre)).astype(np.float32)
        phi = np.concatenate([r, u, [1.0]]).astype(np.float32)
        outs.append((np.dot(H['ESN_WC'], phi), np.dot(H['ESN_WP'][:F], phi), np.dot(H['ESN_WP'][F:], phi)))
    return np.array(outs)


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'esn_weights.h')
    H = parse(path)
    R = np.load(os.path.join(HERE, 'esn_readouts.npz'))
    rng = np.random.default_rng(5)
    # a stretch of plausible measured inputs near the top
    raws = []
    th, w, x, v, vc = 0.1, -0.5, 0.02, 0.0, 0.0
    for k in range(600):
        th += w * 0.002; w += (36 * th - 6 * vc) * 0.002 + rng.normal(0, 0.05)
        vc = np.clip(vc + (40 * th + 6 * w) * 0.002, -0.3, 0.3); x += vc * 0.002; v = vc
        th = max(-0.5, min(0.5, th))
        raws.append(np.array([math.sin(th), 1 - math.cos(th), w, x, v, vc], np.float32))
    c_out = c_style(H, raws)
    esn = T.ESN(); py = []
    for raw in raws:
        esn.step((raw / T.IN_NORM.astype(np.float32)).astype(np.float32))
        phi = esn.features()
        py.append((R['Wc'] @ phi, (R['Wp'] @ phi)[0], (R['Wp'] @ phi)[1]))
    py = np.array(py)
    err = np.max(np.abs(c_out - py), axis=0)
    scale = np.max(np.abs(py), axis=0)
    print('header vs trained network over 600 ticks: max |diff| control %.2e (of %.2f m/s^2), '
          'angle corr %.2e (of %.3f rad), rate corr %.2e (of %.3f rad/s)' % (err[0], scale[0], err[1], scale[1], err[2], scale[2]))
    ok = all(err <= 1e-3 * np.maximum(scale, 1e-3))
    print('MATCH' if ok else 'MISMATCH')
