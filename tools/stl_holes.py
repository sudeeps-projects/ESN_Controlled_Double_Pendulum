"""Measure printed parts: bounding box, and every round hole/boss found by
slicing the mesh across each axis and fitting circles to the section loops.

    python tools/stl_holes.py "STL Files/*.stl"
"""
import glob, math, struct, sys
from collections import defaultdict
import numpy as np


def load_stl(path):
    data = open(path, 'rb').read()
    if data[:5].lower() == b'solid' and b'facet' in data[:400]:
        v = [list(map(float, l.split()[1:4])) for l in data.decode(errors='ignore').splitlines()
             if l.strip().startswith('vertex')]
        return np.array(v, dtype=float).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    a = np.frombuffer(data, dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n, offset=84)
    return a['v'].astype(float).reshape(-1, 3, 3)


def section_loops(tri, axis, c):
    """Segments where plane x[axis]=c cuts the triangles, chained into loops (2-D)."""
    others = [i for i in range(3) if i != axis]
    d = tri[:, :, axis] - c
    segs = []
    for t, dd in zip(tri, d):
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (dd[i] < 0) != (dd[j] < 0):
                s = dd[i] / (dd[i] - dd[j])
                p = t[i] + s * (t[j] - t[i])
                pts.append((round(p[others[0]], 4), round(p[others[1]], 4)))
        if len(pts) == 2 and pts[0] != pts[1]:
            segs.append(pts)
    adj = defaultdict(list)
    for a, b in segs:
        adj[a].append(b); adj[b].append(a)
    seen, loops = set(), []
    for start in list(adj):
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [p for p in adj[cur] if p != prev and p not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            seen.add(cur); loop.append(cur)
        if len(loop) >= 12:
            loops.append(np.array(loop))
    return loops


def circle_fit(p):
    x, y = p[:, 0], p[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    cx, cy, k = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    r = math.sqrt(max(k + cx * cx + cy * cy, 0))
    err = np.max(np.abs(np.hypot(x - cx, y - cy) - r))
    return cx, cy, r, err


def analyse(path):
    tri = load_stl(path)
    pts = tri.reshape(-1, 3)
    lo, hi = pts.min(0), pts.max(0)
    print('\n%s: %d triangles, size %.1f x %.1f x %.1f mm' % (path.split('\\')[-1].split('/')[-1], len(tri), *(hi - lo)))
    found = {}
    for axis in range(3):
        for f in (0.12, 0.3, 0.5, 0.7, 0.88):
            c = lo[axis] + f * (hi[axis] - lo[axis]) + 1e-3
            for loop in section_loops(tri, axis, c):
                cx, cy, r, err = circle_fit(loop)
                if err < max(0.06 * r, 0.15) and 0.8 < r < 40:
                    key = (axis, round(cx, 0), round(cy, 0), round(2 * r, 1))
                    found.setdefault(key, []).append(c)
    rows = {}
    for (axis, cx, cy, dia), cs in found.items():
        k2 = (axis, cx, cy, dia)
        rows[k2] = (min(cs), max(cs))
    for (axis, cx, cy, dia), (c0, c1) in sorted(rows.items(), key=lambda kv: -kv[0][3]):
        print('   round  dia %5.1f mm  along %s  at (%6.1f, %6.1f)  seen %s %.1f..%.1f' % (
            dia, 'xyz'[axis], cx, cy, 'xyz'[axis], c0, c1))


if __name__ == '__main__':
    for pat in sys.argv[1:]:
        for p in sorted(glob.glob(pat)):
            analyse(p)
