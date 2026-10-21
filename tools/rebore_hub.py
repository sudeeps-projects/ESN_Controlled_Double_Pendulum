"""Make the link-2 hub: ZIPY's zipy_dp_hub with its 6.3 mm bore opened up for
the elbow's 8 mm shaft. Moves the bore's vertices only, so everything else
(plate, collar, bolt holes, nut pockets, set-screw hole and nut trap) is ZIPY's
own geometry, untouched.

    python tools/rebore_hub.py            -> STL Files/zipy_dp_hub_8mm.stl

The bore is 8.3: ZIPY's hub gives the 6 mm shaft the same 0.3 mm, and the set
screw takes up the slack.
"""
import struct, sys
import numpy as np
sys.path.insert(0, __file__.replace('\\', '/').rsplit('/', 1)[0])
from stl_holes import load_stl

R_OLD, R_NEW = 3.15, 4.15          # bore radius, mm
CH_BOT, CH_TOP = 0.49, 0.79        # ZIPY's 45 degree entry chamfers, kept
HOLE_R, HOLE_Z = 1.7, 6.0          # set-screw hole: along -y, centred x 0, z 6


def rebore(tri):
    v = tri.reshape(-1, 3).copy()
    r = np.hypot(v[:, 0], v[:, 1])
    out = v.copy()
    bore = np.abs(r - R_OLD) < 0.02
    # where the set-screw hole meets the bore: keep x and z (stay on the hole's
    # cylinder) and slide y onto the new bore
    cross = bore & (v[:, 1] < 0) & (np.abs(np.hypot(v[:, 0], v[:, 2] - HOLE_Z) - HOLE_R) < 0.03)
    plain = bore & ~cross
    out[plain, :2] *= (R_NEW / r[plain])[:, None]
    out[cross, 1] = -np.sqrt(R_NEW ** 2 - v[cross, 0] ** 2)
    # chamfer rims on the two end faces
    for rr, zz, ch in ((R_OLD + CH_BOT, 0.0, CH_BOT), (R_OLD + CH_TOP, 15.0, CH_TOP)):
        rim = (np.abs(r - rr) < 0.02) & (np.abs(v[:, 2] - zz) < 0.01)
        out[rim, :2] *= ((R_NEW + ch) / r[rim])[:, None]
    return retop(out.reshape(-1, 3, 3)), int(plain.sum()), int(cross.sum())


def retop(tri, z=15.0):
    """The collar's top face is a thin ring meshed with long slivers; with its
    inner edge moved out, some fold over. Re-mesh it: zip the inner and outer
    edge loops together by angle. Uses the same edge vertices, so the mesh
    stays closed."""
    flat = np.all(np.abs(tri[:, :, 2] - z) < 1e-4, axis=1)
    v = np.unique(tri[flat].reshape(-1, 3), axis=0)
    r = np.hypot(v[:, 0], v[:, 1])
    inner, outer = v[r < r.mean()], v[r >= r.mean()]
    inner = inner[np.argsort(np.arctan2(inner[:, 1], inner[:, 0]))]
    outer = outer[np.argsort(np.arctan2(outer[:, 1], outer[:, 0]))]
    ang = lambda p: np.arctan2(p[1], p[0])
    new, i, o, ni, no = [], 0, 0, len(inner), len(outer)
    while i < ni or o < no:
        a_i = ang(inner[(i + 1) % ni]) + (2 * np.pi if i + 1 >= ni else 0)
        a_o = ang(outer[(o + 1) % no]) + (2 * np.pi if o + 1 >= no else 0)
        if o >= no or (i < ni and a_i <= a_o):
            t = [inner[i % ni], outer[o % no], inner[(i + 1) % ni]]; i += 1
        else:
            t = [inner[i % ni], outer[o % no], outer[(o + 1) % no]]; o += 1
        if np.cross(t[1] - t[0], t[2] - t[0])[2] < 0:
            t = [t[0], t[2], t[1]]
        new.append(t)
    return np.concatenate([tri[~flat], np.array(new)])


def save_stl(path, tri):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    with open(path, 'wb') as f:
        f.write(b'zipy_dp_hub, 8.3 mm bore (rebore_hub.py)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tri)))
        for nn, t in zip(n, tri):
            f.write(struct.pack('<12fH', *nn, *t.ravel(), 0))


if __name__ == '__main__':
    src, dst = 'STL Files/zipy_dp_hub.stl', 'STL Files/zipy_dp_hub_8mm.stl'
    tri = load_stl(src)
    new, n_plain, n_cross = rebore(tri)
    save_stl(dst, new)
    print('%s -> %s: %d bore vertices moved, %d on the set-screw hole' % (src, dst, n_plain, n_cross))
