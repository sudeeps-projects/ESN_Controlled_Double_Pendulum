"""Mass properties of printed parts from their STLs: volume, centroid and
inertia tensor (about the centroid), by signed tetrahedra.

    python tools/mass_props.py "STL Files/zipy_dp_*.stl"
"""
import glob, sys
import numpy as np
sys.path.insert(0, __file__.rsplit('\\', 1)[0].rsplit('/', 1)[0])
from stl_holes import load_stl


def mass_props(tri):
    """Volume (mm^3), centroid (mm), inertia about the centroid per unit density (mm^5)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))           # 6 x signed volume of each tet with origin
    V = vol6.sum() / 6.0
    cen = ((a + b + c) * vol6[:, None]).sum(0) / (24.0 * V)
    # second moments: integral of x_i x_j over each tet (origin, a, b, c)
    S = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            s = (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]
                 + (a[:, i] + b[:, i] + c[:, i]) * (a[:, j] + b[:, j] + c[:, j]))
            S[i, j] = (vol6 * s).sum() / 120.0
    S_c = S - V * np.outer(cen, cen)                           # about the centroid
    I = np.trace(S_c) * np.eye(3) - S_c
    return V, cen, I


if __name__ == '__main__':
    for pat in sys.argv[1:]:
        for p in sorted(glob.glob(pat)):
            V, cen, I = mass_props(load_stl(p))
            print('%-24s volume %7.2f cm3  solid PLA %5.1f g  centroid (%.1f, %.1f, %.1f) mm' % (
                p.replace('\\', '/').split('/')[-1], V / 1000, V / 1000 * 1.24, *cen))
