"""Print-ready STLs for the second pendulum's elbow (ZIPY route): the bearing
cartridge that bolts into zipy_dp_elbow's encoder face, and the three small
rings that go with it. All dimensions in mm; local z = 0 is the face that sits
on the elbow plate, + toward link 1 (inboard).

    python tools/make_elbow_parts.py     -> STL Files/zipy_dp_cartridge.stl
                                            STL Files/zipy_dp_outer_spacer.stl
                                            STL Files/zipy_dp_inner_spacer.stl
                                            STL Files/zipy_dp_sleeve.stl

Cartridge (print open end DOWN, boss up: no supports; the flange face that
sits on the elbow comes out clean on top):
  boss        O20 x 4 below z 0, into the elbow's O21 hole; O17 through-hole
  flange      O38, z 0..4 under the bearings, 6 mm outside the body, then a
              45 deg slope up to the body (prints without support)
  body        O30, bore O22 x 20 (z 4..24) for two 608s with the outer spacer;
              flat 1.4 mm seat for the outer ring, 45 deg chamfer inside it
  6 holes     O2.7 x 6 deep on O31 every 60 deg, from the flange face, for
              M3 x 8 self-tapping through the elbow's slots
Rings:
  outer spacer  O21.5 / O19.6 x 6.0   between the outer rings (0.25 mm slack in the seat)
  inner spacer  O11.5 / O8.4  x 6.0   between the inner rings, slides on the shaft
  sleeve        O12.0 / O8.4  x 9.0   inner ring of the outer bearing to the hub collar
(printed holes come out ~0.2 small and outsides ~0.1 big: these clear anyway)
"""
import math, struct
import numpy as np

N = 120                     # segments round the axis (multiple of 6)
BORE = 22.0                 # 608 seat: FDM holes print a little small -> snug press
HOLE_D, HOLE_DEPTH, HOLE_PCD = 2.7, 6.0, 31.0
HOLE_SEG = 24


def ang(j):
    return 2 * math.pi * j / N


def revolve(profile, skip=()):
    """Closed (r, z) profile -> triangles; edges listed in skip are left open."""
    tris = []
    for k in range(len(profile)):
        if k in skip:
            continue
        (r1, z1), (r2, z2) = profile[k], profile[(k + 1) % len(profile)]
        for j in range(N):
            a0, a1 = ang(j), ang(j + 1)
            P0 = (r1 * math.cos(a0), r1 * math.sin(a0), z1); P1 = (r1 * math.cos(a1), r1 * math.sin(a1), z1)
            Q0 = (r2 * math.cos(a0), r2 * math.sin(a0), z2); Q1 = (r2 * math.cos(a1), r2 * math.sin(a1), z2)
            if r1 > 1e-9: tris.append((P0, Q0, Q1)) if r2 > 1e-9 else tris.append((P0, Q0, P1))
            if r2 > 1e-9 and r1 > 1e-9: tris.append((P0, Q1, P1))
    return tris


def zipper(inner, outer, c):
    """Triangulate between two loops that both run CCW about centre c."""
    def a(p):
        return math.atan2(p[1] - c[1], p[0] - c[0])
    # start both loops at the point with the smallest angle
    inner = inner[int(np.argmin([a(p) for p in inner])):] + inner[:int(np.argmin([a(p) for p in inner]))]
    outer = outer[int(np.argmin([a(p) for p in outer])):] + outer[:int(np.argmin([a(p) for p in outer]))]
    ni, no = len(inner), len(outer)
    def un(p, k, n):
        return a(p) + (2 * math.pi if k >= n else 0)
    tris, i, o = [], 0, 0
    while i < ni or o < no:
        ai = un(inner[(i + 1) % ni], i + 1, ni) if i < ni else 1e9
        ao = un(outer[(o + 1) % no], o + 1, no) if o < no else 1e9
        if ai <= ao:
            tris.append((inner[i % ni], outer[o % no], inner[(i + 1) % ni])); i += 1
        else:
            tris.append((inner[i % ni], outer[o % no], outer[(o + 1) % no])); o += 1
    return tris


def holed_face(r_in, r_out, z, n_holes, pcd, d_hole, radial_pts=8):
    """Flat ring at height z with n_holes round holes; returns triangles and the hole loops."""
    tris, loops = [], []
    per = N // n_holes
    for k in range(n_holes):
        j0, j1 = k * per, (k + 1) * per
        ac = (ang(j0) + ang(j1)) / 2
        c = (pcd / 2 * math.cos(ac), pcd / 2 * math.sin(ac))
        # sector boundary, CCW about the hole centre: outer arc, far radial edge,
        # inner arc backwards, near radial edge
        bnd = [(r_out * math.cos(ang(j)), r_out * math.sin(ang(j)), z) for j in range(j0, j1 + 1)]
        bnd += [(r * math.cos(ang(j1)), r * math.sin(ang(j1)), z)
                for r in np.linspace(r_out, r_in, radial_pts)[1:-1]]
        bnd += [(r_in * math.cos(ang(j)), r_in * math.sin(ang(j)), z) for j in range(j1, j0 - 1, -1)]
        bnd += [(r * math.cos(ang(j0)), r * math.sin(ang(j0)), z)
                for r in np.linspace(r_in, r_out, radial_pts)[1:-1]]
        hole = [(c[0] + d_hole / 2 * math.cos(2 * math.pi * h / HOLE_SEG),
                 c[1] + d_hole / 2 * math.sin(2 * math.pi * h / HOLE_SEG), z) for h in range(HOLE_SEG)]
        tris += zipper(hole, bnd, c)
        loops.append((c, hole))
    return tris, loops


def blind_hole(c, loop, depth):
    """Wall and flat ceiling of a blind hole going +z from loop."""
    tris, top = [], [(p[0], p[1], p[2] + depth) for p in loop]
    n = len(loop)
    for h in range(n):
        a, b, a2, b2 = loop[h], loop[(h + 1) % n], top[h], top[(h + 1) % n]
        tris += [(a, a2, b2), (a, b2, b)]
    cz = (c[0], c[1], loop[0][2] + depth)
    tris += [(cz, top[(h + 1) % n], top[h]) for h in range(n)]
    return tris


def orient(tris, outward_test):
    """Make every triangle face outward_test's way; returns an array."""
    t = np.array(tris, float)
    return t


def signed_volume(t):
    return np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6


def save(path, t, name):
    if signed_volume(t) < 0:
        t = t[:, ::-1]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
    with open(path, 'wb') as f:
        f.write(name.encode().ljust(80, b' '))
        f.write(struct.pack('<I', len(t)))
        for nn, tt in zip(n, t):
            f.write(struct.pack('<12fH', *nn, *tt.ravel(), 0))
    return t


def ring(od, id_, h):
    return np.array(revolve([(id_ / 2, 0), (od / 2, 0), (od / 2, h), (id_ / 2, h)]), float)


CART_PROFILE = [(8.5, -4), (10, -4), (10, 0), (19, 0), (19, 6), (15, 10), (15, 24),
                (BORE / 2, 24), (BORE / 2, 4), (9.6, 4), (8.5, 2.9)]


def cartridge():
    tris = revolve(CART_PROFILE, skip=(2,))      # edge 2 = flange face (10,0)->(19,0), holed below
    face, loops = holed_face(10, 19, 0.0, 6, HOLE_PCD, HOLE_D)
    # the revolve's faces point one way; build the face and holes the same way
    # (checked by the closed-mesh test), then save() fixes the overall sign
    tris += face
    for c, loop in loops:
        tris += [(a, c2, b) for a, b, c2 in blind_hole(c, loop, HOLE_DEPTH)]
    return np.array(tris, float)


if __name__ == '__main__':
    out = {}
    out['zipy_dp_cartridge'] = save('STL Files/zipy_dp_cartridge.stl', cartridge(), 'ZIPY elbow 608 cartridge')
    out['zipy_dp_outer_spacer'] = save('STL Files/zipy_dp_outer_spacer.stl', ring(21.5, 19.6, 6.0), '608 outer spacer 6.0')
    out['zipy_dp_inner_spacer'] = save('STL Files/zipy_dp_inner_spacer.stl', ring(11.5, 8.4, 6.0), '608 inner spacer 6.0')
    out['zipy_dp_sleeve'] = save('STL Files/zipy_dp_sleeve.stl', ring(12.0, 8.4, 9.0), 'elbow sleeve 9.0')
    for k, t in out.items():
        p = t.reshape(-1, 3)
        print('%-22s %5d triangles  %.1f x %.1f x %.1f mm  %.2f cm3' % (k, len(t), *(p.max(0) - p.min(0)), signed_volume(t) / 1000))
