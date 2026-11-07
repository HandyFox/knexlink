"""3D shapes for every part, used by the 3D view and STL export.

Shapes are built in the part's local frame and placed at its design pose.
They are recognizable stand-ins, not factory CAD: rods and slot positions are
exact, claw and hub details are approximate.
"""
import math
from functools import lru_cache

import numpy as np
import pyvista as pv

from . import catalog as C
from . import geometry as g

HOLE_COLOR = "#26292d"

# K'nex rod end profile for printing custom rods (APPROX - test-fit before a big print run)
END_GROOVE_FROM_TIP = 1.9
END_GROOVE_WIDTH = 1.7
END_GROOVE_DIA = 4.7
END_CHAMFER = 0.6


def _box(center, size, angle_deg=0.0):
    m = pv.Cube(center=(0, 0, 0), x_length=size[0], y_length=size[1], z_length=size[2])
    a = math.radians(angle_deg)
    M = g.mat4(g.rot_matrix(np.array([0, 0, 1.0]), a), np.zeros(3))
    m = m.transform(M, inplace=False)
    return m.translate(center, inplace=False)


def _prism(points2d, z0, z1):
    pts = np.array([[x, y, z0] for x, y in points2d])
    n = len(pts)
    poly = pv.PolyData(pts, faces=np.hstack([[n], np.arange(n)]))
    out = poly.extrude((0, 0, z1 - z0), capping=True)
    return out.triangulate().clean()


def _cyl(center, direction, radius, height, res=32):
    return pv.Cylinder(center=center, direction=direction, radius=radius, height=height,
                       resolution=res, capping=True).triangulate()


@lru_cache(maxsize=None)
def _connector_local(type_key, params_key):
    params = dict(params_key)
    t = C.TYPES[type_key]
    H = C.CONN_T
    reach = C.slot_reach(type_key)
    body = []
    slots = C.local_slots(type_key, params)
    body.append(_cyl((0, 0, 0), (0, 0, 1), 6.0, H))
    for idx, half, lp, ld in slots:
        ang = math.degrees(math.atan2(ld[1], ld[0]))
        r0, r1 = 3.0, reach + 3.2
        body.append(_box(ld * (r0 + r1) / 2, (r1 - r0, 8.6, H), ang))
    idxs = [s[0] for s in slots]
    if len(idxs) > 1 and type_key != "conn2s":
        a0 = math.radians(45 * min(idxs))
        a1 = math.radians(45 * max(idxs))
        if len(idxs) == 8:
            a1 = a0 + 2 * math.pi
        n = max(6, int((a1 - a0) / math.radians(7.5)))
        outer = [((reach - 1.5) * math.cos(a0 + (a1 - a0) * k / n),
                  (reach - 1.5) * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
        inner = [(4.0 * math.cos(a0 + (a1 - a0) * k / n), 4.0 * math.sin(a0 + (a1 - a0) * k / n))
                 for k in range(n, -1, -1)]
        if len(idxs) == 8:
            body.append(_cyl((0, 0, 0), (0, 0, 1), reach - 1.5, H * 0.55, 48))
        else:
            body.append(_prism(outer + inner, -H * 0.275, H * 0.275))
    main = pv.merge(body)
    hole = _cyl((0, 0, 0), (0, 0, 1), C.ROD_DIA / 2 + 0.2, H + 0.3, 20)
    return [(main, t["color"], None), (hole, HOLE_COLOR, None)]


@lru_cache(maxsize=None)
def _joint_local(type_key, params_key):
    params = dict(params_key)
    t = C.TYPES[type_key]
    H = C.CONN_T
    out = []
    slots = C.local_slots(type_key, params)
    for idx, half, lp, ld in slots:
        col = t["color"] if half == "A" else t["color2"]
        r1 = C.REACH + 3.2
        r0 = 4.5
        claw = pv.Cube(center=(0, 0, 0), x_length=r1 - r0, y_length=8.0, z_length=H * 0.9).triangulate()
        M = g.mat4(g.frame_from(ld, np.array([0, 0, 1.0])), ld * (r0 + r1) / 2)
        claw = claw.transform(M, inplace=False)
        if t["joint"] == "rev":
            zc = H * 0.3 if half == "A" else -H * 0.3
            knuckle = _cyl((0, 0, zc), (0, 0, 1), 6.0, H * 0.55)
            out.append((pv.merge([claw, knuckle]), col, half))
        else:
            if half == "A":
                cup = pv.Sphere(radius=6.0, center=(0, 0, 0), theta_resolution=24, phi_resolution=24)
                out.append((pv.merge([claw, cup.triangulate()]), col, half))
            else:
                ball = pv.Sphere(radius=4.6, center=ld * 1.5, theta_resolution=24, phi_resolution=24)
                out.append((pv.merge([claw, ball.triangulate()]), col, half))
    if t["joint"] == "rev":
        pin = _cyl((0, 0, 0), (0, 0, 1), 2.0, H * 1.3, 16)
        out.append((pin, HOLE_COLOR, "A"))
    return out


def _gear_profile(teeth, m):
    rp = m * teeth / 2
    ra, rf = rp + m, rp - 1.25 * m
    pts = []
    for k in range(teeth):
        a = 2 * math.pi * k / teeth
        p = 2 * math.pi / teeth
        for frac, r in ((0.0, rf), (0.18, rf), (0.32, ra), (0.68, ra), (0.82, rf)):
            aa = a + frac * p
            pts.append((r * math.cos(aa), r * math.sin(aa)))
    return pts


def _spur(teeth, m, H):
    """Spur gear: a disc plus one small convex prism per tooth (clean triangles)."""
    rp = m * teeth / 2
    ra, rf = rp + m, rp - 1.25 * m
    parts = [_cyl((0, 0, 0), (0, 0, 1), rf + 0.2, H, max(32, teeth * 2))]
    pitch = 2 * math.pi / teeth
    for k in range(teeth):
        a = k * pitch
        pts = []
        for frac, r in ((0.05, rf), (0.22, ra), (0.48, ra), (0.65, rf)):
            aa = a + frac * pitch
            pts.append((r * math.cos(aa), r * math.sin(aa)))
        parts.append(_prism(pts, -H / 2, H / 2))
    return pv.merge(parts).clean()


@lru_cache(maxsize=None)
def _mount_local(type_key, params_key):
    params = dict(params_key)
    t = C.TYPES[type_key]
    col = t["color"]
    out = []
    if "teeth" in t:
        m = C.GEAR_MODULE
        n = t["teeth"]
        H = t["thick"]
        if t.get("crown"):
            rp = m * n / 2
            disc = _cyl((0, 0, 0), (0, 0, 1), rp + 2.0, H * 0.5, 96)
            teeth = []
            for k in range(n):
                a = 360.0 * k / n
                d = np.array([math.cos(math.radians(a)), math.sin(math.radians(a)), 0])
                teeth.append(_box(d * rp + np.array([0, 0, H * 0.25 + 1.4]),
                                  (4.0, math.pi * m * 0.5, 2.8), a))
            spokes = [_box((0, 0, 0), (2 * rp, 6.0, H * 0.45), a) for a in (0, 60, 120)]
            out.append((pv.merge([disc] + teeth + spokes), col, None))
        else:
            out.append((_spur(n, m, H), col, None))
        out.append((_cyl((0, 0, 0), (0, 0, 1), 6.5, H + 1.0, 24), col, None))
    elif t.get("wheel"):
        od = params.get("od", t["od"])
        w = t["width"]
        out.append((_cyl((0, 0, 0), (0, 0, 1), od / 2, w, 48), "#1f1f1f", None))
        out.append((_cyl((0, 0, 0), (0, 0, 1), od * 0.34, w + 1.0, 40), "#c9ccd1", None))
    elif type_key in ("spacer", "spacer3"):
        out.append((_cyl((0, 0, 0), (0, 0, 1), t["od"] / 2, t["thick"], 24), col, None))
    elif type_key == "clip":
        ring = _cyl((0, 0, 0), (0, 0, 1), 5.6, 6.5, 24)
        r1 = C.slot_reach(type_key) + 3.0
        claw = _box((r1 / 2 + 1.5, 0, 0), (r1 - 3.0, 7.5, 6.0))
        out.append((pv.merge([ring, claw]), col, None))
    elif type_key == "cliphole":
        c2, a2 = t["hole2"]
        ring = _cyl((0, 0, 0), (0, 0, 1), 5.6, 6.5, 24)
        arm = _box((c2[0] / 2, 0, 0), (c2[0], 6.0, 5.0))
        ring2 = _cyl(tuple(c2), tuple(a2), 5.6, 6.0, 24)
        out.append((pv.merge([ring, arm, ring2]), col, None))
        out.append((_cyl(tuple(c2), tuple(a2), C.ROD_DIA / 2 + 0.2, 6.3, 16), HOLE_COLOR, None))
    return out


def _params_key(part):
    keys = ("angle", "tilt", "turn", "od")
    return tuple((k, round(float(part.params[k]), 4)) for k in keys if k in part.params)


def part_meshes(part):
    """[(mesh in design-world coords, color, half)] for a part."""
    fam = part.tdef["family"]
    pk = _params_key(part)
    if fam == "connector":
        local = _connector_local(part.type, pk)
    elif fam == "joint":
        local = _joint_local(part.type, pk)
    else:
        local = _mount_local(part.type, pk)
    M = g.mat4(part.R, part.pos)
    return [(m.transform(M, inplace=False), col, half) for m, col, half in local]


def rod_mesh(rod, detail=False):
    """A rod in design-world coords. detail=True adds the K'nex end grooves (for printing)."""
    if not detail:
        return _cyl(rod.p0 + rod.d * rod.length / 2, rod.d, C.ROD_DIA / 2, rod.length, 24)
    prof = rod_profile(rod.length)
    line = pv.PolyData(np.array([[r, 0.0, z] for z, r in prof]))
    line.lines = np.hstack([[len(prof)], np.arange(len(prof))])
    solid = line.extrude_rotate(resolution=48, capping=True).triangulate().clean()
    M = g.mat4(g.frame_from(g.perp(rod.d), rod.d), rod.p0)
    return solid.transform(M, inplace=False)


def rod_profile(length):
    """(z, radius) outline of a printable rod from z=0 to z=length, closed on the axis."""
    R = C.ROD_DIA / 2
    rg = END_GROOVE_DIA / 2
    c = END_CHAMFER
    a = END_GROOVE_FROM_TIP
    w = END_GROOVE_WIDTH
    one = [(0.0, 0.0), (0.0, R - c), (c, R), (a, R), (a + 0.4, rg), (a + w - 0.4, rg), (a + w, R)]
    other = [(length - z, r) for z, r in reversed(one)]
    return one + other


def custom_rod_solid(length):
    """Printable rod lying along +Z from the origin."""
    prof = rod_profile(length)
    line = pv.PolyData(np.array([[r, 0.0, z] for z, r in prof]))
    line.lines = np.hstack([[len(prof)], np.arange(len(prof))])
    return line.extrude_rotate(resolution=64, capping=True).triangulate().clean()
