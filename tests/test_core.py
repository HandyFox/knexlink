"""Core checks:  python tests/test_core.py   (or python -m pytest tests)"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402

from knexsuite import catalog as C  # noqa: E402
from knexsuite.demo import build_demo  # noqa: E402
from knexsuite.model import Project  # noqa: E402
from knexsuite.solver import Assembly  # noqa: E402
from knexsuite.units import fmt, parse_length  # noqa: E402


def triangle(leg, diag):
    pr = Project()
    c0 = pr.place_start("conn8")
    a, _ = pr.add_rod_at_slot(c0.id, 0, C.RODS[leg][0])
    c1, _ = pr.add_part_at_end(a.id, 1, "conn8", slot=4)
    b, _ = pr.add_rod_at_slot(c0.id, 2, C.RODS[leg][0])
    pr.add_part_at_end(b.id, 1, "conn8", slot=6)
    _, closed = pr.add_rod_at_slot(c1.id, 3, C.RODS[diag][0])
    return pr, closed


def test_units():
    assert abs(parse_length("5.04", "in") - 128.016) < 1e-6
    assert parse_length("128mm", "in") == 128.0
    assert abs(parse_length("3' 2\"", "mm") - 38 * 25.4) < 1e-9
    assert parse_length("junk") is None
    assert fmt(25.4, "in") == '1"'


def test_knex_triangles_close():
    pairs = list(zip(C.ROD_ORDER[:-1], C.ROD_ORDER[1:]))
    for leg, diag in pairs:
        pr, closed = triangle(leg, diag)
        assert closed, (leg, diag)
        gap = closed[0][4]
        assert gap < 3.0
    # the famous short red rod: yellow legs + red diagonal is a forced fit
    pr, closed = triangle("yellow", "red")
    assert closed[0][4] > 2.0
    assert any(lvl == "warn" for lvl, *_ in pr.fit_issues())


def test_rod_length_slides_parts():
    pr = Project()
    c0 = pr.place_start("conn3")
    r, _ = pr.add_rod_at_slot(c0.id, 0, 54.0)
    c1, _ = pr.add_part_at_end(r.id, 1, "conn1")
    x0 = c1.pos.copy()
    assert pr.set_rod_length(r.id, 100.0) is None
    assert np.allclose(c1.pos - x0, r.d * 46.0)
    assert r.color == "custom" and not pr.fit_issues()


def test_demo_motion():
    pr = build_demo()
    A = Assembly(pr)
    assert not pr.fit_issues()
    assert not A.analyze()
    assert A.dof() == 3                     # crank-rocker, gear pair, flap
    crank = [j for j, jt in enumerate(A.joints) if "Frame left" in jt.label][0]
    res = A.sweep(crank, math.radians(360))
    assert all(e["stop"] is None for e in res["ends"].values())   # crank turns all the way
    A.reset()
    ga = [j for j, jt in enumerate(A.joints) if "Gear frame A" in jt.label][0]
    gb = [j for j, jt in enumerate(A.joints) if "Gear frame B" in jt.label][0]
    A.drive(ga, math.radians(60))
    ratio = A.joint_angle(gb) / A.joint_angle(ga)
    assert abs(ratio + 14 / 34) < 0.01, ratio
    A.reset()
    hinge = [j for j, jt in enumerate(A.joints) if jt.hinge][0]
    stop, _ = A.drive(hinge, math.radians(-150))
    assert stop is not None and stop.key.startswith("lim:")


def test_ball_joint_and_3d():
    pr = Project()
    c0 = pr.place_start("conn7_3d")
    p = pr.add_3d_partner(c0.id, "conn4_3d")
    assert len(pr.stacks) == 1
    r, _ = pr.add_rod_at_slot(c0.id, 0, 54.0)
    ball, _ = pr.add_part_at_end(r.id, 1, "ball")
    r2, _ = pr.add_rod_at_slot(ball.id, 1, 86.0)
    tip, _ = pr.add_part_at_end(r2.id, 1, "conn1")
    A = Assembly(pr)
    assert A.dof() == 3                     # a ball joint swings 3 ways
    b = A.body_of_item(tip.id)
    x0 = A.X(b, tip.pos)
    moved, stop, _ = A.drag(b, tip.pos, x0 + np.array([0, 0, 40.0]))
    assert moved > 5
    assert A.ball_tilt([j for j, jt in enumerate(A.joints) if jt.kind == "sph"][0]) > 5


def test_save_load_roundtrip():
    pr = build_demo()
    pr2 = Project.from_json(pr.to_json())
    assert len(pr2.parts) == len(pr.parts) and len(pr2.rods) == len(pr.rods)
    assert Assembly(pr2).dof() == 3


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn()
            print("ok", k)
