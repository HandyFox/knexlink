"""Core checks:  python -m pytest tests   (or just: python tests/test_core.py)"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from linkage.demo import build_demo
from linkage.kinematics import Mechanism
from linkage.model import Project, mirror_copy
from linkage.units import parse_length, fmt_ft_in


def named(pr, name):
    return [p for p in pr.parts.values() if p.name == name][0]


def test_units():
    assert parse_length('38.5') == 38.5
    assert parse_length('3\' 2.5"') == 38.5
    assert parse_length('3ft 2.5in') == 38.5
    assert parse_length('1.5ft') == 18.0
    assert parse_length('-2"') == -2.0
    assert parse_length('abc') is None
    assert fmt_ft_in(38.5) == '3\' 2.5"'
    assert fmt_ft_in(5.25) == '5.25"'


def test_demo_sweeps_clean():
    pr = build_demo()
    m = Mechanism(pr)
    q = m.q0.copy()
    assert not [i for i in m.analyze(q) if i.level == "bind"]
    for d in m.handle_dofs:
        assert m.free_motions(d) == 0
    rod = m.dof_of[named(pr, "Cross rod (left lever)").id]
    res = m.sweep(q, rod)
    for e in res["ends"].values():
        assert e["stop"] is None            # reaches the handle stops
    # concentric: moving the rod leaves the tube alone
    tube = m.dof_of[named(pr, "Cross tube (right lever)").id]
    assert abs(res["ends"][1]["q"][tube]) < 1e-9


def test_lockup_dead_center():
    pr = build_demo()
    sh = named(pr, "Pump R control shaft")
    sh["min_angle"], sh["max_angle"] = -180.0, 180.0
    named(pr, "Pump R arm")["length"] = 1.5      # too short for the tube arm's throw
    pr.settings["stop_on_collision"] = False
    m = Mechanism(pr)
    tube = m.dof_of[named(pr, "Cross tube (right lever)").id]
    res = m.sweep(m.q0.copy(), tube)
    stops = [e["stop"] for e in res["ends"].values()]
    assert all(s is not None and s.key == "lockup" for s in stops), stops
    print("lockup:", stops[0].text)


def test_collision():
    pr = build_demo()
    # a frame bar right in the left lever's path, 6" behind it at grip height
    pr.add("frame", name="Seat edge", start=[-12.0, 3.0, 30.0], end=[0.0, 3.0, 30.0], size=2.0)
    m = Mechanism(pr)
    rod = m.dof_of[named(pr, "Cross rod (left lever)").id]
    res = m.sweep(m.q0.copy(), rod)
    st = res["ends"][1]["stop"]
    assert st is not None and st.key.startswith("hit:"), st
    ang = math.degrees(res["ends"][1]["q"][rod])
    print("hit:", st.text, "at %.1f deg" % ang)
    assert 5 < ang < 30


def test_rod_end_tilt():
    pr = build_demo()
    # move the bellcrank sideways so the rod from the cross shaft runs at a steep angle
    bc = named(pr, "Bellcrank pivot")
    bc["start"], bc["end"] = [6.0, 4.0, 6.0], [6.0, 4.0, 10.0]
    m = Mechanism(pr)
    iss = m.analyze(m.q0.copy())
    print("tilt:", [i.text for i in iss if "tilt" in i.key])
    assert any(i.key.startswith("tilt:") for i in iss)


def test_mirror_and_save():
    pr = build_demo()
    n0 = len(pr.parts)
    new = mirror_copy(pr, [named(pr, "Pump R control shaft").id])
    assert len(new) == 2
    arm = [pr.parts[i] for i in new if pr.parts[i].kind == "arm"][0]
    from linkage.model import arm_tip_neutral
    t_old = arm_tip_neutral(pr, named(pr, "Pump R arm"))
    t_new = arm_tip_neutral(pr, arm)
    assert np.allclose(t_new, [-t_old[0], t_old[1], t_old[2]])
    pr2 = Project.from_json(pr.to_json())
    assert len(pr2.parts) == n0 + 2
    Mechanism(pr2)


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn()
            print("ok", k)
