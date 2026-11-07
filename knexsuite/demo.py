"""Starter model: a K'nex crank-rocker linkage, a gear pair and a hinged flap.

- Turn the crank (green arm, left): the coupler (grey rod) swings the rocker,
  whose arm is a CUSTOM-length rod cut to fit.
- The blue 14T gear drives the red 34T gear on the right (2.43 : 1).
- The flap on top swings on a hinge.
"""
import numpy as np

from . import catalog as C
from .model import Project

ROD = {k: v[0] for k, v in C.RODS.items()}


def _mount_pointing(pr, rod, part_type, t, target, lock=True):
    r = pr.rods[rod.id]
    at = r.p0 + r.d * t
    roll = pr.roll_toward(rod.id, at, target)
    m = pr.mount_on_rod(rod.id, part_type, t=t, roll=roll)
    pr.set_hole_lock(m.id, rod.id, lock)
    return m


def build_demo():
    pr = Project()
    # ---- ground frame: two white connectors joined by a grey rod (slots in the XZ plane)
    g1 = pr.place_start("conn8")
    g1.params["name"] = "Frame left"
    base, _ = pr.add_rod_at_slot(g1.id, 0, ROD["grey"])
    g2, _ = pr.add_part_at_end(base.id, 1, "conn8", slot=4)
    g2.params["name"] = "Frame right"
    # feet so it stands on the table
    for gc in (g1, g2):
        leg, _ = pr.add_rod_at_slot(gc.id, 6, ROD["blue"])
        foot, _ = pr.add_part_at_end(leg.id, 1, "conn5", slot=2)

    # ---- crank: axle through Frame left, hub locked on the axle, green arm up
    axle1 = pr.add_rod_through_hole(g1.id, 0, ROD["white"])
    a1 = pr.rods[axle1.id]
    hub1 = _mount_pointing(pr, axle1, "conn2s", t=32.0, target=a1.p0 + a1.d * 32.0 + np.array([0, 0, 50.0]))
    hub1.params["name"] = "Crank hub"
    arm, _ = pr.add_rod_at_slot(hub1.id, 0, ROD["green"])
    arm.params["name"] = "Crank arm"
    c1, _ = pr.add_part_at_end(arm.id, 1, "conn1")
    c1.params["name"] = "Crank pin block"
    pin1 = pr.add_rod_through_hole(c1.id, 0, ROD["green"])
    p1 = pr.rods[pin1.id]

    # ---- coupler: connector locked on the pin, grey rod at 45 deg up-right
    c2 = _mount_pointing(pr, pin1, "conn3", t=16.0,
                         target=p1.p0 + p1.d * 16.0 + np.array([50.0, 0, 0]))
    c2.params["name"] = "Coupler block"
    coupler, _ = pr.add_rod_at_slot(c2.id, 1, ROD["grey"])
    coupler.params["name"] = "Coupler"
    c4, _ = pr.add_part_at_end(coupler.id, 1, "conn1")
    c4.params["name"] = "Coupler end"
    pin2 = pr.add_rod_through_hole(c4.id, 0, ROD["white"])
    pr.set_hole_lock(c4.id, pin2.id, True)
    p2 = pr.rods[pin2.id]
    target = p2.p0 + p2.d * 8.0          # rocker sits in the crank's plane

    # ---- rocker: axle through Frame right, hub aimed at the pin, custom-length arm
    axle2 = pr.add_rod_through_hole(g2.id, 0, ROD["white"])
    a2 = pr.rods[axle2.id]
    hub_at = a2.p0 + a2.d * 32.0
    hub2 = _mount_pointing(pr, axle2, "conn2s", t=32.0, target=target)
    hub2.params["name"] = "Rocker hub"
    L = float(np.linalg.norm(target - hub_at)) - 2 * C.REACH
    rocker, _ = pr.add_rod_at_slot(hub2.id, 0, round(L, 2))
    rocker.params["name"] = "Rocker arm (custom)"
    c3, _ = pr.add_part_at_end(rocker.id, 1, "conn1")
    c3.params["name"] = "Rocker pin block"

    # ---- gear pair on its own little frame, off to the right
    f1 = pr.place_start("conn5", at=(340.0, 0.0, 0.0))
    f1.params["ground"] = True
    f1.params["name"] = "Gear frame A"
    link, _ = pr.add_rod_at_slot(f1.id, 2, ROD["green"])
    f2, _ = pr.add_part_at_end(link.id, 1, "conn5", slot=0)
    f2.params["name"] = "Gear frame B"
    leg, _ = pr.add_rod_at_slot(f1.id, 0, ROD["white"])
    sh1 = pr.add_rod_through_hole(f1.id, 0, ROD["blue"])
    sh2 = pr.add_rod_through_hole(f2.id, 0, ROD["blue"])
    gA = pr.mount_on_rod(sh1.id, "gear14", t=ROD["blue"] / 2 + 10)
    gB = pr.mount_on_rod(sh2.id, "gear34", t=ROD["blue"] / 2 + 10)
    crank_h = pr.mount_on_rod(sh1.id, "conn1", t=ROD["blue"] / 2 - 12)
    pr.set_hole_lock(crank_h.id, sh1.id, True)
    crank_h.params["name"] = "Gear crank handle"
    h_rod, _ = pr.add_rod_at_slot(crank_h.id, 0, ROD["green"])

    # ---- hinged flap on top of Frame right
    up, _ = pr.add_rod_at_slot(g2.id, 2, ROD["white"])
    hg, _ = pr.add_part_at_end(up.id, 1, "hinge", slot=0, params={"angle": 180.0})
    hg.params["name"] = "Flap hinge"
    flap, _ = pr.add_rod_at_slot(hg.id, 1, ROD["blue"])
    flap.params["name"] = "Flap"
    tip, _ = pr.add_part_at_end(flap.id, 1, "conn3")
    return pr
