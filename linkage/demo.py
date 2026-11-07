"""Starter project: Case-style twin drive levers on a rod-inside-a-tube cross shaft.

Left lever  -> welded to the inner ROD.  The rod runs through the tube to the
               right side, then turns a bellcrank (corner) that pushes a valve spool.
Right lever -> welded to the outer TUBE. Its arm pushes a pump control arm.
Grab either grip: the rod and tube twist independently.
"""
from .model import Project, fit_links


def build_demo():
    pr = Project()
    rod = pr.add("shaft", name="Cross rod (left lever)", style="Solid rod",
                 start=[-10.0, 10.0, 12.0], end=[26.0, 10.0, 12.0], dia=1.0,
                 min_angle=-30.0, max_angle=30.0)
    tube = pr.add("shaft", name="Cross tube (right lever)", style="Tube",
                  start=[2.0, 10.0, 12.0], end=[20.0, 10.0, 12.0], dia=1.75,
                  min_angle=-30.0, max_angle=30.0)
    pr.add("arm", name="Left lever", shaft=rod.id, at=3.0, length=22.0, angle=0.0,
           width=1.25, thick=0.375, grip=True, grip_len=5.0, grip_dia=1.25)
    pr.add("arm", name="Right lever", shaft=tube.id, at=5.0, length=22.0, angle=0.0,
           width=1.25, thick=0.375, grip=True, grip_len=5.0, grip_dia=1.25)
    tube_arm = pr.add("arm", name="Tube output arm", shaft=tube.id, at=16.0, length=4.0,
                      angle=180.0, width=1.0, thick=0.25)
    rod_arm = pr.add("arm", name="Rod output arm", shaft=rod.id, at=34.0, length=4.0,
                     angle=180.0, width=1.0, thick=0.25)

    pump = pr.add("shaft", name="Pump R control shaft", style="Solid rod",
                  start=[17.0, -14.0, 8.0], end=[19.0, -14.0, 8.0], dia=0.875,
                  min_angle=-45.0, max_angle=45.0)
    pump_arm = pr.add("arm", name="Pump R arm", shaft=pump.id, at=1.0, length=3.0,
                      angle=0.0, width=1.0, thick=0.25)
    pr.add("link", name="Drive rod R", a=tube_arm.id, b=pump_arm.id, dia=0.5)

    bc = pr.add("shaft", name="Bellcrank pivot", style="Solid rod",
                start=[21.0, -6.0, 6.0], end=[21.0, -6.0, 10.0], dia=0.75)
    bc_a = pr.add("arm", name="Bellcrank arm A", shaft=bc.id, at=2.0, length=3.0,
                  angle=-90.0, width=1.0, thick=0.25)
    bc_b = pr.add("arm", name="Bellcrank arm B", shaft=bc.id, at=2.0, length=3.0,
                  angle=180.0, width=1.0, thick=0.25)
    pr.add("link", name="Rod to bellcrank", a=rod_arm.id, b=bc_a.id, dia=0.5)

    spool = pr.add("slider", name="Valve spool", start=[8.0, -9.0, 8.0],
                   end=[14.0, -9.0, 8.0], neutral=3.0, block=1.0)
    pr.add("link", name="Bellcrank to spool", a=bc_b.id, b=spool.id, dia=0.5)

    pr.add("frame", name="Left side rail", start=[-26.0, -20.0, 3.0],
           end=[-26.0, 18.0, 3.0], size=2.5)
    pr.add("frame", name="Right side rail", start=[30.0, -20.0, 3.0],
           end=[30.0, 18.0, 3.0], size=2.5)
    fit_links(pr)
    return pr
