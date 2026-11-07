"""K'nex Classic part catalog. All sizes in millimeters.

Rod lengths are the published Classic sizes (green 16 mm ... grey 190 mm).
REACH (connector center to the end of a clicked-in rod) was fitted so that the
standard rods form the K'nex 45-degree triangles: center-to-center spacing goes up by
about the square root of 2 from one rod color to the next. With this value every
standard diagonal closes within 0.6 mm, except yellow/red, which is off by about
2.7 mm - the well-known "red rod is a bit short" K'nex quirk.

Sizes marked APPROX were not published; they are close and can be edited per part.
"""
import math

import numpy as np

ROD_DIA = 6.35            # 1/4"
REACH = 10.93             # connector center -> rod end
CONN_T = 6.4              # connector thickness (APPROX)
SPACER_T = CONN_T / 2     # light blue spacer = half a connector width
GEAR_MODULE = 1.6         # mm per tooth of pitch diameter (fits 82T crown = ~130 mm)
SNAP_TOL = 3.0            # how far a rod end can be off and still click in (mm)
FORCED_FIT = 0.6          # gaps over this are reported as forced fits

ROD_ORDER = ["green", "white", "blue", "yellow", "red", "grey"]
RODS = {
    "green": (16.0, "#36a852"),
    "white": (32.0, "#efefef"),
    "blue": (54.0, "#1f63c6"),
    "yellow": (86.0, "#f3c300"),
    "red": (128.0, "#d5262e"),
    "grey": (190.0, "#8a8d91"),
}
CUSTOM_ROD_COLOR = "#ff8a1a"


def rod_color_for(length):
    for k, (L, _) in RODS.items():
        if abs(L - length) < 0.05:
            return k
    return "custom"


def rod_rgb(key):
    return RODS[key][1] if key in RODS else CUSTOM_ROD_COLOR


def slot_dir(index):
    a = math.radians(45.0 * index)
    return np.array([math.cos(a), math.sin(a), 0.0])


# ---------------------------------------------------------------- part types
# family:
#   connector - flat hub with rod slots in its plane and a center hole (axis = local z)
#   joint     - two halves that move against each other (hinge / ball joint)
#   mount     - slides onto a rod (local z = rod axis): spacers, gears, wheels, clips
#
# slots:  list of (index, half)   direction = slot_dir(index) unless the type says otherwise
# holes:  list of (local center, local axis)
TYPES = {
    "conn1": dict(short="1-way", name="1-way connector", color="#4b4f54", family="connector", slots=[0]),
    "conn2s": dict(short="2-way straight", name="2-way straight (orange)", color="#f07c16", family="connector", slots=[0, 4]),
    "conn2": dict(short="2-way 45", name="2-way 45 deg (light grey)", color="#c4c7cb", family="connector", slots=[0, 1]),
    "conn3": dict(short="3-way", name="3-way 90 deg (red)", color="#d5262e", family="connector", slots=[0, 1, 2]),
    "conn4": dict(short="4-way", name="4-way 135 deg (green)", color="#36a852", family="connector", slots=[0, 1, 2, 3]),
    "conn5": dict(short="5-way", name="5-way 180 deg (yellow)", color="#f3c300", family="connector",
                  slots=[0, 1, 2, 3, 4]),
    "conn8": dict(short="8-way", name="8-way (white)", color="#f7f7f7", family="connector",
                  slots=[0, 1, 2, 3, 4, 5, 6, 7]),
    "conn7_3d": dict(short="7-way 3D", name="7-way 3D (blue)", color="#1e88e5", family="connector",
                     slots=[0, 1, 2, 3, 4, 5, 6], is3d=True),
    "conn4_3d": dict(short="4-way 3D", name="4-way 3D (purple)", color="#8e24aa", family="connector",
                     slots=[0, 1, 2, 3], is3d=True),
    "hinge": dict(short="Hinge", name="Hinge (2 pieces)", color="#1e63c6", color2="#30343a", family="joint",
                  slots=[0, 1], joint="rev", min_angle=45.0, max_angle=315.0),
    "ball": dict(short="Ball joint", name="Ball joint", color="#30343a", color2="#9aa0a6", family="joint",
                 slots=[0, 1], joint="sph", cone=40.0),
    "spacer": dict(short="Spacer", name="Spacer (light blue)", color="#64b5f6", family="mount", thick=SPACER_T,
                   od=10.0, lock=True),
    "spacer3": dict(short="Spacer 3x", name="Spacer (grey, 3x)", color="#9e9e9e", family="mount", thick=3 * SPACER_T,
                    od=10.0, lock=True),
    "gear14": dict(short="Gear 14T", name="Gear 14T (blue)", color="#1e63c6", family="mount", teeth=14,
                   thick=CONN_T, lock=True),
    "gear34": dict(short="Gear 34T", name="Gear 34T (red)", color="#d5262e", family="mount", teeth=34,
                   thick=CONN_T, lock=True),
    "gear82": dict(short="Crown 82T", name="Crown gear 82T (yellow)", color="#f3c300", family="mount", teeth=82,
                   thick=CONN_T, lock=True, crown=True),
    "wheel_s": dict(short="Wheel S", name="Wheel small", color="#2b2b2b", family="mount", od=52.0, width=17.0,
                    lock=True, wheel=True),           # APPROX
    "wheel_m": dict(short="Wheel M", name="Wheel medium", color="#2b2b2b", family="mount", od=76.0, width=20.0,
                    lock=True, wheel=True),           # APPROX
    "wheel_l": dict(short="Wheel L", name="Wheel large", color="#2b2b2b", family="mount", od=106.0, width=24.0,
                    lock=True, wheel=True),           # APPROX
    "clip": dict(short="Clip", name="Rod clip (1 slot)", color="#c8a46e", family="mount", lock=False,
                 slots=[0], reach=9.5),
    "cliphole": dict(short="Clip + hole", name="Clip with hole", color="#8e24aa", family="mount", lock=False,
                     hole2=(np.array([12.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))),
}

CONNECTOR_ORDER = ["conn1", "conn2s", "conn2", "conn3", "conn4", "conn5", "conn8", "conn7_3d",
                   "conn4_3d"]
JOINT_ORDER = ["hinge", "ball"]
MOUNT_ORDER = ["spacer", "spacer3", "gear14", "gear34", "gear82", "wheel_s", "wheel_m", "wheel_l",
               "clip", "cliphole"]


def tdef(type_key):
    return TYPES[type_key]


def pitch_radius(type_key):
    t = TYPES[type_key]
    return GEAR_MODULE * t["teeth"] / 2 if "teeth" in t else None


def slot_reach(type_key):
    return TYPES[type_key].get("reach", REACH)


def local_slots(type_key, params):
    """[(slot index, half, local point, local outward direction)] for a part type."""
    t = TYPES[type_key]
    out = []
    reach = slot_reach(type_key)
    if t["family"] == "joint":
        out.append((0, "A", reach * slot_dir(0), slot_dir(0)))
        if t["joint"] == "rev":
            a = math.radians(params.get("angle", 180.0))
            d = np.array([math.cos(a), math.sin(a), 0.0])
        else:
            tilt = math.radians(params.get("tilt", 0.0))
            turn = math.radians(params.get("turn", 0.0))
            # straight through is -x; tilt swings it, turn spins the tilt direction
            d = np.array([-math.cos(tilt), math.sin(tilt) * math.cos(turn),
                          math.sin(tilt) * math.sin(turn)])
        out.append((1, "B", reach * d, d))
        return out
    for s in t.get("slots", []):
        d = slot_dir(s)
        out.append((s, None, reach * d, d))
    return out


def local_holes(type_key):
    """[(hole index, local center, local axis)]"""
    t = TYPES[type_key]
    if t["family"] == "joint":
        return []
    holes = [(0, np.zeros(3), np.array([0.0, 0.0, 1.0]))]
    if "hole2" in t:
        c, a = t["hole2"]
        holes.append((1, c, a))
    return holes


def missing_slot_dir(type_key):
    """For 3D connectors: the open side where a partner slides in (local)."""
    t = TYPES[type_key]
    used = set(t["slots"])
    for s in range(8):
        if s not in used:
            return slot_dir(s)
    return slot_dir(7)
