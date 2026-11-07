"""Parts kit and project data.

Think K'nex: every part is a simple piece you size however you like.

  Shaft / Tube   - turns on its own axis, held in bearings on the frame.
                   A rod can run through a tube; each twists on its own.
  Arm            - a crank welded to a shaft. Its tip has a rod-end hole.
  Handle lever   - an arm with a grip on the end. Grab the grip and pull.
  Push rod       - a straight rod with ball rod-ends on both ends.
  Slider         - something that moves in a straight line (valve spool).
  Frame bar      - fixed structure, used for interference checks.

All lengths are inches. Angles are degrees.
"""
import copy
import json
import math

import numpy as np

from . import geometry as g

# Field types used by the property editor
TEXT, LEN, ANGLE, VEC, BOOL, CHOICE, REF, NUM = (
    "text", "len", "angle", "vec", "bool", "choice", "ref", "num")


class Part:
    kind = "part"
    title = "Part"
    FIELDS = []          # (key, label, type, extra)
    DEFAULTS = {}

    def __init__(self, pid, **kw):
        self.id = pid
        self.data = copy.deepcopy(self.DEFAULTS)
        self.data.update(kw)

    def __getitem__(self, k):
        return self.data[k]

    def __setitem__(self, k, v):
        self.data[k] = v

    def get(self, k, default=None):
        return self.data.get(k, default)

    @property
    def name(self):
        return self.data.get("name", self.id)

    def type_label(self):
        return self.title

    def to_dict(self):
        return {"id": self.id, "kind": self.kind, **copy.deepcopy(self.data)}


class Shaft(Part):
    kind = "shaft"
    title = "Shaft / Tube"
    FIELDS = [
        ("name", "Name", TEXT, None),
        ("style", "Style", CHOICE, ["Solid rod", "Tube"]),
        ("start", "Start point", VEC, None),
        ("end", "End point", VEC, None),
        ("dia", "Diameter", LEN, None),
        ("min_angle", "Stop (min)", ANGLE, None),
        ("max_angle", "Stop (max)", ANGLE, None),
        ("collide", "Check hits", BOOL, None),
    ]
    DEFAULTS = {"name": "Shaft", "style": "Solid rod", "start": [-12.0, 0.0, 20.0],
                "end": [12.0, 0.0, 20.0], "dia": 1.0, "min_angle": -180.0,
                "max_angle": 180.0, "collide": True}

    def type_label(self):
        return "Tube" if self.data.get("style") == "Tube" else "Shaft"

    def axis(self):
        s, e = g.vec(self["start"]), g.vec(self["end"])
        return s, g.unit(e - s), g.norm(e - s)


class Arm(Part):
    kind = "arm"
    title = "Arm"
    FIELDS = [
        ("name", "Name", TEXT, None),
        ("shaft", "On shaft", REF, ("shaft",)),
        ("at", "Distance from shaft start", LEN, None),
        ("length", "Length (center to hole)", LEN, None),
        ("angle", "Angle (0 = up)", ANGLE, None),
        ("width", "Bar width", LEN, None),
        ("thick", "Bar thickness", LEN, None),
        ("grip", "Has handle grip", BOOL, None),
        ("grip_len", "Grip length", LEN, None),
        ("grip_dia", "Grip diameter", LEN, None),
        ("collide", "Check hits", BOOL, None),
    ]
    DEFAULTS = {"name": "Arm", "shaft": None, "at": 2.0, "length": 4.0, "angle": 0.0,
                "width": 1.0, "thick": 0.25, "grip": False, "grip_len": 5.0,
                "grip_dia": 1.25, "collide": True}

    def type_label(self):
        return "Handle lever" if self.data.get("grip") else "Arm"


class Link(Part):
    kind = "link"
    title = "Push rod"
    FIELDS = [
        ("name", "Name", TEXT, None),
        ("a", "End A connects to", REF, ("arm", "slider")),
        ("b", "End B connects to", REF, ("arm", "slider")),
        ("auto_fit", "Auto length (fits at neutral)", BOOL, None),
        ("length", "Length (hole to hole)", LEN, None),
        ("dia", "Rod diameter", LEN, None),
        ("max_misalign", "Rod-end max tilt", ANGLE, None),
        ("collide", "Check hits", BOOL, None),
    ]
    DEFAULTS = {"name": "Push rod", "a": None, "b": None, "auto_fit": True, "length": 10.0,
                "dia": 0.5, "max_misalign": 20.0, "collide": True}


class Slider(Part):
    kind = "slider"
    title = "Slider (spool)"
    FIELDS = [
        ("name", "Name", TEXT, None),
        ("start", "Track start", VEC, None),
        ("end", "Track end", VEC, None),
        ("neutral", "Neutral position from start", LEN, None),
        ("block", "Block size", LEN, None),
        ("collide", "Check hits", BOOL, None),
    ]
    DEFAULTS = {"name": "Slider", "start": [0.0, -10.0, 10.0], "end": [6.0, -10.0, 10.0],
                "neutral": 3.0, "block": 1.0, "collide": True}

    def track(self):
        s, e = g.vec(self["start"]), g.vec(self["end"])
        return s, g.unit(e - s), g.norm(e - s)


class Frame(Part):
    kind = "frame"
    title = "Frame bar"
    FIELDS = [
        ("name", "Name", TEXT, None),
        ("start", "Start point", VEC, None),
        ("end", "End point", VEC, None),
        ("size", "Square size", LEN, None),
        ("collide", "Check hits", BOOL, None),
    ]
    DEFAULTS = {"name": "Frame bar", "start": [-20.0, -20.0, 0.0], "end": [-20.0, 20.0, 0.0],
                "size": 2.0, "collide": True}


KINDS = {c.kind: c for c in (Shaft, Arm, Link, Slider, Frame)}

DEFAULT_SETTINGS = {
    "stop_on_collision": True,
    "stop_on_overangle": True,
    "warn_push": 0.50,     # push efficiency below this = warning (about 30 deg off)
    "bind_push": 0.17,     # below this = dead center (about 10 deg off)
}


class Project:
    def __init__(self):
        self.parts = {}            # id -> Part, insertion ordered
        self.settings = dict(DEFAULT_SETTINGS)
        self._next = 1

    # ---------- ids / access ----------
    def new_id(self):
        while f"p{self._next}" in self.parts:
            self._next += 1
        pid = f"p{self._next}"
        self._next += 1
        return pid

    def add(self, kind, **kw):
        part = KINDS[kind](self.new_id(), **kw)
        self.parts[part.id] = part
        return part

    def of_kind(self, *kinds):
        return [p for p in self.parts.values() if p.kind in kinds]

    def unique_name(self, base):
        names = {p.name for p in self.parts.values()}
        if base not in names:
            return base
        i = 2
        while f"{base} {i}" in names:
            i += 1
        return f"{base} {i}"

    def arms_on(self, shaft_id):
        return [p for p in self.parts.values() if p.kind == "arm" and p["shaft"] == shaft_id]

    def links_to(self, pid):
        return [p for p in self.parts.values() if p.kind == "link" and pid in (p["a"], p["b"])]

    def is_handle_shaft(self, shaft_id):
        return any(a.get("grip") for a in self.arms_on(shaft_id))

    def remove(self, pid):
        """Remove a part and anything that depends on it."""
        if pid not in self.parts:
            return []
        gone = [pid]
        part = self.parts.pop(pid)
        if part.kind == "shaft":
            for arm in self.arms_on(pid):
                gone += self.remove(arm.id)
        if part.kind in ("arm", "slider"):
            for ln in self.links_to(pid):
                gone += self.remove(ln.id)
        return gone

    # ---------- save / load ----------
    def to_dict(self):
        return {"format": "linkage-designer", "version": 1,
                "settings": dict(self.settings),
                "parts": [p.to_dict() for p in self.parts.values()]}

    def to_json(self):
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_dict(cls, d):
        pr = cls()
        pr.settings.update(d.get("settings", {}))
        for pd in d.get("parts", []):
            pd = dict(pd)
            kind = pd.pop("kind")
            pid = pd.pop("id")
            if kind not in KINDS:
                continue
            part = KINDS[kind](pid, **pd)
            pr.parts[pid] = part
        return pr

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))

    def copy(self):
        return Project.from_dict(self.to_dict())


# ---------------------------------------------------------------- helpers

def arm_tip_neutral(project, arm):
    sh = project.parts.get(arm["shaft"])
    if sh is None:
        return None
    s, a, _ = sh.axis()
    root = s + a * float(arm["at"])
    return root + g.angle_dir(a, float(arm["angle"])) * float(arm["length"])


def attach_point_neutral(project, pid):
    p = project.parts.get(pid)
    if p is None:
        return None
    if p.kind == "arm":
        return arm_tip_neutral(project, p)
    if p.kind == "slider":
        s, d, _ = p.track()
        return s + d * float(p["neutral"])
    return None


def fit_links(project):
    """Set auto-fit push rods to their exact neutral length."""
    for ln in project.of_kind("link"):
        if not ln.get("auto_fit", True):
            continue
        pa = attach_point_neutral(project, ln["a"])
        pb = attach_point_neutral(project, ln["b"])
        if pa is not None and pb is not None:
            ln["length"] = round(g.norm(pa - pb), 4)


def _mx(p):
    return [-float(p[0]), float(p[1]), float(p[2])]


def mirror_copy(project, ids):
    """Mirror-copy parts left<->right across X = 0. Returns new ids.

    Shafts bring their arms. Push rods are copied when both ends are copied too.
    """
    sel = set()
    for pid in ids:
        p = project.parts.get(pid)
        if p is None:
            continue
        sel.add(pid)
        if p.kind == "shaft":
            sel.update(a.id for a in project.arms_on(pid))
        if p.kind == "arm" and p["shaft"] in project.parts:
            sel.add(p["shaft"])
            sel.update(a.id for a in project.arms_on(p["shaft"]))
    mapping = {}

    def mname(n):
        swaps = [("Left", "Right"), ("Right", "Left"), ("left", "right"),
                 ("right", "left"), (" L", " R"), (" R", " L")]
        for a, b in swaps:
            if a in n:
                return project.unique_name(n.replace(a, b, 1))
        return project.unique_name(n + " (mirror)")

    # shafts / sliders / frames first
    for pid in list(sel):
        p = project.parts[pid]
        if p.kind in ("shaft", "slider", "frame"):
            d = copy.deepcopy(p.data)
            d["name"] = mname(p.name)
            d["start"], d["end"] = _mx(p["start"]), _mx(p["end"])
            if p.kind == "shaft":
                # rotation direction flips under a mirror; keep stops matching
                d["min_angle"], d["max_angle"] = -p["max_angle"], -p["min_angle"]
            new = project.add(p.kind, **d)
            mapping[pid] = new.id
    # arms
    for pid in list(sel):
        p = project.parts[pid]
        if p.kind != "arm":
            continue
        tip = arm_tip_neutral(project, p)
        new_shaft = project.parts[mapping[p["shaft"]]]
        s, a, _ = new_shaft.axis()
        root = s + a * float(p["at"])
        mt = g.vec(_mx(tip))
        ang = g.angle_of(a, mt - root)
        d = copy.deepcopy(p.data)
        d["name"] = mname(p.name)
        d["shaft"] = new_shaft.id
        d["angle"] = round(ang, 4)
        new = project.add("arm", **d)
        mapping[pid] = new.id
    # links whose ends were both copied (or one end copied, other end shared)
    for ln in project.of_kind("link"):
        if ln.id in mapping.values():
            continue
        ea, eb = ln["a"], ln["b"]
        if ea in mapping or eb in mapping:
            if ea in mapping and eb in mapping:
                d = copy.deepcopy(ln.data)
                d["name"] = mname(ln.name)
                d["a"], d["b"] = mapping[ea], mapping[eb]
                new = project.add("link", **d)
                mapping[ln.id] = new.id
    fit_links(project)
    return list(mapping.values())
