"""The design: parts, rods, and how they click together.

Everything is stored in its *design pose* (as built). The motion solver moves
rigid groups from there without changing the design.

Connections
  slot  - a rod end clicked into a connector/joint/clip slot (rigid)
  hole  - a rod through a part's hole: connectors and clips spin on it,
          gears / wheels / spacers are locked to it
  stack - two 3D connectors slid together (rigid)
"""
import copy
import json
import math

import numpy as np

from . import catalog as C
from . import geometry as g


class Part:
    def __init__(self, pid, type_key, pos, R, params=None):
        self.id = pid
        self.type = type_key
        self.pos = g.vec(pos)
        self.R = np.array(R, dtype=float).reshape(3, 3)
        self.params = dict(params or {})

    @property
    def tdef(self):
        return C.TYPES[self.type]

    @property
    def name(self):
        return self.params.get("name") or f"{C.TYPES[self.type]['name']} {self.id}"

    def to_dict(self):
        return {"id": self.id, "type": self.type, "pos": self.pos.round(6).tolist(),
                "R": self.R.round(9).tolist(), "params": copy.deepcopy(self.params)}


class Rod:
    def __init__(self, rid, length, p0, d, color=None, params=None):
        self.id = rid
        self.length = float(length)
        self.p0 = g.vec(p0)
        self.d = g.unit(d)
        self.color = color or C.rod_color_for(self.length)
        self.params = dict(params or {})

    @property
    def p1(self):
        return self.p0 + self.d * self.length

    @property
    def name(self):
        if self.params.get("name"):
            return self.params["name"]
        label = self.color.capitalize() if self.color in C.RODS else "Custom"
        return f"{label} rod {self.id}"

    def to_dict(self):
        return {"id": self.id, "length": self.length, "color": self.color,
                "p0": self.p0.round(6).tolist(), "d": self.d.round(9).tolist(),
                "params": copy.deepcopy(self.params)}


class Project:
    def __init__(self):
        self.parts = {}
        self.rods = {}
        self.slots = []      # dict(part, slot, rod, end)
        self.holes = []      # dict(part, hole, rod, t, locked)
        self.stacks = []     # [part a, part b]
        self.settings = {"units": "in", "stop_on_collision": True, "stop_on_limits": True}
        self._n = 1

    # ------------------------------------------------------------ ids / lookup
    def _id(self, prefix):
        while f"{prefix}{self._n}" in self.parts or f"{prefix}{self._n}" in self.rods:
            self._n += 1
        i = f"{prefix}{self._n}"
        self._n += 1
        return i

    def item(self, iid):
        return self.parts.get(iid) or self.rods.get(iid)

    def name_of(self, iid):
        it = self.item(iid)
        return it.name if it else iid

    def has_ground(self):
        return any(x.params.get("ground") for x in list(self.parts.values()) + list(self.rods.values()))

    def _new_part(self, type_key, pos, R, params=None):
        p = Part(self._id("P"), type_key, pos, R, params)
        if not self.has_ground():
            p.params["ground"] = True
        self.parts[p.id] = p
        return p

    def _new_rod(self, length, p0, d, color=None):
        r = Rod(self._id("R"), length, p0, d, color)
        if not self.has_ground():
            r.params["ground"] = True
        self.rods[r.id] = r
        return r

    # ------------------------------------------------------------ geometry of ports
    def slot_world(self, pid, s):
        p = self.parts[pid]
        for idx, half, lp, ld in C.local_slots(p.type, p.params):
            if idx == s:
                return p.pos + p.R @ lp, p.R @ ld, half
        raise KeyError(s)

    def hole_world(self, pid, h):
        p = self.parts[pid]
        for idx, lc, la in C.local_holes(p.type):
            if idx == h:
                return p.pos + p.R @ lc, p.R @ la
        raise KeyError(h)

    def rod_end(self, rid, end):
        r = self.rods[rid]
        return (r.p0, -r.d) if end == 0 else (r.p1, r.d)

    def slot_taken(self, pid, s):
        return any(c["part"] == pid and c["slot"] == s for c in self.slots)

    def end_taken(self, rid, end):
        return any(c["rod"] == rid and c["end"] == end for c in self.slots)

    def hole_taken(self, pid, h):
        return any(c["part"] == pid and c["hole"] == h for c in self.holes)

    def open_slots(self):
        out = []
        for p in self.parts.values():
            for idx, half, _, _ in C.local_slots(p.type, p.params):
                if not self.slot_taken(p.id, idx):
                    out.append((p.id, idx))
        return out

    def free_ends(self):
        return [(r.id, e) for r in self.rods.values() for e in (0, 1) if not self.end_taken(r.id, e)]

    def open_holes(self):
        out = []
        for p in self.parts.values():
            if p.tdef["family"] == "mount" and p.type != "cliphole":
                continue
            for idx, _, _ in C.local_holes(p.type):
                if p.tdef["family"] == "mount" and idx == 0:
                    continue          # a mount's main bore is on its own rod
                if not self.hole_taken(p.id, idx):
                    out.append((p.id, idx))
        return out

    # ------------------------------------------------------------ building
    def _ref_normal(self, rid, end):
        """Hole axis of whatever the rod's other end is in, to keep builds flat by default."""
        o = self.rod_end(rid, end)[1]
        for c in self.slots:
            if c["rod"] == rid and c["end"] != end:
                n = self.parts[c["part"]].R[:, 2]
                if abs(float(n @ o)) < 0.2:
                    return n
        for c in self.holes:
            if c["rod"] == rid:
                n = self.parts[c["part"]].R[:, 2]
                if abs(float(n @ o)) < 0.2:
                    return n
        n = g.FWD if abs(float(g.FWD @ o)) < 0.9 else g.UP
        return n

    def place_start(self, type_key, at=(0.0, 0.0, 0.0)):
        """A first connector standing upright, its hole pointing front-to-back."""
        R = np.column_stack([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])
        return self._new_part(type_key, at, R)

    def place_free_rod(self, length, at=(0.0, 0.0, 0.0)):
        return self._new_rod(length, g.vec(at), [1.0, 0, 0])

    def add_rod_at_slot(self, pid, s, length):
        if self.slot_taken(pid, s):
            raise ValueError("That slot already has a rod in it")
        pt, d, _ = self.slot_world(pid, s)
        r = self._new_rod(length, pt, d)
        self.slots.append({"part": pid, "slot": s, "rod": r.id, "end": 0})
        closed = self.auto_close()
        return r, closed

    def add_rod_through_hole(self, pid, h, length):
        if self.hole_taken(pid, h):
            raise ValueError("That hole already has a rod through it")
        c, a = self.hole_world(pid, h)
        r = self._new_rod(length, c - a * length / 2, a)
        self.holes.append({"part": pid, "hole": h, "rod": r.id, "t": length / 2,
                           "locked": bool(self.parts[pid].tdef.get("lock", False))})
        self.auto_close_holes()
        return r

    def _orient(self, type_key, params, slot, o, n, roll_deg):
        """Rotation putting local `slot` along -o, local z near n, then rolled about o."""
        lp, ld = None, None
        for idx, half, p_, d_ in C.local_slots(type_key, params):
            if idx == slot:
                lp, ld = p_, d_
        z = np.array([0.0, 0.0, 1.0])
        if abs(float(ld @ z)) > 0.9:
            z = np.array([0.0, 1.0, 0.0])
        L = g.frame_from(ld, z)
        n = g.vec(n) - (g.vec(n) @ o) * o
        if np.linalg.norm(n) < 1e-6:
            n = g.perp(o)
        W = g.frame_from(-o, n)
        R = W @ L.T
        if roll_deg:
            R = g.rot_matrix(o, math.radians(roll_deg)) @ R
        center_offset = R @ lp
        return R, center_offset

    def add_part_at_end(self, rid, end, type_key, slot=None, roll=0.0, params=None):
        if self.end_taken(rid, end):
            raise ValueError("That rod end is already clicked into something")
        params = dict(params or {})
        slots = C.local_slots(type_key, params)
        if not slots:
            raise ValueError("That part has no rod slots")
        if slot is None:
            slot = slots[0][0]
        P, o = self.rod_end(rid, end)
        n = self._ref_normal(rid, end)
        R, off = self._orient(type_key, params, slot, o, n, roll)
        part = self._new_part(type_key, P - off, R, params)
        part.params["_attach"] = {"rod": rid, "end": end, "slot": slot, "roll": roll}
        self.slots.append({"part": part.id, "slot": slot, "rod": rid, "end": end})
        closed = self.auto_close()
        return part, closed

    def can_reorient(self, pid):
        p = self.parts.get(pid)
        if p is None or "_attach" not in p.params:
            return False
        a = p.params["_attach"]
        mine = [c for c in self.slots if c["part"] == pid]
        others = [c for c in self.holes if c["part"] == pid] + \
                 [s for s in self.stacks if pid in s]
        return len(mine) == 1 and mine[0]["rod"] == a["rod"] and not others

    def reorient(self, pid, next_slot=False, roll_step=0.0):
        """Turn a just-placed part: next slot onto the rod, or roll it around the rod."""
        if not self.can_reorient(pid):
            raise ValueError("Only a part held by a single rod can be turned")
        p = self.parts[pid]
        a = p.params["_attach"]
        slots = [s[0] for s in C.local_slots(p.type, p.params)]
        slot = a["slot"]
        if next_slot:
            slot = slots[(slots.index(slot) + 1) % len(slots)]
        roll = (a["roll"] + roll_step) % 360.0
        P, o = self.rod_end(a["rod"], a["end"])
        n = self._ref_normal(a["rod"], a["end"])
        R, off = self._orient(p.type, p.params, slot, o, n, roll)
        p.R, p.pos = R, P - off
        a.update(slot=slot, roll=roll)
        for c in self.slots:
            if c["part"] == pid:
                c["slot"] = slot
        return self.auto_close()

    def mount_on_rod(self, rid, type_key, t=None, roll=0.0):
        r = self.rods[rid]
        if t is None:
            t = r.length / 2
        u0 = g.perp(r.d)
        x = g.rot_matrix(r.d, math.radians(roll)) @ u0
        R = g.frame_from(x, r.d)
        part = self._new_part(type_key, r.p0 + r.d * t, R, {"roll": roll})
        self.holes.append({"part": part.id, "hole": 0, "rod": rid, "t": float(t),
                           "locked": bool(C.TYPES[type_key].get("lock", False))})
        return part

    def move_mount(self, pid, t=None, roll=None):
        conn = next((c for c in self.holes if c["part"] == pid and c["hole"] == 0), None)
        if conn is None:
            return
        r = self.rods[conn["rod"]]
        p = self.parts[pid]
        if t is not None:
            conn["t"] = float(min(max(t, 0.0), r.length))
        if roll is not None:
            p.params["roll"] = float(roll)
        u0 = g.perp(r.d)
        x = g.rot_matrix(r.d, math.radians(p.params.get("roll", 0.0))) @ u0
        oldR, oldpos = p.R.copy(), p.pos.copy()
        p.R = g.frame_from(x, r.d)
        p.pos = r.p0 + r.d * conn["t"]
        # anything hanging off this part (clip slots, second holes) moves with it
        moved = self._component_from(pid, exclude={conn["rod"]})
        T = g.mat4(p.R @ oldR.T, p.pos - p.R @ oldR.T @ oldpos)
        self._apply(moved - {pid}, T)
        self.auto_close()

    def add_3d_partner(self, pid, type_key="conn7_3d"):
        a = self.parts[pid]
        if not a.tdef.get("is3d") or not C.TYPES[type_key].get("is3d"):
            raise ValueError("Only 3D connectors (blue / purple) stack together")
        if any(pid in s for s in self.stacks):
            raise ValueError("That connector already has a 3D partner")
        m = a.R @ C.missing_slot_dir(a.type)
        R = g.rot_matrix(m, math.pi / 2) @ a.R
        b = self._new_part(type_key, a.pos.copy(), R)
        self.stacks.append([pid, b.id])
        self.auto_close()
        return b

    def auto_close(self):
        """Click free rod ends into open slots they line up with. Returns [(rod, end, part, slot, gap)]."""
        closed = []
        cos_tol = math.cos(math.radians(6.0))
        opens = self.open_slots()
        world = {(pid, s): self.slot_world(pid, s) for pid, s in opens}
        for rid, end in self.free_ends():
            P, o = self.rod_end(rid, end)
            best = None
            for key, (sp, sd, _) in world.items():
                gap = g.norm(sp - P)
                if gap < C.SNAP_TOL and float(sd @ -o) > cos_tol:
                    if any(c["rod"] == rid and c["part"] == key[0] for c in self.slots):
                        continue
                    if best is None or gap < best[0]:
                        best = (gap, key)
            if best:
                gap, (pid, s) = best
                self.slots.append({"part": pid, "slot": s, "rod": rid, "end": end})
                world.pop((pid, s))
                closed.append((rid, end, pid, s, gap))
        self.auto_close_holes()
        return closed

    def auto_close_holes(self):
        """Hook up rods that run straight through an open hole (spinning fit)."""
        made = []
        for pid, h in self.open_holes():
            hc, ha = self.hole_world(pid, h)
            for r in self.rods.values():
                if any(c["rod"] == r.id and c["part"] == pid for c in self.slots + self.holes):
                    continue
                if abs(abs(float(ha @ r.d)) - 1.0) > 1e-3:
                    continue
                if g.point_line_dist(hc, r.p0, r.d) > C.FORCED_FIT:
                    continue
                t = float((hc - r.p0) @ r.d)
                if -0.5 <= t <= r.length + 0.5:
                    self.holes.append({"part": pid, "hole": h, "rod": r.id, "t": t, "locked": False})
                    made.append((pid, h, r.id))
                    break
        return made

    def roll_toward(self, rid, at_point, target):
        """Mount roll angle (deg) that points a mount's slot 0 from `at_point` toward `target`."""
        r = self.rods[rid]
        w = g.vec(target) - g.vec(at_point)
        w = w - (w @ r.d) * r.d
        u0 = g.perp(r.d)
        v0 = np.cross(r.d, u0)
        return math.degrees(math.atan2(float(w @ v0), float(w @ u0)))

    def set_hole_lock(self, pid, rid, locked):
        for c in self.holes:
            if c["part"] == pid and c["rod"] == rid:
                c["locked"] = bool(locked)

    # ------------------------------------------------------------ editing
    def _neighbors(self, iid):
        out = set()
        for c in self.slots:
            if c["part"] == iid:
                out.add(c["rod"])
            if c["rod"] == iid:
                out.add(c["part"])
        for c in self.holes:
            if c["part"] == iid:
                out.add(c["rod"])
            if c["rod"] == iid:
                out.add(c["part"])
        for a, b in self.stacks:
            if a == iid:
                out.add(b)
            if b == iid:
                out.add(a)
        return out

    def _component_from(self, start, exclude=()):
        seen = {start}
        stack = [start]
        excl = set(exclude)
        while stack:
            x = stack.pop()
            for y in self._neighbors(x):
                if y not in seen and y not in excl:
                    seen.add(y)
                    stack.append(y)
        return seen

    def _apply(self, ids, T):
        R, t = T[:3, :3], T[:3, 3]
        for iid in ids:
            if iid in self.parts:
                p = self.parts[iid]
                p.pos = R @ p.pos + t
                p.R = g.orthonormalize(R @ p.R)
            elif iid in self.rods:
                r = self.rods[iid]
                r.p0 = R @ r.p0 + t
                r.d = g.unit(R @ r.d)

    def set_rod_length(self, rid, length):
        """Change a rod's length and slide whatever hangs off its far end along with it.

        Returns a note if the rod sits in a closed loop (then nothing else moves and
        the fit check reports the mismatch)."""
        r = self.rods[rid]
        length = float(length)
        dL = length - r.length
        if abs(dL) < 1e-9:
            return None
        end1 = [c["part"] for c in self.slots if c["rod"] == rid and c["end"] == 1]
        end0 = [c["part"] for c in self.slots if c["rod"] == rid and c["end"] == 0]
        on_rod = [c["part"] for c in self.holes if c["rod"] == rid]
        side1 = set()
        for pid in end1:
            side1 |= self._component_from(pid, exclude={rid})
        anchor = set(end0) | set(on_rod)
        r.length = length
        r.color = C.rod_color_for(length)
        if not (side1 & anchor):
            self._apply(side1, g.mat4(np.eye(3), r.d * dL))
            return None
        side0 = set()
        for pid in end0:
            side0 |= self._component_from(pid, exclude={rid})
        if not (side0 & set(end1)):
            r.p0 = r.p0 - r.d * dL
            self._apply(side0, g.mat4(np.eye(3), -r.d * dL))
            for c in self.holes:
                if c["rod"] == rid:
                    c["t"] += dL
            return None
        return "This rod is part of a closed loop, so the other parts can't slide to fit."

    def set_joint_param(self, pid, key, value):
        """Change a hinge angle or ball-joint tilt, swinging the B side to match."""
        p = self.parts[pid]
        old_slots = {i: (lp, ld) for i, h, lp, ld in C.local_slots(p.type, p.params)}
        newp = dict(p.params)
        newp[key] = float(value)
        new_slots = {i: (lp, ld) for i, h, lp, ld in C.local_slots(p.type, newp)}
        (_, d_old), (_, d_new) = old_slots[1], new_slots[1]
        side_b = set()
        side_a = set()
        for c in self.slots:
            if c["part"] == pid:
                comp = self._component_from(c["rod"], exclude={pid})
                (side_b if c["slot"] == 1 else side_a).update(comp)
        if side_b & side_a:
            return "Both sides of this joint are tied together, so it can't swing here."
        # rotation taking old direction to new about the joint center
        a, b = p.R @ d_old, p.R @ d_new
        axis = np.cross(a, b)
        s = np.linalg.norm(axis)
        ang = math.atan2(s, float(a @ b))
        Rm = np.eye(3) if s < 1e-9 else g.rot_matrix(axis / s, ang)
        p.params[key] = float(value)
        self._apply(side_b, g.mat4(Rm, p.pos - Rm @ p.pos))
        self.auto_close()
        return None

    def delete(self, iid):
        self.parts.pop(iid, None)
        self.rods.pop(iid, None)
        self.slots = [c for c in self.slots if iid not in (c["part"], c["rod"])]
        self.holes = [c for c in self.holes if iid not in (c["part"], c["rod"])]
        self.stacks = [s for s in self.stacks if iid not in s]
        for p in self.parts.values():
            a = p.params.get("_attach")
            if a and a["rod"] == iid:
                p.params.pop("_attach")

    def translate_all(self, ids, delta):
        self._apply(ids, g.mat4(np.eye(3), g.vec(delta)))

    # ------------------------------------------------------------ checks
    def fit_issues(self):
        """Rods forced into slots or holes they don't line up with."""
        out = []
        for c in self.slots:
            P, o = self.rod_end(c["rod"], c["end"])
            sp, sd, _ = self.slot_world(c["part"], c["slot"])
            gap = g.norm(sp - P)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(sd @ -o)))))
            if gap > C.FORCED_FIT or ang > 2.0:
                lvl = "bind" if gap > C.SNAP_TOL or ang > 8 else "warn"
                out.append((lvl, f"fit:{c['part']}:{c['slot']}",
                            f"Forced fit: {self.name_of(c['rod'])} into {self.name_of(c['part'])} "
                            f"is off {gap:.1f} mm / {ang:.1f} deg", [c["rod"], c["part"]]))
        for c in self.holes:
            r = self.rods[c["rod"]]
            hc, ha = self.hole_world(c["part"], c["hole"])
            off = g.point_line_dist(hc, r.p0, r.d)
            ang = math.degrees(math.acos(min(1.0, abs(float(ha @ r.d)))))
            if off > C.FORCED_FIT or ang > 2.0:
                out.append(("bind", f"hole:{c['part']}:{c['hole']}",
                            f"{self.name_of(c['rod'])} doesn't line up with the hole in "
                            f"{self.name_of(c['part'])} ({off:.1f} mm / {ang:.1f} deg)",
                            [c["rod"], c["part"]]))
            tt = float((hc - r.p0) @ r.d)
            if tt < -1 or tt > r.length + 1:
                out.append(("bind", f"off:{c['part']}", f"{self.name_of(c['part'])} is off the end "
                            f"of {self.name_of(c['rod'])}", [c["rod"], c["part"]]))
        return out

    # ------------------------------------------------------------ files
    def to_dict(self):
        return {"format": "knexsuite", "version": 1, "settings": dict(self.settings),
                "parts": [p.to_dict() for p in self.parts.values()],
                "rods": [r.to_dict() for r in self.rods.values()],
                "slots": copy.deepcopy(self.slots), "holes": copy.deepcopy(self.holes),
                "stacks": copy.deepcopy(self.stacks)}

    def to_json(self):
        return json.dumps(self.to_dict(), indent=1)

    @classmethod
    def from_dict(cls, d):
        pr = cls()
        pr.settings.update(d.get("settings", {}))
        for pd in d.get("parts", []):
            if pd["type"] in C.TYPES:
                pr.parts[pd["id"]] = Part(pd["id"], pd["type"], pd["pos"], pd["R"], pd.get("params"))
        for rd in d.get("rods", []):
            pr.rods[rd["id"]] = Rod(rd["id"], rd["length"], rd["p0"], rd["d"], rd.get("color"),
                                    rd.get("params"))
        ok = lambda c: c["part"] in pr.parts and c["rod"] in pr.rods  # noqa: E731
        pr.slots = [c for c in d.get("slots", []) if ok(c)]
        pr.holes = [c for c in d.get("holes", []) if ok(c)]
        pr.stacks = [s for s in d.get("stacks", []) if all(x in pr.parts for x in s)]
        return pr

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))

    def copy(self):
        return Project.from_dict(self.to_dict())

    def bounds(self):
        pts = [p.pos for p in self.parts.values()]
        for r in self.rods.values():
            pts += [r.p0, r.p1]
        if not pts:
            return np.zeros(3), np.zeros(3)
        a = np.array(pts)
        return a.min(axis=0), a.max(axis=0)
