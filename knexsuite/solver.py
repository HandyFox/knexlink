"""Motion: rigid groups, joints, gears, dragging, driving and binding checks.

Parts that click together rigidly form one *body*. Bodies move against each other at:
  - a rod spinning in a connector's / clip's hole (revolute)
  - a hinge (revolute, 45..315 deg)
  - a ball joint (spherical, 40 deg cone)
  - meshing gears (their contact points roll together)
Ground bodies never move.

Each body's pose is a rotation R and offset t applied to its design pose.
The solver uses small Newton steps with minimum change, so anything that
isn't pushed stays where it is.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import catalog as C
from . import geometry as g

SUB_STEP_MM = 1.5           # max grabbed-point motion per solve
DRIVE_STEP = math.radians(1.0)
SNAP_ROT = math.radians(20.0)


@dataclass
class Issue:
    level: str          # "warn" | "bind"
    key: str
    text: str
    parts: list = field(default_factory=list)
    stop: bool = False


@dataclass
class Joint:
    kind: str           # "rev" | "sph"
    a: int              # body index
    b: int
    p: np.ndarray       # design point
    axis: np.ndarray    # design axis (rev)
    label: str
    part: str           # part id that defines it
    rod: str = None
    hinge: bool = False
    base_angle: float = 0.0   # hinge opening at design


class Assembly:
    def __init__(self, project):
        self.pr = project
        P = project.parts
        parent = {}

        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        def pnode(pid, slot=None):
            p = P[pid]
            if p.tdef["family"] == "joint":
                return f"{pid}:{'A' if slot in (0, None) else 'B'}"
            return pid

        for r in project.rods:
            find(r)
        for pid, p in P.items():
            if p.tdef["family"] == "joint":
                find(f"{pid}:A")
                find(f"{pid}:B")
            else:
                find(pid)
        for c in project.slots:
            union(pnode(c["part"], c["slot"]), c["rod"])
        for c in project.holes:
            if c["locked"]:
                union(pnode(c["part"]), c["rod"])
        for a, b in project.stacks:
            union(a, b)

        roots = {}
        self.node_body = {}
        for x in list(parent.keys()):
            r = find(x)
            if r not in roots:
                roots[r] = len(roots)
            self.node_body[x] = roots[r]
        self.nb = len(roots)
        self.members = [[] for _ in range(self.nb)]
        for x, b in self.node_body.items():
            self.members[b].append(x)

        # ground
        ground = set()
        for iid, it in list(P.items()) + list(project.rods.items()):
            if it.params.get("ground"):
                ground.add(self.body_of_item(iid))
        if not ground and self.nb:
            first = next(iter(list(P.keys()) + list(project.rods.keys())))
            ground.add(self.body_of_item(first))
        self.ground = ground
        self.free = [b for b in range(self.nb) if b not in ground]
        self.var = {b: i for i, b in enumerate(self.free)}
        self.n = 6 * len(self.free)

        # body reference points (design)
        self.o_d = np.zeros((self.nb, 3))
        for b in range(self.nb):
            pts = [self._node_point(x) for x in self.members[b]]
            self.o_d[b] = np.mean(pts, axis=0) if pts else np.zeros(3)

        # joints
        self.joints = []
        for c in project.holes:
            if c["locked"]:
                continue
            a, b = self.node_body[pnode(c["part"])], self.node_body[c["rod"]]
            if a == b:
                continue
            hc, ha = project.hole_world(c["part"], c["hole"])
            self.joints.append(Joint("rev", b, a, hc, project.rods[c["rod"]].d.copy(),
                                     f"{project.name_of(c['part'])} on {project.name_of(c['rod'])}",
                                     c["part"], rod=c["rod"]))
        for pid, p in P.items():
            if p.tdef["family"] != "joint":
                continue
            a, b = self.node_body[f"{pid}:A"], self.node_body[f"{pid}:B"]
            if a == b:
                continue
            if p.tdef["joint"] == "rev":
                self.joints.append(Joint("rev", a, b, p.pos.copy(), p.R[:, 2].copy(), p.name, pid,
                                         hinge=True, base_angle=p.params.get("angle", 180.0)))
            else:
                self.joints.append(Joint("sph", a, b, p.pos.copy(), p.R[:, 0].copy(), p.name, pid))

        # gears
        self.meshes = self._find_meshes()

        self.reset()

    # ------------------------------------------------------------ structure helpers
    def _node_point(self, x):
        pid = x.split(":")[0]
        if pid in self.pr.parts:
            return self.pr.parts[pid].pos
        r = self.pr.rods[pid]
        return (r.p0 + r.p1) / 2

    def body_of_item(self, iid, half=None):
        if iid in self.pr.rods:
            return self.node_body[iid]
        p = self.pr.parts[iid]
        if p.tdef["family"] == "joint":
            return self.node_body[f"{iid}:{half or 'A'}"]
        return self.node_body[iid]

    def _find_meshes(self):
        gears = []
        for pid, p in self.pr.parts.items():
            if "teeth" in p.tdef:
                gears.append((pid, p.pos.copy(), p.R[:, 2].copy(), C.pitch_radius(p.type),
                              bool(p.tdef.get("crown")), self.body_of_item(pid)))
        out = []
        for i in range(len(gears)):
            for j in range(i + 1, len(gears)):
                gi, gj = gears[i], gears[j]
                if gi[5] == gj[5]:
                    continue
                ci, ai, ri, ki = gi[1], gi[2], gi[3], gi[4]
                cj, aj, rj, kj = gj[1], gj[2], gj[3], gj[4]
                dot = abs(float(ai @ aj))
                if dot > 0.995 and not (ki or kj):
                    off = abs(float((cj - ci) @ ai))
                    dist = g.point_line_dist(cj, ci, ai)
                    if off < 4.0 and abs(dist - (ri + rj)) < 3.0:
                        out.append((gi[0], gj[0]))
                elif dot < 0.1 and (ki != kj):
                    crown, small = (gi, gj) if ki else (gj, gi)
                    P = self._contact(small[1], small[2], small[3], crown[1])
                    rad = g.point_line_dist(P, crown[1], crown[2])
                    h = abs(float((P - crown[1]) @ crown[2]))
                    if abs(rad - crown[3]) < 6.0 and h < C.CONN_T * 1.5:
                        out.append((gi[0], gj[0]))
        return out

    @staticmethod
    def _contact(c_s, a_s, r_s, c_other):
        w = c_other - c_s
        w = w - (w @ a_s) * a_s
        return c_s + g.unit(w, g.perp(a_s)) * r_s

    # ------------------------------------------------------------ state
    def reset(self):
        self.R = np.array([np.eye(3) for _ in range(self.nb)])
        self.t = np.zeros((self.nb, 3))

    def save(self):
        return self.R.copy(), self.t.copy()

    def restore(self, s):
        self.R, self.t = s[0].copy(), s[1].copy()

    def X(self, b, p):
        return self.R[b] @ p + self.t[b]

    def o(self, b):
        return self.R[b] @ self.o_d[b] + self.t[b]

    def T(self, b):
        return g.mat4(self.R[b], self.t[b])

    def _apply(self, d):
        for b, i in self.var.items():
            w = d[6 * i:6 * i + 3]
            dt = d[6 * i + 3:6 * i + 6]
            if not (np.any(w) or np.any(dt)):
                continue
            o = self.o(b)
            E = g.exp_rot(w)
            self.R[b] = E @ self.R[b]
            self.t[b] = E @ (self.t[b] - o) + o + dt

    def tidy(self):
        for b in self.free:
            self.R[b] = g.orthonormalize(self.R[b])

    # ------------------------------------------------------------ constraint rows
    def _point_block(self, J, row, b, x, sign=1.0):
        """Rows for d(x)/d(vars) of material point x on body b."""
        if b not in self.var:
            return
        i = self.var[b]
        J[row:row + 3, 6 * i:6 * i + 3] += -sign * g.skew(x - self.o(b))
        J[row:row + 3, 6 * i + 3:6 * i + 6] += sign * np.eye(3)

    def _rot_row(self, J, row, b, coeff):
        if b in self.var:
            i = self.var[b]
            J[row, 6 * i:6 * i + 3] += coeff

    def joint_angle(self, j):
        jt = self.joints[j]
        u0 = g.perp(jt.axis)
        a = self.R[jt.a] @ jt.axis
        ua, ub = self.R[jt.a] @ u0, self.R[jt.b] @ u0
        return math.atan2(float(a @ g.cross(ua, ub)), float(ua @ ub))

    def evaluate(self, drive=None):
        rows = 0
        for jt in self.joints:
            rows += 5 if jt.kind == "rev" else 3
        if drive is not None:
            rows += 1
        r = np.zeros(rows)
        J = np.zeros((rows, self.n))
        k = 0
        for j, jt in enumerate(self.joints):
            pa, pb = self.X(jt.a, jt.p), self.X(jt.b, jt.p)
            r[k:k + 3] = pa - pb
            self._point_block(J, k, jt.a, pa, 1.0)
            self._point_block(J, k, jt.b, pb, -1.0)
            k += 3
            if jt.kind == "rev":
                aa, ab = self.R[jt.a] @ jt.axis, self.R[jt.b] @ jt.axis
                u = g.perp(ab)
                v = g.cross(ab, u)
                for w in (u, v):
                    r[k] = float(aa @ w)
                    self._rot_row(J, k, jt.a, g.cross(aa, w))
                    self._rot_row(J, k, jt.b, g.cross(w, aa))
                    k += 1
        if drive is not None:
            j, target = drive
            jt = self.joints[j]
            phi = self.joint_angle(j)
            r[k] = (phi - target + math.pi) % (2 * math.pi) - math.pi
            a = self.R[jt.a] @ jt.axis
            self._rot_row(J, k, jt.a, -a)
            self._rot_row(J, k, jt.b, a)
        return r, J

    def gear_rows(self):
        """Linear rows: contact points of meshing gears move together (rolling)."""
        G = np.zeros((len(self.meshes), self.n))
        for k, (pa, pb) in enumerate(self.meshes):
            A, B = self.pr.parts[pa], self.pr.parts[pb]
            ba, bb = self.body_of_item(pa), self.body_of_item(pb)
            ca, cb = self.X(ba, A.pos), self.X(bb, B.pos)
            aa, ab = self.R[ba] @ A.R[:, 2], self.R[bb] @ B.R[:, 2]
            ra, rb = C.pitch_radius(A.type), C.pitch_radius(B.type)
            if not (A.tdef.get("crown") or B.tdef.get("crown")):
                # spur pair: pitch point splits the center distance by tooth count
                w = cb - ca
                w = w - (w @ aa) * aa
                P = ca + w * (ra / (ra + rb))
                t = g.unit(g.cross(aa, P - ca))
            elif (ra <= rb and not A.tdef.get("crown")) or B.tdef.get("crown"):
                P = self._contact(ca, aa, ra, cb)
                t = g.unit(g.cross(aa, P - ca))
            else:
                P = self._contact(cb, ab, rb, ca)
                t = g.unit(g.cross(ab, P - cb))
            for b, s in ((ba, 1.0), (bb, -1.0)):
                if b in self.var:
                    i = self.var[b]
                    G[k, 6 * i:6 * i + 3] += s * g.cross(P - self.o(b), t)
                    G[k, 6 * i + 3:6 * i + 6] += s * t
        return G

    def _limit(self, d):
        m = 1.0
        for i in range(len(self.free)):
            w = np.linalg.norm(d[6 * i:6 * i + 3])
            dt = np.linalg.norm(d[6 * i + 3:6 * i + 6])
            m = max(m, w / 0.15, dt / 15.0)
        return d / m

    def project(self, drive=None, G=None, z=None, iters=30):
        """Newton back onto the constraints. Returns (ok, z)."""
        if self.n == 0:
            return True, z
        if z is None:
            z = np.zeros(self.n)
        for _ in range(iters):
            r, J = self.evaluate(drive)
            if G is not None and len(G):
                r = np.concatenate([r, G @ z])
                J = np.vstack([J, G])
            if len(r) == 0 or np.max(np.abs(r)) < 1e-6:
                self.tidy()
                return True, z
            A = J @ J.T
            lam = 1e-9 * (np.trace(A) / max(len(A), 1) + 1.0)
            try:
                d = -J.T @ np.linalg.solve(A + lam * np.eye(len(A)), r)
            except np.linalg.LinAlgError:
                return False, z
            d = self._limit(d)
            self._apply(d)
            z = z + d
        r, _ = self.evaluate(drive)
        if G is not None and len(G):
            r = np.concatenate([r, G @ z])
        self.tidy()
        return bool(len(r) == 0 or np.max(np.abs(r)) < 1e-3), z

    def dof(self):
        if self.n == 0:
            return 0
        _, J = self.evaluate()
        G = self.gear_rows()
        if len(G):
            J = np.vstack([J, G])
        if len(J) == 0:
            return self.n
        return int(self.n - np.linalg.matrix_rank(J, tol=1e-6))

    # ------------------------------------------------------------ dragging
    def drag(self, body, p_design, target, analyze=True):
        """Pull material point p_design (on `body`) toward world `target`.

        Returns (moved_mm, stop_issue_or_None, issues)."""
        if body not in self.var:
            return 0.0, Issue("warn", "ground", "That part is fixed to the ground"), []
        start_issues = self.analyze() if analyze else []
        start_keys = {i.key for i in start_issues if i.stop}
        x0 = self.X(body, p_design)
        total = g.norm(target - x0)
        nsub = int(min(40, max(1, math.ceil(total / SUB_STEP_MM))))
        issues = start_issues
        i = self.var[body]
        for s in range(nsub):
            x = self.X(body, p_design)
            want = x + (target - x) / (nsub - s)
            dp = want - x
            if g.norm(dp) < 1e-4:
                break
            saved = self.save()
            r, Jc = self.evaluate()
            G = self.gear_rows()
            if len(G):
                Jc = np.vstack([Jc, G])
                r = np.concatenate([r, np.zeros(len(G))])
            Jp = np.zeros((3, self.n))
            Jp[:, 6 * i:6 * i + 3] = -g.skew(x - self.o(body))
            Jp[:, 6 * i + 3:6 * i + 6] = np.eye(3)
            if len(Jc):
                Jpi = np.linalg.pinv(Jc, rcond=1e-9)
                d0 = -Jpi @ r
                N = np.eye(self.n) - Jpi @ Jc
            else:
                d0 = np.zeros(self.n)
                N = np.eye(self.n)
            A = Jp @ N
            d = d0 + N @ (np.linalg.pinv(A, rcond=1e-4) @ (dp - Jp @ d0))
            d = self._limit(d)
            self._apply(d)
            ok, _ = self.project(G=G if len(G) else None, z=d)
            moved = g.norm(self.X(body, p_design) - x)
            if not ok or self._snapped(saved):
                self.restore(saved)
                return g.norm(self.X(body, p_design) - x0), Issue(
                    "bind", "lockup", "Locks up here (can't go further without bending a part)",
                    [], True), issues
            if analyze:
                new = self.analyze()
                stops = [it for it in new if it.stop and it.key not in start_keys]
                if stops:
                    self.restore(saved)
                    return g.norm(self.X(body, p_design) - x0), stops[0], issues
                issues = new
            if moved < 0.15 * g.norm(dp):
                return g.norm(self.X(body, p_design) - x0), Issue(
                    "warn", "stall", "Won't move that way (locked, at a limit, or held by the frame)",
                    [], False), issues
        return g.norm(self.X(body, p_design) - x0), None, issues

    def _snapped(self, saved):
        R0 = saved[0]
        for b in self.free:
            c = (np.trace(R0[b].T @ self.R[b]) - 1) / 2
            if math.acos(max(-1.0, min(1.0, c))) > SNAP_ROT:
                return True
        return False

    # ------------------------------------------------------------ driving a joint
    def drive(self, j, target, analyze=True):
        """Turn revolute joint j to `target` (radians, relative to design). Steps 1 deg at a time."""
        cur = self.joint_angle(j)
        diff = target - cur
        diff = (diff + math.pi) % (2 * math.pi) - math.pi   # shortest way round
        n = max(1, int(math.ceil(abs(diff) / DRIVE_STEP)))
        start_keys = {i.key for i in self.analyze() if i.stop} if analyze else set()
        issues = []
        for k in range(n):
            saved = self.save()
            goal = cur + diff * (k + 1) / n
            G = self.gear_rows()
            ok, _ = self.project(drive=(j, goal), G=G if len(G) else None,
                                 z=np.zeros(self.n))
            if not ok or self._snapped(saved):
                self.restore(saved)
                return Issue("bind", "lockup", "Locks up here (dead center or something too tight)",
                             [self.joints[j].part], True), issues
            if analyze:
                new = self.analyze()
                stops = [it for it in new if it.stop and it.key not in start_keys]
                if stops:
                    self.restore(saved)
                    return stops[0], issues
                issues = new
        return None, issues

    def sweep(self, j, span=math.radians(360)):
        """Drive joint j forward then backward from where it is. Returns ends and traces."""
        start = self.save()
        a0 = self.joint_angle(j)
        out = {"ends": {}, "states": []}
        for sign in (+1, -1):
            self.restore(start)
            states = [self.save()]
            stop = None
            turned = 0.0
            while turned < span - 1e-9:
                step = min(math.radians(5), span - turned)
                stop, _ = self.drive(j, a0 + sign * (turned + step))
                if stop is not None:
                    break
                turned += step
                states.append(self.save())
            out["ends"][sign] = {"angle": a0 + sign * turned, "stop": stop, "state": self.save()}
            out["states"].append(states)
        self.restore(start)
        return out

    # ------------------------------------------------------------ checks
    def joint_reading(self, j):
        """(label, value in degrees, kind)"""
        jt = self.joints[j]
        if jt.kind == "rev":
            phi = math.degrees(self.joint_angle(j))
            if jt.hinge:
                return jt.label, (jt.base_angle + phi) % 360.0, "hinge"
            return jt.label, phi, "spin"
        return jt.label, self.ball_tilt(j), "ball"

    def ball_tilt(self, j):
        jt = self.joints[j]
        p = self.pr.parts[jt.part]
        sl = {i: d for i, h, lp, d in C.local_slots(p.type, p.params)}
        da = self.R[jt.a] @ (p.R @ -sl[0])
        db = self.R[jt.b] @ (p.R @ sl[1])
        return math.degrees(math.acos(max(-1.0, min(1.0, float(da @ db)))))

    def analyze(self):
        st = self.pr.settings
        issues = []
        for j, jt in enumerate(self.joints):
            p = self.pr.parts[jt.part]
            if jt.hinge:
                _, ang, _ = self.joint_reading(j)
                lo, hi = p.tdef["min_angle"], p.tdef["max_angle"]
                if ang < lo - 0.5 or ang > hi + 0.5:
                    issues.append(Issue("bind", f"lim:{jt.part}",
                                        f"{p.name} is at its limit ({ang:.0f} deg)", [jt.part],
                                        bool(st.get("stop_on_limits", True))))
            elif jt.kind == "sph":
                tilt = self.ball_tilt(j)
                if tilt > p.tdef["cone"] + 0.5:
                    issues.append(Issue("bind", f"lim:{jt.part}",
                                        f"{p.name} tilted past {p.tdef['cone']:.0f} deg", [jt.part],
                                        bool(st.get("stop_on_limits", True))))
        issues += self.collisions()
        return issues

    def capsules(self):
        caps = []   # (item id, body, p0, p1, radius)
        for rid, r in self.pr.rods.items():
            b = self.node_body[rid]
            caps.append((rid, b, self.X(b, r.p0 + r.d * 1.0), self.X(b, r.p1 - r.d * 1.0),
                         C.ROD_DIA / 2))
        for pid, p in self.pr.parts.items():
            fam = p.tdef["family"]
            if fam in ("connector", "joint") or p.type == "clip":
                # each claw of a flat connector is a short capsule as thick as the connector
                for idx, half, lp, ld in C.local_slots(p.type, p.params):
                    b = self.body_of_item(pid, half)
                    a0 = self.X(b, p.pos + p.R @ (ld * 2.5))
                    a1 = self.X(b, p.pos + p.R @ (ld * (C.slot_reach(p.type) + 0.5)))
                    caps.append((pid, b, a0, a1, C.CONN_T / 2))
            elif p.type in ("spacer", "spacer3", "cliphole"):
                b = self.body_of_item(pid)
                c = self.X(b, p.pos)
                caps.append((pid, b, c, c, 5.0))
        return caps

    def collisions(self):
        caps = self.capsules()
        if len(caps) < 2:
            return []
        lo = np.array([np.minimum(c[2], c[3]) - c[4] for c in caps])
        hi = np.array([np.maximum(c[2], c[3]) + c[4] for c in caps])
        over = np.all((lo[:, None, :] <= hi[None, :, :]) & (lo[None, :, :] <= hi[:, None, :]), axis=2)
        ii, jj = np.nonzero(np.triu(over, 1))
        out, seen = [], set()
        nb_cache = {}

        def nbrs(x):
            if x not in nb_cache:
                nb_cache[x] = self.pr._neighbors(x)
            return nb_cache[x]

        for i, j in zip(ii, jj):
            a, b = caps[i], caps[j]
            if a[1] == b[1] or a[0] == b[0]:
                continue
            if b[0] in nbrs(a[0]) or (nbrs(a[0]) & nbrs(b[0]) & set(self.pr.parts)):
                continue
            d = g.seg_seg_dist(a[2], a[3], b[2], b[3])
            if d < a[4] + b[4] - 0.3:
                x, y = sorted((a[0], b[0]))
                key = f"hit:{x}:{y}"
                if key in seen:
                    continue
                seen.add(key)
                out.append(Issue("bind", key, f"{self.pr.name_of(x)} hits {self.pr.name_of(y)}",
                                 [x, y], bool(self.pr.settings.get("stop_on_collision", True))))
        return out

    def item_point(self, iid):
        if iid in self.pr.rods:
            r = self.pr.rods[iid]
            return self.X(self.node_body[iid], (r.p0 + r.p1) / 2), (r.p0 + r.p1) / 2
        p = self.pr.parts[iid]
        return self.X(self.body_of_item(iid), p.pos), p.pos
