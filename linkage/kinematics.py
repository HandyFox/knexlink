"""Motion solver and binding checks.

Every shaft has one unknown (its twist angle) and every slider has one (its
position). Every push rod says "these two holes stay exactly L apart".
Moving a handle = forcing one unknown and solving the rest, one small step at
a time so the linkage can't jump through a dead-center position unnoticed.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import geometry as g
from .model import fit_links

STEP_ANGLE = math.radians(0.75)   # max driver step per solve (shafts)
STEP_SLIDE = 0.05                 # inches (sliders)
SNAP_ANGLE = math.radians(15.0)   # a follower jumping more than this in one step = snap-through
SNAP_SLIDE = 1.5


@dataclass
class Issue:
    level: str          # "warn" or "bind"
    key: str            # stable id for de-duplication
    text: str
    parts: list = field(default_factory=list)
    stop: bool = False  # True = motion stops here


class Mechanism:
    """Compiled, solvable version of a Project."""

    def __init__(self, project):
        fit_links(project)
        self.project = project
        P = project.parts
        self.dofs = []            # (part_id, "rot"|"lin")
        self.dof_of = {}          # part id (shaft/slider) -> dof index
        for p in P.values():
            if p.kind == "shaft":
                s, a, L = p.axis()
                if L < 1e-6:
                    continue
                self.dof_of[p.id] = len(self.dofs)
                self.dofs.append((p.id, "rot"))
            elif p.kind == "slider":
                s, d, L = p.track()
                if L < 1e-6:
                    continue
                self.dof_of[p.id] = len(self.dofs)
                self.dofs.append((p.id, "lin"))
        n = len(self.dofs)
        self.q0 = np.zeros(n)
        self.lo = np.full(n, -np.inf)
        self.hi = np.full(n, np.inf)
        self.axis = {}            # shaft id -> (c, a, L)
        self.track = {}           # slider id -> (s, d, L)
        for i, (pid, kind) in enumerate(self.dofs):
            p = P[pid]
            if kind == "rot":
                self.axis[pid] = p.axis()
                self.lo[i] = math.radians(float(p["min_angle"]))
                self.hi[i] = math.radians(float(p["max_angle"]))
            else:
                s, d, L = p.track()
                self.track[pid] = (s, d, L)
                self.q0[i] = float(p["neutral"])
                self.lo[i], self.hi[i] = 0.0, L

        # attachment points (arm tips, slider blocks): pid -> (dof, neutral point)
        self.attach = {}
        self.arm_info = {}        # arm id -> (dof, root0, tip0, length)
        for p in P.values():
            if p.kind == "arm" and p["shaft"] in self.dof_of:
                c, a, _ = self.axis[p["shaft"]]
                root = c + a * float(p["at"])
                tip = root + g.angle_dir(a, float(p["angle"])) * float(p["length"])
                d = self.dof_of[p["shaft"]]
                self.attach[p.id] = (d, tip)
                self.arm_info[p.id] = (d, root, tip, float(p["length"]))
            elif p.kind == "slider" and p.id in self.dof_of:
                self.attach[p.id] = (self.dof_of[p.id], None)

        self.links = []           # (link id, a id, b id, length)
        for p in P.values():
            if p.kind == "link" and p["a"] in self.attach and p["b"] in self.attach \
                    and p["a"] != p["b"]:
                self.links.append((p.id, p["a"], p["b"], float(p["length"])))

        self.handle_dofs = [self.dof_of[s.id] for s in project.of_kind("shaft")
                            if s.id in self.dof_of and project.is_handle_shaft(s.id)]
        self._build_graph()

    # ------------------------------------------------------------ kinematics
    def _build_graph(self):
        self.adj = {i: set() for i in range(len(self.dofs))}
        for _, a, b, _ in self.links:
            da, db = self.attach[a][0], self.attach[b][0]
            if da != db:
                self.adj[da].add(db)
                self.adj[db].add(da)

    def component(self, d):
        seen, stack = {d}, [d]
        while stack:
            x = stack.pop()
            for y in self.adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        return seen

    def distances_from(self, d):
        dist, frontier = {d: 0}, [d]
        while frontier:
            nxt = []
            for x in frontier:
                for y in self.adj[x]:
                    if y not in dist:
                        dist[y] = dist[x] + 1
                        nxt.append(y)
            frontier = nxt
        return dist

    def body_matrix(self, pid, q):
        """4x4 world transform for a shaft or slider body at state q."""
        i = self.dof_of[pid]
        if self.dofs[i][1] == "rot":
            c, a, _ = self.axis[pid]
            R = g.rot_matrix(a, q[i] - self.q0[i])
            return g.rigid(R, c - R @ c)
        s, d, _ = self.track[pid]
        return g.rigid(np.eye(3), d * (q[i] - self.q0[i]))

    def point(self, pid, q):
        d, p0 = self.attach[pid]
        sid = self.dofs[d][0]
        if self.dofs[d][1] == "rot":
            c, a, _ = self.axis[sid]
            return c + g.rot_matrix(a, q[d]) @ (p0 - c)
        s, dirv, _ = self.track[sid]
        return s + dirv * q[d]

    def dpoint(self, pid, q, P=None):
        """d(point)/d(own dof)."""
        d, _ = self.attach[pid]
        sid = self.dofs[d][0]
        if P is None:
            P = self.point(pid, q)
        if self.dofs[d][1] == "rot":
            c, a, _ = self.axis[sid]
            return np.cross(a, P - c)
        return self.track[sid][1]

    def residual(self, q):
        r = np.zeros(len(self.links))
        for k, (_, a, b, L) in enumerate(self.links):
            r[k] = g.norm(self.point(a, q) - self.point(b, q)) - L
        return r

    def jacobian(self, q):
        J = np.zeros((len(self.links), len(self.dofs)))
        for k, (_, a, b, L) in enumerate(self.links):
            Pa, Pb = self.point(a, q), self.point(b, q)
            e = g.unit(Pa - Pb)
            da, db = self.attach[a][0], self.attach[b][0]
            J[k, da] += e @ self.dpoint(a, q, Pa)
            J[k, db] -= e @ self.dpoint(b, q, Pb)
        return J

    def solve(self, q, fixed, tol=1e-7, iters=40):
        """Newton (minimum change) with the `fixed` dofs locked. Returns (q, ok)."""
        q = q.copy()
        free = np.array([i for i in range(len(self.dofs)) if i not in fixed], dtype=int)
        if len(self.links) == 0 or len(free) == 0:
            r = self.residual(q)
            return q, (len(r) == 0 or np.max(np.abs(r)) < 1e-5)
        rot = np.array([self.dofs[i][1] == "rot" for i in free])
        for _ in range(iters):
            r = self.residual(q)
            if np.max(np.abs(r)) < tol:
                return q, True
            J = self.jacobian(q)[:, free]
            A = J @ J.T
            lam = 1e-10 * (np.trace(A) + 1.0)
            try:
                dq = -J.T @ np.linalg.solve(A + lam * np.eye(len(A)), r)
            except np.linalg.LinAlgError:
                return q, False
            m = max(np.max(np.abs(dq[rot])) / 0.3 if rot.any() else 0.0,
                    np.max(np.abs(dq[~rot])) / 2.0 if (~rot).any() else 0.0, 1.0)
            q[free] += dq / m
        r = self.residual(q)
        return q, bool(np.max(np.abs(r)) < 1e-5)

    def velocities(self, q, driver, fixed):
        """dq / d(driver): how every dof moves per unit of driver motion."""
        n = len(self.dofs)
        v = np.zeros(n)
        v[driver] = 1.0
        if not self.links:
            return v
        free = np.array([i for i in range(n) if i not in fixed and i != driver], dtype=int)
        if len(free) == 0:
            return v
        J = self.jacobian(q)
        Jf = J[:, free]
        A = Jf @ Jf.T
        lam = 1e-10 * (np.trace(A) + 1.0)
        try:
            v[free] = -Jf.T @ np.linalg.solve(A + lam * np.eye(len(A)), J[:, driver])
        except np.linalg.LinAlgError:
            pass
        return v

    def held_for(self, driver):
        """Other handles are held still while you move one."""
        if driver in self.handle_dofs:
            return {d for d in self.handle_dofs if d != driver}
        return set()

    def free_motions(self, driver):
        """Extra floppy motions in the driver's linkage (0 = fully determined)."""
        comp = sorted(self.component(driver))
        held = self.held_for(driver)
        unknown = [i for i in comp if i != driver and i not in held]
        if not unknown:
            return 0
        rows = [k for k, (_, a, b, _) in enumerate(self.links)
                if self.attach[a][0] in comp or self.attach[b][0] in comp]
        if not rows:
            return len(unknown)
        J = self.jacobian(self.q0)[np.ix_(rows, unknown)]
        rank = np.linalg.matrix_rank(J, tol=1e-6)
        return len(unknown) - rank

    # ------------------------------------------------------------ motion
    def move(self, q, driver, target, prev_issues=None):
        """Walk the driver toward `target`. Stops at the first new binding.

        Returns (q_reached, issues_at_reached, stop_issue_or_None)
        """
        held = self.held_for(driver)
        fixed = held | {driver}
        lin = self.dofs[driver][1] == "lin"
        target = float(np.clip(target, self.lo[driver], self.hi[driver]))
        cur = float(q[driver])
        step = STEP_SLIDE if lin else STEP_ANGLE
        nsteps = max(1, int(math.ceil(abs(target - cur) / step)))
        delta = (target - cur) / nsteps
        if prev_issues is None:
            prev_issues = self.analyze(q, driver)
        prev_keys = {i.key for i in prev_issues if i.stop}
        issues = prev_issues
        if abs(target - cur) < 1e-12:
            return q, issues, None
        rotmask = np.array([k == "rot" for _, k in self.dofs])
        for k in range(nsteps):
            v = self.velocities(q, driver, fixed)
            guess = q + v * delta
            guess[driver] = cur + (k + 1) * delta
            for h in held:
                guess[h] = q[h]
            qs, ok = self.solve(guess, fixed)
            if ok:
                dq = np.abs(qs - q)
                dq[driver] = 0.0
                if np.any(dq[rotmask] > SNAP_ANGLE) or np.any(dq[~rotmask] > SNAP_SLIDE):
                    ok = False
            if not ok:
                stop = self._cant_reach_issue(guess, fixed)
                return q, issues, stop
            new_issues = self.analyze(qs, driver)
            new_stops = [i for i in new_issues if i.stop and i.key not in prev_keys]
            if new_stops:
                return q, issues, new_stops[0]
            q, issues = qs, new_issues
        return q, issues, None

    def _cant_reach_issue(self, guess, fixed):
        qs, _ = self.solve(guess, fixed, iters=60)
        r = np.abs(self.residual(qs))
        parts = []
        text = "Linkage locks up here (can't move any further)"
        if len(r):
            k = int(np.argmax(r))
            lid = self.links[k][0]
            parts = [lid]
            name = self.project.parts[lid].name
            text = (f"Locks up: '{name}' would need to change length "
                    f"(dead center / over-travel)")
        return Issue("bind", "lockup", text, parts, True)

    def sweep(self, q_start, driver):
        """Run the driver to both ends. Returns dict with ranges and bind points."""
        out = {"driver": driver, "ends": {}, "traces": []}
        for sign in (+1, -1):
            q = q_start.copy()
            issues = self.analyze(q, driver)
            trace = [q.copy()]
            lim = self.hi[driver] if sign > 0 else self.lo[driver]
            if not np.isfinite(lim):
                lim = q[driver] + sign * math.radians(360)
            chunk = math.radians(5) if self.dofs[driver][1] == "rot" else 0.25
            stop = None
            while (lim - q[driver]) * sign > 1e-9:
                tgt = q[driver] + sign * min(chunk, abs(lim - q[driver]))
                q2, issues, stop = self.move(q, driver, tgt, issues)
                if np.allclose(q2, q) and stop is None:
                    break
                q = q2
                trace.append(q.copy())
                if stop is not None:
                    break
            out["ends"][sign] = {"q": q.copy(), "stop": stop,
                                 "at_limit": stop is None}
            out["traces"].append(trace)
        return out

    # ------------------------------------------------------------ checks
    def analyze(self, q, driver=None):
        P = self.project.parts
        st = self.project.settings
        issues = []
        # 1. assembly
        if self.links:
            r = self.residual(q)
            for k, (lid, a, b, L) in enumerate(self.links):
                if abs(r[k]) > 1e-3:
                    issues.append(Issue("bind", f"len:{lid}",
                                        f"'{P[lid].name}' doesn't fit (off by {abs(r[k]):.3f}\")",
                                        [lid], True))
        # 2. push efficiency and rod-end tilt
        dist = self.distances_from(driver) if driver is not None else {}
        for lid, a, b, L in self.links:
            Pa, Pb = self.point(a, q), self.point(b, q)
            e = g.unit(Pb - Pa)
            da, db = self.attach[a][0], self.attach[b][0]
            ends = [(a, da, Pa), (b, db, Pb)]
            if driver is not None and da in dist and db in dist and dist[da] != dist[db]:
                driven = [ends[0] if dist[da] > dist[db] else ends[1]]
            else:
                driven = ends
            for pid, d, Pp in driven:
                if d == driver:
                    continue
                t = self.dpoint(pid, q, Pp)
                tn = g.norm(t)
                if tn < 1e-9:
                    continue
                eff = abs(float(e @ t)) / tn
                if eff < st["bind_push"]:
                    issues.append(Issue("bind", f"dc:{lid}:{pid}",
                                        f"Dead center: '{P[lid].name}' pushes '{P[pid].name}' "
                                        f"at {eff * 100:.0f}% (toggle point)", [lid, pid], False))
                elif eff < st["warn_push"]:
                    issues.append(Issue("warn", f"dc:{lid}:{pid}",
                                        f"Poor push angle: '{P[lid].name}' on '{P[pid].name}' "
                                        f"only {eff * 100:.0f}% effective", [lid, pid], False))
            lim = float(P[lid].get("max_misalign", 20.0))
            for pid, d, Pp in ends:
                if P[pid].kind != "arm":
                    continue
                ax = self.axis[self.dofs[d][0]][1]
                tilt = math.degrees(math.asin(min(1.0, abs(float(e @ ax)))))
                if tilt > lim:
                    issues.append(Issue("bind", f"tilt:{lid}:{pid}",
                                        f"Rod end over-angled: '{P[lid].name}' at '{P[pid].name}' "
                                        f"tilts {tilt:.0f} deg (max {lim:.0f})", [lid, pid],
                                        bool(st.get("stop_on_overangle", True))))
                elif tilt > 0.8 * lim:
                    issues.append(Issue("warn", f"tilt:{lid}:{pid}",
                                        f"Rod end near its limit: '{P[lid].name}' at '{P[pid].name}' "
                                        f"tilts {tilt:.0f} deg", [lid, pid], False))
        # 3. stops and track ends (followers only; the driver is clamped)
        for i, (pid, kind) in enumerate(self.dofs):
            if i == driver:
                continue
            if q[i] > self.hi[i] + 1e-6 or q[i] < self.lo[i] - 1e-6:
                txt = (f"'{P[pid].name}' hits the end of its travel" if kind == "lin"
                       else f"'{P[pid].name}' hits its stop ({math.degrees(q[i]):.0f} deg)")
                issues.append(Issue("bind", f"stop:{pid}", txt, [pid], True))
        # 4. interference
        issues += self.collisions(q)
        return issues

    def capsules(self, q):
        """(owner, part id, p0, p1, radius, exclude-set) for interference checks."""
        P = self.project.parts
        caps = []
        for pid, (c, a, L) in self.axis.items():
            p = P[pid]
            if not p.get("collide", True):
                continue
            r = float(p["dia"]) / 2
            trim = min(r, L / 2 - 1e-3)
            caps.append((pid, pid, c + a * trim, c + a * (L - trim), r, set()))
        for aid, (d, root0, tip0, length) in self.arm_info.items():
            p = P[aid]
            if not p.get("collide", True):
                continue
            sid = self.dofs[d][0]
            M = self.body_matrix(sid, q)
            root = M[:3, :3] @ root0 + M[:3, 3]
            tip = M[:3, :3] @ tip0 + M[:3, 3]
            u = g.unit(tip - root)
            shaft_r = float(P[sid]["dia"]) / 2
            start = root + u * min(shaft_r, length * 0.5)
            rr = float(p["thick"]) / 2
            caps.append((sid, aid, start, tip, rr, set()))
            if p.get("grip"):
                gl = float(p["grip_len"])
                caps.append((sid, aid, tip, tip + u * gl, float(p["grip_dia"]) / 2, set()))
        for sid, (s, dv, L) in self.track.items():
            p = P[sid]
            if not p.get("collide", True):
                continue
            c = self.point(sid, q)
            b = float(p["block"])
            caps.append((sid, sid, c - dv * b * 0.25, c + dv * b * 0.25, b / 2, set()))
        for lid, a, b, L in self.links:
            p = P[lid]
            if not p.get("collide", True):
                continue
            Pa, Pb = self.point(a, q), self.point(b, q)
            u = g.unit(Pb - Pa)
            trim = min(0.9, 0.3 * L)
            caps.append((lid, lid, Pa + u * trim, Pb - u * trim, float(p["dia"]) / 2, {a, b}))
        for p in self.project.of_kind("frame"):
            if not p.get("collide", True):
                continue
            s, e = g.vec(p["start"]), g.vec(p["end"])
            L = g.norm(e - s)
            if L < 1e-6:
                continue
            u = (e - s) / L
            r = float(p["size"]) / 2
            trim = min(r, L / 2 - 1e-3)
            caps.append(("ground", p.id, s + u * trim, e - u * trim, r * 1.2, set()))
        return caps

    def collisions(self, q):
        caps = self.capsules(q)
        if len(caps) < 2:
            return []
        P = self.project.parts
        lo = np.array([np.minimum(c[2], c[3]) - c[4] for c in caps])
        hi = np.array([np.maximum(c[2], c[3]) + c[4] for c in caps])
        over = np.all((lo[:, None, :] <= hi[None, :, :]) & (lo[None, :, :] <= hi[:, None, :]), axis=2)
        idx_i, idx_j = np.nonzero(np.triu(over, 1))
        # concentric shafts (rod inside tube) never "hit" each other
        concentric = set()
        ids = list(self.axis.keys())
        for x in range(len(ids)):
            for y in range(x + 1, len(ids)):
                c1, a1, _ = self.axis[ids[x]]
                c2, a2, _ = self.axis[ids[y]]
                if abs(abs(float(a1 @ a2)) - 1) < 1e-6 and g.point_line_dist(c2, c1, a1) < 1e-3:
                    concentric.add(frozenset((ids[x], ids[y])))
        out = []
        for i, j in zip(idx_i, idx_j):
            ci, cj = caps[i], caps[j]
            if ci[0] == cj[0] or ci[1] == cj[1]:
                continue
            if cj[1] in ci[5] or ci[1] in cj[5]:
                continue
            if frozenset((ci[0], cj[0])) in concentric:
                # arms on a nested rod vs the tube: only a hit if inside its length
                pass
            if frozenset((ci[1], cj[1])) in concentric:
                continue
            # links sharing a hole
            if ci[5] and cj[5] and (ci[5] & cj[5]):
                continue
            dmin = g.seg_seg_dist(ci[2], ci[3], cj[2], cj[3])
            if dmin < ci[4] + cj[4] - 0.02:
                a, b = sorted((ci[1], cj[1]))
                out.append(Issue("bind", f"hit:{a}:{b}",
                                 f"Interference: '{P[a].name}' hits '{P[b].name}'",
                                 [a, b], bool(self.project.settings.get("stop_on_collision", True))))
        # de-duplicate (grip + bar can both hit)
        seen, uniq = set(), []
        for it in out:
            if it.key not in seen:
                seen.add(it.key)
                uniq.append(it)
        return uniq

    # ------------------------------------------------------------ readouts
    def readouts(self, q, driver=None):
        """Rows of (part id, label, value text, travel inches or None)."""
        P = self.project.parts
        rows = []
        for i, (pid, kind) in enumerate(self.dofs):
            if kind == "rot":
                deg = math.degrees(q[i] - self.q0[i])
                rows.append((pid, P[pid].name, deg, "deg"))
            else:
                rows.append((pid, P[pid].name, q[i] - self.q0[i], "in"))
        for aid, (d, root0, tip0, length) in self.arm_info.items():
            travel = self.arm_radius(aid) * (q[d] - self.q0[d])
            rows.append((aid, P[aid].name, travel, "in"))
        return rows

    def arm_radius(self, aid):
        """Radius used for travel: the hole, or the middle of the grip on a handle."""
        p = self.project.parts[aid]
        r = float(p["length"])
        if p.get("grip"):
            r += float(p["grip_len"]) * 0.5
        return r

    def grab_point(self, pid, q):
        """World point you grab: grip center, arm hole, or slider block."""
        p = self.project.parts[pid]
        if p.kind == "arm":
            d, root0, tip0, length = self.arm_info[pid]
            u = g.unit(tip0 - root0)
            local = root0 + u * self.arm_radius(pid)
            M = self.body_matrix(self.dofs[d][0], q)
            return M[:3, :3] @ local + M[:3, 3]
        return self.point(pid, q)

    def driver_scale(self, driver):
        """Inches of travel per unit of driver dof (grip radius for a handle)."""
        pid, kind = self.dofs[driver]
        if kind == "lin":
            return 1.0
        best = None
        for aid, (d, _, _, length) in self.arm_info.items():
            if d != driver:
                continue
            if self.project.parts[aid].get("grip"):
                return self.arm_radius(aid)
            best = max(best or 0.0, length)
        return best or 1.0
