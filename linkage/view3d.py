"""3D view: draws the parts and lets you grab a handle and pull it.

Mouse:
  left-drag on a grip / arm hole / slider block  -> move that part
  left-click a part                              -> select it
  left-drag empty space                          -> orbit
  shift+left-drag or middle-drag                 -> pan
  wheel                                          -> zoom
"""
import math

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from qtpy import QtCore

from . import geometry as g

COLORS = {
    "rod": "#5b6770",
    "tube": "#8fa3b3",
    "stripe": "#f2b705",
    "arm": "#3a6ea5",
    "grip": "#1d1d1d",
    "link": "#d9d9d9",
    "ball": "#7a7a7a",
    "slider": "#2e7d32",
    "track": "#a5a5a5",
    "frame": "#4a4a4a",
    "bearing": "#3b3b3b",
    "warn": "#ff9800",
    "bind": "#ff1744",
    "select": "#d500f9",
    "trace": "#00897b",
    "grid": "#c9cdd2",
}

GRAB_PIXELS = 22


class SceneView(QtInteractor):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.set_background("#f4f6f8", top="#dfe6ee")
        try:
            self.enable_anti_aliasing("ssaa")
        except Exception:
            pass
        self.add_axes(line_width=3, labels_off=False)
        self.mech = None
        self.project = None
        self.part_actors = {}      # pid -> list of (actor, base color, mover)
        self.actor_owner = {}      # actor address -> pid
        self.trace_actors = []
        self.grid_actor = None
        self.selected = set()
        self.status = {}
        self.q = None
        # callbacks set by the main window
        self.on_pick = None        # fn(pid or None)
        self.on_drag_start = None  # fn(pid) -> bool
        self.on_drag = None        # fn(target_value)
        self.on_drag_end = None    # fn()
        self._press = None
        self._drag = None
        self._pending_target = None

    # ------------------------------------------------------------ building
    def clear_parts(self):
        for lst in self.part_actors.values():
            for actor, _, _ in lst:
                self.remove_actor(actor, render=False)
        self.part_actors.clear()
        self.actor_owner.clear()
        self.clear_traces(render=False)

    def _add(self, pid, mesh, color, mover, **kw):
        kw.setdefault("smooth_shading", True)
        kw.setdefault("specular", 0.3)
        actor = self.add_mesh(mesh, color=color, render=False, reset_camera=False, **kw)
        self.part_actors.setdefault(pid, []).append((actor, color, mover))
        self.actor_owner[actor.GetAddressAsString("vtkActor")] = pid
        return actor

    def build(self, mech, reset_camera=False):
        self.mech = mech
        self.project = mech.project
        self.clear_parts()
        P = self.project.parts
        # shafts / tubes
        for pid, (c, a, L) in mech.axis.items():
            p = P[pid]
            r = float(p["dia"]) / 2
            tube = p.get("style") == "Tube"
            mover = ("body", pid)
            cyl = pv.Cylinder(center=c + a * L / 2, direction=a, radius=r, height=L,
                              resolution=40, capping=True)
            self._add(pid, cyl, COLORS["tube" if tube else "rod"], mover)
            # stripe so you can see it twist
            u0, v0 = g.axis_frame(a)
            stripe = pv.Cylinder(center=c + a * L / 2 + u0 * r * 0.93, direction=a,
                                 radius=max(0.06, r * 0.16), height=L * 0.96, resolution=12)
            self._add(pid, stripe, COLORS["stripe"], mover)
            # bearing blocks at both ends (fixed)
            bsize = max(1.0, r * 2.6)
            for t in (min(0.6, L * 0.1), L - min(0.6, L * 0.1)):
                if tube or not self._inside_tube(pid, c + a * t):
                    blk = self._box(c + a * t, a, u0, (bsize * 0.5, bsize, bsize))
                    self._add(pid, blk, COLORS["bearing"], None, smooth_shading=False)
        # arms and handle levers
        for aid, (d, root, tip, length) in mech.arm_info.items():
            p = P[aid]
            sid = mech.dofs[d][0]
            c, a, _ = mech.axis[sid]
            mover = ("body", sid)
            u = g.unit(tip - root)
            w, t = float(p["width"]), float(p["thick"])
            ext = w * 0.5
            bar = self._box(root + u * (length + ext) / 2, u, a, (length + ext, t, w), along_first=True)
            self._add(aid, bar, COLORS["arm"], mover, smooth_shading=False)
            hub_r = max(float(P[sid]["dia"]) / 2 + 0.2, w * 0.6)
            hub = pv.Cylinder(center=root, direction=a, radius=hub_r, height=t * 2.2, resolution=30)
            self._add(aid, hub, COLORS["arm"], mover)
            if p.get("grip"):
                gl, gd = float(p["grip_len"]), float(p["grip_dia"])
                grip = pv.Cylinder(center=tip + u * gl / 2, direction=u, radius=gd / 2,
                                   height=gl, resolution=30)
                self._add(aid, grip, COLORS["grip"], mover)
                knob = pv.Sphere(radius=gd * 0.62, center=tip + u * gl, theta_resolution=24,
                                 phi_resolution=24)
                self._add(aid, knob, COLORS["grip"], mover)
            else:
                pin = pv.Cylinder(center=tip, direction=a, radius=max(0.12, w * 0.22),
                                  height=t * 3.0, resolution=20)
                self._add(aid, pin, COLORS["ball"], mover)
        # sliders
        for sid, (s, dv, L) in mech.track.items():
            p = P[sid]
            b = float(p["block"])
            u0, _ = g.axis_frame(dv)
            rail = self._box(s + dv * L / 2 - u0 * b * 0.55, dv, u0, (L + b, b * 0.15, b * 0.5),
                             along_first=True)
            self._add(sid, rail, COLORS["track"], None, smooth_shading=False)
            neutral = s + dv * float(p["neutral"])
            blk = self._box(neutral, dv, u0, (b, b, b), along_first=True)
            self._add(sid, blk, COLORS["slider"], ("body", sid), smooth_shading=False)
        # push rods (unit cylinder along +Z, scaled every frame)
        unit_rod = pv.Cylinder(center=(0, 0, 0.5), direction=(0, 0, 1), radius=0.5, height=1.0,
                               resolution=24)
        for lid, a_id, b_id, L in mech.links:
            p = P[lid]
            r = float(p["dia"]) / 2
            self._add(lid, unit_rod.copy(), COLORS["link"], ("rod", lid, r))
            for end in (a_id, b_id):
                ball = pv.Sphere(radius=r * 1.5, center=(0, 0, 0), theta_resolution=20,
                                 phi_resolution=20)
                self._add(lid, ball, COLORS["ball"], ("ball", end))
        # frame
        for p in self.project.of_kind("frame"):
            s, e = g.vec(p["start"]), g.vec(p["end"])
            L = g.norm(e - s)
            if L < 1e-6:
                continue
            u = (e - s) / L
            z = g.UP if abs(u @ g.UP) < 0.9 else g.FWD
            sz = float(p["size"])
            box = self._box((s + e) / 2, u, z, (L, sz, sz), along_first=True)
            self._add(p.id, box, COLORS["frame"], None, smooth_shading=False, opacity=0.85)
        self._build_grid()
        if reset_camera:
            self.view_preset("3d")

    def _inside_tube(self, pid, point):
        """True if a point on rod `pid` sits inside another (larger) concentric shaft."""
        for other, (c, a, L) in self.mech.axis.items():
            if other == pid:
                continue
            if g.point_line_dist(point, c, a) < 1e-3:
                t = float((point - c) @ a)
                if 0 <= t <= L and self.project.parts[other]["dia"] > self.project.parts[pid]["dia"]:
                    return True
        return False

    def _box(self, center, d1, d2, size, along_first=True):
        """Box with edge lengths size=(along d1, along d2, along d1 x d2)."""
        e1 = g.unit(d1)
        e2 = g.vec(d2) - (g.vec(d2) @ e1) * e1
        e2 = g.unit(e2, g.axis_frame(e1)[0])
        e3 = np.cross(e1, e2)
        box = pv.Cube(center=(0, 0, 0), x_length=size[0], y_length=size[1], z_length=size[2])
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = e1, e2, e3, g.vec(center)
        return box.transform(M, inplace=False)

    def _build_grid(self):
        if self.grid_actor is not None:
            self.remove_actor(self.grid_actor, render=False)
            self.grid_actor = None
        pts = self._all_points()
        if pts is None:
            lo, hi = np.array([-36, -36, 0.0]), np.array([36, 36, 0.0])
        else:
            lo, hi = pts.min(axis=0), pts.max(axis=0)
        x0 = math.floor((lo[0] - 12) / 12) * 12
        x1 = math.ceil((hi[0] + 12) / 12) * 12
        y0 = math.floor((lo[1] - 12) / 12) * 12
        y1 = math.ceil((hi[1] + 12) / 12) * 12
        z = min(0.0, math.floor(lo[2] / 12) * 12) if pts is not None else 0.0
        lines = []
        for x in np.arange(x0, x1 + 0.1, 12):
            lines.append(([x, y0, z], [x, y1, z]))
        for y in np.arange(y0, y1 + 0.1, 12):
            lines.append(([x0, y, z], [x1, y, z]))
        points = np.array([p for ln in lines for p in ln], dtype=float)
        cells = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(lines))])
        grid = pv.PolyData(points, lines=cells)
        self.grid_actor = self.add_mesh(grid, color=COLORS["grid"], line_width=1, render=False,
                                        reset_camera=False, pickable=False)

    def _all_points(self):
        pts = []
        for lst in self.part_actors.values():
            for actor, _, _ in lst:
                b = actor.GetBounds()
                if b[0] <= b[1]:
                    pts.append([b[0], b[2], b[4]])
                    pts.append([b[1], b[3], b[5]])
        return np.array(pts) if pts else None

    # ------------------------------------------------------------ updating
    def update_pose(self, q, status=None, selected=None, render=True):
        self.q = q
        if status is not None:
            self.status = status
        if selected is not None:
            self.selected = set(selected)
        mech = self.mech
        if mech is None:
            return
        mats = {pid: mech.body_matrix(pid, q) for pid in mech.dof_of}
        link_pts = {}
        for lid, a, b, L in mech.links:
            link_pts[lid] = (mech.point(a, q), mech.point(b, q))
        for pid, lst in self.part_actors.items():
            st = self.status.get(pid)
            for actor, base, mover in lst:
                if mover is not None:
                    if mover[0] == "body":
                        actor.user_matrix = mats[mover[1]]
                    elif mover[0] == "rod":
                        pa, pb = link_pts[mover[1]]
                        r = mover[2]
                        M = g.frame_along(pa, pb)
                        S = np.diag([2 * r, 2 * r, g.norm(pb - pa), 1.0])
                        actor.user_matrix = M @ S
                    elif mover[0] == "ball":
                        M = np.eye(4)
                        M[:3, 3] = mech.point(mover[1], q)
                        actor.user_matrix = M
                if st == "bind":
                    col = COLORS["bind"]
                elif st == "warn":
                    col = COLORS["warn"]
                elif pid in self.selected:
                    col = COLORS["select"]
                else:
                    col = base
                actor.prop.color = col
        if render:
            self.render()

    def set_traces(self, paths):
        """paths: list of (N,3) point arrays."""
        self.clear_traces(render=False)
        for pts in paths:
            pts = np.asarray(pts, dtype=float)
            if len(pts) < 2 or np.allclose(pts.min(axis=0), pts.max(axis=0), atol=1e-3):
                continue
            line = pv.lines_from_points(pts)
            act = self.add_mesh(line, color=COLORS["trace"], line_width=3, render=False,
                                reset_camera=False, pickable=False)
            self.trace_actors.append(act)
        self.render()

    def clear_traces(self, render=True):
        for act in self.trace_actors:
            self.remove_actor(act, render=False)
        self.trace_actors = []
        if render:
            self.render()

    # ------------------------------------------------------------ camera
    def view_preset(self, name):
        pts = self._all_points()
        if pts is None:
            center, radius = np.zeros(3), 36.0
        else:
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            center, radius = (lo + hi) / 2, max(12.0, g.norm(hi - lo) / 2)
        cam = self.camera
        dirs = {
            "3d": (np.array([0.75, 1.0, 0.65]), (0, 0, 1)),
            "front": (np.array([0, 1.0, 0]), (0, 0, 1)),
            "back": (np.array([0, -1.0, 0]), (0, 0, 1)),
            "side": (np.array([1.0, 0, 0]), (0, 0, 1)),
            "top": (np.array([0, 0, 1.0]), (0, 1, 0)),
        }
        d, up = dirs[name]
        d = d / np.linalg.norm(d)
        dist = radius * 3.2
        cam.focal_point = tuple(center)
        cam.position = tuple(center + d * dist)
        cam.up = up
        cam.parallel_projection = name != "3d"
        if name != "3d":
            cam.parallel_scale = radius * 1.15
        self.reset_camera_clipping_range()
        self.render()

    # ------------------------------------------------------------ screen math
    def _scale(self):
        try:
            return float(self.devicePixelRatioF())
        except Exception:
            return 1.0

    def _display_xy(self, ev):
        pos = ev.position() if hasattr(ev, "position") else ev.pos()
        s = self._scale()
        h = self.render_window.GetSize()[1]
        return pos.x() * s, h - 1 - pos.y() * s

    def world_to_display(self, p):
        ren = self.renderer
        ren.SetWorldPoint(float(p[0]), float(p[1]), float(p[2]), 1.0)
        ren.WorldToDisplay()
        x, y, z = ren.GetDisplayPoint()
        return np.array([x, y])

    def display_ray(self, x, y):
        ren = self.renderer
        out = []
        for z in (0.0, 1.0):
            ren.SetDisplayPoint(x, y, z)
            ren.DisplayToWorld()
            w = ren.GetWorldPoint()
            out.append(np.array(w[:3]) / (w[3] if abs(w[3]) > 1e-12 else 1.0))
        o, f = out
        return o, g.unit(f - o)

    def _find_grab(self, x, y):
        if self.mech is None or self.q is None:
            return None
        best, best_d = None, GRAB_PIXELS * self._scale()
        P = self.project.parts
        cands = []
        for aid in self.mech.arm_info:
            cands.append((0 if P[aid].get("grip") else 1, aid))
        for sid in self.mech.track:
            cands.append((1, sid))
        for prio, pid in sorted(cands):
            sp = self.world_to_display(self.mech.grab_point(pid, self.q))
            dd = math.hypot(sp[0] - x, sp[1] - y) - (6 if prio == 0 else 0)
            if dd < best_d:
                best, best_d = pid, dd
        return best

    def _pick_part(self, x, y):
        import vtk
        picker = vtk.vtkPropPicker()
        if not picker.Pick(x, y, 0, self.renderer):
            return None
        act = picker.GetActor()
        if act is None:
            return None
        return self.actor_owner.get(act.GetAddressAsString("vtkActor"))

    def drag_target(self, x, y):
        """Turn the mouse position into a new value for the grabbed dof."""
        dr = self._drag
        mech, q = self.mech, self.q
        d = dr["dof"]
        o, ray = self.display_ray(x, y)
        if dr["kind"] == "rot":
            c, a = dr["c"], dr["a"]
            denom = float(ray @ a)
            if abs(denom) > 0.25:
                t = float((dr["center"] - o) @ a) / denom
                hit = o + ray * t
                w = hit - dr["center"]
                if g.norm(w) > 0.5:
                    ang = math.atan2(float(w @ dr["v"]), float(w @ dr["u"]))
                    cur = q[d]
                    diff = (ang - cur + math.pi) % (2 * math.pi) - math.pi
                    return cur + diff
            # looking along the lever's swing plane: use screen motion along the swing
            gp = mech.grab_point(dr["pid"], q)
            tang = np.cross(a, gp - dr["center"])
            r = max(g.norm(tang), 1e-6)
            s0 = self.world_to_display(gp)
            s1 = self.world_to_display(gp + tang / r)
            sv = s1 - s0
            px_per_in = max(np.linalg.norm(sv), 1e-6)
            dm = np.array([x, y]) - dr["last"]
            dr["last"] = np.array([x, y])
            inches = float(dm @ (sv / px_per_in)) / px_per_in
            return q[d] + inches / r
        else:
            s, dv = dr["s"], dr["dv"]
            if abs(float(dv @ ray)) < 0.97:
                # closest point between the mouse ray and the track line
                return self._track_param(o, ray, s, dv) + dr["offset"]
            s0 = self.world_to_display(mech.point(dr["pid"], q))
            s1 = self.world_to_display(mech.point(dr["pid"], q) + dv)
            sv = s1 - s0
            ppi = max(np.linalg.norm(sv), 1e-6)
            dm = np.array([x, y]) - dr["last"]
            dr["last"] = np.array([x, y])
            return q[d] + float(dm @ (sv / ppi)) / ppi

    @staticmethod
    def _track_param(o, ray, s, dv):
        # parameter along track (from track start) of the point closest to the ray
        w0 = s - o
        b = float(dv @ ray)
        d = float(dv @ w0)
        e = float(ray @ w0)
        den = 1 - b * b
        tl = (b * e - d) / den
        return tl

    # ------------------------------------------------------------ mouse
    def mousePressEvent(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.LeftButton and \
                not (ev.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier):
            x, y = self._display_xy(ev)
            pid = self._find_grab(x, y)
            if pid is not None and self.on_drag_start and self.on_drag_start(pid):
                self._start_drag(pid, x, y)
                ev.accept()
                return
            self._press = (x, y)
        super().mousePressEvent(ev)

    def _start_drag(self, pid, x, y):
        mech = self.mech
        p = self.project.parts[pid]
        if p.kind == "arm":
            d = mech.arm_info[pid][0]
            sid = mech.dofs[d][0]
            c, a, _ = mech.axis[sid]
            root0, tip0 = mech.arm_info[pid][1], mech.arm_info[pid][2]
            u = g.unit(tip0 - root0)
            self._drag = {"pid": pid, "dof": d, "kind": "rot", "c": c, "a": a,
                          "center": root0, "u": u, "v": np.cross(a, u),
                          "last": np.array([x, y])}
        else:
            d = mech.dof_of[pid]
            s, dv, L = mech.track[pid]
            o, ray = self.display_ray(x, y)
            grab_t = self._track_param(o, ray, s, dv) if abs(float(dv @ ray)) < 0.97 else None
            self._drag = {"pid": pid, "dof": d, "kind": "lin", "s": s, "dv": dv,
                          "last": np.array([x, y]),
                          "offset": (self.q[d] - grab_t) if grab_t is not None else 0.0}
        self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        x, y = self._display_xy(ev)
        if self._drag is not None:
            self._pending_target = self.drag_target(x, y)
            QtCore.QTimer.singleShot(0, self._flush_drag)
            ev.accept()
            return
        if self._press is None:
            hover = self._find_grab(x, y)
            self.setCursor(QtCore.Qt.CursorShape.OpenHandCursor if hover
                           else QtCore.Qt.CursorShape.ArrowCursor)
        super().mouseMoveEvent(ev)

    def _flush_drag(self):
        if self._pending_target is None or self._drag is None:
            return
        tgt, self._pending_target = self._pending_target, None
        if self.on_drag:
            self.on_drag(tgt)

    def mouseReleaseEvent(self, ev):
        if self._drag is not None:
            self._flush_drag()
            self._drag = None
            self.setCursor(QtCore.Qt.CursorShape.ArrowCursor)
            if self.on_drag_end:
                self.on_drag_end()
            ev.accept()
            return
        if self._press is not None and ev.button() == QtCore.Qt.MouseButton.LeftButton:
            x, y = self._display_xy(ev)
            if math.hypot(x - self._press[0], y - self._press[1]) < 4 * self._scale():
                pid = self._pick_part(x, y)
                if self.on_pick:
                    self.on_pick(pid)
        self._press = None
        super().mouseReleaseEvent(ev)
