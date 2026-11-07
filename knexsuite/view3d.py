"""3D view.

Build mode: green dots = open slots, orange dots = free rod ends, blue discs = open
holes. Click one to make it active, then pick a part from the kit.
Move mode: grab any part that isn't on the ground and drag it.
Both modes: drag empty space to orbit, Shift+drag to pan, wheel to zoom.
"""
import math

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from qtpy import QtCore

from . import catalog as C
from . import geometry as g
from .meshes import part_meshes, rod_mesh

PORT_COLORS = {"slot": "#00c853", "end": "#ff6d00", "hole": "#2979ff"}
STATUS = {"bind": "#ff1744", "warn": "#ff9100", "select": "#d500f9"}
PICK_PX = 11


class KnexView(QtInteractor):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.set_background("#f3f5f8", top="#dce4ee")
        try:
            self.enable_anti_aliasing("ssaa")
        except Exception:
            pass
        self.add_axes(line_width=3)
        self.item_actors = {}          # item id -> [(actor, base color, body)]
        self.owner = {}           # actor address -> item id
        self.port_actors = []
        self.ports = []           # (kind, a, b, world point)
        self.active_actor = None
        self.trace_actors = []
        self.grid_actor = None
        self.asm = None
        self.project = None
        self.edit_mode = "build"
        self.on_port = None       # fn(port tuple)
        self.on_pick = None       # fn(item id or None)
        self.on_drag_start = None  # fn(item id, world point) -> bool
        self.on_drag = None       # fn(world target)
        self.on_drag_end = None
        self._press = None
        self._drag = None
        self._pending = None

    # ------------------------------------------------------------ build scene
    def _add(self, iid, mesh, color, body, **kw):
        kw.setdefault("smooth_shading", True)
        kw.setdefault("specular", 0.35)
        a = self.add_mesh(mesh, color=color, render=False, reset_camera=False, **kw)
        self.item_actors.setdefault(iid, []).append((a, color, body))
        self.owner[a.GetAddressAsString("vtkActor")] = iid
        return a

    def clear_scene(self):
        for lst in self.item_actors.values():
            for a, _, _ in lst:
                self.remove_actor(a, render=False)
        self.item_actors.clear()
        self.owner.clear()
        self.clear_ports(render=False)
        self.clear_traces(render=False)

    def build(self, project, asm, reset_camera=False):
        self.project, self.asm = project, asm
        self.clear_scene()
        for r in project.rods.values():
            self._add(r.id, rod_mesh(r), C.rod_rgb(r.color), asm.body_of_item(r.id))
        for p in project.parts.values():
            for mesh, col, half in part_meshes(p):
                self._add(p.id, mesh, col, asm.body_of_item(p.id, half))
        self._grid()
        if reset_camera:
            self.view_preset("3d")

    def _grid(self):
        if self.grid_actor is not None:
            self.remove_actor(self.grid_actor, render=False)
        lo, hi = self.project.bounds() if self.project else (np.zeros(3), np.zeros(3))
        step = 25.4
        pad = 2 * step
        x0, x1 = math.floor((lo[0] - pad) / step) * step, math.ceil((hi[0] + pad) / step) * step
        y0, y1 = math.floor((lo[1] - pad) / step) * step, math.ceil((hi[1] + pad) / step) * step
        z = lo[2] - 12.0
        pts, cells = [], []
        for x in np.arange(x0, x1 + 0.1, step):
            pts += [[x, y0, z], [x, y1, z]]
        for y in np.arange(y0, y1 + 0.1, step):
            pts += [[x0, y, z], [x1, y, z]]
        for i in range(0, len(pts), 2):
            cells += [2, i, i + 1]
        grid = pv.PolyData(np.array(pts, dtype=float), lines=np.array(cells))
        self.grid_actor = self.add_mesh(grid, color="#c5ccd5", line_width=1, render=False,
                                        reset_camera=False, pickable=False)

    # ------------------------------------------------------------ ports (build mode)
    def clear_ports(self, render=True):
        for a in self.port_actors:
            self.remove_actor(a, render=False)
        self.port_actors = []
        if self.active_actor is not None:
            self.remove_actor(self.active_actor, render=False)
            self.active_actor = None
        self.ports = []
        if render:
            self.render()

    def show_ports(self, project, active=None):
        self.clear_ports(render=False)
        groups = {"slot": [], "end": [], "hole": []}
        for pid, s in project.open_slots():
            pt, d, _ = project.slot_world(pid, s)
            p = pt + d * 2.0
            self.ports.append(("slot", pid, s, p))
            groups["slot"].append(p)
        for rid, e in project.free_ends():
            pt, o = project.rod_end(rid, e)
            p = pt + o * 1.5
            self.ports.append(("end", rid, e, p))
            groups["end"].append(p)
        for pid, h in project.open_holes():
            c, a = project.hole_world(pid, h)
            p = c + a * (C.CONN_T / 2 + 1.2)
            self.ports.append(("hole", pid, h, p))
            groups["hole"].append(p)
        geo = {"slot": pv.Sphere(radius=2.4, theta_resolution=12, phi_resolution=12),
               "end": pv.Sphere(radius=2.8, theta_resolution=12, phi_resolution=12),
               "hole": pv.Cube(x_length=3.4, y_length=3.4, z_length=3.4)}
        for kind, pts in groups.items():
            if not pts:
                continue
            cloud = pv.PolyData(np.array(pts, dtype=float))
            glyphs = cloud.glyph(geom=geo[kind], scale=False, orient=False)
            a = self.add_mesh(glyphs, color=PORT_COLORS[kind], render=False, reset_camera=False,
                              pickable=False, lighting=True)
            self.port_actors.append(a)
        if active is not None:
            for kind, a_, b_, p in self.ports:
                if (kind, a_, b_) == tuple(active):
                    s = pv.Sphere(radius=4.2, center=p, theta_resolution=20, phi_resolution=20)
                    self.active_actor = self.add_mesh(s, color="#ffd600", render=False,
                                                      reset_camera=False, pickable=False,
                                                      opacity=0.9)
        self.render()

    # ------------------------------------------------------------ pose / colors
    def update_pose(self, asm, status=None, selected=None, render=True):
        self.asm = asm
        status = status or {}
        for iid, lst in self.item_actors.items():
            st = status.get(iid)
            for a, base, body in lst:
                a.user_matrix = asm.T(body)
                if st == "bind":
                    col = STATUS["bind"]
                elif st == "warn":
                    col = STATUS["warn"]
                elif iid == selected:
                    col = STATUS["select"]
                else:
                    col = base
                a.prop.color = col
        if render:
            self.render()

    def set_traces(self, paths):
        self.clear_traces(render=False)
        for pts in paths:
            pts = np.asarray(pts, dtype=float)
            if len(pts) < 2 or np.ptp(pts, axis=0).max() < 0.5:
                continue
            self.trace_actors.append(self.add_mesh(pv.lines_from_points(pts), color="#00897b",
                                                   line_width=3, render=False,
                                                   reset_camera=False, pickable=False))
        self.render()

    def clear_traces(self, render=True):
        for a in self.trace_actors:
            self.remove_actor(a, render=False)
        self.trace_actors = []
        if render:
            self.render()

    def hud(self, text, alarm=False):
        try:
            self.add_text(text, position="upper_left", font_size=10, name="hud",
                          color="#b00020" if alarm else "#222222")
        except Exception:
            pass

    # ------------------------------------------------------------ camera
    def view_preset(self, name):
        lo, hi = self.project.bounds() if self.project else (np.zeros(3), np.zeros(3))
        center = (lo + hi) / 2
        radius = max(60.0, g.norm(hi - lo) / 2)
        dirs = {"3d": (np.array([0.55, -1.0, 0.6]), (0, 0, 1)),
                "front": (np.array([0, -1.0, 0]), (0, 0, 1)),
                "side": (np.array([1.0, 0, 0]), (0, 0, 1)),
                "top": (np.array([0, 0, 1.0]), (0, 1, 0))}
        d, up = dirs[name]
        d = d / np.linalg.norm(d)
        cam = self.camera
        cam.focal_point = tuple(center)
        cam.position = tuple(center + d * radius * 3.4)
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

    def _xy(self, ev):
        pos = ev.position() if hasattr(ev, "position") else ev.pos()
        s = self._scale()
        return pos.x() * s, self.render_window.GetSize()[1] - 1 - pos.y() * s

    def world_to_display(self, p):
        ren = self.renderer
        ren.SetWorldPoint(float(p[0]), float(p[1]), float(p[2]), 1.0)
        ren.WorldToDisplay()
        x, y, _ = ren.GetDisplayPoint()
        return np.array([x, y])

    def display_ray(self, x, y):
        ren = self.renderer
        out = []
        for z in (0.0, 1.0):
            ren.SetDisplayPoint(x, y, z)
            ren.DisplayToWorld()
            w = ren.GetWorldPoint()
            out.append(np.array(w[:3]) / (w[3] if abs(w[3]) > 1e-12 else 1.0))
        return out[0], g.unit(out[1] - out[0])

    def find_port(self, x, y):
        best, bd = None, PICK_PX * self._scale()
        for kind, a, b, p in self.ports:
            sp = self.world_to_display(p)
            d = math.hypot(sp[0] - x, sp[1] - y)
            if d < bd:
                best, bd = (kind, a, b), d
        return best

    def pick(self, x, y):
        """(item id, world point) under the mouse, or (None, None)."""
        import vtk
        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.0015)
        if not picker.Pick(x, y, 0, self.renderer):
            return None, None
        act = picker.GetActor()
        if act is None:
            return None, None
        iid = self.owner.get(act.GetAddressAsString("vtkActor"))
        return iid, np.array(picker.GetPickPosition())

    # ------------------------------------------------------------ mouse
    def mousePressEvent(self, ev):
        if ev.button() == QtCore.Qt.MouseButton.LeftButton and \
                not (ev.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier):
            x, y = self._xy(ev)
            if self.edit_mode == "move":
                iid, wp = self.pick(x, y)
                if iid is not None and self.on_drag_start and self.on_drag_start(iid, wp):
                    cam = self.camera
                    n = g.unit(np.array(cam.focal_point) - np.array(cam.position))
                    self._drag = {"point": wp, "normal": n}
                    self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
                    ev.accept()
                    return
            self._press = (x, y)
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._drag is not None:
            x, y = self._xy(ev)
            o, ray = self.display_ray(x, y)
            n = self._drag["normal"]
            den = float(ray @ n)
            if abs(den) > 1e-6:
                t = float((self._drag["point"] - o) @ n) / den
                self._pending = o + ray * t
                QtCore.QTimer.singleShot(0, self._flush)
            ev.accept()
            return
        super().mouseMoveEvent(ev)

    def _flush(self):
        if self._pending is None or self._drag is None:
            return
        tgt, self._pending = self._pending, None
        if self.on_drag:
            self.on_drag(tgt)

    def mouseReleaseEvent(self, ev):
        if self._drag is not None:
            self._flush()
            self._drag = None
            self.setCursor(QtCore.Qt.CursorShape.ArrowCursor)
            if self.on_drag_end:
                self.on_drag_end()
            ev.accept()
            return
        if self._press is not None and ev.button() == QtCore.Qt.MouseButton.LeftButton:
            x, y = self._xy(ev)
            if math.hypot(x - self._press[0], y - self._press[1]) < 4 * self._scale():
                port = self.find_port(x, y) if self.edit_mode == "build" else None
                if port is not None and self.on_port:
                    self.on_port(port)
                elif self.on_pick:
                    iid, _ = self.pick(x, y)
                    self.on_pick(iid)
        self._press = None
        super().mouseReleaseEvent(ev)
