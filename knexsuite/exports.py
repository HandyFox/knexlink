"""Outputs for real builds: parts / cut list, STL, and dimensioned drawings (DXF, PDF)."""
import csv
import datetime
import math
import os
from collections import Counter, OrderedDict

import numpy as np
import pyvista as pv

from . import catalog as C
from . import geometry as g
from .meshes import (END_CHAMFER, END_GROOVE_DIA, END_GROOVE_FROM_TIP, END_GROOVE_WIDTH,
                     custom_rod_solid, part_meshes, rod_mesh)
from .units import fmt, fmt_both

APP = "KnexLink"


# ================================================================== parts / cut list

def bom_rows(project, units="in"):
    """[(category, part, size, qty)]"""
    rows = []
    std = Counter(r.color for r in project.rods.values() if r.color in C.RODS)
    for key in C.ROD_ORDER:
        if std[key]:
            rows.append(("Rods", f"{key.capitalize()} rod", fmt_both(C.RODS[key][0], units), std[key]))
    cust = Counter(round(r.length, 1) for r in project.rods.values() if r.color not in C.RODS)
    for L in sorted(cust):
        rows.append(("Rods (custom - cut or print)", "Custom rod",
                     fmt_both(L, units) + f"   [hole-to-hole {fmt(L + 2 * C.REACH, units)}]", cust[L]))
    kinds = Counter()
    sizes = {}
    for p in project.parts.values():
        key = p.type
        if p.tdef.get("wheel") and "od" in p.params:
            key = (p.type, round(p.params["od"], 1))
            sizes[key] = fmt_both(p.params["od"], units) + " dia"
        kinds[key] += 1
    order = {k: i for i, k in enumerate(C.CONNECTOR_ORDER + C.JOINT_ORDER + C.MOUNT_ORDER)}
    for key in sorted(kinds, key=lambda k: order.get(k if isinstance(k, str) else k[0], 99)):
        tk = key if isinstance(key, str) else key[0]
        fam = C.TYPES[tk]["family"]
        cat = {"connector": "Connectors", "joint": "Joints", "mount": "On-rod parts"}[fam]
        size = sizes.get(key, "")
        if not size and "teeth" in C.TYPES[tk]:
            size = f"{C.TYPES[tk]['teeth']} teeth, pitch dia {fmt(2 * C.pitch_radius(tk), units)}"
        elif not size and C.TYPES[tk].get("wheel"):
            size = fmt_both(C.TYPES[tk]["od"], units) + " dia"
        rows.append((cat, C.TYPES[tk]["name"], size, kinds[key]))
    return rows


def write_bom_csv(project, path, units="in"):
    rows = bom_rows(project, units)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["Category", "Part", "Size", "Qty"])
        for r in rows:
            w.writerow(r)
        w.writerow([])
        w.writerow(["Total pieces", "", "", sum(r[3] for r in rows)])
    return path


# ================================================================== STL

def model_meshes(project, asm=None):
    out = []
    for r in project.rods.values():
        m = rod_mesh(r)
        if asm is not None:
            m = m.transform(asm.T(asm.body_of_item(r.id)), inplace=False)
        out.append(m)
    for p in project.parts.values():
        for m, col, half in part_meshes(p):
            if asm is not None:
                m = m.transform(asm.T(asm.body_of_item(p.id, half)), inplace=False)
            out.append(m)
    return out


def export_model_stl(project, path, asm=None):
    meshes = model_meshes(project, asm)
    if not meshes:
        raise ValueError("Nothing to export")
    merged = pv.merge(meshes)
    if not isinstance(merged, pv.PolyData):
        merged = merged.extract_surface(algorithm="dataset_surface")
    merged.triangulate().save(path, binary=True)
    return path


def export_custom_rods(project, folder):
    lengths = sorted({round(r.length, 1) for r in project.rods.values() if r.color not in C.RODS})
    paths = []
    for L in lengths:
        path = os.path.join(folder, f"knex_custom_rod_{L:.1f}mm.stl")
        custom_rod_solid(L).save(path, binary=True)
        paths.append(path)
    return paths


# ================================================================== drawing geometry

VIEWS = OrderedDict([
    ("FRONT", (np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]))),
    ("TOP", (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, -1.0]))),
    ("RIGHT SIDE", (np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), np.array([-1.0, 0, 0]))),
])


def _circle3d(c, n, r, k=32):
    u = g.perp(n)
    v = g.cross(n, u)
    return [c + r * (math.cos(2 * math.pi * i / k) * u + math.sin(2 * math.pi * i / k) * v)
            for i in range(k + 1)]


def view_polylines(project, view):
    """[(points Nx2, layer)] for one orthographic view of the design pose (mm)."""
    ex, ey, _ = VIEWS[view]
    P = lambda p: np.array([float(p @ ex), float(p @ ey)])  # noqa: E731
    out = []
    rr = C.ROD_DIA / 2
    for r in project.rods.values():
        a, b = P(r.p0), P(r.p1)
        L = np.linalg.norm(b - a)
        if L < 0.5:
            out.append((np.array([P(q) for q in _circle3d(r.p0, r.d, rr)]), "RODS"))
            continue
        u = (b - a) / L
        n = np.array([-u[1], u[0]]) * rr
        out.append((np.array([a + n, b + n, b - n, a - n, a + n]), "RODS"))
    for p in project.parts.values():
        fam = p.tdef["family"]
        if fam in ("connector", "joint") or p.type == "clip":
            out.append((np.array([P(q) for q in _circle3d(p.pos, p.R[:, 2], 6.0, 24)]), "PARTS"))
            for idx, half, lp, ld in C.local_slots(p.type, p.params):
                a = P(p.pos + p.R @ (ld * 5.0))
                b = P(p.pos + p.R @ (ld * (C.slot_reach(p.type) + 3.0)))
                out.append((np.array([a, b]), "PARTS"))
        else:
            t = p.tdef
            if "teeth" in t:
                rad = C.pitch_radius(p.type) + C.GEAR_MODULE
            elif t.get("wheel"):
                rad = p.params.get("od", t["od"]) / 2
            else:
                rad = t.get("od", 11.0) / 2
            out.append((np.array([P(q) for q in _circle3d(p.pos, p.R[:, 2], rad, 40)]), "PARTS"))
    return out


def _extent(polys):
    pts = np.vstack([pl for pl, _ in polys]) if polys else np.zeros((1, 2))
    return pts.min(axis=0), pts.max(axis=0)


# ================================================================== DXF

def _dxf_line(f, a, b, layer):
    f.write(f"0\nLINE\n8\n{layer}\n10\n{a[0]:.4f}\n20\n{a[1]:.4f}\n30\n0.0\n"
            f"11\n{b[0]:.4f}\n21\n{b[1]:.4f}\n31\n0.0\n")


def _dxf_text(f, at, h, text, layer="TEXT"):
    text = text.replace("\n", " ")
    f.write(f"0\nTEXT\n8\n{layer}\n10\n{at[0]:.4f}\n20\n{at[1]:.4f}\n30\n0.0\n40\n{h:.3f}\n1\n{text}\n")


def _dxf_dim(f, a, b, offset_dir, label, h=3.5):
    """Simple dimension: extension lines, dimension line with ticks, text."""
    a, b, od = np.array(a), np.array(b), np.array(offset_dir)
    da, db = a + od, b + od
    _dxf_line(f, a, da + od * 0.15, "DIMS")
    _dxf_line(f, b, db + od * 0.15, "DIMS")
    _dxf_line(f, da, db, "DIMS")
    u = (db - da) / max(np.linalg.norm(db - da), 1e-9)
    n = np.array([-u[1], u[0]])
    for p in (da, db):
        _dxf_line(f, p - (u + n) * 1.2, p + (u + n) * 1.2, "DIMS")
    mid = (da + db) / 2 + od / max(np.linalg.norm(od), 1e-9) * 2.0
    _dxf_text(f, mid - u * len(label) * h * 0.3, h, label, "DIMS")


def export_dxf(project, path, units="in"):
    """Front, top and right-side views in millimeters, with overall dimensions and a parts list."""
    views = {v: view_polylines(project, v) for v in VIEWS}
    ext = {v: _extent(views[v]) for v in VIEWS}
    fw = ext["FRONT"][1] - ext["FRONT"][0]
    gap = 60.0
    origin = {
        "FRONT": -ext["FRONT"][0],
        "TOP": np.array([-ext["TOP"][0][0], fw[1] + gap - ext["TOP"][0][1]]),
        "RIGHT SIDE": np.array([fw[0] + gap - ext["RIGHT SIDE"][0][0], -ext["RIGHT SIDE"][0][1]]),
    }
    with open(path, "w", encoding="ascii", errors="replace") as f:
        f.write("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nENTITIES\n")
        for v, polys in views.items():
            off = origin[v]
            for pl, layer in polys:
                for i in range(len(pl) - 1):
                    _dxf_line(f, pl[i] + off, pl[i + 1] + off, f"{layer}_{v.replace(' ', '_')}")
            lo, hi = ext[v][0] + off, ext[v][1] + off
            _dxf_text(f, (lo[0], lo[1] - 22), 5.0, f"{v} VIEW")
            w, h = hi - lo
            _dxf_dim(f, (lo[0], lo[1]), (hi[0], lo[1]), (0, -12), fmt(w, units))
            _dxf_dim(f, (lo[0], lo[1]), (lo[0], hi[1]), (-12, 0), fmt(h, units))
        # parts list
        x0 = origin["RIGHT SIDE"][0] + (ext["RIGHT SIDE"][1] - ext["RIGHT SIDE"][0])[0] + gap
        y = fw[1] + gap + 80
        _dxf_text(f, (x0, y), 5.0, f"{APP} PARTS LIST")
        y -= 9
        for cat, part, size, qty in bom_rows(project, units):
            _dxf_text(f, (x0, y), 3.2, f"{qty:>3} x  {part}   {size}")
            y -= 6
        _dxf_text(f, (x0, y - 6), 3.0, "Drawing units: mm. Design pose. Sizes from the K'nex Classic system.")
        f.write("0\nENDSEC\n0\nEOF\n")
    return path


# ================================================================== PDF (Qt)

def export_pdf(project, path, units="in", title="KnexLink design"):
    """Letter landscape sheets: 3 views + dims, parts list, custom rod details."""
    from qtpy import QtCore, QtGui

    writer = QtGui.QPdfWriter(path)
    writer.setPageSize(QtGui.QPageSize(QtGui.QPageSize.PageSizeId.Letter))
    writer.setPageOrientation(QtGui.QPageLayout.Orientation.Landscape)
    writer.setResolution(300)
    writer.setTitle(title)
    p = QtGui.QPainter(writer)
    W, H = writer.width(), writer.height()
    dpmm = 300 / 25.4
    margin = 10 * dpmm

    def frame(sheet, n_sheets, caption):
        p.setPen(QtGui.QPen(QtGui.QColor("#000"), 2))
        p.drawRect(QtCore.QRectF(margin, margin, W - 2 * margin, H - 2 * margin))
        tb = QtCore.QRectF(W - margin - 95 * dpmm, H - margin - 18 * dpmm, 95 * dpmm, 18 * dpmm)
        p.drawRect(tb)
        p.setFont(QtGui.QFont("Helvetica", 11, QtGui.QFont.Weight.Bold))
        p.drawText(tb.adjusted(3 * dpmm, 1 * dpmm, 0, 0), QtCore.Qt.AlignmentFlag.AlignLeft, title)
        p.setFont(QtGui.QFont("Helvetica", 8))
        p.drawText(tb.adjusted(3 * dpmm, 7 * dpmm, 0, 0), QtCore.Qt.AlignmentFlag.AlignLeft,
                   f"{caption}\n{datetime.date.today().isoformat()}   Sheet {sheet} of {n_sheets}   {APP}")

    custom = sorted({round(r.length, 1) for r in project.rods.values() if r.color not in C.RODS})
    n_sheets = 3 if custom else 2

    # ---------------- sheet 1: views
    views = {v: view_polylines(project, v) for v in VIEWS}
    ext = {v: _extent(views[v]) for v in VIEWS}
    size = {v: ext[v][1] - ext[v][0] for v in VIEWS}
    gap_pg = 32.0                      # page mm between views (room for dims and labels)
    total_w = size["FRONT"][0] + size["RIGHT SIDE"][0]
    total_h = size["FRONT"][1] + size["TOP"][1]
    avail_w = (W - 2 * margin) / dpmm - 40 - gap_pg
    avail_h = (H - 2 * margin) / dpmm - 62 - gap_pg
    scale = min(avail_w / max(total_w, 1), avail_h / max(total_h, 1))
    nice = [10, 5, 4, 3, 2, 1.5, 1, 0.75, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05, 0.02]
    scale = next((s_ for s_ in nice if s_ <= scale), scale)
    ox = margin + 28 * dpmm
    oy = H - margin - 38 * dpmm       # page y of the front view's bottom edge
    place = {
        "FRONT": (ox, oy),
        "TOP": (ox, oy - (size["FRONT"][1] * scale + gap_pg) * dpmm),
        "RIGHT SIDE": (ox + (size["FRONT"][0] * scale + gap_pg) * dpmm, oy),
    }

    def to_page(v, pt):
        bx, by = place[v]
        x = bx + (pt[0] - ext[v][0][0]) * scale * dpmm
        y = by - (pt[1] - ext[v][0][1]) * scale * dpmm
        return QtCore.QPointF(x, y)

    def dim(a, b, off, label):
        pen = QtGui.QPen(QtGui.QColor("#1565c0"), 1.2)
        p.setPen(pen)
        da = QtCore.QPointF(a.x() + off[0], a.y() + off[1])
        db = QtCore.QPointF(b.x() + off[0], b.y() + off[1])
        p.drawLine(a, da)
        p.drawLine(b, db)
        p.drawLine(da, db)
        for q in (da, db):
            p.drawEllipse(q, 4, 4)
        p.setFont(QtGui.QFont("Helvetica", 8))
        mid = QtCore.QPointF((da.x() + db.x()) / 2, (da.y() + db.y()) / 2)
        r = QtCore.QRectF(mid.x() - 200, mid.y() - 60, 400, 50) if off[1] else \
            QtCore.QRectF(mid.x() - 420, mid.y() - 25, 400, 50)
        p.drawText(r, QtCore.Qt.AlignmentFlag.AlignCenter if off[1] else
                   (QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter), label)

    frame(1, n_sheets, f"Views - scale {scale:g}:1 - dimensions in {'inches' if units == 'in' else 'mm'}")
    for v, polys in views.items():
        for pl, layer in polys:
            col = "#222" if layer == "RODS" else "#555"
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1.4 if layer == "RODS" else 1.0))
            pts = [to_page(v, q) for q in pl]
            p.drawPolyline(QtGui.QPolygonF(pts))
        lo, hi = ext[v]
        a = to_page(v, lo)
        b = to_page(v, (hi[0], lo[1]))
        c = to_page(v, (lo[0], hi[1]))
        dim(a, b, (0, 8 * dpmm), fmt(hi[0] - lo[0], units))
        dim(a, c, (-8 * dpmm, 0), fmt(hi[1] - lo[1], units))
        p.setPen(QtGui.QColor("#000"))
        p.setFont(QtGui.QFont("Helvetica", 9, QtGui.QFont.Weight.Bold))
        p.drawText(QtCore.QPointF(a.x(), a.y() + 16 * dpmm), f"{v} VIEW")
    p.setFont(QtGui.QFont("Helvetica", 7))
    p.drawText(QtCore.QRectF(margin + 3 * dpmm, margin + 2 * dpmm, 150 * dpmm, 12 * dpmm),
               "Third-angle layout: TOP above FRONT, RIGHT SIDE to the right. Shown as designed "
               "(not in a moved position).")

    # ---------------- sheet 2: parts list
    writer.newPage()
    frame(2, n_sheets, "Parts list")
    rows = bom_rows(project, units)
    y = margin + 14 * dpmm
    x = [margin + 6 * dpmm, margin + 70 * dpmm, margin + 150 * dpmm, margin + 245 * dpmm]
    p.setFont(QtGui.QFont("Helvetica", 10, QtGui.QFont.Weight.Bold))
    for xi, head in zip(x, ["Category", "Part", "Size", "Qty"]):
        p.drawText(QtCore.QPointF(xi, y), head)
    p.drawLine(QtCore.QPointF(x[0], y + 2 * dpmm), QtCore.QPointF(x[3] + 8 * dpmm, y + 2 * dpmm))
    p.setFont(QtGui.QFont("Helvetica", 9))
    for row in rows:
        y += 6 * dpmm
        if y > H - margin - 25 * dpmm:
            break
        for xi, val in zip(x, row):
            p.drawText(QtCore.QPointF(xi, y), str(val))
    y += 8 * dpmm
    p.setFont(QtGui.QFont("Helvetica", 10, QtGui.QFont.Weight.Bold))
    p.drawText(QtCore.QPointF(x[2], y), "Total pieces")
    p.drawText(QtCore.QPointF(x[3], y), str(sum(r[3] for r in rows)))

    # ---------------- sheet 3: custom rods
    if custom:
        writer.newPage()
        frame(3, n_sheets, "Custom rods - cut or 3D print")
        y = margin + 20 * dpmm
        p.setFont(QtGui.QFont("Helvetica", 9))
        p.drawText(QtCore.QRectF(margin + 6 * dpmm, margin + 4 * dpmm, 240 * dpmm, 14 * dpmm),
                   f"Rod diameter {fmt_both(C.ROD_DIA, units)}. Each end: groove {fmt(END_GROOVE_WIDTH, units)} "
                   f"wide, {fmt(END_GROOVE_DIA, units)} dia, starting {fmt(END_GROOVE_FROM_TIP, units)} "
                   f"from the tip; {fmt(END_CHAMFER, units)} chamfer. End details are approximate - "
                   "test-fit one rod first.")
        cnt = Counter(round(r.length, 1) for r in project.rods.values() if r.color not in C.RODS)
        for L in custom:
            if y > H - margin - 40 * dpmm:
                break
            avail = W - 2 * margin - 80 * dpmm
            s = min(1.0, avail / (L * dpmm)) * dpmm
            x0 = margin + 10 * dpmm
            h = C.ROD_DIA * s
            p.setPen(QtGui.QPen(QtGui.QColor("#000"), 1.4))
            p.setBrush(QtGui.QColor(C.CUSTOM_ROD_COLOR))
            p.drawRect(QtCore.QRectF(x0, y, L * s, h))
            p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            gx = END_GROOVE_FROM_TIP * s
            for gxx in (gx, L * s - gx - END_GROOVE_WIDTH * s):
                p.drawLine(QtCore.QPointF(x0 + gxx, y), QtCore.QPointF(x0 + gxx, y + h))
                p.drawLine(QtCore.QPointF(x0 + gxx + END_GROOVE_WIDTH * s, y),
                           QtCore.QPointF(x0 + gxx + END_GROOVE_WIDTH * s, y + h))
            a = QtCore.QPointF(x0, y + h)
            b = QtCore.QPointF(x0 + L * s, y + h)
            dim(a, b, (0, 7 * dpmm), fmt_both(L, units))
            p.setPen(QtGui.QColor("#000"))
            p.setFont(QtGui.QFont("Helvetica", 9, QtGui.QFont.Weight.Bold))
            p.drawText(QtCore.QPointF(x0 + L * s + 8 * dpmm, y + h), f"Qty {cnt[L]}")
            p.setFont(QtGui.QFont("Helvetica", 9))
            y += 30 * dpmm
    p.end()
    return path
