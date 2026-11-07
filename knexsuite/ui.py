"""Main window: K'nex kit, build/move modes, properties, motion and binding, parts list, exports."""
import math
import os

import numpy as np
from qtpy import QtCore, QtGui, QtWidgets

from . import catalog as C
from . import exports as X
from .demo import build_demo
from .model import Project
from .solver import Assembly
from .units import fmt, fmt_both, parse_length
from .view3d import KnexView

APP = "KnexLink"
FILE_FILTER = "KnexLink designs (*.knex.json *.json);;All files (*)"

HELP = """
<h3>KnexLink - K'nex design suite</h3>
<p><b>Build mode (B)</b> - parts click together like real K'nex:</p>
<ul>
<li><span style="color:#00a844"><b>Green dot</b></span> = open connector slot. Click it, then click a rod color.</li>
<li><span style="color:#ff6d00"><b>Orange dot</b></span> = free rod end. Click it, then click a connector, hinge or ball joint.</li>
<li><span style="color:#2979ff"><b>Blue cube</b></span> = open hole. Click it, then a rod color to push a rod (axle) through.</li>
<li>Select a <b>rod</b> (click it), then click a spacer, gear, wheel, clip or connector to slide it onto the rod.</li>
<li>Rods whose ends land on a slot click in by themselves, just like closing a K'nex triangle.</li>
<li><b>T</b> turns the part you just placed to its next slot, <b>Y</b> rolls it 45 deg around the rod.</li>
<li>Any rod can be a <b>custom length</b>: pick Custom in the kit, or change a rod's length on the right.</li>
</ul>
<p><b>Move mode (M)</b> - grab any part that isn't on the ground and drag. Joints, gears and
hinges move like the real thing. Motion stops at binding points and turns those parts red.
Click a joint in the Motion tab, then use Sweep (S) to run it end to end.</p>
<p><b>Ground</b>: parts marked Ground (the first part, by default) never move.</p>
<p>Camera: drag empty space to orbit, Shift+drag to pan, wheel to zoom. Views: 1-4.</p>
<p>Lengths: type 5.04, 5.04", 128mm, 3' 2" - a bare number uses the display unit.</p>
"""


# =================================================================== small widgets

class LengthEdit(QtWidgets.QLineEdit):
    valueEdited = QtCore.Signal(float)

    def __init__(self, units_fn, value=0.0, parent=None):
        super().__init__(parent)
        self.units_fn = units_fn
        self._v = float(value)
        self.editingFinished.connect(self._commit)
        self.setToolTip('Type 5.04, 5.04", 128mm or 3\' 2" (bare numbers use the display unit)')
        self.setValue(value)

    def setValue(self, v):
        self._v = float(v)
        self.setText(fmt(self._v, self.units_fn(), 3))

    def value(self):
        return self._v

    def _commit(self):
        v = parse_length(self.text(), self.units_fn())
        if v is None:
            self.setValue(self._v)
            return
        changed = abs(v - self._v) > 1e-6
        self.setValue(v)
        if changed:
            self.valueEdited.emit(v)


def _swatch_button(text, color, tip, fn, dark_text=None):
    b = QtWidgets.QPushButton(text)
    b.setToolTip(tip)
    b.setMinimumHeight(30)
    qc = QtGui.QColor(color)
    light = qc.lightness() > 150 if dark_text is None else dark_text
    b.setStyleSheet(
        f"QPushButton {{ background: {color}; color: {'#111' if light else '#fff'};"
        f" border: 1px solid #8a9099; border-radius: 4px; font-weight: 600; padding: 2px 6px; }}"
        f"QPushButton:hover {{ border: 2px solid #222; }}"
        f"QPushButton:disabled {{ background: #e6e8eb; color: #9aa0a6; border-color: #d0d4d9; }}")
    b.clicked.connect(fn)
    return b


# =================================================================== properties

class Props(QtWidgets.QScrollArea):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.show_item(None)

    def _later(self, fn):
        QtCore.QTimer.singleShot(0, fn)

    def show_item(self, iid):
        pr = self.win.project
        box = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(box)
        form.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        it = pr.item(iid) if iid else None
        if it is None:
            lab = QtWidgets.QLabel("Click a part to see and change it.\n\nIn Build mode, click a dot to "
                                   "start adding parts there.")
            lab.setWordWrap(True)
            form.addRow(lab)
            self.setWidget(box)
            return
        units = self.win.units
        is_rod = iid in pr.rods
        title = QtWidgets.QLabel(f"<b>{'Rod' if is_rod else it.tdef['name']}</b> "
                                 f"<span style='color:#888'>{iid}</span>")
        form.addRow(title)
        name = QtWidgets.QLineEdit(it.params.get("name", ""))
        name.setPlaceholderText(it.name)
        name.editingFinished.connect(lambda: self._later(lambda: self.win.edit(
            "Rename", lambda: it.params.__setitem__("name", name.text().strip()))))
        form.addRow("Name", name)
        ground = QtWidgets.QCheckBox("Fixed to the ground (never moves)")
        ground.setChecked(bool(it.params.get("ground")))
        ground.toggled.connect(lambda v: self._later(lambda: self.win.edit(
            "Ground", lambda: it.params.__setitem__("ground", bool(v)))))
        form.addRow("", ground)
        track = QtWidgets.QCheckBox("Track its travel")
        track.setChecked(iid in self.win.tracked)
        track.toggled.connect(lambda v: self.win.set_tracked(iid, v))
        form.addRow("", track)

        if is_rod:
            combo = QtWidgets.QComboBox()
            for k in C.ROD_ORDER:
                combo.addItem(f"{k.capitalize()}  ({fmt(C.RODS[k][0], units)})", k)
            combo.addItem("Custom length", "custom")
            combo.setCurrentIndex(max(0, combo.findData(it.color)))
            combo.activated.connect(lambda i: self._later(lambda: self.win.set_rod_kind(iid, combo.itemData(i))))
            form.addRow("Rod", combo)
            le = LengthEdit(lambda: self.win.units, it.length)
            le.valueEdited.connect(lambda v: self._later(lambda: self.win.set_rod_length(iid, v)))
            form.addRow("Length", le)
            info = QtWidgets.QLabel(f"{fmt_both(it.length, units)}\nHole-to-hole when clicked into two "
                                    f"connectors: {fmt(it.length + 2 * C.REACH, units)}")
            info.setStyleSheet("color:#666")
            info.setWordWrap(True)
            form.addRow("", info)
        else:
            p = it
            fam = p.tdef["family"]
            if p.type == "hinge":
                sp = self._spin(p.params.get("angle", 180.0), 45, 315, 15, " deg")
                sp.valueChanged.connect(lambda v: self._later(lambda: self.win.set_joint(iid, "angle", v)))
                form.addRow("Opening angle", sp)
            if p.type == "ball":
                sp = self._spin(p.params.get("tilt", 0.0), 0, C.TYPES["ball"]["cone"], 5, " deg")
                sp.valueChanged.connect(lambda v: self._later(lambda: self.win.set_joint(iid, "tilt", v)))
                form.addRow("Tilt", sp)
                sp2 = self._spin(p.params.get("turn", 0.0), -180, 180, 15, " deg")
                sp2.valueChanged.connect(lambda v: self._later(lambda: self.win.set_joint(iid, "turn", v)))
                form.addRow("Tilt direction", sp2)
            if p.tdef.get("wheel"):
                le = LengthEdit(lambda: self.win.units, p.params.get("od", p.tdef["od"]))
                le.valueEdited.connect(lambda v: self._later(lambda: self.win.edit(
                    "Wheel size", lambda: p.params.__setitem__("od", float(v)))))
                form.addRow("Tire diameter", le)
            if "teeth" in p.tdef:
                form.addRow("Gear", QtWidgets.QLabel(
                    f"{p.tdef['teeth']} teeth, pitch dia {fmt(2 * C.pitch_radius(p.type), units)}"))
            conns = [c for c in pr.holes if c["part"] == iid]
            for c in conns:
                rod = pr.rods[c["rod"]]
                lab = f"On {rod.name}" if c["hole"] == 0 else f"Hole 2: {rod.name}"
                lock = QtWidgets.QCheckBox("Locked (turns with the rod)" if fam != "mount" or
                                           p.type in ("clip", "cliphole") else "Locked to the rod")
                lock.setChecked(bool(c["locked"]))
                lock.toggled.connect(lambda v, c=c: self._later(lambda: self.win.edit(
                    "Lock", lambda: c.__setitem__("locked", bool(v)))))
                form.addRow(lab, lock)
                if c["hole"] == 0 and (fam == "mount" or "_attach" not in p.params):
                    pos = LengthEdit(lambda: self.win.units, c["t"])
                    pos.valueEdited.connect(lambda v: self._later(lambda: self.win.edit(
                        "Slide", lambda: pr.move_mount(iid, t=v))))
                    form.addRow("Position along rod", pos)
                    roll = self._spin(p.params.get("roll", 0.0), -360, 360, 15, " deg")
                    roll.valueChanged.connect(lambda v: self._later(lambda: self.win.edit(
                        "Roll", lambda: pr.move_mount(iid, roll=v))))
                    form.addRow("Turned around rod", roll)
            if p.tdef.get("is3d"):
                b = QtWidgets.QPushButton("Add 3D partner (blue)")
                b.clicked.connect(lambda: self.win.add_partner(iid, "conn7_3d"))
                form.addRow("", b)
                b2 = QtWidgets.QPushButton("Add 3D partner (purple)")
                b2.clicked.connect(lambda: self.win.add_partner(iid, "conn4_3d"))
                form.addRow("", b2)
            if pr.can_reorient(iid):
                row = QtWidgets.QHBoxLayout()
                t1 = QtWidgets.QPushButton("Turn to next slot (T)")
                t1.clicked.connect(lambda: self.win.turn(True, 0))
                t2 = QtWidgets.QPushButton("Roll 45 deg (Y)")
                t2.clicked.connect(lambda: self.win.turn(False, 45))
                row.addWidget(t1)
                row.addWidget(t2)
                w = QtWidgets.QWidget()
                w.setLayout(row)
                form.addRow(w)
        dele = QtWidgets.QPushButton("Delete (Del)")
        dele.clicked.connect(self.win.delete_selected)
        form.addRow(dele)
        self.setWidget(box)

    @staticmethod
    def _spin(v, lo, hi, step, suffix):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setSingleStep(step)
        sp.setDecimals(1)
        sp.setSuffix(suffix)
        sp.setKeyboardTracking(False)
        sp.setValue(float(v))
        return sp


# =================================================================== main window

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, project=None, path=None):
        super().__init__()
        self.resize(1560, 960)
        self.project = project or build_demo()
        self.path = path
        self.dirty = False
        self.undo, self.redo = [], []
        self.mode = "build"
        self.active = None
        self.selected = None
        self.tracked = set()
        self.ranges = {}
        self.issues = []
        self.last_stop = None
        self.drive_joint = None
        self.drag = None
        self._log_keys = set()

        self.view = KnexView(self)
        self.view.on_port = self.on_port
        self.view.on_pick = self.select
        self.view.on_drag_start = self.drag_start
        self.view.on_drag = self.drag_to
        self.view.on_drag_end = self.drag_end
        self.setCentralWidget(self.view)
        self._docks()
        self._actions()
        self._tab_timer = QtCore.QTimer(self)
        self._tab_timer.setSingleShot(True)
        self._tab_timer.timeout.connect(self.refresh_motion)
        self.rebuild(reset_camera=True)
        self._title()

    @property
    def units(self):
        return self.project.settings.get("units", "in")

    # ------------------------------------------------------------ layout
    def _docks(self):
        kit = QtWidgets.QWidget()
        kv = QtWidgets.QVBoxLayout(kit)
        kv.setContentsMargins(6, 6, 6, 6)
        self.kit_hint = QtWidgets.QLabel()
        self.kit_hint.setWordWrap(True)
        self.kit_hint.setStyleSheet("background:#fff8e1; border:1px solid #ffe082; padding:5px;")
        kv.addWidget(self.kit_hint)
        self.kit_buttons = {}

        def section(title, items, cols=2):
            kv.addWidget(QtWidgets.QLabel(f"<b>{title}</b>"))
            grid = QtWidgets.QGridLayout()
            grid.setSpacing(4)
            for i, (key, text, color, tip) in enumerate(items):
                b = _swatch_button(text, color, tip, lambda _=False, k=key: self.kit(k))
                self.kit_buttons[key] = b
                grid.addWidget(b, i // cols, i % cols)
            kv.addLayout(grid)

        rods = [(f"rod:{k}", f"{k.capitalize()} {fmt(C.RODS[k][0], 'in', 2)}", C.RODS[k][1],
                 f"{k.capitalize()} rod, {fmt_both(C.RODS[k][0])}") for k in C.ROD_ORDER]
        rods.append(("rod:custom", "Custom...", C.CUSTOM_ROD_COLOR, "A rod cut to any length"))
        section("Rods", rods, cols=3)
        section("Connectors", [(k, C.TYPES[k]["short"], C.TYPES[k]["color"],
                                C.TYPES[k]["name"]) for k in C.CONNECTOR_ORDER], cols=3)
        section("Joints", [(k, C.TYPES[k]["short"], C.TYPES[k]["color"], C.TYPES[k]["name"])
                           for k in C.JOINT_ORDER])
        section("On a rod", [(k, C.TYPES[k]["short"], C.TYPES[k]["color"],
                              C.TYPES[k]["name"] + " - select a rod first") for k in C.MOUNT_ORDER], cols=3)
        kv.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(kit)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(330)
        d = QtWidgets.QDockWidget("K'nex kit", self)
        d.setObjectName("kit")
        d.setWidget(scroll)
        d.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea, d)

        right = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.props = Props(self)
        right.addWidget(self.props)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Part", "Type"])
        self.tree.setColumnWidth(0, 170)
        self.tree.itemSelectionChanged.connect(self._tree_pick)
        right.addWidget(self.tree)
        right.setSizes([520, 300])
        d = QtWidgets.QDockWidget("Selected part / all parts", self)
        d.setObjectName("props")
        d.setWidget(right)
        d.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        right.setMinimumWidth(330)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, d)

        self.tabs = QtWidgets.QTabWidget()
        # motion
        mw = QtWidgets.QWidget()
        ml = QtWidgets.QVBoxLayout(mw)
        ml.setContentsMargins(3, 3, 3, 3)
        row = QtWidgets.QHBoxLayout()
        self.dof_label = QtWidgets.QLabel()
        row.addWidget(self.dof_label)
        row.addStretch(1)
        for txt, deg in (("<< 15", -15), ("< 1", -1), ("1 >", 1), ("15 >>", 15)):
            b = QtWidgets.QPushButton(txt)
            b.setToolTip("Turn the selected joint")
            b.clicked.connect(lambda _=False, d_=deg: self.nudge(d_))
            row.addWidget(b)
        sw = QtWidgets.QPushButton("Sweep joint (S)")
        sw.clicked.connect(self.sweep)
        row.addWidget(sw)
        ml.addLayout(row)
        self.mtable = QtWidgets.QTableWidget(0, 5)
        self.mtable.setHorizontalHeaderLabels(["Joint / tracked part", "Now", "Range seen", "Total travel",
                                               "Notes"])
        hh = self.mtable.horizontalHeader()
        hh.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Stretch)
        for c in range(1, 5):
            hh.setSectionResizeMode(c, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.mtable.verticalHeader().setVisible(False)
        self.mtable.verticalHeader().setDefaultSectionSize(22)
        self.mtable.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.mtable.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.mtable.cellClicked.connect(self._mtable_clicked)
        ml.addWidget(self.mtable)
        self.tabs.addTab(mw, "Motion")
        # binding
        bw = QtWidgets.QWidget()
        bl = QtWidgets.QVBoxLayout(bw)
        bl.setContentsMargins(3, 3, 3, 3)
        bl.addWidget(QtWidgets.QLabel("<b>Right now</b>"))
        self.now = QtWidgets.QListWidget()
        bl.addWidget(self.now, 1)
        h = QtWidgets.QHBoxLayout()
        h.addWidget(QtWidgets.QLabel("<b>Binding points found</b>"))
        h.addStretch(1)
        clr = QtWidgets.QPushButton("Clear")
        clr.clicked.connect(self._clear_log)
        h.addWidget(clr)
        bl.addLayout(h)
        self.log = QtWidgets.QListWidget()
        bl.addWidget(self.log, 1)
        for lw in (self.now, self.log):
            lw.itemClicked.connect(lambda it: self.select((it.data(QtCore.Qt.ItemDataRole.UserRole) or [None])[0]))
        self.tabs.addTab(bw, "Binding / fit")
        # bom
        pw = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(pw)
        pl.setContentsMargins(3, 3, 3, 3)
        self.bom = QtWidgets.QTableWidget(0, 4)
        self.bom.setHorizontalHeaderLabels(["Category", "Part", "Size", "Qty"])
        hh = self.bom.horizontalHeader()
        hh.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.bom.verticalHeader().setVisible(False)
        self.bom.verticalHeader().setDefaultSectionSize(22)
        self.bom.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        pl.addWidget(self.bom)
        h = QtWidgets.QHBoxLayout()
        self.bom_total = QtWidgets.QLabel()
        h.addWidget(self.bom_total)
        h.addStretch(1)
        b = QtWidgets.QPushButton("Save parts list (CSV)...")
        b.clicked.connect(self.export_csv)
        h.addWidget(b)
        pl.addLayout(h)
        self.tabs.addTab(pw, "Parts list")
        self.report = QtWidgets.QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.tabs.addTab(self.report, "Sweep report")
        d = QtWidgets.QDockWidget("Motion, binding and parts", self)
        d.setObjectName("bottom")
        d.setWidget(self.tabs)
        d.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea, d)
        self.resizeDocks([d], [250], QtCore.Qt.Orientation.Vertical)

    def _actions(self):
        tb = self.addToolBar("Main")
        tb.setObjectName("main")
        tb.setMovable(False)
        mb = self.menuBar()
        fm, em, vm, xm, hm = (mb.addMenu(t) for t in ("&File", "&Edit", "&View", "E&xport", "&Help"))

        def act(text, fn, key=None, menu=None, tool=True, tip=None, checkable=False):
            a = QtGui.QAction(text, self)
            if key:
                a.setShortcut(QtGui.QKeySequence(key))
            if tip:
                a.setToolTip(tip)
            a.setCheckable(checkable)
            a.triggered.connect(fn)
            if menu is not None:
                menu.addAction(a)
            if tool:
                tb.addAction(a)
            return a

        act("New", self.new_file, "Ctrl+N", fm, tool=False)
        act("Open...", self.open_file, "Ctrl+O", fm)
        act("Save", self.save_file, "Ctrl+S", fm)
        act("Save As...", self.save_as, "Ctrl+Shift+S", fm, tool=False)
        fm.addSeparator()
        act("Load demo", self.load_demo, None, fm, tool=False)
        fm.addSeparator()
        act("Exit", self.close, None, fm, tool=False)
        tb.addSeparator()
        act("Undo", self.do_undo, "Ctrl+Z", em)
        act("Redo", self.do_redo, "Ctrl+Y", em)
        act("Delete", self.delete_selected, "Del", em, tool=False)
        act("Turn part to next slot", lambda: self.turn(True, 0), "T", em, tool=False)
        act("Roll part 45 deg", lambda: self.turn(False, 45), "Y", em, tool=False)
        act("Clear active dot", lambda: self.set_active(None), "Esc", em, tool=False)
        tb.addSeparator()
        grp = QtGui.QActionGroup(self)
        self.a_build = act("Build", lambda: self.set_mode("build"), "B", vm, checkable=True,
                           tip="Click dots and add parts (B)")
        self.a_move = act("Move", lambda: self.set_mode("move"), "M", vm, checkable=True,
                          tip="Grab and drag parts to see them move (M)")
        grp.addAction(self.a_build)
        grp.addAction(self.a_move)
        self.a_build.setChecked(True)
        tb.addSeparator()
        for name, label, key in (("3d", "3D", "1"), ("front", "Front", "2"), ("side", "Side", "3"),
                                 ("top", "Top", "4")):
            act(label, lambda _=False, n=name: self.view.view_preset(n), key, vm)
        tb.addSeparator()
        act("Reset pose", self.reset_pose, "R", vm, tip="Put everything back as built")
        act("Sweep", self.sweep, "S", vm, tip="Run the selected joint end to end")
        tb.addSeparator()
        self.a_units = act('Units: in', self.toggle_units, None, vm, tip="Switch inches / millimeters")
        self.a_hits = QtGui.QAction("Stop at hits", self, checkable=True)
        self.a_hits.setChecked(self.project.settings.get("stop_on_collision", True))
        self.a_hits.toggled.connect(lambda v: self._setting("stop_on_collision", v))
        tb.addAction(self.a_hits)
        vm.addAction(self.a_hits)
        tb.addSeparator()
        act("Parts list (CSV)...", self.export_csv, None, xm, tool=False)
        act("Model as STL...", self.export_stl, None, xm, tool=False)
        act("Custom rods as STL (for printing)...", self.export_rods, None, xm, tool=False)
        act("Drawings (PDF)...", self.export_pdf, None, xm, tool=False)
        act("Drawings (DXF)...", self.export_dxf, None, xm, tool=False)
        ex = QtWidgets.QToolButton()
        ex.setText("Export")
        ex.setMenu(xm)
        ex.setPopupMode(QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup)
        tb.addWidget(ex)
        act("How to use", lambda: QtWidgets.QMessageBox.about(self, APP, HELP), "F1", hm, tool=False)

    # ------------------------------------------------------------ core refresh
    def rebuild(self, reset_camera=False):
        self.asm = Assembly(self.project)
        if self.selected and not self.project.item(self.selected):
            self.selected = None
        if self.active and not self._port_valid(self.active):
            self.active = None
        self.ranges = {}
        self.last_stop = None
        self.tracked = {t for t in self.tracked if self.project.item(t)}
        if self.drive_joint is not None and self.drive_joint >= len(self.asm.joints):
            self.drive_joint = None
        self.view.edit_mode = self.mode
        self.view.build(self.project, self.asm, reset_camera=reset_camera)
        self.issues = self._all_issues()
        self._show_ports()
        self._paint()
        self.props.show_item(self.selected)
        self._fill_tree()
        self._fill_bom()
        self.refresh_motion()
        self._hint()
        self.a_units.setText(f"Units: {self.units}")

    def _port_valid(self, port):
        kind, a, b = port
        pr = self.project
        if kind == "slot":
            return a in pr.parts and not pr.slot_taken(a, b)
        if kind == "end":
            return a in pr.rods and not pr.end_taken(a, b)
        return a in pr.parts and not pr.hole_taken(a, b)

    def _show_ports(self):
        if self.mode == "build":
            self.view.show_ports(self.project, self.active)
        else:
            self.view.clear_ports()

    def _all_issues(self):
        from .solver import Issue
        out = [Issue(lvl, key, text, parts, False) for lvl, key, text, parts in self.project.fit_issues()]
        return out + self.asm.analyze()

    def _status(self):
        st = {}
        items = list(self.issues) + ([self.last_stop] if self.last_stop else [])
        for it in items:
            for pid in it.parts:
                if it.level == "bind" or st.get(pid) != "bind":
                    st[pid] = it.level
        return st

    def _paint(self):
        self.view.update_pose(self.asm, self._status(), self.selected, render=False)
        if self.mode == "build":
            txt = "BUILD  -  " + self._active_text()
        else:
            txt = "MOVE  -  grab a part and drag it"
            if self.last_stop:
                txt += f"\nSTOPPED: {self.last_stop.text}"
        self.view.hud(txt, alarm=bool(self.last_stop) and self.mode == "move")
        self.view.render()

    def _active_text(self):
        if not self.active:
            return "click a dot to start adding parts there"
        kind, a, b = self.active
        return {"slot": "slot picked - choose a rod",
                "end": "rod end picked - choose a connector, hinge or ball joint",
                "hole": "hole picked - choose a rod to push through"}[kind]

    def _hint(self):
        sel_rod = self.selected in self.project.rods
        kind = self.active[0] if self.active else None
        if self.mode == "move":
            txt = "Move mode: drag parts in the 3D view. Switch to Build (B) to add parts."
        elif kind == "slot":
            txt = "A slot is picked. Click a <b>rod</b>."
        elif kind == "hole":
            txt = "A hole is picked. Click a <b>rod</b> to push through it."
        elif kind == "end":
            txt = "A rod end is picked. Click a <b>connector</b>, <b>hinge</b> or <b>ball joint</b>."
        elif sel_rod:
            txt = "A rod is selected. Click a part under <b>On a rod</b> (or a connector) to slide it on."
        elif not self.project.parts and not self.project.rods:
            txt = "Empty design. Click a <b>connector</b> to start."
        else:
            txt = "Click a dot in the 3D view: <span style='color:#00a844'>green</span> slot, " \
                  "<span style='color:#ff6d00'>orange</span> rod end, <span style='color:#2979ff'>blue</span> hole. " \
                  "Or click a rod to slide parts onto it."
        self.kit_hint.setText(txt)
        build = self.mode == "build"
        empty = not self.project.parts and not self.project.rods
        for key, b in self.kit_buttons.items():
            if key.startswith("rod:"):
                ok = kind in ("slot", "hole")
            elif key in C.CONNECTOR_ORDER:
                ok = kind == "end" or sel_rod or empty or kind is None
            elif key in C.JOINT_ORDER:
                ok = kind == "end"
            else:
                ok = sel_rod or kind == "end"
            b.setEnabled(build and ok)

    # ------------------------------------------------------------ editing
    def snapshot(self):
        self.undo.append(self.project.to_json())
        self.undo = self.undo[-200:]
        self.redo.clear()
        self.dirty = True
        self._title()

    def edit(self, label, fn, after=None):
        """Run an edit with undo; `after(result)` runs before the redraw. Shows any note."""
        self.snapshot()
        try:
            note = fn()
        except ValueError as ex:
            self.undo.pop()
            self.statusBar().showMessage(str(ex), 6000)
            return
        if self.mode != "build":
            self.mode = "build"
            self.a_build.setChecked(True)
            self.asm.reset()
        if after is not None:
            after(note)
            note = None if not isinstance(note, str) else note
        self.rebuild(reset_camera=len(self.project.parts) + len(self.project.rods) == 1)
        if isinstance(note, str):
            self.statusBar().showMessage(note, 8000)

    def do_undo(self):
        if self.undo:
            self.redo.append(self.project.to_json())
            self.project = Project.from_json(self.undo.pop())
            self.dirty = True
            self.rebuild()
            self._title()

    def do_redo(self):
        if self.redo:
            self.undo.append(self.project.to_json())
            self.project = Project.from_json(self.redo.pop())
            self.dirty = True
            self.rebuild()
            self._title()

    def set_active(self, port):
        self.active = port
        self._show_ports()
        self._paint()
        self._hint()

    def on_port(self, port):
        self.set_active(port)

    def select(self, iid):
        self.selected = iid if iid and self.project.item(iid) else None
        self.props.show_item(self.selected)
        self.tree.blockSignals(True)
        self.tree.clearSelection()
        it = self._tree_items.get(self.selected)
        if it is not None:
            it.setSelected(True)
            self.tree.scrollToItem(it)
        self.tree.blockSignals(False)
        self._paint()
        self._hint()

    def _closed_note(self, closed):
        if not closed:
            return None
        worst = max(c[4] for c in closed)
        msg = f"Clicked {len(closed)} rod end(s) into place"
        if worst > C.FORCED_FIT:
            msg += f" - forced fit of {worst:.1f} mm (K'nex flex)"
        return msg

    def kit(self, key):
        pr = self.project
        kind = self.active[0] if self.active else None
        res = {}

        def keep(name, fn):
            def run():
                out = fn()
                if isinstance(out, tuple):
                    res[name], closed = out
                    return self._closed_note(closed)
                res[name] = out
                return None
            return run

        if key.startswith("rod:"):
            color = key.split(":")[1]
            L = self._ask_length() if color == "custom" else C.RODS[color][0]
            if L is None:
                return
            if kind == "slot":
                _, pid, s = self.active

                def after(_):
                    r = res["r"]
                    self.selected = r.id
                    self.active = None if pr.end_taken(r.id, 1) else ("end", r.id, 1)
                self.edit("Add rod", keep("r", lambda: pr.add_rod_at_slot(pid, s, L)), after)
            elif kind == "hole":
                _, pid, h = self.active

                def after(_):
                    self.selected = res["r"].id
                    self.active = None
                self.edit("Add axle", keep("r", lambda: pr.add_rod_through_hole(pid, h, L)), after)
            else:
                self.statusBar().showMessage("Click a green slot dot or a blue hole first.", 5000)
            return
        tdef = C.TYPES[key]
        if kind == "end" and tdef["family"] in ("connector", "joint"):
            _, rid, end = self.active

            def after(_):
                p = res["p"]
                self.selected = p.id
                opens = [s for (pid, s) in pr.open_slots() if pid == p.id]
                self.active = ("slot", p.id, opens[0]) if opens else None
            self.edit("Add part", keep("p", lambda: pr.add_part_at_end(rid, end, key)), after)
            return
        if tdef["family"] != "joint" and (self.selected in pr.rods or kind == "end"):
            rid = self.selected if self.selected in pr.rods else self.active[1]
            r = pr.rods[rid]
            t = r.length / 2
            if kind == "end" and self.active[1] == rid:
                t = 6.0 if self.active[2] == 0 else r.length - 6.0

            def after(_):
                self.active = None
                self.selected = res["p"].id
            self.edit("Slide on", keep("p", lambda: pr.mount_on_rod(rid, key, t=t)), after)
            return
        if tdef["family"] == "connector":
            lo, hi = pr.bounds()
            at = (0.0, 0.0, 0.0) if not pr.parts and not pr.rods else (hi[0] + 60.0, 0.0, lo[2])

            def after(_):
                p = res["p"]
                self.selected = p.id
                self.active = ("slot", p.id, C.TYPES[key]["slots"][0])
            self.edit("Start", keep("p", lambda: pr.place_start(key, at)), after)
            return
        self.statusBar().showMessage("Pick a dot or a rod first (see the yellow hint).", 5000)

    def _ask_length(self):
        txt, ok = QtWidgets.QInputDialog.getText(
            self, "Custom rod", f"Rod length ({'inches' if self.units == 'in' else 'mm'}; "
            "you can also type 128mm or 5.04\"):", text=fmt(100.0, self.units, 3))
        if not ok:
            return None
        v = parse_length(txt, self.units)
        if v is None or v <= 1.0:
            QtWidgets.QMessageBox.warning(self, APP, "That length didn't read right.")
            return None
        return v

    def turn(self, next_slot, roll):
        pid = self.selected
        if not self.project.can_reorient(pid):
            self.statusBar().showMessage("Select a part you just added at a rod end to turn it.", 5000)
            return
        def after(_):
            opens = [s for (p, s) in self.project.open_slots() if p == pid]
            if self.active and self.active[1] == pid:
                self.active = ("slot", pid, opens[0]) if opens else None
        self.edit("Turn", lambda: self._closed_note(self.project.reorient(pid, next_slot, roll)), after)

    def add_partner(self, pid, type_key):
        res = {}
        self.edit("3D partner", lambda: res.__setitem__("p", self.project.add_3d_partner(pid, type_key)),
                  lambda _: setattr(self, "selected", res["p"].id))

    def set_rod_kind(self, rid, kind):
        if kind == "custom":
            L = self._ask_length()
            if L is None:
                self.props.show_item(rid)
                return
        else:
            L = C.RODS[kind][0]
        self.set_rod_length(rid, L)

    def set_rod_length(self, rid, L):
        if L <= 1.0:
            self.statusBar().showMessage("Rods need some length.", 5000)
            return
        self.edit("Rod length", lambda: self.project.set_rod_length(rid, L))

    def set_joint(self, pid, key, value):
        self.edit("Joint", lambda: self.project.set_joint_param(pid, key, value))

    def delete_selected(self):
        if not self.selected:
            return
        iid = self.selected
        self.selected = None
        self.edit("Delete", lambda: self.project.delete(iid))

    def set_tracked(self, iid, on):
        (self.tracked.add if on else self.tracked.discard)(iid)
        self.refresh_motion()

    def _setting(self, key, value):
        self.project.settings[key] = bool(value)
        self.issues = self._all_issues()
        self._paint()
        self.refresh_motion()

    def toggle_units(self):
        self.project.settings["units"] = "mm" if self.units == "in" else "in"
        self.rebuild()

    # ------------------------------------------------------------ modes / motion
    def set_mode(self, mode, rebuild=True):
        self.mode = mode
        self.view.edit_mode = mode
        (self.a_build if mode == "build" else self.a_move).setChecked(True)
        if mode == "build":
            self.asm.reset()
            self.last_stop = None
            self.view.clear_traces(render=False)
        self.issues = self._all_issues()
        self._show_ports()
        self._paint()
        self._hint()
        self.refresh_motion()

    def drag_start(self, iid, world):
        if self.mode != "move" or iid is None:
            return False
        half = None
        p = self.project.parts.get(iid)
        if p is not None and p.tdef["family"] == "joint":
            # pick the half whose body is nearest the click
            ba, bb = self.asm.body_of_item(iid, "A"), self.asm.body_of_item(iid, "B")
            half = "B" if bb in self.asm.var and ba not in self.asm.var else "A"
        b = self.asm.body_of_item(iid, half)
        if b in self.asm.ground:
            self.statusBar().showMessage(f"{self.project.name_of(iid)} is on the ground - grab a moving part.", 4000)
            self.select(iid)
            return False
        self.drag = (b, self.asm.R[b].T @ (world - self.asm.t[b]), iid)
        self.tracked.add(iid)
        self.select(iid)
        return True

    def drag_to(self, target):
        if not self.drag:
            return
        b, pd, iid = self.drag
        moved, stop, issues = self.asm.drag(b, pd, target)
        if stop is not None and stop.stop:
            self.last_stop = stop
            self._log(stop, f"dragging {self.project.name_of(iid)}")
        elif stop is not None:
            self.statusBar().showMessage(stop.text, 2500)
        elif moved > 0.01:
            self.last_stop = None
        from .solver import Issue
        self.issues = [Issue(l, k, t, pp, False) for l, k, t, pp in self.project.fit_issues()] + issues
        self._ranges()
        self._paint()
        if not self._tab_timer.isActive():
            self._tab_timer.start(80)

    def drag_end(self):
        self.drag = None
        self.refresh_motion()

    def nudge(self, deg):
        j = self._joint_for_drive()
        if j is None:
            return
        if self.mode != "move":
            self.set_mode("move")
        target = self.asm.joint_angle(j) + math.radians(deg)
        stop, issues = self.asm.drive(j, target)
        from .solver import Issue
        self.issues = [Issue(l, k, t, pp, False) for l, k, t, pp in self.project.fit_issues()] + issues
        self.last_stop = stop if (stop and stop.stop) else None
        if stop:
            self._log(stop, f"turning {self.asm.joints[j].label}")
        self._ranges()
        self._paint()
        self.refresh_motion()

    def _joint_for_drive(self):
        if self.drive_joint is not None:
            return self.drive_joint
        if self.selected:
            for j, jt in enumerate(self.asm.joints):
                if jt.kind == "rev" and self.selected in (jt.part, jt.rod):
                    return j
        revs = [j for j, jt in enumerate(self.asm.joints) if jt.kind == "rev"]
        if revs:
            self.statusBar().showMessage("Click a joint in the Motion tab to choose which one to turn.", 5000)
        else:
            self.statusBar().showMessage("No turning joints yet: push a rod through a hole, or add a hinge.", 6000)
        return None

    def reset_pose(self):
        self.asm.reset()
        self.ranges = {}
        self.last_stop = None
        self.view.clear_traces(render=False)
        self.issues = self._all_issues()
        self._paint()
        self.refresh_motion()

    def _log(self, stop, what):
        key = (stop.key, what)
        if key in self._log_keys:
            return
        self._log_keys.add(key)
        it = QtWidgets.QListWidgetItem(f"While {what}: {stop.text}")
        it.setForeground(QtGui.QColor("#c62828"))
        it.setData(QtCore.Qt.ItemDataRole.UserRole, stop.parts)
        self.log.addItem(it)

    def _clear_log(self):
        self.log.clear()
        self._log_keys.clear()

    def _rows(self):
        """[(key, label, value, unit, note)]"""
        rows = []
        for j, jt in enumerate(self.asm.joints):
            label, val, kind = self.asm.joint_reading(j)
            note = {"hinge": "hinge opening", "spin": "turns", "ball": "ball tilt"}[kind]
            rows.append((f"j{j}", label, val, "deg", note))
        for iid in sorted(self.tracked):
            if not self.project.item(iid):
                continue
            cur, des = self.asm.item_point(iid)
            d = cur - des
            rows.append((f"t:{iid}", self.project.name_of(iid), float(np.linalg.norm(d)), "mm",
                         f"dx {fmt(d[0], self.units)}  dy {fmt(d[1], self.units)}  dz {fmt(d[2], self.units)}"))
        return rows

    def _ranges(self):
        for key, _, v, unit, _ in self._rows():
            lo, hi = self.ranges.get(key, (v, v))
            self.ranges[key] = (min(lo, v), max(hi, v))

    def _fv(self, v, unit):
        return f"{v:.1f} deg" if unit == "deg" else fmt(v, self.units)

    def refresh_motion(self):
        dof = self.asm.dof()
        self.dof_label.setText(
            f"<b>{len(self.asm.free)}</b> moving group(s), <b>{len(self.asm.joints)}</b> joint(s), "
            f"<b>{len(self.asm.meshes)}</b> gear mesh(es) - <b>{dof}</b> way(s) it can move")
        self._ranges()
        rows = self._rows()
        self.mtable.setRowCount(len(rows))
        st = self._status()
        for r, (key, label, v, unit, note) in enumerate(rows):
            lo, hi = self.ranges.get(key, (v, v))
            cells = [label + ("   << selected" if key == f"j{self.drive_joint}" else ""),
                     self._fv(v, unit), f"{self._fv(lo, unit)} to {self._fv(hi, unit)}",
                     self._fv(hi - lo, unit), note]
            for c, txt in enumerate(cells):
                it = self.mtable.item(r, c)
                if it is None:
                    it = QtWidgets.QTableWidgetItem()
                    self.mtable.setItem(r, c, it)
                it.setText(txt)
                it.setData(QtCore.Qt.ItemDataRole.UserRole, key)
                part = None
                if key.startswith("j"):
                    part = self.asm.joints[int(key[1:])].part
                else:
                    part = key[2:]
                col = {"bind": "#ffcdd2", "warn": "#ffe0b2"}.get(st.get(part))
                it.setBackground(QtGui.QColor(col) if col else QtGui.QBrush())
        self.now.clear()
        items = list(self.issues)
        if self.last_stop and all(i.key != self.last_stop.key for i in items):
            items.insert(0, self.last_stop)
        if not items:
            li = QtWidgets.QListWidgetItem("No binding or forced fits.")
            li.setForeground(QtGui.QColor("#2e7d32"))
            self.now.addItem(li)
        for i in items:
            li = QtWidgets.QListWidgetItem(("STOP  " if i.stop else "") + i.text)
            li.setForeground(QtGui.QColor("#c62828" if i.level == "bind" else "#e65100"))
            li.setData(QtCore.Qt.ItemDataRole.UserRole, i.parts)
            self.now.addItem(li)
        n = sum(1 for i in items if i.level == "bind")
        self.tabs.setTabText(1, f"Binding / fit ({n})" if n else "Binding / fit")

    def _mtable_clicked(self, row, col):
        it = self.mtable.item(row, 0)
        key = it.data(QtCore.Qt.ItemDataRole.UserRole) if it else None
        if key and key.startswith("j"):
            j = int(key[1:])
            if self.asm.joints[j].kind == "rev":
                self.drive_joint = j
                self.statusBar().showMessage(f"Turning joint: {self.asm.joints[j].label}. "
                                             "Use the arrow buttons or Sweep.", 5000)
            self.select(self.asm.joints[j].part)
            self.refresh_motion()
        elif key:
            self.select(key[2:])

    def sweep(self):
        j = self._joint_for_drive()
        if j is None:
            return
        if self.mode != "move":
            self.set_mode("move")
        jt = self.asm.joints[j]
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        try:
            span = math.radians(360) if not jt.hinge else math.radians(270)
            res = self.asm.sweep(j, span)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        start = self.asm.save()
        watch = sorted(self.tracked) or [jt.part]
        paths = {iid: [] for iid in watch}
        ranges = {}
        for states in res["states"]:
            for s in states:
                self.asm.restore(s)
                for key, _, v, unit, _ in self._rows():
                    lo, hi = ranges.get(key, (v, v))
                    ranges[key] = (min(lo, v), max(hi, v))
        for sign in (-1, +1):
            seq = res["states"][0 if sign > 0 else 1]
            seq = seq if sign > 0 else list(reversed(seq))
            for s in seq:
                self.asm.restore(s)
                for iid in watch:
                    paths[iid].append(self.asm.item_point(iid)[0])
        self.asm.restore(start)
        for k, v in ranges.items():
            a, b = self.ranges.get(k, v)
            self.ranges[k] = (min(a, v[0]), max(b, v[1]))
        # paths: walk -end -> start -> +end
        self.view.set_traces([np.array(p) for p in paths.values()])
        lines = [f"SWEEP: {jt.label}", "=" * (7 + len(jt.label)), ""]
        for sign, name in ((+1, "Forward"), (-1, "Backward")):
            e = res["ends"][sign]
            deg = math.degrees(e["angle"] - self.asm.joint_angle(j))
            stop = e["stop"]
            if stop:
                self._log(stop, f"sweeping {jt.label}")
            lines.append(f"{name}: {deg:+.1f} deg  -  " + (stop.text if stop else "full turn, no binding"))
        lines += ["", f"{'Joint / part':44s} {'Low':>12s} {'High':>12s} {'Travel':>12s}", "-" * 84]
        for key, label, v, unit, _ in self._rows():
            lo, hi = ranges.get(key, (v, v))
            lines.append(f"{label[:44]:44s} {self._fv(lo, unit):>12s} {self._fv(hi, unit):>12s} "
                         f"{self._fv(hi - lo, unit):>12s}")
        if not self.tracked:
            lines += ["", "Tip: tick 'Track its travel' on a part (or drag it) to add it to this report."]
        self.report.setPlainText("\n".join(lines))
        self.tabs.setCurrentWidget(self.report)
        self.drive_joint = j
        self.issues = self._all_issues()
        self._paint()
        self.refresh_motion()

    # ------------------------------------------------------------ lists
    def _fill_tree(self):
        self.tree.blockSignals(True)
        self.tree.clear()
        self._tree_items = {}
        pr = self.project
        groups = [("Rods", list(pr.rods.values())),
                  ("Connectors", [p for p in pr.parts.values() if p.tdef["family"] == "connector"]),
                  ("Joints", [p for p in pr.parts.values() if p.tdef["family"] == "joint"]),
                  ("On-rod parts", [p for p in pr.parts.values() if p.tdef["family"] == "mount"])]
        for title, items in groups:
            if not items:
                continue
            top = QtWidgets.QTreeWidgetItem([f"{title} ({len(items)})", ""])
            self.tree.addTopLevelItem(top)
            for x in items:
                if x.id in pr.rods:
                    typ = fmt(x.length, self.units) + ("" if x.color in C.RODS else " custom")
                else:
                    typ = x.tdef["name"]
                if x.params.get("ground"):
                    typ += "  [ground]"
                it = QtWidgets.QTreeWidgetItem([x.name, typ])
                it.setData(0, QtCore.Qt.ItemDataRole.UserRole, x.id)
                top.addChild(it)
                self._tree_items[x.id] = it
            top.setExpanded(len(items) < 40)
        if self.selected in self._tree_items:
            self._tree_items[self.selected].setSelected(True)
        self.tree.blockSignals(False)

    def _tree_pick(self):
        its = self.tree.selectedItems()
        iid = its[0].data(0, QtCore.Qt.ItemDataRole.UserRole) if its else None
        if iid and iid != self.selected:
            self.select(iid)

    def _fill_bom(self):
        rows = X.bom_rows(self.project, self.units)
        self.bom.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c, v in enumerate(row):
                it = QtWidgets.QTableWidgetItem(str(v))
                self.bom.setItem(r, c, it)
        self.bom.resizeColumnToContents(0)
        self.bom.resizeColumnToContents(1)
        self.bom_total.setText(f"<b>{sum(r[3] for r in rows)}</b> pieces")

    # ------------------------------------------------------------ exports
    def _save_name(self, title, default, filt):
        base = os.path.splitext(os.path.splitext(os.path.basename(self.path or "knex_design"))[0])[0]
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, title, base + default, filt)
        return path

    def export_csv(self):
        path = self._save_name("Save parts list", "_parts.csv", "CSV (*.csv)")
        if path:
            X.write_bom_csv(self.project, path, self.units)
            self.statusBar().showMessage(f"Saved {path}", 6000)

    def export_stl(self):
        path = self._save_name("Save model STL", ".stl", "STL (*.stl)")
        if path:
            X.export_model_stl(self.project, path, self.asm if self.mode == "move" else None)
            self.statusBar().showMessage(f"Saved {path}", 6000)

    def export_rods(self):
        if not any(r.color not in C.RODS for r in self.project.rods.values()):
            QtWidgets.QMessageBox.information(self, APP, "There are no custom-length rods in this design.")
            return
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Folder for the rod STL files")
        if folder:
            paths = X.export_custom_rods(self.project, folder)
            QtWidgets.QMessageBox.information(
                self, APP, f"Saved {len(paths)} rod file(s) to {folder}.\n\nThe end grooves are close to "
                "real K'nex but approximate - print one and test the fit before printing a batch.")

    def export_pdf(self):
        path = self._save_name("Save drawings (PDF)", "_drawings.pdf", "PDF (*.pdf)")
        if path:
            X.export_pdf(self.project, path, self.units,
                         title=os.path.basename(self.path or "KnexLink design"))
            self.statusBar().showMessage(f"Saved {path}", 6000)

    def export_dxf(self):
        path = self._save_name("Save drawings (DXF)", "_drawings.dxf", "DXF (*.dxf)")
        if path:
            X.export_dxf(self.project, path, self.units)
            self.statusBar().showMessage(f"Saved {path}", 6000)

    # ------------------------------------------------------------ files
    def _title(self):
        nm = os.path.basename(self.path) if self.path else "Untitled"
        self.setWindowTitle(f"{APP} - {nm}{' *' if self.dirty else ''}")

    def _maybe_save(self):
        if not self.dirty:
            return True
        r = QtWidgets.QMessageBox.question(
            self, APP, "Save changes first?",
            QtWidgets.QMessageBox.StandardButton.Save | QtWidgets.QMessageBox.StandardButton.Discard |
            QtWidgets.QMessageBox.StandardButton.Cancel)
        if r == QtWidgets.QMessageBox.StandardButton.Save:
            return self.save_file()
        return r == QtWidgets.QMessageBox.StandardButton.Discard

    def _load(self, project, path):
        self.project, self.path = project, path
        self.dirty = False
        self.undo.clear()
        self.redo.clear()
        self.selected = self.active = None
        self.tracked = set()
        self.drive_joint = None
        self._clear_log()
        self.set_mode("build", rebuild=False)
        self.rebuild(reset_camera=True)
        self._title()

    def new_file(self):
        if self._maybe_save():
            self._load(Project(), None)

    def load_demo(self):
        if self._maybe_save():
            self._load(build_demo(), None)

    def open_file(self):
        if not self._maybe_save():
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open design", "", FILE_FILTER)
        if path:
            try:
                with open(path, encoding="utf-8") as fh:
                    pr = Project.from_json(fh.read())
            except Exception as ex:
                QtWidgets.QMessageBox.critical(self, APP, f"Couldn't open that file:\n{ex}")
                return
            self._load(pr, path)

    def save_file(self):
        if not self.path:
            return self.save_as()
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write(self.project.to_json())
        self.dirty = False
        self._title()
        self.statusBar().showMessage(f"Saved {self.path}", 4000)
        return True

    def save_as(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save design", "my_design.knex.json", FILE_FILTER)
        if not path:
            return False
        if not path.lower().endswith(".json"):
            path += ".knex.json"
        self.path = path
        return self.save_file()

    def closeEvent(self, ev):
        if self._maybe_save():
            try:
                self.view.close()
            except Exception:
                pass
            ev.accept()
        else:
            ev.ignore()
