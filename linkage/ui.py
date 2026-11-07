"""Main window: parts kit, part list, properties, 3D view, travel and binding readouts."""
import copy
import math
import os

import numpy as np
from qtpy import QtCore, QtGui, QtWidgets

from . import geometry as g
from .demo import build_demo
from .kinematics import Mechanism
from .model import (ANGLE, BOOL, CHOICE, LEN, NUM, REF, TEXT, VEC, KINDS, Project,
                    attach_point_neutral, fit_links, mirror_copy)
from .units import fmt_ft_in, parse_length
from .view3d import SceneView

APP_NAME = "Linkage Designer"
POSITIVE_KEYS = {"dia", "length", "width", "thick", "grip_len", "grip_dia", "block", "size"}
FILE_FILTER = "Linkage designs (*.linkage.json *.json);;All files (*)"

HELP_TEXT = """
<h3>Linkage Designer</h3>
<p><b>Directions:</b> X = left/right (right is +), Y = back/front (front is +), Z = up.
All sizes are inches. You can type <i>38.5</i>, <i>3' 2.5"</i>, <i>3ft 2.5in</i> or <i>1.5ft</i>.</p>
<p><b>Move things:</b> put the mouse over a black handle grip, an arm's hole or a slider block
(the cursor turns into a hand) and drag. Everything connected follows. Other handles are
held still while you move one. Motion stops at the first binding point and the parts
that bind turn <span style="color:#ff1744"><b>red</b></span>. Orange = getting close. The selected part is <span style="color:#d500f9"><b>magenta</b></span>.</p>
<p><b>Camera:</b> drag empty space to orbit, Shift+drag or middle-drag to pan, wheel to zoom.
Click a part to select it and edit its sizes on the right.</p>
<p><b>Parts kit</b> (size everything however you want):</p>
<ul>
<li><b>Shaft / Tube</b> - turns in bearings. Put a rod inside a tube (same axis, smaller
diameter) and each twists on its own. The yellow stripe shows the twist.</li>
<li><b>Arm</b> - crank welded to a shaft. Angle 0 points straight up (or forward on an
upright shaft). The hole is at the end.</li>
<li><b>Handle lever</b> - an arm with a grip. This is what you grab.</li>
<li><b>Bellcrank</b> - short pivot with two arms, for turning a corner.</li>
<li><b>Push rod</b> - connects two holes with ball rod-ends. Auto length makes it fit
exactly at the neutral position; turn that off to set your own length.</li>
<li><b>Slider</b> - straight-line motion, like a valve spool.</li>
<li><b>Frame bar</b> - fixed structure, checked for interference.</li>
</ul>
<p><b>Binding checks:</b> lock-up (dead center / can't reach), poor push angle, rod-end
over-tilt, shaft stops, slider end of travel, and parts hitting each other.</p>
<p><b>Sweep</b> runs the selected handle stop-to-stop, draws the paths in teal, and
writes a travel report.</p>
"""


# =================================================================== widgets

class LengthEdit(QtWidgets.QLineEdit):
    """Line edit for inches. Accepts 38.5, 38.5", 3' 2.5", 3ft 2in, 1.5ft."""
    valueEdited = QtCore.Signal(float)

    def __init__(self, value=0.0, parent=None):
        super().__init__(parent)
        self._value = float(value)
        self.setMinimumWidth(60)
        self.setToolTip('Inches. Also accepts feet: 3\' 2.5"  or  1.5ft')
        self.editingFinished.connect(self._commit)
        self.setValue(value)

    def value(self):
        return self._value

    def setValue(self, v):
        self._value = float(v)
        self.setText(fmt_ft_in(self._value, 3))

    def _commit(self):
        v = parse_length(self.text())
        if v is None:
            self.setValue(self._value)
            return
        changed = abs(v - self._value) > 1e-9
        self.setValue(v)
        if changed:
            self.valueEdited.emit(v)


class PropertyPanel(QtWidgets.QScrollArea):
    """Edits the fields of one part. Calls on_change(pid, key, value)."""

    def __init__(self, on_change, parent=None):
        super().__init__(parent)
        self.on_change = on_change
        self.setWidgetResizable(True)
        self.project = None
        self.part = None
        self.widgets = {}
        self._show_empty()

    def _show_empty(self):
        w = QtWidgets.QLabel("Select a part to edit its sizes.\n\n"
                             "Click a part in the 3D view or the parts list.")
        w.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft)
        w.setWordWrap(True)
        w.setMargin(10)
        self.setWidget(w)
        self.widgets = {}
        self.part = None

    def set_part(self, project, part):
        self.project = project
        if part is None:
            self._show_empty()
            return
        self.part = part
        box = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(box)
        form.setLabelAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        title = QtWidgets.QLabel(f"<b>{part.type_label()}</b>  <span style='color:#888'>{part.id}</span>")
        form.addRow(title)
        self.widgets = {}
        for key, label, ftype, extra in part.FIELDS:
            w = self._make(key, ftype, extra)
            self.widgets[key] = (w, ftype, extra)
            form.addRow(label, w)
        if part.kind == "shaft":
            hint = QtWidgets.QLabel("Tip: to nest a rod inside a tube, give it the same axis line "
                                    "and a smaller diameter. Stops limit how far it can twist.")
            hint.setWordWrap(True)
            hint.setStyleSheet("color:#666")
            form.addRow(hint)
        if part.kind == "arm":
            hint = QtWidgets.QLabel("Angle 0 = straight up (forward on an upright shaft). "
                                    "Positive angles follow the right-hand rule around the "
                                    "shaft's start-to-end direction.")
            hint.setWordWrap(True)
            hint.setStyleSheet("color:#666")
            form.addRow(hint)
        self.setWidget(box)
        self.refresh()

    def _emit(self, key, value):
        pid = self.part.id
        QtCore.QTimer.singleShot(0, lambda: self.on_change(pid, key, value))

    def _make(self, key, ftype, extra):
        if ftype == TEXT:
            w = QtWidgets.QLineEdit()
            w.editingFinished.connect(lambda: self._emit(key, w.text().strip() or key))
            return w
        if ftype == LEN:
            w = LengthEdit()
            w.valueEdited.connect(lambda v: self._emit(key, v))
            return w
        if ftype in (ANGLE, NUM):
            w = QtWidgets.QDoubleSpinBox()
            w.setRange(-360.0, 360.0)
            w.setDecimals(1)
            w.setSingleStep(5.0)
            w.setSuffix(" deg")
            w.setKeyboardTracking(False)
            w.valueChanged.connect(lambda v: self._emit(key, float(v)))
            return w
        if ftype == BOOL:
            w = QtWidgets.QCheckBox()
            w.toggled.connect(lambda v: self._emit(key, bool(v)))
            return w
        if ftype == CHOICE:
            w = QtWidgets.QComboBox()
            w.addItems(extra)
            w.currentTextChanged.connect(lambda t: self._emit(key, t))
            return w
        if ftype == REF:
            w = QtWidgets.QComboBox()
            w.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            w.setMinimumContentsLength(12)
            w.addItem("(none)", None)
            for p in self.project.parts.values():
                if p.kind in extra and p.id != self.part.id:
                    w.addItem(f"{p.name}  [{p.type_label()}]", p.id)
            w.currentIndexChanged.connect(lambda i: self._emit(key, w.itemData(i)))
            return w
        if ftype == VEC:
            w = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(3)
            edits = []
            tips = ["X: left - / right +", "Y: back - / front +", "Z: down - / up +"]
            for i, axis in enumerate("XYZ"):
                lab = QtWidgets.QLabel(axis)
                lab.setStyleSheet("color:#888")
                e = LengthEdit()
                e.setToolTip(tips[i] + '   (inches, or 3\' 2")')
                e.valueEdited.connect(lambda v, i=i: self._vec_changed(key, i, v))
                h.addWidget(lab)
                h.addWidget(e, 1)
                edits.append(e)
            w.edits = edits
            return w
        return QtWidgets.QLabel("?")

    def _vec_changed(self, key, i, v):
        vec = list(self.part[key])
        vec[i] = v
        self._emit(key, vec)

    def refresh(self):
        """Reload displayed values without rebuilding (keeps keyboard focus)."""
        if self.part is None:
            return
        part = self.part
        for key, (w, ftype, extra) in self.widgets.items():
            val = part.get(key)
            w.blockSignals(True)
            try:
                if ftype == TEXT:
                    w.setText(str(val))
                elif ftype == LEN:
                    w.setValue(float(val))
                elif ftype in (ANGLE, NUM):
                    w.setValue(float(val))
                elif ftype == BOOL:
                    w.setChecked(bool(val))
                elif ftype == CHOICE:
                    w.setCurrentText(str(val))
                elif ftype == REF:
                    idx = w.findData(val)
                    w.setCurrentIndex(max(idx, 0))
                elif ftype == VEC:
                    for e, x in zip(w.edits, val):
                        e.blockSignals(True)
                        e.setValue(float(x))
                        e.blockSignals(False)
            finally:
                w.blockSignals(False)
        if part.kind == "link":
            w = self.widgets["length"][0]
            w.setEnabled(not part.get("auto_fit", True))


class LinkDialog(QtWidgets.QDialog):
    def __init__(self, project, a=None, b=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add push rod")
        lay = QtWidgets.QFormLayout(self)
        self.ca, self.cb = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        for c in (self.ca, self.cb):
            for p in project.parts.values():
                if p.kind in ("arm", "slider"):
                    c.addItem(f"{p.name}  [{p.type_label()}]", p.id)
        if a is not None and self.ca.findData(a) >= 0:
            self.ca.setCurrentIndex(self.ca.findData(a))
        if b is not None and self.cb.findData(b) >= 0:
            self.cb.setCurrentIndex(self.cb.findData(b))
        elif self.cb.count() > 1 and self.cb.currentIndex() == self.ca.currentIndex():
            self.cb.setCurrentIndex(1 if self.ca.currentIndex() == 0 else 0)
        lay.addRow(QtWidgets.QLabel("Connect a rod between two holes:"))
        lay.addRow("End A", self.ca)
        lay.addRow("End B", self.cb)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok |
                                        QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addRow(bb)

    def ends(self):
        return self.ca.currentData(), self.cb.currentData()


# =================================================================== main window

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, project=None, path=None):
        super().__init__()
        self.resize(1500, 920)
        self.project = project or build_demo()
        self.path = path
        self.dirty = False
        self.undo_stack, self.redo_stack = [], []
        self.selected = None
        self.attach_history = []
        self.mech = None
        self.q = None
        self.issues = []
        self.driver = None
        self.last_stop = None
        self.ranges = {}
        self.bind_log = []
        self._bind_keys = set()

        self.view = SceneView(self)
        self.view.on_pick = self.select
        self.view.on_drag_start = self._drag_start
        self.view.on_drag = self._drag
        self.view.on_drag_end = self._drag_end
        self.setCentralWidget(self.view)

        self._build_docks()
        self._build_actions()
        self._table_timer = QtCore.QTimer(self)
        self._table_timer.setSingleShot(True)
        self._table_timer.timeout.connect(self._refresh_readouts)
        self.statusBar().showMessage("Grab a black handle grip and drag it.  "
                                     "Drag empty space to orbit.  Help menu has the details.")
        self.rebuild(reset_camera=True)
        self._update_title()

    # ------------------------------------------------------------ layout
    def _build_docks(self):
        # parts kit + parts list
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(6, 6, 6, 6)
        kit_label = QtWidgets.QLabel("<b>Parts kit</b> - click to add, then size it")
        lv.addWidget(kit_label)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)
        kit = [
            ("Shaft", "#5b6770", lambda: self.add_shaft("Solid rod"), "Solid shaft that turns in bearings"),
            ("Tube", "#8fa3b3", lambda: self.add_shaft("Tube"), "Hollow tube. Put a shaft through it on the same axis"),
            ("Arm", "#3a6ea5", lambda: self.add_arm(False), "Crank arm on the selected shaft"),
            ("Handle lever", "#1d1d1d", lambda: self.add_arm(True), "Arm with a grip you can drag"),
            ("Bellcrank", "#3a6ea5", self.add_bellcrank, "Pivot with two arms - turns motion around a corner"),
            ("Push rod", "#9e9e9e", self.add_link, "Rod with ball ends connecting two holes"),
            ("Slider", "#2e7d32", self.add_slider, "Straight-line motion, like a valve spool"),
            ("Frame bar", "#4a4a4a", self.add_frame, "Fixed structure for interference checks"),
        ]
        for i, (txt, col, fn, tip) in enumerate(kit):
            b = QtWidgets.QPushButton(txt)
            b.setToolTip(tip)
            b.setMinimumHeight(34)
            b.setStyleSheet(
                f"QPushButton {{ background: #ffffff; border: 1px solid #b8bec6;"
                f" border-left: 7px solid {col}; border-radius: 3px; text-align: left;"
                f" padding-left: 8px; font-weight: 600; }}"
                f"QPushButton:hover {{ background: #eef4fb; }}"
                f"QPushButton:pressed {{ background: #dce8f5; }}")
            b.clicked.connect(fn)
            grid.addWidget(b, i // 2, i % 2)
        lv.addLayout(grid)
        lv.addSpacing(6)
        lv.addWidget(QtWidgets.QLabel("<b>Parts in this design</b>"))
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Part", "Type"])
        self.tree.setColumnWidth(0, 190)
        self.tree.itemSelectionChanged.connect(self._tree_selected)
        lv.addWidget(self.tree, 1)
        row = QtWidgets.QHBoxLayout()
        for txt, fn, tip in (("Duplicate", self.duplicate, "Copy the selected part"),
                             ("Mirror L/R", self.mirror, "Make a left/right mirror copy across X = 0"),
                             ("Delete", self.delete, "Delete the selected part")):
            b = QtWidgets.QPushButton(txt)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            row.addWidget(b)
        lv.addLayout(row)
        dock = QtWidgets.QDockWidget("Parts", self)
        dock.setObjectName("parts")
        dock.setWidget(left)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        left.setMinimumWidth(300)

        # properties
        self.props = PropertyPanel(self.change_property)
        dock = QtWidgets.QDockWidget("Sizes and settings", self)
        dock.setObjectName("props")
        dock.setWidget(self.props)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        self.props.setMinimumWidth(340)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, dock)

        # readouts
        tabs = QtWidgets.QTabWidget()
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Part", "Now", "Range seen", "Total travel",
                                              "Ratio to handle"])
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.Stretch)
        for c in range(1, 5):
            self.table.horizontalHeader().setSectionResizeMode(
                c, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(22)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.horizontalHeaderItem(4).setToolTip(
            "Inches this part moves per inch of handle-grip travel, right now\n"
            "(degrees per degree for shafts). Minus = moves the opposite way.\n"
            "Force multiplier is about 1 / ratio (ignoring friction).")
        self.table.horizontalHeaderItem(2).setToolTip(
            "Lowest and highest positions seen since the last Reset")
        self.table.cellClicked.connect(self._table_clicked)
        tabs.addTab(self.table, "Travel")

        prob = QtWidgets.QWidget()
        pv_ = QtWidgets.QVBoxLayout(prob)
        pv_.setContentsMargins(4, 4, 4, 4)
        pv_.addWidget(QtWidgets.QLabel("<b>Right now</b>"))
        self.now_list = QtWidgets.QListWidget()
        pv_.addWidget(self.now_list, 1)
        hl = QtWidgets.QHBoxLayout()
        hl.addWidget(QtWidgets.QLabel("<b>Binding points found</b> (where motion stopped)"))
        hl.addStretch(1)
        clr = QtWidgets.QPushButton("Clear")
        clr.clicked.connect(self._clear_log)
        hl.addWidget(clr)
        pv_.addLayout(hl)
        self.log_list = QtWidgets.QListWidget()
        pv_.addWidget(self.log_list, 1)
        self.now_list.itemClicked.connect(self._issue_clicked)
        self.log_list.itemClicked.connect(self._issue_clicked)
        tabs.addTab(prob, "Binding / problems")
        self.prob_tab_index = 1
        self.tabs = tabs

        self.report = QtWidgets.QPlainTextEdit()
        self.report.setReadOnly(True)
        f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        self.report.setFont(f)
        self.report.setPlainText("Select a handle and press Sweep to get a travel report.")
        tabs.addTab(self.report, "Sweep report")

        dock = QtWidgets.QDockWidget("Travel and binding", self)
        dock.setObjectName("readouts")
        dock.setWidget(tabs)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea, dock)
        self.resizeDocks([dock], [270], QtCore.Qt.Orientation.Vertical)

    def _build_actions(self):
        tb = self.addToolBar("Main")
        tb.setObjectName("main")
        tb.setMovable(False)
        tb.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextOnly)
        mb = self.menuBar()
        fm = mb.addMenu("&File")
        em = mb.addMenu("&Edit")
        vm = mb.addMenu("&View")
        hm = mb.addMenu("&Help")

        def act(text, fn, shortcut=None, menu=None, tool=True, tip=None):
            a = QtGui.QAction(text, self)
            if shortcut:
                a.setShortcut(QtGui.QKeySequence(shortcut))
            if tip:
                a.setToolTip(tip)
                a.setStatusTip(tip)
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
        act("Load demo (Case-style levers)", self.load_demo, None, fm, tool=False)
        fm.addSeparator()
        act("Exit", self.close, None, fm, tool=False)
        tb.addSeparator()
        act("Undo", self.do_undo, "Ctrl+Z", em)
        act("Redo", self.do_redo, "Ctrl+Y", em)
        em.addSeparator()
        act("Duplicate", self.duplicate, "Ctrl+D", em, tool=False)
        act("Mirror left/right copy", self.mirror, "Ctrl+M", em, tool=False)
        act("Delete", self.delete, "Del", em, tool=False)
        tb.addSeparator()
        for name, label, key in (("3d", "3D", "1"), ("front", "Front", "2"),
                                 ("side", "Side", "3"), ("top", "Top", "4")):
            act(label, lambda _=False, n=name: self.view.view_preset(n), key, vm,
                tip=f"{label} view ({key})")
        tb.addSeparator()
        act("Reset", self.reset_pose, "R", vm,
            tip="Back to neutral, clear the travel ranges and swept paths")
        act("Sweep", self.sweep, "S", vm,
            tip="Run the selected handle stop-to-stop and report travel and binding")
        act("Clear paths", lambda: self.view.clear_traces(), None, vm)
        tb.addSeparator()
        self.a_hits = QtGui.QAction("Stop at hits", self, checkable=True)
        self.a_hits.setToolTip("Stop motion when parts run into each other")
        self.a_hits.setChecked(self.project.settings.get("stop_on_collision", True))
        self.a_hits.toggled.connect(lambda v: self._set_setting("stop_on_collision", v))
        tb.addAction(self.a_hits)
        self.a_tilt = QtGui.QAction("Stop at rod-end tilt", self, checkable=True)
        self.a_tilt.setToolTip("Stop motion when a rod end tilts past its limit")
        self.a_tilt.setChecked(self.project.settings.get("stop_on_overangle", True))
        self.a_tilt.toggled.connect(lambda v: self._set_setting("stop_on_overangle", v))
        tb.addAction(self.a_tilt)
        act("How to use", self.show_help, "F1", hm, tool=False)

    # ------------------------------------------------------------ model plumbing
    def rebuild(self, reset_camera=False, keep_selection=True):
        fit_links(self.project)
        self.mech = Mechanism(self.project)
        self.q = self.mech.q0.copy()
        self.driver = None
        self.last_stop = None
        self.issues = self.mech.analyze(self.q)
        self.ranges = {}
        if not keep_selection or self.selected not in self.project.parts:
            self.selected = None
        self.view.build(self.mech, reset_camera=reset_camera)
        self._refresh_tree()
        self.props.set_part(self.project, self.project.parts.get(self.selected))
        self._update_view()
        self._refresh_readouts()

    def _status_map(self):
        st = {}
        issues = list(self.issues)
        if self.last_stop is not None:
            issues.append(self.last_stop)
        for it in issues:
            for pid in it.parts:
                if it.level == "bind" or st.get(pid) != "bind":
                    st[pid] = it.level
        return st

    def _update_view(self, render=True):
        sel = {self.selected} if self.selected else set()
        self.view.update_pose(self.q, self._status_map(), sel, render=False)
        self._update_hud()
        if render:
            self.view.render()

    def _update_hud(self):
        if self.driver is None:
            txt = "Grab a handle grip, arm hole or slider block and drag"
        else:
            pid, kind = self.mech.dofs[self.driver]
            name = self.project.parts[pid].name
            v = self.q[self.driver] - self.mech.q0[self.driver]
            if kind == "rot":
                travel = v * self.mech.driver_scale(self.driver)
                txt = f"{name}: {math.degrees(v):+.1f} deg   grip travel {travel:+.2f}\""
            else:
                txt = f"{name}: {v:+.2f}\""
            if self.last_stop is not None:
                txt += f"\nSTOPPED: {self.last_stop.text}"
        try:
            self.view.add_text(txt, position="upper_left", font_size=10, name="hud",
                               color="#b00020" if self.last_stop else "#222222")
        except Exception:
            pass

    def _snapshot(self):
        self.undo_stack.append(self.project.to_json())
        if len(self.undo_stack) > 200:
            self.undo_stack.pop(0)
        self.redo_stack.clear()
        self.dirty = True
        self._update_title()

    def do_undo(self):
        if not self.undo_stack:
            return
        self.redo_stack.append(self.project.to_json())
        self.project = Project.from_json(self.undo_stack.pop())
        self.dirty = True
        self.rebuild()
        self._update_title()

    def do_redo(self):
        if not self.redo_stack:
            return
        self.undo_stack.append(self.project.to_json())
        self.project = Project.from_json(self.redo_stack.pop())
        self.dirty = True
        self.rebuild()
        self._update_title()

    def change_property(self, pid, key, value):
        part = self.project.parts.get(pid)
        if part is None or part.get(key) == value:
            return
        if key in POSITIVE_KEYS and float(value) <= 0:
            self.statusBar().showMessage("That size has to be bigger than zero.", 6000)
            self.props.refresh()
            return
        self._snapshot()
        part[key] = value
        if part.kind == "link" and key == "length":
            part["auto_fit"] = False
        if part.kind == "link" and key in ("a", "b") and part["a"] == part["b"]:
            self.statusBar().showMessage("Both ends of a push rod can't go to the same hole.", 6000)
        self.rebuild()
        self.props.refresh()
        if key == "name":
            self._refresh_tree()

    def _set_setting(self, key, value):
        self.project.settings[key] = bool(value)
        self.dirty = True
        self.issues = self.mech.analyze(self.q, self.driver)
        self._update_view()
        self._refresh_readouts()

    # ------------------------------------------------------------ selection
    def select(self, pid):
        self.selected = pid if pid in self.project.parts else None
        if self.selected and self.project.parts[self.selected].kind in ("arm", "slider"):
            if not self.attach_history or self.attach_history[-1] != self.selected:
                self.attach_history.append(self.selected)
                self.attach_history = self.attach_history[-6:]
        self.tree.blockSignals(True)
        self.tree.clearSelection()
        it = self._tree_items.get(self.selected)
        if it is not None:
            it.setSelected(True)
            self.tree.scrollToItem(it)
        self.tree.blockSignals(False)
        self.props.set_part(self.project, self.project.parts.get(self.selected))
        self._update_view()

    def _tree_selected(self):
        items = self.tree.selectedItems()
        pid = items[0].data(0, QtCore.Qt.ItemDataRole.UserRole) if items else None
        if pid != self.selected:
            self.select(pid)

    def _refresh_tree(self):
        self.tree.blockSignals(True)
        self.tree.clear()
        self._tree_items = {}
        P = self.project

        def item(parent, part):
            it = QtWidgets.QTreeWidgetItem([part.name, part.type_label()])
            it.setData(0, QtCore.Qt.ItemDataRole.UserRole, part.id)
            if parent is None:
                self.tree.addTopLevelItem(it)
            else:
                parent.addChild(it)
            self._tree_items[part.id] = it
            return it

        for sh in P.of_kind("shaft"):
            it = item(None, sh)
            for arm in P.arms_on(sh.id):
                item(it, arm)
            it.setExpanded(True)
        for arm in P.of_kind("arm"):
            if arm["shaft"] not in P.parts:
                item(None, arm).setText(1, "Arm (not on a shaft)")
        for kind in ("link", "slider", "frame"):
            for p in P.of_kind(kind):
                item(None, p)
        it = self._tree_items.get(self.selected)
        if it is not None:
            it.setSelected(True)
        self.tree.blockSignals(False)

    # ------------------------------------------------------------ adding parts
    def _spot(self):
        """A y-offset away from existing parts so new parts don't land on top of others."""
        ys = []
        for p in self.project.parts.values():
            for k in ("start", "end"):
                if k in p.data and p.kind != "frame":
                    ys.append(p.data[k][1])
        return (min(ys) - 10.0) if ys else 0.0

    def _selected_shaft(self):
        p = self.project.parts.get(self.selected)
        if p is None:
            return None
        if p.kind == "shaft":
            return p
        if p.kind == "arm":
            return self.project.parts.get(p["shaft"])
        return None

    def _added(self, part):
        self.rebuild()
        self.select(part.id)
        self.statusBar().showMessage(f"Added {part.type_label().lower()} '{part.name}'. "
                                     "Set its sizes on the right.", 6000)

    def add_shaft(self, style):
        self._snapshot()
        y = self._spot()
        tube = style == "Tube"
        p = self.project.add("shaft", name=self.project.unique_name("Tube" if tube else "Shaft"),
                             style=style, start=[-12.0, y, 20.0], end=[12.0, y, 20.0],
                             dia=1.5 if tube else 1.0)
        self._added(p)

    def add_arm(self, grip):
        sh = self._selected_shaft()
        self._snapshot()
        if sh is None:
            y = self._spot()
            sh = self.project.add("shaft", name=self.project.unique_name("Lever pivot" if grip else "Shaft"),
                                  start=[-2.0, y, 12.0], end=[2.0, y, 12.0], dia=1.0)
        s, a, L = sh.axis()
        if grip:
            p = self.project.add("arm", name=self.project.unique_name("Handle lever"), shaft=sh.id,
                                 at=round(L / 2, 3), length=18.0, angle=0.0, width=1.25,
                                 thick=0.375, grip=True)
            if sh["min_angle"] <= -180 and sh["max_angle"] >= 180:
                sh["min_angle"], sh["max_angle"] = -30.0, 30.0
        else:
            p = self.project.add("arm", name=self.project.unique_name("Arm"), shaft=sh.id,
                                 at=round(L / 2, 3), length=4.0, angle=180.0)
        self._added(p)

    def add_bellcrank(self):
        self._snapshot()
        y = self._spot()
        name = self.project.unique_name("Bellcrank")
        sh = self.project.add("shaft", name=name, start=[0.0, y, 8.0], end=[0.0, y, 12.0],
                              dia=0.75)
        self.project.add("arm", name=f"{name} arm A", shaft=sh.id, at=2.0, length=3.0, angle=0.0)
        self.project.add("arm", name=f"{name} arm B", shaft=sh.id, at=2.0, length=3.0, angle=90.0)
        self._added(sh)

    def add_link(self):
        holes = self.project.of_kind("arm", "slider")
        if len(holes) < 2:
            QtWidgets.QMessageBox.information(self, APP_NAME,
                                              "Add at least two arms (or an arm and a slider) first.")
            return
        hist = [h for h in self.attach_history if h in self.project.parts]
        a = hist[-1] if hist else None
        b = hist[-2] if len(hist) > 1 else None
        dlg = LinkDialog(self.project, a, b, self)
        if dlg.exec() != QtWidgets.QDialog.DialogCode.Accepted:
            return
        ea, eb = dlg.ends()
        if ea == eb:
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Pick two different holes.")
            return
        self._snapshot()
        p = self.project.add("link", name=self.project.unique_name("Push rod"), a=ea, b=eb)
        self._added(p)

    def add_slider(self):
        self._snapshot()
        y = self._spot()
        p = self.project.add("slider", name=self.project.unique_name("Slider"),
                             start=[-3.0, y, 10.0], end=[3.0, y, 10.0], neutral=3.0)
        self._added(p)

    def add_frame(self):
        self._snapshot()
        y = self._spot()
        p = self.project.add("frame", name=self.project.unique_name("Frame bar"),
                             start=[-24.0, y, 2.0], end=[24.0, y, 2.0], size=2.0)
        self._added(p)

    # ------------------------------------------------------------ editing parts
    def duplicate(self):
        p = self.project.parts.get(self.selected)
        if p is None:
            return
        if p.kind == "link":
            QtWidgets.QMessageBox.information(self, APP_NAME,
                                              "Push rods connect two holes - add a new one instead.")
            return
        self._snapshot()
        d = copy.deepcopy(p.data)
        d["name"] = self.project.unique_name(p.name)
        if p.kind == "arm":
            sh = self.project.parts.get(p["shaft"])
            L = sh.axis()[2] if sh else 0
            d["at"] = min(L, float(p["at"]) + 2.0)
            new = self.project.add("arm", **d)
        else:
            for k in ("start", "end"):
                d[k] = [d[k][0], d[k][1] - 6.0, d[k][2]]
            new = self.project.add(p.kind, **d)
            if p.kind == "shaft":
                for arm in self.project.arms_on(p.id):
                    ad = copy.deepcopy(arm.data)
                    ad["name"] = self.project.unique_name(arm.name)
                    ad["shaft"] = new.id
                    self.project.add("arm", **ad)
        self._added(new)

    def mirror(self):
        if self.selected not in self.project.parts:
            QtWidgets.QMessageBox.information(self, APP_NAME, "Select a shaft, arm or slider to mirror.")
            return
        self._snapshot()
        new = mirror_copy(self.project, [self.selected])
        self.rebuild()
        if new:
            self.select(new[0])
        self.statusBar().showMessage(f"Mirrored {len(new)} part(s) across the center line (X = 0).", 6000)

    def delete(self):
        p = self.project.parts.get(self.selected)
        if p is None:
            return
        trial = self.project.copy()
        gone = trial.remove(p.id)
        if len(gone) > 1:
            names = ", ".join(trial_name for trial_name in
                              (self.project.parts[x].name for x in gone[1:]))
            r = QtWidgets.QMessageBox.question(
                self, APP_NAME, f"Delete '{p.name}'?\nThis also removes: {names}")
            if r != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        self._snapshot()
        self.project.remove(p.id)
        self.selected = None
        self.rebuild()

    # ------------------------------------------------------------ dragging
    def _dof_for(self, pid):
        p = self.project.parts.get(pid)
        if p is None:
            return None
        if p.kind == "arm" and pid in self.mech.arm_info:
            return self.mech.arm_info[pid][0]
        if p.kind in ("shaft", "slider"):
            return self.mech.dof_of.get(pid)
        if p.kind == "link":
            a = self.mech.attach.get(p["a"])
            return a[0] if a else None
        return None

    def _drag_start(self, pid):
        d = self._dof_for(pid)
        if d is None:
            return False
        if self.driver != d:
            self.issues = self.mech.analyze(self.q, d)
        self.driver = d
        self.select(pid)
        free = self.mech.free_motions(d)
        if free > 0:
            self.statusBar().showMessage(
                f"Note: this linkage has {free} loose motion(s) - something isn't connected, "
                "so loose parts just stay put.", 8000)
        return True

    def _drag(self, target):
        q, issues, stop = self.mech.move(self.q, self.driver, target, self.issues)
        moved = not np.allclose(q, self.q)
        self.q, self.issues = q, issues
        if stop is not None:
            self.last_stop = stop
            self._log_bind(stop)
        elif moved:
            self.last_stop = None
        self._track_ranges()
        self._update_view()
        if not self._table_timer.isActive():
            self._table_timer.start(60)

    def _drag_end(self):
        self._refresh_readouts()
        if self.last_stop is not None:
            self.statusBar().showMessage("Binding: " + self.last_stop.text, 10000)

    def _driver_pos_text(self, d, q):
        pid, kind = self.mech.dofs[d]
        v = q[d] - self.mech.q0[d]
        if kind == "rot":
            return (f"{math.degrees(v):+.1f} deg / grip {v * self.mech.driver_scale(d):+.2f}\"")
        return f"{v:+.2f}\""

    def _log_bind(self, stop, d=None, q=None):
        d = self.driver if d is None else d
        q = self.q if q is None else q
        key = (stop.key, d, round(float(q[d]), 2))
        if key in self._bind_keys:
            return
        self._bind_keys.add(key)
        name = self.project.parts[self.mech.dofs[d][0]].name
        text = f"{name} at {self._driver_pos_text(d, q)}:  {stop.text}"
        self.bind_log.append((text, stop.parts))
        it = QtWidgets.QListWidgetItem(text)
        it.setForeground(QtGui.QColor("#c62828"))
        it.setData(QtCore.Qt.ItemDataRole.UserRole, stop.parts)
        self.log_list.addItem(it)
        self.log_list.scrollToBottom()

    def _clear_log(self):
        self.bind_log.clear()
        self._bind_keys.clear()
        self.log_list.clear()

    def reset_pose(self):
        self.q = self.mech.q0.copy()
        self.driver = None
        self.last_stop = None
        self.issues = self.mech.analyze(self.q)
        self.ranges = {}
        self.view.clear_traces(render=False)
        self._update_view()
        self._refresh_readouts()

    # ------------------------------------------------------------ readouts
    def _rows(self, q):
        """(pid, label, value, unit, is_driver) in display order."""
        m = self.mech
        P = self.project.parts
        rows = []
        for i, (pid, kind) in enumerate(m.dofs):
            v = q[i] - m.q0[i]
            if kind == "rot":
                rows.append((pid, P[pid].name, math.degrees(v), "deg", i))
                for aid, (d, _, _, _) in m.arm_info.items():
                    if d == i:
                        rows.append((aid, "    " + P[aid].name, m.arm_radius(aid) * v, "in", i))
            else:
                rows.append((pid, P[pid].name, v, "in", i))
        return rows

    def _track_ranges(self):
        for pid, _, v, unit, _ in self._rows(self.q):
            lo, hi = self.ranges.get(pid, (v, v))
            self.ranges[pid] = (min(lo, v), max(hi, v))

    @staticmethod
    def _fmt(v, unit):
        return f"{v:+.1f} deg" if unit == "deg" else f"{v:+.2f}\""

    def _refresh_readouts(self):
        m = self.mech
        self._track_ranges()
        rows = self._rows(self.q)
        v = None
        scale = 1.0
        if self.driver is not None and m.links:
            v = m.velocities(self.q, self.driver, m.held_for(self.driver) | {self.driver})
            scale = m.driver_scale(self.driver)
        self.table.setRowCount(len(rows))
        st = self._status_map()
        for r, (pid, label, val, unit, dof) in enumerate(rows):
            lo, hi = self.ranges.get(pid, (val, val))
            span = hi - lo
            ratio = ""
            if v is not None:
                if unit == "in":
                    p = self.project.parts[pid]
                    per = (m.arm_radius(pid) * v[dof]) if p.kind == "arm" else v[dof]
                    ratio = f"{per / scale:+.3f}" if abs(per) > 1e-9 else "-"
                elif m.dofs[self.driver][1] == "rot":
                    ratio = f"{v[dof]:+.3f}" if abs(v[dof]) > 1e-9 else "-"
            cells = [label, self._fmt(val, unit), f"{self._fmt(lo, unit)} to {self._fmt(hi, unit)}",
                     (f"{span:.1f} deg" if unit == "deg" else
                      (f"{span:.2f}\"  ({fmt_ft_in(span)})" if span >= 12 else f"{span:.2f}\"")),
                     ratio]
            if dof == self.driver and unit == "deg":
                cells[0] += "   << moving"
            for c, txt in enumerate(cells):
                it = self.table.item(r, c)
                if it is None:
                    it = QtWidgets.QTableWidgetItem()
                    self.table.setItem(r, c, it)
                it.setText(txt)
                it.setData(QtCore.Qt.ItemDataRole.UserRole, pid)
                col = {"bind": "#ffcdd2", "warn": "#ffe0b2"}.get(st.get(pid))
                it.setBackground(QtGui.QColor(col) if col else QtGui.QBrush())
                f = it.font()
                f.setBold(unit == "deg" or self.project.parts[pid].kind == "slider")
                it.setFont(f)
        # problems right now
        self.now_list.clear()
        items = list(self.issues)
        if self.last_stop is not None and all(i.key != self.last_stop.key for i in items):
            items.insert(0, self.last_stop)
        if self.driver is not None:
            free = m.free_motions(self.driver)
            if free:
                self.now_list.addItem(f"Note: {free} loose motion(s) in this linkage "
                                      "(a part isn't tied down by push rods)")
        if not items:
            ok = QtWidgets.QListWidgetItem("No binding here.")
            ok.setForeground(QtGui.QColor("#2e7d32"))
            self.now_list.addItem(ok)
        for it_ in items:
            li = QtWidgets.QListWidgetItem(("STOP  " if it_.stop else "") + it_.text)
            li.setForeground(QtGui.QColor("#c62828" if it_.level == "bind" else "#e65100"))
            li.setData(QtCore.Qt.ItemDataRole.UserRole, it_.parts)
            self.now_list.addItem(li)
        n_bind = sum(1 for i in items if i.level == "bind")
        self.tabs.setTabText(self.prob_tab_index,
                             f"Binding / problems ({n_bind})" if n_bind else "Binding / problems")

    def _table_clicked(self, row, col):
        it = self.table.item(row, 0)
        if it is not None:
            self.select(it.data(QtCore.Qt.ItemDataRole.UserRole))

    def _issue_clicked(self, item):
        parts = item.data(QtCore.Qt.ItemDataRole.UserRole)
        if parts:
            self.select(parts[0])

    # ------------------------------------------------------------ sweep
    def sweep(self):
        d = self._dof_for(self.selected) if self.selected else None
        if d is None:
            d = self.driver
        if d is None and self.mech.handle_dofs:
            d = self.mech.handle_dofs[0]
        if d is None:
            QtWidgets.QMessageBox.information(self, APP_NAME, "Select a handle (or any moving part) first.")
            return
        m = self.mech
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.CursorShape.WaitCursor)
        try:
            start_q = self.q.copy()
            res = m.sweep(start_q, d)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        pid = m.dofs[d][0]
        name = self.project.parts[pid].name
        all_q = [qq for tr in res["traces"] for qq in tr]
        # ranges over the whole sweep
        rng = {}
        for qq in all_q:
            for rp, label, val, unit, _ in self._rows(qq):
                lo, hi = rng.get(rp, (val, val))
                rng[rp] = (min(lo, val), max(hi, val))
        for rp, (lo, hi) in rng.items():
            a, b = self.ranges.get(rp, (lo, hi))
            self.ranges[rp] = (min(a, lo), max(b, hi))
        # paths
        paths = []
        for tr in res["traces"]:
            for gp in list(m.arm_info.keys()) + list(m.track.keys()):
                paths.append([m.grab_point(gp, qq) for qq in tr])
        # merge both directions into one path per point
        merged = []
        n = len(m.arm_info) + len(m.track)
        if len(res["traces"]) == 2 and n:
            for k in range(n):
                a, b = paths[k], paths[n + k]
                merged.append(list(reversed(b)) + a)
        self.view.set_traces(merged or paths)
        # report
        lines = [f"SWEEP: {name}", "=" * (7 + len(name)), ""]
        for sign, label in ((+1, "Positive direction"), (-1, "Negative direction")):
            e = res["ends"][sign]
            pos = self._driver_pos_text(d, e["q"])
            if e["stop"] is None:
                why = "reached its stop (no binding)"
            else:
                why = "BINDS: " + e["stop"].text
                self._log_bind(e["stop"], d, e["q"])
            lines.append(f"{label}: goes to {pos}")
            lines.append(f"    {why}")
        lines.append("")
        lines.append(f"{'Part':38s} {'Low':>11s} {'High':>11s} {'Total':>11s}")
        lines.append("-" * 74)
        for rp, label, val, unit, dof in self._rows(start_q):
            if dof not in m.component(d):
                continue
            lo, hi = rng[rp]
            if unit == "deg":
                lines.append(f"{label[:38]:38s} {lo:+10.1f}d {hi:+10.1f}d {hi - lo:10.1f}d")
            else:
                lines.append(f"{label[:38]:38s} {lo:+10.2f}\" {hi:+10.2f}\" {hi - lo:10.2f}\"")
        lines.append("")
        warn = set()
        for qq in all_q[:: max(1, len(all_q) // 60)]:
            for it_ in m.analyze(qq, d):
                if it_.level == "warn" or (it_.level == "bind" and not it_.stop):
                    warn.add(it_.text.split(" at ")[0].split(" only ")[0].split(" tilts ")[0])
        if warn:
            lines.append("Along the way:")
            for w in sorted(warn):
                lines.append(f"  - {w}")
        else:
            lines.append("No warnings along the way.")
        self.report.setPlainText("\n".join(lines))
        self.tabs.setCurrentWidget(self.report)
        self.q = start_q
        self.driver = d
        self.issues = m.analyze(self.q, d)
        self.last_stop = None
        self._update_view()
        self._refresh_readouts()

    # ------------------------------------------------------------ files
    def _update_title(self):
        nm = os.path.basename(self.path) if self.path else "Untitled"
        self.setWindowTitle(f"{APP_NAME} - {nm}{' *' if self.dirty else ''}")

    def _maybe_save(self):
        if not self.dirty:
            return True
        r = QtWidgets.QMessageBox.question(
            self, APP_NAME, "Save changes to this design first?",
            QtWidgets.QMessageBox.StandardButton.Save | QtWidgets.QMessageBox.StandardButton.Discard |
            QtWidgets.QMessageBox.StandardButton.Cancel)
        if r == QtWidgets.QMessageBox.StandardButton.Save:
            return self.save_file()
        return r == QtWidgets.QMessageBox.StandardButton.Discard

    def _load(self, project, path):
        self.project = project
        self.path = path
        self.dirty = False
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.selected = None
        self._clear_log()
        self.a_hits.setChecked(project.settings.get("stop_on_collision", True))
        self.a_tilt.setChecked(project.settings.get("stop_on_overangle", True))
        self.rebuild(reset_camera=True)
        self._update_title()

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
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                pr = Project.from_json(fh.read())
        except Exception as ex:
            QtWidgets.QMessageBox.critical(self, APP_NAME, f"Couldn't open that file:\n{ex}")
            return
        self._load(pr, path)

    def save_file(self):
        if not self.path:
            return self.save_as()
        try:
            with open(self.path, "w", encoding="utf-8") as fh:
                fh.write(self.project.to_json())
        except Exception as ex:
            QtWidgets.QMessageBox.critical(self, APP_NAME, f"Couldn't save:\n{ex}")
            return False
        self.dirty = False
        self._update_title()
        self.statusBar().showMessage(f"Saved {self.path}", 4000)
        return True

    def save_as(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save design", "my_linkage.linkage.json",
                                                        FILE_FILTER)
        if not path:
            return False
        if not path.lower().endswith(".json"):
            path += ".linkage.json"
        self.path = path
        return self.save_file()

    def show_help(self):
        QtWidgets.QMessageBox.about(self, f"How to use {APP_NAME}", HELP_TEXT)

    def closeEvent(self, ev):
        if self._maybe_save():
            try:
                self.view.close()
            except Exception:
                pass
            ev.accept()
        else:
            ev.ignore()
