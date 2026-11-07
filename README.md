# Linkage Designer

A K'nex-style 3D layout tool for control linkages: shafts, tubes with rods running
through them, arms, handle levers, bellcranks, push rods and sliders. Grab a handle,
pull it, and watch everything move. It shows inches of travel and stops at binding points.

## Start it

**Easiest:** double-click `run_windows.bat`. The first run installs what it needs into a
`.venv` folder next to the app (a few minutes). After that it starts right up.

**Or by hand:**

```
pip install -r requirements.txt
python main.py
```

Python 3.10 or newer. It opens with a demo: Case-style twin levers on a rod-inside-a-tube
cross shaft. The left lever twists the inner rod, the right lever twists the outer tube,
and the rod's far end turns a bellcrank that pushes a valve spool around a corner.

## Using it

| Do this | How |
|---|---|
| Move a handle | Hover a black grip (cursor becomes a hand) and drag |
| Move any arm or slider | Drag its hole or block the same way |
| Orbit / pan / zoom | Drag empty space / Shift+drag / mouse wheel |
| Select a part | Click it, or pick it in the parts list |
| Change sizes | Edit on the right. Type `38.5`, `3' 2.5"`, `3ft 2.5in` or `1.5ft` |
| Views | Toolbar or keys 1 (3D), 2 (Front), 3 (Side), 4 (Top) |
| Full stop-to-stop run | Select a handle, press **Sweep** (S). Paths draw in teal and a report appears |
| Back to neutral | **Reset** (R) |
| Left/right copy | **Mirror L/R** copies the selected shaft and its arms across the center line |

Directions: **X** = left/right (right is +), **Y** = back/front (front is +), **Z** = up.

**Rod inside a tube:** give the rod the same axis line as the tube and a smaller
diameter. Each twists on its own; the yellow stripe on each shows its twist.

**Arm angles:** 0 = straight up (straight forward on an upright shaft). Positive follows
the right-hand rule around the shaft's start-to-end direction.

**Push rods** auto-size to fit at the neutral position. Turn off "Auto length" to set
your own length (like adjusting a turnbuckle).

## What it checks

Red = binding, orange = getting close. Motion stops at the first binding point and it is
logged on the **Binding / problems** tab with the handle position where it happened.

- **Lock-up / dead center** - the linkage can't go further without a rod changing length
- **Poor push angle** - a rod pushing an arm mostly along the arm instead of across it
- **Rod-end over-tilt** - a ball rod end tipped past its limit (20 deg default, per rod)
- **Stops** - a shaft hitting its min/max angle, a slider at the end of its track
- **Interference** - parts running into each other or into frame bars

Turn off "Stop at hits" or "Stop at rod-end tilt" on the toolbar to drag past those and
just see them highlighted.

## Travel tab

For every shaft, arm and slider: where it is now, the lowest and highest positions seen,
total travel in inches, and the ratio to the handle. Ratio = inches this part moves per
inch of grip travel; the force multiplier is roughly 1 / ratio (ignoring friction).

## Files

Designs save as `.linkage.json` (File > Save). No CAD export yet.

## Tests

`python tests/test_core.py`
