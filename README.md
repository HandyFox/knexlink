# KnexLink

A K'nex design suite for real-life development. Build with standard K'nex Classic
parts that snap together the way the real ones do, cut any rod to a custom length,
drag the model to watch it move, and get a parts list, STL files and dimensioned
drawings out the other end.

## Start it

**Windows:** double-click `run_windows.bat`. The first run sets up what it needs in a
`.venv` folder (a few minutes); after that it starts right up.

**By hand (any OS):**

```
pip install -r requirements.txt
python main.py
```

Python 3.10 or newer. It opens with a demo: a crank-rocker linkage (with one
custom-length rod), a 14T/34T gear pair and a hinged flap.

## The kit

| Family | Parts |
|---|---|
| Rods | green 16 mm, white 32, blue 54, yellow 86, red 128, grey 190 - plus **custom length** |
| Connectors | 1-way, 2-way straight, 2-way 45, 3-way (red), 4-way (green), 5-way (yellow), 8-way (white), 7-way 3D (blue), 4-way 3D (purple) |
| Joints | hinge (45-315 deg), ball joint (40 deg cone) |
| On a rod | spacers, gears 14T / 34T / crown 82T, wheels S / M / L (tire size editable), rod clip, clip with hole |

Rod lengths are the published K'nex Classic sizes. The connector geometry is fitted so
standard rods form the K'nex 45-degree triangles: every standard diagonal closes within
0.6 mm, except yellow legs with a red diagonal, which is about 2.7 mm short - the same
"red rod is a bit short" quirk the real set has. KnexLink reports that as a forced fit.

## Building (Build mode, key B)

1. Click a dot in the 3D view:
   - **green** = open connector slot - then click a rod color
   - **orange** = free rod end - then click a connector, hinge or ball joint
   - **blue** = open hole - then click a rod to push an axle through it
2. Click a **rod** (no dot) and then a spacer, gear, wheel, clip or connector to slide it onto that rod.
   Set its position along the rod, its turn, and whether it is locked to the rod on the right.
3. **T** turns the part you just placed to its next slot; **Y** rolls it 45 deg around the rod.
4. Rod ends that land on a slot click in automatically, like closing a real K'nex triangle.
5. Change any rod's length on the right. Whatever hangs off its far end slides along; if the
   rod is in a closed loop, the mismatch shows up as a forced fit instead.

Lengths: type `5.04`, `5.04"`, `128mm` or `3' 2"`. A bare number uses the display unit
(toolbar **Units** switches inches / mm).

## Moving it (Move mode, key M)

Grab any part that isn't on the ground and drag. Rigidly clicked parts move together;
connectors spin on rods through their holes (unless locked), hinges and ball joints swing,
and meshing gears turn each other at their tooth ratio. Parts marked **Ground** never move
(the first part you place is ground by default).

The **Motion** tab lists every joint and every tracked part (anything you drag gets tracked)
with its range and total travel. Click a joint, then use the arrow buttons or **Sweep** (S)
to run it end to end and get a report with travel paths drawn in teal.

## Binding and fit checks

Red = binding, orange = warning. Motion stops at the first binding point and logs where:

- **Lock-up** - dead center, or nothing can move without bending a part
- **Joint limits** - hinge outside 45-315 deg, ball joint past its 40 deg cone
- **Interference** - rods and connector claws running into each other (toggle "Stop at hits")
- **Forced fits** - a rod pushed into a slot or hole it doesn't line up with

## Export (menu or toolbar)

- **Parts list (CSV)** - counts by color/type, custom rods by length with hole-to-hole size
- **Model as STL** - the whole model, in its moved position if you're in Move mode
- **Custom rods as STL** - one printable rod per custom length, with K'nex-style end grooves
  (approximate - print one and test the fit first)
- **Drawings (PDF)** - Letter sheets: front/top/side views with overall dimensions, parts
  list, custom rod details
- **Drawings (DXF)** - the same views and parts list in millimeters for CAD/CAM

## Accuracy notes

Rod lengths and rod diameter are published values. Connector reach was fitted from them.
Connector thickness, wheel sizes and the rod end groove are close approximations; the
shapes in the 3D view are recognizable stand-ins, not factory CAD.

## Tests

`python tests/test_core.py`
