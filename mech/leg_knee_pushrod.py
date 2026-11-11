#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum Sagittal Knee Actuation Pushrod (`ALU-080`, 2.032 mm thick)
for Day 06 Walking Legs (`mech/leg_knee_pushrod.py`).

Couples the `12.0 mm` KneeServo crank arm (`J3`) to the `12.0 mm` integral 90-deg bellcrank lobe
on `leg_tibia_strut` at a ~97 deg transmission angle, forming a non-crossing sagittal 4-bar
knee drive with zero dead-center singularity.

Coordinate convention: mm, Z up, front facing -Y.
Exports:
  - mech/leg_knee_pushrod.FCStd
  - mech/leg_knee_pushrod.step (colored brushed aluminum)
  - mech/leg_knee_pushrod.dxf  (1:1 flat laser-cut profile)
"""
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Mechanical Parameters (mm) ---
THICKNESS_MM       = 2.032   # SendCutSend ALU-080 (0.080" 5052-H32)
PITCH_LEN_MM       = 36.0    # Servo-crank pin to tibia-bellcrank pin center-to-center pitch
END_RADIUS_MM      = 4.5     # 9.0 mm link width x 45.0 mm total length (> 9.47 x 6.30 mm min part size)
PIVOT_HOLE_DIA_MM  = 3.10    # M3 shoulder-screw clearance holes at (0,0) and (36.0, 0)
SLOT_HALF_SPAN_MM  = 8.50    # Central weight-reduction stadium slot half-length
SLOT_RADIUS_MM     = 1.50    # 3.00 mm wide lightening slot (leaves 3.00 mm wall bridges)
ALU_COLOR          = (0.68, 0.74, 0.82)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_knee_pushrod.log"


def make_stadium_face(x0: float, x1: float, r: float) -> Part.Face:
    p1 = App.Vector(x0, -r, 0)
    p2 = App.Vector(x1, -r, 0)
    p3 = App.Vector(x1 + r, 0, 0)
    p4 = App.Vector(x1, r, 0)
    p5 = App.Vector(x0, r, 0)
    p6 = App.Vector(x0 - r, 0, 0)

    e1 = Part.LineSegment(p1, p2).toShape()
    e2 = Part.Arc(p2, p3, p4).toShape()
    e3 = Part.LineSegment(p4, p5).toShape()
    e4 = Part.Arc(p5, p6, p1).toShape()
    return Part.Face(Part.Wire([e1, e2, e3, e4]))


def write_pushrod_dxf(path: Path) -> None:
    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    def dxf_stadium(x0, x1, r):
        dxf_line(x0, -r, x1, -r)
        dxf_arc(x1, 0.0, r, -90.0, 90.0)
        dxf_line(x1, r, x0, r)
        dxf_arc(x0, 0.0, r, 90.0, 270.0)

    dxf_stadium(0.0, PITCH_LEN_MM, END_RADIUS_MM)
    dxf_circle(0.0, 0.0, PIVOT_HOLE_DIA_MM * 0.5)
    dxf_circle(PITCH_LEN_MM, 0.0, PIVOT_HOLE_DIA_MM * 0.5)
    mid = PITCH_LEN_MM * 0.5
    dxf_stadium(mid - SLOT_HALF_SPAN_MM, mid + SLOT_HALF_SPAN_MM, SLOT_RADIUS_MM)

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_knee_pushrod")
    base_face = make_stadium_face(0.0, PITCH_LEN_MM, END_RADIUS_MM)

    cuts = []
    for cx in (0.0, PITCH_LEN_MM):
        c_edge = Part.Circle(App.Vector(cx, 0.0, 0.0), App.Vector(0, 0, 1), PIVOT_HOLE_DIA_MM * 0.5).toShape()
        cuts.append(Part.Face(Part.Wire([c_edge])))

    mid = PITCH_LEN_MM * 0.5
    cuts.append(make_stadium_face(mid - SLOT_HALF_SPAN_MM, mid + SLOT_HALF_SPAN_MM, SLOT_RADIUS_MM))

    profile_face = base_face
    for cf in cuts:
        profile_face = profile_face.cut(cf)

    solid = profile_face.extrude(App.Vector(0, 0, THICKNESS_MM))
    assert solid.isValid(), "leg_knee_pushrod shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegKneePushrod")
    part_obj.Shape = solid
    doc.recompute()

    write_pushrod_dxf(MECH_DIR / "leg_knee_pushrod.dxf")
    doc.saveAs(str(MECH_DIR / "leg_knee_pushrod.FCStd"))

    step_path = MECH_DIR / "leg_knee_pushrod.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_knee_pushrod valid={solid.isValid()} "
        f"bbox=({bb.XLength:.2f} x {bb.YLength:.2f} x {bb.ZLength:.2f} mm) "
        f"vol={solid.Volume / 1000.0:.3f} cm3\n"
    )


exit_code = 0
try:
    build()
except Exception:
    LOG_PATH.write_text(traceback.format_exc())
    exit_code = 1
finally:
    if App.GuiUp:
        os._exit(exit_code)
