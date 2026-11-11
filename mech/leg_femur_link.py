#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum Sagittal Upper Femur Link (`ALU-080`, 2.032 mm thick)
for Day 06 Walking Legs (`mech/leg_femur_link.py`).

Operates in paired double-shear (inboard plate driven directly by the HipServo horn +
outboard plate supported by `leg_hip_bracket`), swinging in the X-Z sagittal walking plane.

Coordinate convention: mm, Z up, front facing -Y.
Exports:
  - mech/leg_femur_link.FCStd
  - mech/leg_femur_link.step (colored brushed aluminum)
  - mech/leg_femur_link.dxf  (1:1 flat laser-cut profile)
"""
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Mechanical Parameters (mm) ---
THICKNESS_MM       = 2.032   # SendCutSend ALU-080 (0.080" 5052-H32)
PITCH_LEN_MM       = 38.0    # Hip-to-knee center-to-center distance
END_RADIUS_MM      = 6.0     # Rounded link ends -> total width = 12.0 mm, length = 50.0 mm
PIVOT_HOLE_DIA_MM  = 3.10    # M3 shoulder-screw / bearing clearance hole
HORN_BOLT_RAD_MM   = 7.50    # Dual M2 servo-horn drive bolt pattern around hip pivot (x=0)
HORN_HOLE_DIA_MM   = 2.20    # M2 clearance (> 0.014" min hole)
SLOT_HALF_SPAN_MM  = 8.00    # Central weight-reduction stadium slot half-length
SLOT_RADIUS_MM     = 2.20    # 4.40 mm wide lightening slot (leaves 3.80 mm wall bridges)
ALU_COLOR          = (0.76, 0.79, 0.82)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_femur_link.log"


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


def write_femur_dxf(path: Path) -> None:
    """Writes clean millimeter R12 DXF with closed outer stadium + circular/stadium cutouts."""
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
    dxf_circle(HORN_BOLT_RAD_MM, 3.0, HORN_HOLE_DIA_MM * 0.5)
    dxf_circle(HORN_BOLT_RAD_MM, -3.0, HORN_HOLE_DIA_MM * 0.5)
    slot_mid = PITCH_LEN_MM * 0.5 + 3.0
    dxf_stadium(slot_mid - SLOT_HALF_SPAN_MM, slot_mid + SLOT_HALF_SPAN_MM, SLOT_RADIUS_MM)

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_femur_link")
    base_face = make_stadium_face(0.0, PITCH_LEN_MM, END_RADIUS_MM)

    cuts = []
    for cx, cy, dia in [
        (0.0, 0.0, PIVOT_HOLE_DIA_MM),
        (PITCH_LEN_MM, 0.0, PIVOT_HOLE_DIA_MM),
        (HORN_BOLT_RAD_MM, 3.0, HORN_HOLE_DIA_MM),
        (HORN_BOLT_RAD_MM, -3.0, HORN_HOLE_DIA_MM),
    ]:
        c_edge = Part.Circle(App.Vector(cx, cy, 0), App.Vector(0, 0, 1), dia * 0.5).toShape()
        cuts.append(Part.Face(Part.Wire([c_edge])))

    slot_mid = PITCH_LEN_MM * 0.5 + 3.0
    cuts.append(make_stadium_face(slot_mid - SLOT_HALF_SPAN_MM, slot_mid + SLOT_HALF_SPAN_MM, SLOT_RADIUS_MM))

    profile_face = base_face
    for cf in cuts:
        profile_face = profile_face.cut(cf)

    solid = profile_face.extrude(App.Vector(0, 0, THICKNESS_MM))
    assert solid.isValid(), "leg_femur_link shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegFemurLink")
    part_obj.Shape = solid
    doc.recompute()

    write_femur_dxf(MECH_DIR / "leg_femur_link.dxf")
    doc.saveAs(str(MECH_DIR / "leg_femur_link.FCStd"))

    step_path = MECH_DIR / "leg_femur_link.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_femur_link valid={solid.isValid()} "
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
