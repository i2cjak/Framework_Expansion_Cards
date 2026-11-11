#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum CNC-Bent Tibia Strut (`ALU-063`, 1.600 mm thick)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Coordinate convention: mm, Z up (strut axis along Z from ankle Z=0 to knee Z=58.0 mm),
front facing -Y (web at front -Y, two 90-deg stiffening/clevis ribs extending +Y).

Exports:
  - mech/leg_tibia_strut.FCStd
  - mech/leg_tibia_strut.step     (3D bent U-channel solid)
  - mech/leg_tibia_strut.flat.dxf (1:1 flat pattern with BEND lines on layer 'BEND')
"""
import math
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named SendCutSend ALU-063 Parameters (mm) ---
THICKNESS_MM       = 1.6002   # 0.063" 5052-H32 Aluminum
BEND_RADIUS_IN_MM  = 0.8890   # 0.035" effective inside bend radius
K_FACTOR           = 0.42     # SendCutSend ALU-063 K-factor
WEB_OUT_WIDTH_MM   = 22.0     # Outside U-channel width along X (inside gap = 18.8 mm)
FLANGE_OUT_D_MM    = 15.5     # Outside flange depth along Y (> 7.70 mm min formed flange)
STRUT_LEN_MM       = 58.0     # Vertical tibia height along Z (> 38.10 mm min flat length)
PIVOT_HOLE_DIA_MM  = 3.10     # M3 knee & TPU-foot cross-axle holes
PIVOT_FROM_TIP_MM  = 4.00     # 4.0 mm from flange rear edge (8.69 mm from bend > 6.45 mm!)
KNEE_Z_MM          = 52.0     # Knee M3 axle height (6.0 mm below top edge)
ANKLE_Z_MM         = 6.0      # TPU foot M3 cross-bolt height (6.0 mm above bottom edge)
WEB_HOLE_DIA_MM    = 3.10     # 3x M3 linkage / weight-relief holes along web centerline X=0
ALU_COLOR          = (0.74, 0.77, 0.81)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_tibia_strut.log"


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_WIDTH_MM * 0.5 - r_out

    # Build XY U-channel cross-section at Z = 0, front web outer face at Y = -FLANGE_OUT_D_MM * 0.5
    # and flanges extending +Y, then extrude +Z by STRUT_LEN_MM!
    y_front = -FLANGE_OUT_D_MM * 0.5
    y_bend_c = y_front + r_out
    y_tip = y_front + FLANGE_OUT_D_MM
    x_w = half_flat_web
    x_out = WEB_OUT_WIDTH_MM * 0.5
    x_in = x_out - t
    s45 = math.sqrt(0.5)

    pts_outer = [
        App.Vector(-x_out, y_tip, 0.0),
        App.Vector(-x_out, y_bend_c, 0.0),
        App.Vector(-x_w - r_out * s45, y_bend_c - r_out * s45, 0.0),
        App.Vector(-x_w, y_front, 0.0),
        App.Vector(x_w, y_front, 0.0),
        App.Vector(x_w + r_out * s45, y_bend_c - r_out * s45, 0.0),
        App.Vector(x_out, y_bend_c, 0.0),
        App.Vector(x_out, y_tip, 0.0),
    ]
    pts_inner = [
        App.Vector(x_in, y_tip, 0.0),
        App.Vector(x_in, y_bend_c, 0.0),
        App.Vector(x_w + r_in * s45, y_bend_c - r_in * s45, 0.0),
        App.Vector(x_w, y_front + t, 0.0),
        App.Vector(-x_w, y_front + t, 0.0),
        App.Vector(-x_w - r_in * s45, y_bend_c - r_in * s45, 0.0),
        App.Vector(-x_in, y_bend_c, 0.0),
        App.Vector(-x_in, y_tip, 0.0),
    ]

    edges = [
        Part.LineSegment(pts_outer[0], pts_outer[1]).toShape(),
        Part.Arc(pts_outer[1], pts_outer[2], pts_outer[3]).toShape(),
        Part.LineSegment(pts_outer[3], pts_outer[4]).toShape(),
        Part.Arc(pts_outer[4], pts_outer[5], pts_outer[6]).toShape(),
        Part.LineSegment(pts_outer[6], pts_outer[7]).toShape(),
        Part.LineSegment(pts_outer[7], pts_inner[0]).toShape(),
        Part.LineSegment(pts_inner[0], pts_inner[1]).toShape(),
        Part.Arc(pts_inner[1], pts_inner[2], pts_inner[3]).toShape(),
        Part.LineSegment(pts_inner[3], pts_inner[4]).toShape(),
        Part.Arc(pts_inner[4], pts_inner[5], pts_inner[6]).toShape(),
        Part.LineSegment(pts_inner[6], pts_inner[7]).toShape(),
        Part.LineSegment(pts_inner[7], pts_outer[0]).toShape(),
    ]
    face = Part.Face(Part.Wire(edges))
    solid = face.extrude(App.Vector(0.0, 0.0, STRUT_LEN_MM))

    # Cut coaxial knee (Z=52) & ankle/foot (Z=6) M3 holes through both side flanges (along X)
    y_piv = y_tip - PIVOT_FROM_TIP_MM
    for hz in (ANKLE_Z_MM, KNEE_Z_MM):
        cyl = Part.makeCylinder(
            PIVOT_HOLE_DIA_MM * 0.5,
            WEB_OUT_WIDTH_MM * 2.0,
            App.Vector(-WEB_OUT_WIDTH_MM, y_piv, hz),
            App.Vector(1, 0, 0),
        )
        solid = solid.cut(cyl)

    # Cut 3x centerline M3 harness/mount holes through front web (X = 0, Z = 17, 29, 41 mm)
    for wz in (17.0, 29.0, 41.0):
        wcyl = Part.makeCylinder(
            WEB_HOLE_DIA_MM * 0.5,
            t * 4.0,
            App.Vector(0.0, y_front - t, wz),
            App.Vector(0, 1, 0),
        )
        solid = solid.cut(wcyl)

    return solid


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_WIDTH_MM * 0.5 - r_out
    straight_ear  = FLANGE_OUT_D_MM - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)

    half_w = half_flat_web + ba + straight_ear
    bend_x = half_flat_web + ba * 0.5
    piv_x  = half_w - PIVOT_FROM_TIP_MM
    cr     = 1.5

    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed rounded rectangle (-half_w..+half_w in X, 0..STRUT_LEN_MM in Y)
    dxf_line(-half_w + cr, 0.0, half_w - cr, 0.0)
    dxf_arc(half_w - cr, cr, cr, -90.0, 0.0)
    dxf_line(half_w, cr, half_w, STRUT_LEN_MM - cr)
    dxf_arc(half_w - cr, STRUT_LEN_MM - cr, cr, 0.0, 90.0)
    dxf_line(half_w - cr, STRUT_LEN_MM, -half_w + cr, STRUT_LEN_MM)
    dxf_arc(-half_w + cr, STRUT_LEN_MM - cr, cr, 90.0, 180.0)
    dxf_line(-half_w, STRUT_LEN_MM - cr, -half_w, cr)
    dxf_arc(-half_w + cr, cr, cr, 180.0, 270.0)

    # 4x flange pivot holes + 3x web centerline holes
    for hz in (ANKLE_Z_MM, KNEE_Z_MM):
        dxf_circle(-piv_x, hz, PIVOT_HOLE_DIA_MM * 0.5)
        dxf_circle(piv_x, hz, PIVOT_HOLE_DIA_MM * 0.5)
    for wz in (17.0, 29.0, 41.0):
        dxf_circle(0.0, wz, WEB_HOLE_DIA_MM * 0.5)

    # Bend lines on layer 'BEND'
    dxf_line(-bend_x, 0.0, -bend_x, STRUT_LEN_MM, layer="BEND")
    dxf_line(bend_x, 0.0, bend_x, STRUT_LEN_MM, layer="BEND")

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_tibia_strut")
    solid = build_bent_solid()
    assert solid.isValid(), "leg_tibia_strut shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegTibiaStrut")
    part_obj.Shape = solid
    doc.recompute()

    write_flat_dxf(MECH_DIR / "leg_tibia_strut.flat.dxf")
    doc.saveAs(str(MECH_DIR / "leg_tibia_strut.FCStd"))

    step_path = MECH_DIR / "leg_tibia_strut.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_tibia_strut valid={solid.isValid()} "
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
