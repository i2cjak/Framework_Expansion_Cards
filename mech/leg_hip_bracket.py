#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum CNC-Bent Hip Clevis Bracket (`ALU-063`, 1.600 mm thick)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Coordinate convention: mm, Z up, front facing -Y.
Forms an inverted U-clevis (top web + two downward -Z side flanges, or upward +Z clevis ears)
bolting to `leg_bay_housing` with dual M2.5 fasteners and supporting the M3 hip pitch axle.

Exports:
  - mech/leg_hip_bracket.FCStd
  - mech/leg_hip_bracket.step     (3D bent solid with R=0.889mm inside bend radii)
  - mech/leg_hip_bracket.flat.dxf (1:1 flat pattern with BEND lines on layer 'BEND')
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
WEB_OUT_WIDTH_MM   = 28.0     # Outside width across clevis
FLANGE_OUT_H_MM    = 18.5     # Outside height of each 90-deg ear (> 7.70 mm min flange)
BRACKET_DEPTH_MM   = 20.0     # Fore-aft depth along Y (> 9.53 mm min flat width)
MOUNT_HOLE_DIA_MM  = 2.70     # M2.5 housing attachment holes on center web
MOUNT_HOLE_Y_MM    = 5.50     # +/- 5.50 mm along Y on X=0 centerline (11.5 mm from bend!)
PIVOT_HOLE_DIA_MM  = 3.10     # M3 hip pitch axle clearance holes on both ears
PIVOT_FROM_TIP_MM  = 5.00     # 5.0 mm from ear tip (11.0 mm from bend tangent > die_width/2!)
ALU_COLOR          = (0.72, 0.75, 0.79)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_hip_bracket.log"


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_WIDTH_MM * 0.5 - r_out          # 14.0 - 2.4892 = 11.5108 mm
    straight_ear  = FLANGE_OUT_H_MM - r_out                 # 18.5 - 2.4892 = 16.0108 mm

    # Build XZ cross-section profile (at Y = -BRACKET_DEPTH_MM/2) and extrude +Y by BRACKET_DEPTH_MM!
    # Web top at Z = 0, inner face at Z = -t; ears bend DOWNWARD (-Z) so clevis straddles hip link below housing!
    y0 = -BRACKET_DEPTH_MM * 0.5
    x_w = half_flat_web
    x_out = WEB_OUT_WIDTH_MM * 0.5
    x_in  = x_out - t
    z_bend_c = -r_out
    z_tip = -FLANGE_OUT_H_MM

    s45 = math.sqrt(0.5)

    pts_outer = [
        App.Vector(-x_out, y0, z_tip),
        App.Vector(-x_out, y0, z_bend_c),
        # Left outer 90-deg arc centered at (-x_w, y0, z_bend_c) with radius r_out
        App.Vector(-x_w - r_out * s45, y0, z_bend_c + r_out * s45),
        App.Vector(-x_w, y0, 0.0),
        App.Vector(x_w, y0, 0.0),
        # Right outer 90-deg arc centered at (+x_w, y0, z_bend_c) with radius r_out
        App.Vector(x_w + r_out * s45, y0, z_bend_c + r_out * s45),
        App.Vector(x_out, y0, z_bend_c),
        App.Vector(x_out, y0, z_tip),
    ]
    pts_inner = [
        App.Vector(x_in, y0, z_tip),
        App.Vector(x_in, y0, z_bend_c),
        App.Vector(x_w + r_in * s45, y0, z_bend_c + r_in * s45),
        App.Vector(x_w, y0, -t),
        App.Vector(-x_w, y0, -t),
        App.Vector(-x_w - r_in * s45, y0, z_bend_c + r_in * s45),
        App.Vector(-x_in, y0, z_bend_c),
        App.Vector(-x_in, y0, z_tip),
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
    solid = face.extrude(App.Vector(0, BRACKET_DEPTH_MM, 0))

    # Cut 2x M2.5 mounting holes through top web (along Z)
    for hy in (-MOUNT_HOLE_Y_MM, MOUNT_HOLE_Y_MM):
        cyl = Part.makeCylinder(MOUNT_HOLE_DIA_MM * 0.5, t * 4.0, App.Vector(0.0, hy, -t * 2.0), App.Vector(0, 0, 1))
        solid = solid.cut(cyl)

    # Cut coaxial M3 hip axle holes through both side ears (along X at Z = z_tip + PIVOT_FROM_TIP_MM, Y = 0)
    z_piv = z_tip + PIVOT_FROM_TIP_MM
    axle_cyl = Part.makeCylinder(
        PIVOT_HOLE_DIA_MM * 0.5,
        WEB_OUT_WIDTH_MM * 2.0,
        App.Vector(-WEB_OUT_WIDTH_MM, 0.0, z_piv),
        App.Vector(1, 0, 0),
    )
    solid = solid.cut(axle_cyl)
    return solid


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_WIDTH_MM * 0.5 - r_out
    straight_ear  = FLANGE_OUT_H_MM - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)

    half_len = half_flat_web + ba + straight_ear
    half_y   = BRACKET_DEPTH_MM * 0.5
    bend_x   = half_flat_web + ba * 0.5
    piv_x    = half_len - PIVOT_FROM_TIP_MM
    cr       = 1.5  # 1.5 mm corner fillets on outer corners

    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed rounded rectangle outer contour (-half_len..+half_len, -half_y..+half_y)
    dxf_line(-half_len + cr, -half_y, half_len - cr, -half_y)
    dxf_arc(half_len - cr, -half_y + cr, cr, -90.0, 0.0)
    dxf_line(half_len, -half_y + cr, half_len, half_y - cr)
    dxf_arc(half_len - cr, half_y - cr, cr, 0.0, 90.0)
    dxf_line(half_len - cr, half_y, -half_len + cr, half_y)
    dxf_arc(-half_len + cr, half_y - cr, cr, 90.0, 180.0)
    dxf_line(-half_len, half_y - cr, -half_len, -half_y + cr)
    dxf_arc(-half_len + cr, -half_y + cr, cr, 180.0, 270.0)

    # Interior holes (2x web M2.5, 2x ear M3)
    dxf_circle(0.0, -MOUNT_HOLE_Y_MM, MOUNT_HOLE_DIA_MM * 0.5)
    dxf_circle(0.0, MOUNT_HOLE_Y_MM, MOUNT_HOLE_DIA_MM * 0.5)
    dxf_circle(-piv_x, 0.0, PIVOT_HOLE_DIA_MM * 0.5)
    dxf_circle(piv_x, 0.0, PIVOT_HOLE_DIA_MM * 0.5)

    # Bend centerlines on non-cutting layer 'BEND'
    dxf_line(-bend_x, -half_y, -bend_x, half_y, layer="BEND")
    dxf_line(bend_x, -half_y, bend_x, half_y, layer="BEND")

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_hip_bracket")
    solid = build_bent_solid()
    assert solid.isValid(), "leg_hip_bracket shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegHipBracket")
    part_obj.Shape = solid
    doc.recompute()

    write_flat_dxf(MECH_DIR / "leg_hip_bracket.flat.dxf")
    doc.saveAs(str(MECH_DIR / "leg_hip_bracket.FCStd"))

    step_path = MECH_DIR / "leg_hip_bracket.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_hip_bracket valid={solid.isValid()} "
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
