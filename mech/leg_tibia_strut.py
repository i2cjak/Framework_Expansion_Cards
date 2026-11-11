#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum CNC-Bent Sagittal Bellcrank Tibia Strut (`ALU-063`, 1.600 mm thick)
for Day 06 Walking Legs (`mech/leg_tibia_strut.py`).

Coordinate convention: mm, Z up (strut axis along Z from Z=0 to Z=60.0 mm),
front facing -Y (U-channel centered on Y=0 with `20.0 mm` outside width / `16.8 mm` inside gap,
front web at `X = 0` facing forward `-X`, and two 90-deg side flanges extending `+X` in the
sagittal `X-Z` plane!).
The upper flanges feature an integral `12.0 mm` 90-deg bellcrank lobe (`X = 22.5 mm, Z = 53.0 mm`)
driven by `leg_knee_pushrod`, while the lower shin tapers cleanly down to `15.5 mm` depth.

Exports:
  - mech/leg_tibia_strut.FCStd
  - mech/leg_tibia_strut.step     (3D bent U-channel solid with tapered bellcrank lobe)
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
WEB_OUT_Y_MM       = 20.0     # Outside U-channel width across Y (inside gap = 16.80 mm)
SHIN_FLANGE_X_MM   = 15.5     # Lower shin outside flange depth along +X (> 7.70 mm min flange)
BELL_FLANGE_X_MM   = 26.5     # Upper bellcrank lobe outside flange depth along +X
WEB_TOP_Z_MM       = 39.5     # Front closed U-channel web top (> 38.10 mm min bend; open clevis above!)
CLEVIS_RELIEF_X_MM = 3.80     # Front clevis throat cut X above WEB_TOP_Z_MM (clears bend zone cleanly)
TAPER_Z0_MM        = 35.5     # Height where shin flange begins widening into bellcrank lobe
TAPER_Z1_MM        = 46.5     # Height where full 26.5 mm bellcrank lobe depth is reached
STRUT_LEN_MM       = 61.5     # Vertical tibia height along Z (> 38.10 mm min flat length)

PIVOT_HOLE_DIA_MM  = 3.10     # M3 knee, bellcrank & ankle cross-axle holes (along Y)
SHIN_PIVOT_X_MM    = 10.5     # Knee & ankle bore X (9.24 mm from bend centerline >> 7.55 mm min)
BELL_PIVOT_X_MM    = 22.5     # Bellcrank pushrod bore X (12.0 mm lever arm behind knee!)
KNEE_Z_MM          = 54.5     # Knee & bellcrank M3 axle height (15.0 mm open clevis throat above web!)
ANKLE_Z_MM         = 7.5      # TPU foot M3 cross-bolt height (7.5 mm above bottom -> exact 47.0 mm tibia)
WEB_HOLE_DIA_MM    = 3.10     # 2x M3 weight-relief holes along front web centerline (Y=0)
ALU_COLOR          = (0.74, 0.77, 0.81)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_tibia_strut.log"


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_Y_MM * 0.5 - r_out
    s45 = math.sqrt(0.5)

    # Build XY U-channel cross-section out to full BELL_FLANGE_X_MM, extrude +Z by STRUT_LEN_MM,
    # then trim the lower shin rear edge (`X > SHIN_FLANGE_X_MM` below `TAPER_Z0_MM` with smooth ramp to `TAPER_Z1_MM`)!
    x_front = 0.0
    x_bend_c = r_out
    x_tip = BELL_FLANGE_X_MM
    y_w = half_flat_web
    y_out = WEB_OUT_Y_MM * 0.5
    y_in = y_out - t

    pts_outer = [
        App.Vector(x_tip, -y_out, 0.0),
        App.Vector(x_bend_c, -y_out, 0.0),
        App.Vector(x_bend_c - r_out * s45, -y_w - r_out * s45, 0.0),
        App.Vector(x_front, -y_w, 0.0),
        App.Vector(x_front, y_w, 0.0),
        App.Vector(x_bend_c - r_out * s45, y_w + r_out * s45, 0.0),
        App.Vector(x_bend_c, y_out, 0.0),
        App.Vector(x_tip, y_out, 0.0),
    ]
    pts_inner = [
        App.Vector(x_tip, y_in, 0.0),
        App.Vector(x_bend_c, y_in, 0.0),
        App.Vector(x_bend_c - r_in * s45, y_w + r_in * s45, 0.0),
        App.Vector(x_front + t, y_w, 0.0),
        App.Vector(x_front + t, -y_w, 0.0),
        App.Vector(x_bend_c - r_in * s45, -y_w - r_in * s45, 0.0),
        App.Vector(x_bend_c, -y_in, 0.0),
        App.Vector(x_tip, -y_in, 0.0),
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

    # Trim lower shin profile in X-Z (extruded across all Y) so shin is 15.5 mm deep below Z=34
    # and widens smoothly along a straight ramp to 26.5 mm at Z=45.
    cut_pts = [
        App.Vector(SHIN_FLANGE_X_MM, -WEB_OUT_Y_MM, -1.0),
        App.Vector(BELL_FLANGE_X_MM + 5.0, -WEB_OUT_Y_MM, -1.0),
        App.Vector(BELL_FLANGE_X_MM + 5.0, -WEB_OUT_Y_MM, TAPER_Z1_MM),
        App.Vector(BELL_FLANGE_X_MM, -WEB_OUT_Y_MM, TAPER_Z1_MM),
        App.Vector(SHIN_FLANGE_X_MM, -WEB_OUT_Y_MM, TAPER_Z0_MM),
    ]
    cut_wire = Part.makePolygon(cut_pts + [cut_pts[0]])
    taper_cut = Part.Face(cut_wire).extrude(App.Vector(0.0, WEB_OUT_Y_MM * 2.0, 0.0))
    solid = solid.cut(taper_cut)

    # Open upper knee clevis throat (`Z = WEB_TOP_Z_MM .. STRUT_LEN_MM`, `X < CLEVIS_RELIEF_X_MM`)
    # so `FemurIn` and `FemurOut` swing +/-80 deg with zero front-web collision!
    clevis_throat = Part.makeBox(
        CLEVIS_RELIEF_X_MM + 2.0,
        WEB_OUT_Y_MM * 2.0,
        STRUT_LEN_MM - WEB_TOP_Z_MM + 2.0,
        App.Vector(-2.0, -WEB_OUT_Y_MM, WEB_TOP_Z_MM),
    )
    solid = solid.cut(clevis_throat)

    # Cut coaxial Ankle (10.5, 7.5), Knee (10.5, 54.5), & Bellcrank (22.5, 54.5) M3 bores along Y
    for hx, hz in (
        (SHIN_PIVOT_X_MM, ANKLE_Z_MM),
        (SHIN_PIVOT_X_MM, KNEE_Z_MM),
        (BELL_PIVOT_X_MM, KNEE_Z_MM),
    ):
        cyl = Part.makeCylinder(
            PIVOT_HOLE_DIA_MM * 0.5,
            WEB_OUT_Y_MM * 2.0,
            App.Vector(hx, -WEB_OUT_Y_MM, hz),
            App.Vector(0, 1, 0),
        )
        solid = solid.cut(cyl)

    # Cut 2x centerline M3 weight-relief holes through lower closed front web (along X at Y=0)
    for wz in (18.0, 30.0):
        wcyl = Part.makeCylinder(
            WEB_HOLE_DIA_MM * 0.5,
            t * 4.0,
            App.Vector(-t, 0.0, wz),
            App.Vector(1, 0, 0),
        )
        solid = solid.cut(wcyl)

    return solid.removeSplitter()


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    half_flat_web = WEB_OUT_Y_MM * 0.5 - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)

    straight_shin = SHIN_FLANGE_X_MM - r_out
    straight_bell = BELL_FLANGE_X_MM - r_out

    half_w_shin = half_flat_web + ba + straight_shin
    half_w_bell = half_flat_web + ba + straight_bell
    bend_x = half_flat_web + ba * 0.5

    shin_piv_x = half_flat_web + ba + (SHIN_PIVOT_X_MM - r_out)
    bell_piv_x = half_flat_web + ba + (BELL_PIVOT_X_MM - r_out)

    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed symmetric polygon outer contour with open upper knee clevis throat (`Z = 39.5 .. 61.5 mm`)
    clevis_inner_x = half_flat_web + ba + (CLEVIS_RELIEF_X_MM - r_out)
    contour = [
        (-half_w_shin, 0.0),
        (half_w_shin, 0.0),
        (half_w_shin, TAPER_Z0_MM),
        (half_w_bell, TAPER_Z1_MM),
        (half_w_bell, STRUT_LEN_MM),
        (clevis_inner_x, STRUT_LEN_MM),
        (clevis_inner_x, WEB_TOP_Z_MM),
        (-clevis_inner_x, WEB_TOP_Z_MM),
        (-clevis_inner_x, STRUT_LEN_MM),
        (-half_w_bell, STRUT_LEN_MM),
        (-half_w_bell, TAPER_Z1_MM),
        (-half_w_shin, TAPER_Z0_MM),
    ]
    for i in range(len(contour)):
        x1, y1 = contour[i]
        x2, y2 = contour[(i + 1) % len(contour)]
        dxf_line(x1, y1, x2, y2)

    # 6x flange M3 pivot holes (Ankle, Knee, Bellcrank on left & right flanges) + 2x web holes
    for hz in (ANKLE_Z_MM, KNEE_Z_MM):
        dxf_circle(-shin_piv_x, hz, PIVOT_HOLE_DIA_MM * 0.5)
        dxf_circle(shin_piv_x, hz, PIVOT_HOLE_DIA_MM * 0.5)
    dxf_circle(-bell_piv_x, KNEE_Z_MM, PIVOT_HOLE_DIA_MM * 0.5)
    dxf_circle(bell_piv_x, KNEE_Z_MM, PIVOT_HOLE_DIA_MM * 0.5)

    for wz in (18.0, 30.0):
        dxf_circle(0.0, wz, WEB_HOLE_DIA_MM * 0.5)

    # Bend centerlines on layer 'BEND' along the 39.5 mm closed U-channel web
    dxf_line(-bend_x, 0.0, -bend_x, WEB_TOP_Z_MM, layer="BEND")
    dxf_line(bend_x, 0.0, bend_x, WEB_TOP_Z_MM, layer="BEND")

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
