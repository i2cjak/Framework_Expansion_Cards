#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum CNC-Bent Outboard Double-Shear Bearing Outrigger (`ALU-063`, 1.600 mm thick)
for Day 06 Walking Legs (`mech/leg_hip_bracket.py`).

Coordinate convention: mm, Z up, front/outboard facing -Y.
Forms a rigid 90-deg overhead outrigger bridge:
  - Horizontal top flange (`28.0 x 30.0 mm`) bolts to the top roof canopy of `leg_bay_housing`
    with 4x M2.5 fasteners (`X = +/-9.0 mm`, `Y_local = +19.6` & `+27.6 mm` from outboard face),
    leaving `-X` (forward), `+X` (aft), and `-Z` (downward) 100% unobstructed for +/-75 deg sagittal swing.
  - Vertical outboard bearing plate (`28.0 x 19.6 mm` at `Y_local = 0..+1.6 mm`) supports **both**
    outboard M3 servo axle pins (`X = -7.30 mm` Hip & `X = +7.30 mm` Knee, `12.60 mm` below top face)
    in double shear so radial/ground loads bypass the servo output splines into the chassis shelf.

Exports:
  - mech/leg_hip_bracket.FCStd
  - mech/leg_hip_bracket.step     (3D bent solid with R=0.889mm inside bend radius)
  - mech/leg_hip_bracket.flat.dxf (1:1 flat pattern with BEND line on layer 'BEND')
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
WIDTH_X_MM         = 28.0     # Bracket width along X (> 9.53 mm min flat width)
FLANGE_Y_OUT_MM    = 30.0     # Top mounting flange outside length along +Y (Y_local = 0 .. +30.0)
FLANGE_Z_OUT_MM    = 19.6     # Vertical outboard bearing plate outside height (Z_local = -19.6 .. 0.0)

MOUNT_HOLE_DIA_MM  = 2.70     # 4x M2.5 clearance holes on top flange
MOUNT_HOLE_X_MM    = 9.00     # +/- 9.00 mm in X
MOUNT_HOLE_Y1_MM   = 19.60    # Matches bay housing Y = -26.0 when outboard face is at Y = -45.6
MOUNT_HOLE_Y2_MM   = 27.60    # Matches bay housing Y = -18.0 when outboard face is at Y = -45.6

AXLE_HOLE_DIA_MM   = 3.10     # 2x M3 coaxial outboard bearing support holes
AXLE_HOLE_X_MM     = 7.30     # Matches +/- 7.30 mm Hip & Knee servo axes
AXLE_HOLE_Z_MM     = -12.60   # Matches Z_assy = +9.50 when top outer face Z_local=0 is at Z_assy = +22.10

ALU_COLOR          = (0.72, 0.75, 0.79)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_hip_bracket.log"


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    s45 = math.sqrt(0.5)

    # Build YZ profile at X = -WIDTH_X_MM/2 and extrude +X by WIDTH_X_MM.
    # Outer corner at (Y=0, Z=0):
    #   - Top flange outer face: Z = 0, Y = r_out .. FLANGE_Y_OUT_MM
    #   - Vertical bearing flange outer face: Y = 0, Z = -FLANGE_Z_OUT_MM .. -r_out
    #   - 90-deg bend center at (Y = r_out, Z = -r_out)
    x0 = -WIDTH_X_MM * 0.5
    yc = r_out
    zc = -r_out

    p_o0 = App.Vector(x0, 0.0, -FLANGE_Z_OUT_MM)
    p_o1 = App.Vector(x0, 0.0, zc)
    p_om = App.Vector(x0, yc - r_out * s45, zc + r_out * s45)
    p_o2 = App.Vector(x0, yc, 0.0)
    p_o3 = App.Vector(x0, FLANGE_Y_OUT_MM, 0.0)

    p_i3 = App.Vector(x0, FLANGE_Y_OUT_MM, -t)
    p_i2 = App.Vector(x0, yc, -t)
    p_im = App.Vector(x0, yc - r_in * s45, zc + r_in * s45)
    p_i1 = App.Vector(x0, t, zc)
    p_i0 = App.Vector(x0, t, -FLANGE_Z_OUT_MM)

    edges = [
        Part.LineSegment(p_o0, p_o1).toShape(),
        Part.Arc(p_o1, p_om, p_o2).toShape(),
        Part.LineSegment(p_o2, p_o3).toShape(),
        Part.LineSegment(p_o3, p_i3).toShape(),
        Part.LineSegment(p_i3, p_i2).toShape(),
        Part.Arc(p_i2, p_im, p_i1).toShape(),
        Part.LineSegment(p_i1, p_i0).toShape(),
        Part.LineSegment(p_i0, p_o0).toShape(),
    ]
    face = Part.Face(Part.Wire(edges))
    solid = face.extrude(App.Vector(WIDTH_X_MM, 0.0, 0.0))

    # Cut 4x M2.5 mounting holes through top flange (along Z)
    for hx in (-MOUNT_HOLE_X_MM, MOUNT_HOLE_X_MM):
        for hy in (MOUNT_HOLE_Y1_MM, MOUNT_HOLE_Y2_MM):
            cyl = Part.makeCylinder(
                MOUNT_HOLE_DIA_MM * 0.5,
                t * 4.0,
                App.Vector(hx, hy, -t * 2.5),
                App.Vector(0, 0, 1),
            )
            solid = solid.cut(cyl)

    # Cut 2x M3 coaxial outboard support bearing holes through vertical flange (along Y)
    for ax in (-AXLE_HOLE_X_MM, AXLE_HOLE_X_MM):
        acyl = Part.makeCylinder(
            AXLE_HOLE_DIA_MM * 0.5,
            t * 4.0,
            App.Vector(ax, -t, AXLE_HOLE_Z_MM),
            App.Vector(0, 1, 0),
        )
        solid = solid.cut(acyl)

    return solid


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    flat_z = FLANGE_Z_OUT_MM - r_out
    flat_y = FLANGE_Y_OUT_MM - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)

    # Unfold along Y_flat: 0 .. total_len, X_flat: -WIDTH_X_MM/2 .. +WIDTH_X_MM/2
    # Segment 1 (vertical bearing flange): Y_flat = 0 .. flat_z (tip at 0)
    # Bend zone: Y_flat = flat_z .. flat_z + ba (centerline at flat_z + 0.5*ba)
    # Segment 2 (top roof flange): Y_flat = flat_z + ba .. total_len (tip at total_len)
    total_len = flat_z + ba + flat_y
    bend_y = flat_z + 0.5 * ba
    half_x = WIDTH_X_MM * 0.5
    cr = 1.5

    axle_tip_dist = FLANGE_Z_OUT_MM + AXLE_HOLE_Z_MM  # 19.6 - 12.6 = 7.0 mm from vertical tip
    y1_tip_dist = FLANGE_Y_OUT_MM - MOUNT_HOLE_Y1_MM  # 30.0 - 19.6 = 10.4 mm from roof tip
    y2_tip_dist = FLANGE_Y_OUT_MM - MOUNT_HOLE_Y2_MM  # 30.0 - 27.6 = 2.4 mm from roof tip

    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed rounded rectangle (-half_x..+half_x, 0..total_len)
    dxf_line(-half_x + cr, 0.0, half_x - cr, 0.0)
    dxf_arc(half_x - cr, cr, cr, -90.0, 0.0)
    dxf_line(half_x, cr, half_x, total_len - cr)
    dxf_arc(half_x - cr, total_len - cr, cr, 0.0, 90.0)
    dxf_line(half_x - cr, total_len, -half_x + cr, total_len)
    dxf_arc(-half_x + cr, total_len - cr, cr, 90.0, 180.0)
    dxf_line(-half_x, total_len - cr, -half_x, cr)
    dxf_arc(-half_x + cr, cr, cr, 180.0, 270.0)

    # 2x M3 axle support holes
    for ax in (-AXLE_HOLE_X_MM, AXLE_HOLE_X_MM):
        dxf_circle(ax, axle_tip_dist, AXLE_HOLE_DIA_MM * 0.5)

    # 4x M2.5 roof mounting holes
    for hx in (-MOUNT_HOLE_X_MM, MOUNT_HOLE_X_MM):
        dxf_circle(hx, total_len - y1_tip_dist, MOUNT_HOLE_DIA_MM * 0.5)
        dxf_circle(hx, total_len - y2_tip_dist, MOUNT_HOLE_DIA_MM * 0.5)

    # Single 90-deg bend centerline on layer 'BEND'
    dxf_line(-half_x, bend_y, half_x, bend_y, layer="BEND")

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
