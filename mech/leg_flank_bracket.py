#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum Reversible Flank Stance-Extension Rail (`ALU-080`, 2.032 mm / 12-gauge)
for Day 06 Walking Legs (`mech/leg_flank_bracket.py`, `qty: 2` — 1 Left + 1 Right, 100% reversible!).

Coordinate convention: mm, Z up, outboard facing -Y, under-chassis inboard facing +Y,
fore-aft laptop flank axis along +/-X (`166.0 mm` total span).

Why this bracket is essential on a real Framework Laptop 13:
  On the Framework Laptop 13 (`228.9 mm` deep, `Y_laptop = -114.45 .. +114.45 mm`), **both** physical
  Expansion Card bays on each side sit in the rear half of the chassis (`Y_laptop = +36.0 mm` Front Port
  and `Y_laptop = +76.6 mm` Rear Port, `40.6 mm` center-to-center per Framework's `FW_MAIN_PCB_GENERIC_2`
  mainboard drawing), while the entire front half (`Y_laptop = -114.45 .. +21.0 mm`) holds the battery
  and palmrest with zero ports. Mounting legs directly at the ports would leave `150 mm` of unsupported
  front laptop mass ahead of the front legs.

  Each rigid `166.0 x 57.0 x 12.5 mm` CNC-bent `ALU-080` inverted U-channel rail:
    1. Bolts across the underside of **both** rear-set `leg_bay_housing` Expansion Card modules on that
       flank (`8x M2.5` bolts across the `40.6 mm` two-bay baseline at `Y_laptop = +36.0` & `+76.6 mm`),
       locking out yaw twist.
    2. Extends **`93 mm` forward** along the laptop flank (`Y_laptop = +21.0 -> -72.0 mm`) with a
       `28.5 mm` wide inboard plate tucked flush underneath the laptop's solid CNC aluminum bottom cover
       (stopping clear of the `|X| = 112 mm` card-release buttons and the `Y = +/-103 mm` rubber foot
       strips) plus twin full-length `12.5 mm` `90 deg` downward stiffening girder flanges (`I_xx ~ 820 mm^4`).
    3. Carries the **Front `leg_servo_pod` at `Y_laptop = -45.0 mm`** (`81.0 mm` forward of the Front Port,
       reacting vertical ground loads straight into the solid aluminum bottom cover right above the front leg
       with zero port prying!) and the **Rear `leg_servo_pod` at `Y_laptop = +67.0 mm`**, giving a balanced
       `112.0 mm` quadruped wheelbase centered at `Y_laptop = +11.0 mm`.
    4. Uses a mirror-symmetric hole layout so **one single part (`qty: 2`)** fits both Left and Right
       sides of the laptop via a `180 deg` Z-rotation.

Exports:
  - mech/leg_flank_bracket.FCStd
  - mech/leg_flank_bracket.step      (colored 5052-H32 aluminum 3D bent channel)
  - mech/leg_flank_bracket.flat.dxf  (1:1 flat pattern with 2x BEND lines on layer 'BEND')
"""
import math
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named SendCutSend ALU-080 Parameters (mm, verified against `scs.py material ALU-080`) ---
SKU                = "ALU-080"
THICKNESS_MM       = 2.0320   # 0.080" (12-gauge) 5052-H32 Aluminum
BEND_RADIUS_IN_MM  = 0.9652   # 0.038" effective inside bend radius
K_FACTOR           = 0.48     # SendCutSend ALU-080 K-factor

RAIL_LEN_X_MM      = 166.00   # Spans X = -83.0 .. +83.0 mm (Y_laptop = -72.0 .. +94.0 mm when centered at +11.0 mm)
CHAN_OUT_W_Y_MM    = 57.00    # Outside channel width: Y = -28.50 (outboard) .. +28.50 (inboard under bottom cover)
CHAN_IN_W_Y_MM     = CHAN_OUT_W_Y_MM - 2.0 * THICKNESS_MM  # 52.936 mm inside channel width
FLANGE_OUT_H_Z_MM  = 12.50    # Downward 90-deg stiffening girder flanges: Z = -12.50 .. 0.00 mm (> 7.95 mm min!)

HOLE_DIA_M25_MM    = 2.70     # M2.5 clearance holes
BOLT_DX_MM         = 7.00     # +/- 7.00 mm fore-aft bolt pair spacing per module/pod
INBOARD_BOLT_Y_MM  = (8.00, 17.00)    # Matches `leg_bay_housing` floor inserts (> 7.1 mm clear of +25.50 mm bend!)
OUTBOARD_BOLT_Y_MM = (-8.00, -17.00)  # Matches `leg_servo_pod` floor inserts (> 7.1 mm clear of -25.50 mm bend!)

# Symmetric X centers so one SKU (`qty: 2`) works on both Left (`RotZ(-90)`) & Right (`RotZ(+90)`) flanks:
#   Centered at Y_laptop = +11.0 mm:
#     - Rear Bay (+76.6 mm) & Front Bay (+36.0 mm) -> X_local = +/-65.60 mm & +/-25.00 mm
#     - Rear Leg (+67.0 mm) & Front Leg (-45.0 mm) -> X_local = +/-56.00 mm
BAY_CENTERS_X_MM   = (-65.60, -25.00, 25.00, 65.60)
POD_CENTERS_X_MM   = (-56.00, 56.00)

# Center harness tie-down / weight-relief slot along neutral axis Y = 0
SLOT_LEN_X_MM      = 20.00
SLOT_W_Y_MM        = 6.00
SLOT_R_MM          = 3.00

ALU_RAIL_COLOR     = (0.66, 0.71, 0.77)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_flank_bracket.log"


def make_slot_prism(len_x: float, w_y: float, h_z: float, cx: float, cy: float, z0: float) -> Part.Shape:
    r = w_y * 0.5
    dx = max(0.01, len_x * 0.5 - r)
    c1 = Part.makeCylinder(r, h_z, App.Vector(cx - dx, cy, z0), App.Vector(0, 0, 1))
    c2 = Part.makeCylinder(r, h_z, App.Vector(cx + dx, cy, z0), App.Vector(0, 0, 1))
    box = Part.makeBox(2.0 * dx, w_y, h_z, App.Vector(cx - dx, cy - r, z0))
    return c1.fuse([c2, box]).removeSplitter()


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    s45 = math.sqrt(0.5)

    hw_out = CHAN_OUT_W_Y_MM * 0.5     # 28.50 mm
    hw_in = CHAN_IN_W_Y_MM * 0.5       # 26.468 mm
    yc = hw_out - r_out                # 25.5028 mm flat-top bend tangent
    z_top = 0.0
    z_bot = -FLANGE_OUT_H_Z_MM         # -12.50 mm
    zc = -r_out                        # -2.9972 mm bend arc Z center
    x0 = -RAIL_LEN_X_MM * 0.5

    # YZ cross-section at x0 extruded +X by RAIL_LEN_X_MM (inverted U-channel: flat top Z=0, twin flanges down to -12.5)
    p_lo0 = App.Vector(x0, -hw_out, z_bot)
    p_lo1 = App.Vector(x0, -hw_out, zc)
    p_lom = App.Vector(x0, -yc - r_out * s45, zc + r_out * s45)
    p_lo2 = App.Vector(x0, -yc, z_top)

    p_ro2 = App.Vector(x0, yc, z_top)
    p_rom = App.Vector(x0, yc + r_out * s45, zc + r_out * s45)
    p_ro1 = App.Vector(x0, hw_out, zc)
    p_ro0 = App.Vector(x0, hw_out, z_bot)

    p_ri0 = App.Vector(x0, hw_in, z_bot)
    p_ri1 = App.Vector(x0, hw_in, zc)
    p_rim = App.Vector(x0, yc + r_in * s45, zc + r_in * s45)
    p_ri2 = App.Vector(x0, yc, -t)

    p_li2 = App.Vector(x0, -yc, -t)
    p_lim = App.Vector(x0, -yc - r_in * s45, zc + r_in * s45)
    p_li1 = App.Vector(x0, -hw_in, zc)
    p_li0 = App.Vector(x0, -hw_in, z_bot)

    u_edges = [
        Part.LineSegment(p_lo0, p_lo1).toShape(),
        Part.Arc(p_lo1, p_lom, p_lo2).toShape(),
        Part.LineSegment(p_lo2, p_ro2).toShape(),
        Part.Arc(p_ro2, p_rom, p_ro1).toShape(),
        Part.LineSegment(p_ro1, p_ro0).toShape(),
        Part.LineSegment(p_ro0, p_ri0).toShape(),
        Part.LineSegment(p_ri0, p_ri1).toShape(),
        Part.Arc(p_ri1, p_rim, p_ri2).toShape(),
        Part.LineSegment(p_ri2, p_li2).toShape(),
        Part.Arc(p_li2, p_lim, p_li1).toShape(),
        Part.LineSegment(p_li1, p_li0).toShape(),
        Part.LineSegment(p_li0, p_lo0).toShape(),
    ]
    solid = Part.Face(Part.Wire(u_edges)).extrude(App.Vector(RAIL_LEN_X_MM, 0.0, 0.0))

    # Center servo-cable tie-down / relief slot
    solid = solid.cut(make_slot_prism(SLOT_LEN_X_MM, SLOT_W_Y_MM, t * 4.0, 0.0, 0.0, -t * 2.5))

    # Inboard Expansion Card bay mounting holes (16x M2.5 clearance)
    for cx in BAY_CENTERS_X_MM:
        for dx in (-BOLT_DX_MM, BOLT_DX_MM):
            for hy in INBOARD_BOLT_Y_MM:
                cyl = Part.makeCylinder(HOLE_DIA_M25_MM * 0.5, t * 4.0, App.Vector(cx + dx, hy, -t * 2.5), App.Vector(0, 0, 1))
                solid = solid.cut(cyl)

    # Outboard Dual-Servo Pod mounting holes (8x M2.5 clearance: Front Leg at -56 & Rear Leg at +56)
    for cx in POD_CENTERS_X_MM:
        for dx in (-BOLT_DX_MM, BOLT_DX_MM):
            for hy in OUTBOARD_BOLT_Y_MM:
                cyl = Part.makeCylinder(HOLE_DIA_M25_MM * 0.5, t * 4.0, App.Vector(cx + dx, hy, -t * 2.5), App.Vector(0, 0, 1))
                solid = solid.cut(cyl)

    return solid.removeSplitter()


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    flat_web_half = CHAN_OUT_W_Y_MM * 0.5 - r_out
    flat_flange = FLANGE_OUT_H_Z_MM - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)
    half_y = flat_web_half + ba + flat_flange
    bend_y = flat_web_half + 0.5 * ba
    half_x = RAIL_LEN_X_MM * 0.5
    cr = 2.0

    ents = []

    def dxf_line(x1, y1, x2, y2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{y1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{y2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed rounded-rectangle outer perimeter (`166.0 x ~73.5 mm`)
    dxf_line(-half_x + cr, -half_y, half_x - cr, -half_y)
    dxf_arc(half_x - cr, -half_y + cr, cr, 270.0, 360.0)
    dxf_line(half_x, -half_y + cr, half_x, half_y - cr)
    dxf_arc(half_x - cr, half_y - cr, cr, 0.0, 90.0)
    dxf_line(half_x - cr, half_y, -half_x + cr, half_y)
    dxf_arc(-half_x + cr, half_y - cr, cr, 90.0, 180.0)
    dxf_line(-half_x, half_y - cr, -half_x, -half_y + cr)
    dxf_arc(-half_x + cr, -half_y + cr, cr, 180.0, 270.0)

    # Center stadium slot
    sr = SLOT_W_Y_MM * 0.5
    sdx = SLOT_LEN_X_MM * 0.5 - sr
    dxf_line(-sdx, -sr, sdx, -sr)
    dxf_arc(sdx, 0.0, sr, 270.0, 90.0)
    dxf_line(sdx, sr, -sdx, sr)
    dxf_arc(-sdx, 0.0, sr, 90.0, 270.0)

    # 24x M2.5 mounting holes
    for cx in BAY_CENTERS_X_MM:
        for dx in (-BOLT_DX_MM, BOLT_DX_MM):
            for hy in INBOARD_BOLT_Y_MM:
                dxf_circle(cx + dx, hy, HOLE_DIA_M25_MM * 0.5)
    for cx in POD_CENTERS_X_MM:
        for dx in (-BOLT_DX_MM, BOLT_DX_MM):
            for hy in OUTBOARD_BOLT_Y_MM:
                dxf_circle(cx + dx, hy, HOLE_DIA_M25_MM * 0.5)

    # 2x full-length 90-deg bend lines on layer 'BEND'
    dxf_line(-half_x, -bend_y, half_x, -bend_y, layer="BEND")
    dxf_line(-half_x, bend_y, half_x, bend_y, layer="BEND")

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_flank_bracket")
    solid = build_bent_solid()
    assert solid.isValid(), "leg_flank_bracket shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegFlankBracket")
    part_obj.Shape = solid
    doc.recompute()

    write_flat_dxf(MECH_DIR / "leg_flank_bracket.flat.dxf")
    doc.saveAs(str(MECH_DIR / "leg_flank_bracket.FCStd"))

    step_path = MECH_DIR / "leg_flank_bracket.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_RAIL_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_flank_bracket valid={solid.isValid()} "
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
