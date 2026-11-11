#!/usr/bin/env python3
"""
SendCutSend 5052-H32 Aluminum Rigid Bay-Extension Cantilever Outrigger (`ALU-080`, 2.032 mm / 12-gauge)
for Day 06 Walking Legs (`mech/leg_bay_outrigger.py`).

Coordinate convention: mm, Z up, front/outboard facing -Y, bay insertion + under-chassis anchor facing +Y.

Why this part exists:
  Framework Expansion Card modules sit recessed inside a 6.8 mm underside pocket beneath the laptop's
  overhanging beveled palmrest rim. Rather than cantilevering the leg on printed plastic or colliding
  with the upper chassis lip, this 12-gauge (2.032 mm) CNC-bent 5052-H32 aluminum U-channel spine:
    1. Extends a 26.0 mm wide inboard anchor plate (`Y = -11.5 .. +44.0 mm`, `Z = -2.032 .. 0.0 mm`)
       bolted across the underside of the locked-in bay cartridge (`4x M2.5`) AND bearing 17.5 mm directly
       underneath the laptop's solid CNC aluminum bottom cover (`Y = +26.5 .. +44.0 mm`, with a `10 x 7 mm`
       access window clearing the chassis expansion-card release button) to react vertical walking loads
       and roll bending moments straight into the chassis floor.
    2. Projects 11.5 mm horizontally outboard past the recessed module face and overhanging chassis rim
       while staying strictly below `Z = 0.0 mm` (zero upper-lip interference), then widens into a
       `35.66 mm` outside (`31.60 mm` inside) U-channel cradle (`Y = -37.5 .. -11.5 mm`) with twin `90 deg`
       vertical side walls (`18.50 mm` outside height) that carry the dual horizontal outboard-facing
       MG90S servo saddle completely clear of the laptop body.

Exports:
  - mech/leg_bay_outrigger.FCStd
  - mech/leg_bay_outrigger.step      (3D bent solid with R=0.9652mm inside bend radius)
  - mech/leg_bay_outrigger.flat.dxf  (1:1 flat pattern with 2x BEND lines on layer 'BEND')
"""
import math
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named SendCutSend ALU-080 Parameters (mm, from live `scs.py material ALU-080`) ---
SKU                = "ALU-080"
THICKNESS_MM       = 2.0320   # 0.080" (12-gauge) 5052-H32 Aluminum
BEND_RADIUS_IN_MM  = 0.9652   # 0.038" effective inside bend radius
K_FACTOR           = 0.48     # SendCutSend ALU-080 K-factor

CRADLE_IN_W_X_MM   = 31.60    # Inside width between vertical cradle walls (matches servo saddle)
CRADLE_OUT_W_X_MM  = CRADLE_IN_W_X_MM + 2.0 * THICKNESS_MM  # 35.664 mm outside width
WALL_OUT_H_Z_MM    = 18.50    # Outside vertical wall height (Z = -2.032 .. +16.468 mm, > 7.95 mm min flange)
CRADLE_Y0_MM       = -37.50   # Outboard front edge of metal servo cradle
CRADLE_Y1_MM       = -11.50   # Inboard end of upward walls (4.0 mm clear of laptop beveled rim!)

TONGUE_W_X_MM      = 26.00    # Inboard under-chassis tongue width (inside bend tangent = 29.67 mm!)
TONGUE_Y_END_MM    = 44.00    # Extends 17.5 mm past bay pocket (26.5..44.0) under solid laptop bottom floor!

LATCH_WIN_W_X_MM   = 10.00    # Clearance window over Framework underside card-release button
LATCH_WIN_L_Y_MM   = 7.00     # Y = +29.5 .. +36.5 mm
LATCH_WIN_Y0_MM    = 29.50
LATCH_WIN_R_MM     = 2.00

HOLE_DIA_M25_MM    = 2.70     # M2.5 clearance holes
MOUNT_X_MM         = 7.00     # +/- 7.00 mm (> 8.0 mm clear of bend lines!)
BAY_BOLT_Y_MM      = (5.00, 19.00)     # 4x M2.5 bolts into locked-in PA12 bay cartridge
CRADLE_BOLT_Y_MM   = (-31.00, -18.00)  # 4x M2.5 bolts into 3.2mm solid PA12 servo saddle floor

ALU_SPINE_COLOR    = (0.66, 0.70, 0.76)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_bay_outrigger.log"


def make_rounded_rect_prism(w: float, d: float, h: float, r: float, x0: float, y0: float, z0: float) -> Part.Shape:
    x1, y1 = x0 + w, y0 + d
    s45 = math.sqrt(0.5)
    edges = [
        Part.LineSegment(App.Vector(x0 + r, y0, z0), App.Vector(x1 - r, y0, z0)).toShape(),
        Part.Arc(App.Vector(x1 - r, y0, z0), App.Vector(x1 - r + r * s45, y0 + r - r * s45, z0), App.Vector(x1, y0 + r, z0)).toShape(),
        Part.LineSegment(App.Vector(x1, y0 + r, z0), App.Vector(x1, y1 - r, z0)).toShape(),
        Part.Arc(App.Vector(x1, y1 - r, z0), App.Vector(x1 - r + r * s45, y1 - r + r * s45, z0), App.Vector(x1 - r, y1, z0)).toShape(),
        Part.LineSegment(App.Vector(x1 - r, y1, z0), App.Vector(x0 + r, y1, z0)).toShape(),
        Part.Arc(App.Vector(x0 + r, y1, z0), App.Vector(x0 + r - r * s45, y1 - r + r * s45, z0), App.Vector(x0, y1 - r, z0)).toShape(),
        Part.LineSegment(App.Vector(x0, y1 - r, z0), App.Vector(x0, y0 + r, z0)).toShape(),
        Part.Arc(App.Vector(x0, y0 + r, z0), App.Vector(x0 + r - r * s45, y0 + r - r * s45, z0), App.Vector(x0 + r, y0, z0)).toShape(),
    ]
    return Part.Face(Part.Wire(edges)).extrude(App.Vector(0, 0, h))


def build_bent_solid() -> Part.Shape:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    s45 = math.sqrt(0.5)
    hw_out = CRADLE_OUT_W_X_MM * 0.5
    hw_in = CRADLE_IN_W_X_MM * 0.5
    xc = hw_out - r_out
    z_bot = -t
    z_top = z_bot + WALL_OUT_H_Z_MM
    zc = z_bot + r_out

    p_lo0 = App.Vector(-hw_out, CRADLE_Y0_MM, z_top)
    p_lo1 = App.Vector(-hw_out, CRADLE_Y0_MM, zc)
    p_lom = App.Vector(-xc - r_out * s45, CRADLE_Y0_MM, zc - r_out * s45)
    p_lo2 = App.Vector(-xc, CRADLE_Y0_MM, z_bot)

    p_ro2 = App.Vector(xc, CRADLE_Y0_MM, z_bot)
    p_rom = App.Vector(xc + r_out * s45, CRADLE_Y0_MM, zc - r_out * s45)
    p_ro1 = App.Vector(hw_out, CRADLE_Y0_MM, zc)
    p_ro0 = App.Vector(hw_out, CRADLE_Y0_MM, z_top)

    p_ri0 = App.Vector(hw_in, CRADLE_Y0_MM, z_top)
    p_ri1 = App.Vector(hw_in, CRADLE_Y0_MM, zc)
    p_rim = App.Vector(xc + r_in * s45, CRADLE_Y0_MM, zc - r_in * s45)
    p_ri2 = App.Vector(xc, CRADLE_Y0_MM, 0.0)

    p_li2 = App.Vector(-xc, CRADLE_Y0_MM, 0.0)
    p_lim = App.Vector(-xc - r_in * s45, CRADLE_Y0_MM, zc - r_in * s45)
    p_li1 = App.Vector(-hw_in, CRADLE_Y0_MM, zc)
    p_li0 = App.Vector(-hw_in, CRADLE_Y0_MM, z_top)

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
    cradle_len = CRADLE_Y1_MM - CRADLE_Y0_MM
    cradle = Part.Face(Part.Wire(u_edges)).extrude(App.Vector(0.0, cradle_len, 0.0))

    tongue_len = TONGUE_Y_END_MM - CRADLE_Y1_MM
    tongue = Part.makeBox(
        TONGUE_W_X_MM,
        tongue_len + 1.0,
        t,
        App.Vector(-TONGUE_W_X_MM * 0.5, CRADLE_Y1_MM - 1.0, -t),
    )
    solid = cradle.fuse(tongue).removeSplitter()

    # Release-button access cutout in inboard under-chassis plate
    latch_cut = make_rounded_rect_prism(
        LATCH_WIN_W_X_MM,
        LATCH_WIN_L_Y_MM,
        t * 4.0,
        LATCH_WIN_R_MM,
        -LATCH_WIN_W_X_MM * 0.5,
        LATCH_WIN_Y0_MM,
        -t * 2.5,
    )
    solid = solid.cut(latch_cut)

    # 8x M2.5 floor mounting holes (4 in bay + 4 in outboard cradle)
    for hy in BAY_BOLT_Y_MM + CRADLE_BOLT_Y_MM:
        for hx in (-MOUNT_X_MM, MOUNT_X_MM):
            cyl = Part.makeCylinder(
                HOLE_DIA_M25_MM * 0.5,
                t * 4.0,
                App.Vector(hx, hy, -t * 2.5),
                App.Vector(0, 0, 1),
            )
            solid = solid.cut(cyl)

    return solid.removeSplitter()


def write_flat_dxf(path: Path) -> None:
    t = THICKNESS_MM
    r_in = BEND_RADIUS_IN_MM
    r_out = r_in + t
    flat_floor_half = CRADLE_OUT_W_X_MM * 0.5 - r_out
    flat_wall = WALL_OUT_H_Z_MM - r_out
    ba = (math.pi * 0.5) * (r_in + K_FACTOR * t)
    wing_half = flat_floor_half + ba + flat_wall
    bend_x = flat_floor_half + 0.5 * ba
    tongue_half = TONGUE_W_X_MM * 0.5

    y0 = CRADLE_Y0_MM
    y1 = CRADLE_Y1_MM
    y2 = TONGUE_Y_END_MM
    cr = 1.5
    ir = 0.8

    ents = []

    def dxf_line(x1, yy1, x2, yy2, layer="0"):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{x1:.6f}\n20\n{yy1:.6f}\n30\n0.0\n11\n{x2:.6f}\n21\n{yy2:.6f}\n31\n0.0\n")

    def dxf_arc(cx, cy, r, a0, a1, layer="0"):
        ents.append(f"0\nARC\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n50\n{a0:.6f}\n51\n{a1:.6f}\n")

    def dxf_circle(cx, cy, r, layer="0"):
        ents.append(f"0\nCIRCLE\n8\n{layer}\n10\n{cx:.6f}\n20\n{cy:.6f}\n30\n0.0\n40\n{r:.6f}\n")

    # Closed T-shaped outline
    dxf_line(-wing_half + cr, y0, wing_half - cr, y0)
    dxf_arc(wing_half - cr, y0 + cr, cr, 270.0, 360.0)
    dxf_line(wing_half, y0 + cr, wing_half, y1 - cr)
    dxf_arc(wing_half - cr, y1 - cr, cr, 0.0, 90.0)
    dxf_line(wing_half - cr, y1, tongue_half + ir, y1)
    dxf_arc(tongue_half + ir, y1 + ir, ir, 180.0, 270.0)
    dxf_line(tongue_half, y1 + ir, tongue_half, y2 - cr)
    dxf_arc(tongue_half - cr, y2 - cr, cr, 0.0, 90.0)
    dxf_line(tongue_half - cr, y2, -tongue_half + cr, y2)
    dxf_arc(-tongue_half + cr, y2 - cr, cr, 90.0, 180.0)
    dxf_line(-tongue_half, y2 - cr, -tongue_half, y1 + ir)
    dxf_arc(-tongue_half - ir, y1 + ir, ir, 270.0, 360.0)
    dxf_line(-tongue_half - ir, y1, -wing_half + cr, y1)
    dxf_arc(-wing_half + cr, y1 - cr, cr, 90.0, 180.0)
    dxf_line(-wing_half, y1 - cr, -wing_half, y0 + cr)
    dxf_arc(-wing_half + cr, y0 + cr, cr, 180.0, 270.0)

    # Release-button rounded-rectangle cutout
    lx0, lx1 = -LATCH_WIN_W_X_MM * 0.5, LATCH_WIN_W_X_MM * 0.5
    ly0, ly1 = LATCH_WIN_Y0_MM, LATCH_WIN_Y0_MM + LATCH_WIN_L_Y_MM
    lr = LATCH_WIN_R_MM
    dxf_line(lx0 + lr, ly0, lx1 - lr, ly0)
    dxf_arc(lx1 - lr, ly0 + lr, lr, 270.0, 360.0)
    dxf_line(lx1, ly0 + lr, lx1, ly1 - lr)
    dxf_arc(lx1 - lr, ly1 - lr, lr, 0.0, 90.0)
    dxf_line(lx1 - lr, ly1, lx0 + lr, ly1)
    dxf_arc(lx0 + lr, ly1 - lr, lr, 90.0, 180.0)
    dxf_line(lx0, ly1 - lr, lx0, ly0 + lr)
    dxf_arc(lx0 + lr, ly0 + lr, lr, 180.0, 270.0)

    # 8x M2.5 floor mounting holes
    for hy in BAY_BOLT_Y_MM + CRADLE_BOLT_Y_MM:
        for hx in (-MOUNT_X_MM, MOUNT_X_MM):
            dxf_circle(hx, hy, HOLE_DIA_M25_MM * 0.5)

    # 2x 90-deg bend centerlines on layer 'BEND'
    dxf_line(-bend_x, y0, -bend_x, y1, layer="BEND")
    dxf_line(bend_x, y0, bend_x, y1, layer="BEND")

    dxf = (
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        + "".join(ents)
        + "0\nENDSEC\n0\nEOF\n"
    )
    path.write_text(dxf)


def build():
    doc = App.newDocument("leg_bay_outrigger")
    solid = build_bent_solid()
    assert solid.isValid(), "leg_bay_outrigger shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegBayOutrigger")
    part_obj.Shape = solid
    doc.recompute()

    write_flat_dxf(MECH_DIR / "leg_bay_outrigger.flat.dxf")
    doc.saveAs(str(MECH_DIR / "leg_bay_outrigger.FCStd"))

    step_path = MECH_DIR / "leg_bay_outrigger.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = ALU_SPINE_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    LOG_PATH.write_text(
        f"OK: leg_bay_outrigger valid={solid.isValid()} "
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
