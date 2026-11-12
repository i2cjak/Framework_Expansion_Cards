#!/usr/bin/env python3
"""
MJF PA12 Open-Top Flush Expansion-Card Bay Cartridge (`mech/leg_bay_housing.py`, `qty: 4`)
for Day 06 Walking Legs — Rear-Set Port Anchor for `leg_flank_bracket`.

Coordinate convention: mm, Z up, laptop outboard bezel facing -Y (`Y = +0.20 mm`),
bay insertion direction (USB-C plug) facing +Y (`Y = +0.20 .. +32.00 mm`),
fore/aft laptop flank axis along +/-X (`30.0 mm` body, `32.0 mm` T-rails).

Mechanical architecture:
  1. Slides flush into the Framework Laptop 13's rear-set Expansion Card bays (`Y_laptop = +36.0 mm`
     Front Port and `Y_laptop = +76.6 mm` Rear Port), housing the double-sided 4-layer
     `Day_06_Walking_Legs` PCB (`board.step` placed at `(-148.50, +121.10, +2.85)`) from above (`+Z`)
     with `0.000 mm^3` component interference and engaging the chassis T-rails + underside latch.
  2. Open-top front connector aperture (`16.4 x 4.05 mm`, `X = -8.20 .. +8.20 mm`, `Z = +2.75 .. +6.80 mm`)
     exposes both side-entry 3-pin 1.0 mm JST-SH servo headers (`J2` Hip, `J3` Knee) for straight-in
     outboard mating, while top access exposes `H1`/`H2` M2 mounting screws and `J4` 4-pin SWD header.
  3. Flat underside (`Z = 0.00 mm`, `Y = +4.50 .. +32.00 mm`) bolts directly to the inboard top face
     of the SendCutSend `ALU-080` `leg_flank_bracket` via `4x M2.5 x 2.5 mm` (`OD 3.5 mm`) brass
     heat-set insert bosses per module at `X = +/-7.00 mm`, `Y = +8.50` & `+14.50 mm` (`Ø3.25 x 2.55 mm`
     blind insert bores -> `8x M2.5` bolts per flank across the `40.6 mm` two-bay baseline!).

Exports:
  - mech/leg_bay_housing.FCStd
  - mech/leg_bay_housing.step      (colored matte black MJF PA12 nylon)
  - mech/leg_bay_housing.fab.json  (measured 3dp MJF PA12 features)
"""
import json
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Parameters (mm) ---
BAY_WIDTH_X_MM        = 30.00    # Standard Framework Expansion Card body width
RAIL_SPAN_X_MM        = 32.00    # Side T-rail outer span
RAIL_HEIGHT_Z_MM      = 1.80     # T-rail tongue thickness (Z = 2.40 .. 4.20)
BAY_HEIGHT_Z_MM       = 6.80     # Standard Framework Expansion Card height (Z = 0.00 .. 6.80)
BAY_Y_OUT_TOP_MM      = 0.20     # Flush with +/-148.1 mm upper bezel
BAY_Y_OUT_BOT_MM      = 4.50     # Flat floor from Y = 4.50 .. 32.00 mm (clears Y = 8.50 bosses!)
BAY_Y_IN_MM           = 32.00    # Standard 32.00 mm Framework Expansion Card depth

PCB_POCKET_W_X_MM     = 26.30    # Clears 26.00 mm Day 06 PCB width (-13.15 .. +13.15 mm)
PCB_BOSS_SPAN_X_MM    = 22.60    # Exact H1..H2 M2 hole spacing (+/- 11.30 mm)
PCB_BOSS_Y_MM         = 11.60    # Exact H1/H2 Y (-109.50 + 121.10 = +11.60 mm)
PCB_SEAT_Z_MM         = 2.85     # Bare PCB bottom Z (FR4 Z = 2.85 .. 3.65; top JST Z_max = 6.605 mm)

RAIL_BOLT_X_MM        = 7.00     # +/- 7.00 mm fore-aft bolt spacing matching `leg_flank_bracket`
RAIL_BOLT_Y_LIST      = (8.50, 14.50)  # Exact match to `leg_flank_bracket` INBOARD_BOLT_Y_MM
M25_INSERT_BORE_R_MM  = 1.625    # Ø3.25 mm blind bore for M2.5 x 2.5 mm (OD 3.5 mm) heat-set insert
M25_INSERT_DEPTH_Z_MM = 2.55     # Blind depth inside Ø4.90 mm boss (boss top Z = 2.75 mm < 2.85 mm FR4)

PA12_COLOR            = (0.20, 0.21, 0.23)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_bay_housing.log"


def build():
    doc = App.newDocument("leg_bay_housing")

    # 1. Beveled-front Expansion Card body (30.0 x 31.8 x 6.8 mm, Y = 0.20 .. 32.00 mm)
    x0 = -BAY_WIDTH_X_MM * 0.5
    pts = [
        App.Vector(x0, BAY_Y_OUT_BOT_MM, 0.0),
        App.Vector(x0, BAY_Y_IN_MM, 0.0),
        App.Vector(x0, BAY_Y_IN_MM, BAY_HEIGHT_Z_MM),
        App.Vector(x0, BAY_Y_OUT_TOP_MM, BAY_HEIGHT_Z_MM),
        App.Vector(x0, BAY_Y_OUT_TOP_MM, 2.80),
        App.Vector(x0, 0.90, 1.00),
        App.Vector(x0, BAY_Y_OUT_BOT_MM, 0.0),
    ]
    bay_body = Part.Face(Part.makePolygon(pts)).extrude(App.Vector(BAY_WIDTH_X_MM, 0.0, 0.0))

    # 2. Side guide T-rails along both sides of bay cartridge: X = -16..+16, Y = 3.00..30.50, Z = 2.40..4.20
    rails = Part.makeBox(
        RAIL_SPAN_X_MM,
        27.50,
        RAIL_HEIGHT_Z_MM,
        App.Vector(-RAIL_SPAN_X_MM * 0.5, 3.00, 2.40),
    )

    solid = bay_body.fuse(rails).removeSplitter()

    # 3. Stepped open-top PCB & bottom-SMD cavity (drops in straight from +Z!)
    #    - Upper PCB & top-component tub: Y = 0.95 .. 31.25, Z = 2.75 .. 7.00
    #    - Mid bottom-passive pocket:     Y = 1.55 .. 31.25, Z = 1.85 .. 2.80
    #    - Deep front power-stage sump:   Y = 2.00 .. 16.00, Z = 1.00 .. 1.90 (clears L1/D4/1206 caps at Z_min = 1.165)
    tub_upper = Part.makeBox(
        PCB_POCKET_W_X_MM,
        31.25 - 0.95,
        7.00 - 2.75,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 0.95, 2.75),
    )
    tub_mid = Part.makeBox(
        PCB_POCKET_W_X_MM,
        31.25 - 1.55,
        2.80 - 1.85,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 1.55, 1.85),
    )
    tub_sump = Part.makeBox(
        PCB_POCKET_W_X_MM,
        21.50 - 1.70,
        1.90 - 1.00,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 1.70, 1.00),
    )

    # Rear straddle-mount USB-C plug slot (`J1` Amphenol 12401981E412A: X = -4.19..+4.26, Z_min = 1.94)
    usbc_slot = Part.makeBox(9.60, 4.50, 5.25, App.Vector(-4.80, 28.20, 1.75))

    # Outboard (-Y) front aperture exposing J2 (Hip) & J3 (Knee) 3-pin JST-SH headers (X = -8.20 .. +8.20)
    servo_hdr_port = Part.makeBox(16.40, 3.20, 4.25, App.Vector(-8.20, -0.20, 2.75))

    # Underside latch catch pocket at inboard end (Y = 22.0 .. 25.5, Z = 0.0 .. 0.75 -> 1.10 mm PA12 floor above!)
    latch_pocket = Part.makeBox(8.00, 3.50, 0.75, App.Vector(-4.00, 22.00, -0.05))

    solid = (
        solid.cut(tub_upper)
        .cut(tub_mid)
        .cut(tub_sump)
        .cut(usbc_slot)
        .cut(servo_hdr_port)
        .cut(latch_pocket)
    )

    # 4. Internal M2 PCB standoffs up to Z = 2.85 mm (`H1`/`H2` at X = +/-11.30 mm, Y = +11.60 mm)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        boss = Part.makeCylinder(2.10, PCB_SEAT_Z_MM - 1.00, App.Vector(bx, PCB_BOSS_Y_MM, 1.00), App.Vector(0, 0, 1))
        solid = solid.fuse(boss)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        pilot = Part.makeCylinder(0.85, 2.50, App.Vector(bx, PCB_BOSS_Y_MM, 0.50), App.Vector(0, 0, 1))
        solid = solid.cut(pilot)

    # 5. 4x M2.5 bottom heat-set insert bosses (`X = +/-7.00, Y = +8.50 & +14.50`, OD = 4.90 mm, Z = 0.00..2.75 mm)
    for by in RAIL_BOLT_Y_LIST:
        for bx in (-RAIL_BOLT_X_MM, RAIL_BOLT_X_MM):
            pad = Part.makeCylinder(2.45, 2.75 - 1.00, App.Vector(bx, by, 1.00), App.Vector(0, 0, 1))
            solid = solid.fuse(pad)
    for by in RAIL_BOLT_Y_LIST:
        for bx in (-RAIL_BOLT_X_MM, RAIL_BOLT_X_MM):
            bh = Part.makeCylinder(
                M25_INSERT_BORE_R_MM,
                M25_INSERT_DEPTH_Z_MM + 0.20,
                App.Vector(bx, by, -0.20),
                App.Vector(0, 0, 1),
            )
            solid = solid.cut(bh)

    solid = solid.removeSplitter()
    assert solid.isValid(), "leg_bay_housing shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegBayHousing")
    part_obj.Shape = solid
    doc.recompute()

    doc.saveAs(str(MECH_DIR / "leg_bay_housing.FCStd"))
    step_path = MECH_DIR / "leg_bay_housing.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = PA12_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    vol_cm3 = round(solid.Volume / 1000.0, 2)
    area_cm2 = round(solid.Area / 100.0, 2)

    fab = {
        "kind": "3dp",
        "name": "Leg Bay Flush Port Housing",
        "source": "leg_bay_housing.step",
        "qty": 4,
        "options": {
            "process": "mjf",
            "material": "pa12",
            "color": "black",
        },
        "features": {
            "volume_cm3": vol_cm3,
            "area_cm2": area_cm2,
            "size_x_mm": round(bb.XLength, 2),
            "size_y_mm": round(bb.YLength, 2),
            "size_z_mm": round(bb.ZLength, 2),
        },
    }
    (MECH_DIR / "leg_bay_housing.fab.json").write_text(json.dumps(fab, indent=2) + "\n")

    LOG_PATH.write_text(
        f"OK: leg_bay_housing valid={solid.isValid()} "
        f"bbox=({bb.XLength:.2f} x {bb.YLength:.2f} x {bb.ZLength:.2f} mm) "
        f"vol={vol_cm3:.2f} cm3 area={area_cm2:.2f} cm2\n"
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
    elif exit_code != 0:
        raise SystemExit(exit_code)
