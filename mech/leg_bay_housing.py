#!/usr/bin/env python3
"""
MJF PA12 Flush Expansion-Card Bay Plug (`mech/leg_bay_housing.py`, `qty: 4`)
for Day 06 Walking Legs — Rear-Set Port Anchor for `leg_flank_bracket`.

Coordinate convention: mm, Z up, laptop outboard bezel facing -Y (`Y = +0.20 mm`),
bay insertion direction (USB-C plug) facing +Y (`Y = +0.20 .. +30.00 mm`),
fore/aft laptop flank axis along +/-X (`30.0 mm` body, `32.0 mm` T-rails).

Mechanical architecture:
  1. Slides flush into the Framework Laptop 13's rear-set Expansion Card bays (`Y_laptop = +36.0 mm`
     Front Port and `Y_laptop = +76.6 mm` Rear Port), housing the 4-layer `Day_06_Walking_Legs` PCB
     and engaging the chassis T-rails and underside release latch.
  2. Beveled outboard face (`Y = +0.20 mm` at `Z = +6.80 mm` tapering to `Y = +6.80 mm` at `Z = 0.00 mm`)
     matches the Framework Laptop 13 lower chassis bevel while providing a recessed `10.0 x 3.0 mm`
     front wire-exit port (`Z = +1.40 .. +4.40 mm`) for the 4-wire servo harness running along the
     top deck of `leg_flank_bracket`.
  3. Flat underside (`Z = 0.00 mm`, `Y = +6.80 .. +30.00 mm`) bolts directly to the inboard top face
     of the SendCutSend `ALU-080` `leg_flank_bracket` (`4x M2.5` heat-set insert bosses per module at
     `X = +/-7.00 mm`, `Y = +8.00` & `+17.00 mm` -> `8x M2.5` bolts per flank across the `40.6 mm`
     two-bay baseline!).

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
BAY_Y_OUT_BOT_MM      = 6.60     # Matches laptop lower beveled skirt so rail sits flush at Z = 0
BAY_Y_IN_MM           = 30.00    # Full 30 mm bay depth

PCB_POCKET_W_X_MM     = 26.40    # Clears 26.0 mm Day 06 PCB width
PCB_POCKET_Y0_MM      = 3.40
PCB_POCKET_Y1_MM      = 28.60
PCB_BOSS_SPAN_X_MM    = 22.60    # Exact H1..H2 spacing (22.60 mm)
PCB_BOSS_Y_MM         = 11.50    # M2 boss Y position inside bay pocket

RAIL_BOLT_X_MM        = 7.00     # +/- 7.00 mm fore-aft bolt spacing matching `leg_flank_bracket`
RAIL_BOLT_Y_LIST      = (8.00, 17.00)  # Exact match to `leg_flank_bracket` INBOARD_BOLT_Y_MM

PA12_COLOR            = (0.20, 0.21, 0.23)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_bay_housing.log"


def build():
    doc = App.newDocument("leg_bay_housing")

    # 1. Beveled-front Expansion Card body matching Framework Laptop 13 side profile
    x0 = -BAY_WIDTH_X_MM * 0.5
    pts = [
        App.Vector(x0, BAY_Y_OUT_BOT_MM, 0.0),
        App.Vector(x0, BAY_Y_IN_MM, 0.0),
        App.Vector(x0, BAY_Y_IN_MM, BAY_HEIGHT_Z_MM),
        App.Vector(x0, BAY_Y_OUT_TOP_MM, BAY_HEIGHT_Z_MM),
        App.Vector(x0, BAY_Y_OUT_TOP_MM, BAY_HEIGHT_Z_MM - 1.40),
        App.Vector(x0, BAY_Y_OUT_BOT_MM, 0.0),
    ]
    bay_body = Part.Face(Part.makePolygon(pts)).extrude(App.Vector(BAY_WIDTH_X_MM, 0.0, 0.0))

    # 2. Side guide T-rails along both sides of bay cartridge: X = -16..+16, Y = 4.0..28.5, Z = 2.4..4.2
    rails = Part.makeBox(
        RAIL_SPAN_X_MM,
        24.50,
        RAIL_HEIGHT_Z_MM,
        App.Vector(-RAIL_SPAN_X_MM * 0.5, 4.00, 2.40),
    )

    solid = bay_body.fuse(rails).removeSplitter()

    # 3. Internal PCB & component cavity (Y = 3.40 .. 28.60, Z = 2.00 .. 5.80; 2.0 mm floor for M2.5 inserts)
    pcb_cavity = Part.makeBox(
        PCB_POCKET_W_X_MM,
        PCB_POCKET_Y1_MM - PCB_POCKET_Y0_MM,
        3.80,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, PCB_POCKET_Y0_MM, 2.00),
    )
    # Rear USB-C tongue opening (Y = 27.5 .. 30.5, Z = 2.0 .. 5.6)
    usbc_opening = Part.makeBox(10.50, 3.50, 3.60, App.Vector(-5.25, 27.50, 2.00))
    # Outboard (-Y) low-profile servo harness exit port (10.0 x 2.8 mm, Z = 2.10 .. 4.90)
    harness_exit = Part.makeBox(10.00, 5.00, 2.80, App.Vector(-5.00, -0.50, 2.10))
    # Underside latch catch pocket at inboard end (Y = 21.5 .. 25.0, Z = 0.0 .. 0.8, clear of Y=8,17 bolts!)
    latch_pocket = Part.makeBox(8.00, 3.50, 0.80, App.Vector(-4.00, 21.50, 0.00))

    solid = solid.cut(pcb_cavity).cut(usbc_opening).cut(harness_exit).cut(latch_pocket)

    # 4. Internal M2 PCB standoffs (X = +/-11.30 mm, Y = 11.50 mm) + 4x internal M2.5 floor bosses
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        boss = Part.makeCylinder(2.10, 0.80, App.Vector(bx, PCB_BOSS_Y_MM, 2.00), App.Vector(0, 0, 1))
        solid = solid.fuse(boss)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        pilot = Part.makeCylinder(0.85, 2.00, App.Vector(bx, PCB_BOSS_Y_MM, 1.00), App.Vector(0, 0, 1))
        solid = solid.cut(pilot)

    # 5. 4x M2.5 bottom mounting holes matching `leg_flank_bracket` (`X = +/-7.00, Y = +8.00 & +17.00`)
    for by in RAIL_BOLT_Y_LIST:
        for bx in (-RAIL_BOLT_X_MM, RAIL_BOLT_X_MM):
            # Reinforcing internal boss pad up to Z = 2.60 mm (clears 0.8 mm PCB bottom at Z = 2.80 mm!)
            pad = Part.makeCylinder(2.60, 0.60, App.Vector(bx, by, 2.00), App.Vector(0, 0, 1))
            solid = solid.fuse(pad)
    for by in RAIL_BOLT_Y_LIST:
        for bx in (-RAIL_BOLT_X_MM, RAIL_BOLT_X_MM):
            bh = Part.makeCylinder(1.05, 2.50, App.Vector(bx, by, -0.30), App.Vector(0, 0, 1))
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
