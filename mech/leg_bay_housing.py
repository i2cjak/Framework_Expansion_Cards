#!/usr/bin/env python3
"""
MJF PA12 Recessed Expansion-Bay Cartridge & Outboard Dual-Servo Saddle (`mech/leg_bay_housing.py`)
for Day 06 Walking Legs (mounted on the rigid SendCutSend `ALU-080` `leg_bay_outrigger` spine).

Coordinate convention: mm, Z up, front (outboard leg turret & servo splines) facing -Y,
bay insertion direction (USB-C plug) facing +Y, fore/aft walking axis along +/-X.

Mechanical architecture:
  1. Low-profile bay cartridge (`30.0 x 26.5 x 6.8 mm`) slides into the recessed Framework Laptop
     underside expansion bay T-rails (`32.0 mm` span) and engages the latch pocket.
  2. Low-profile wiring neck (`24.0 x 12.0 x 6.4 mm`, `Y = -12.0 .. 0.0 mm`, `Z = 0.0 .. +6.4 mm`)
     exits cleanly underneath the laptop's overhanging beveled upper palmrest rim (`Z >= +6.8 mm` out
     to `Y = -7.5 mm`) with zero chassis interference while enclosing the 6-wire servo harness.
  3. Outboard dual-servo saddle (`31.2 x 25.5 x 31.3 mm`, `Y = -37.5 .. -12.0 mm`, `Z = 0.0 .. +31.3 mm`)
     seats snugly inside the 12-gauge `5052-H32` aluminum `leg_bay_outrigger` U-channel cradle (`8x M2.5`
     floor bolts at `Y = +19, +5, -18, -31 mm`), housing two horizontal MG90S servos facing outboard (`-Y`).

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
BAY_WIDTH_X_MM        = 30.0     # Standard Framework Expansion Card body width
RAIL_SPAN_X_MM        = 32.0     # Side T-rail outer span
RAIL_HEIGHT_Z_MM      = 1.8      # T-rail tongue thickness
BAY_HEIGHT_Z_MM       = 6.8      # Standard Framework Expansion Card height
BAY_INSERT_LEN_Y_MM   = 26.5     # Inserted bay body length along +Y (Y = 0 .. +26.5)

NECK_WIDTH_X_MM       = 25.6     # Low-profile wiring neck clearing beveled chassis lip
NECK_Y0_MM            = -12.0    # Outboard end of neck where turret rises clear of laptop overhang
NECK_HEIGHT_Z_MM      = 6.4      # Strictly < 6.8 mm bay height so it clears overhanging upper rim!

PCB_POCKET_W_X_MM     = 26.4     # Clears 26.0 mm PCB width
PCB_POCKET_L_Y_MM     = 24.0     # Internal PCB pocket (Y = 2.0 .. 26.0)
PCB_BOSS_SPAN_X_MM    = 22.60    # Exact H1..H2 spacing (22.60 mm)
PCB_BOSS_Y_MM         = 9.0      # Boss Y position inside bay pocket

TURRET_WIDTH_X_MM     = 31.2     # Fits inside 31.60 mm `leg_bay_outrigger` U-channel with 0.2 mm/side clearance
TURRET_FRONT_Y_MM     = -37.5    # Shifted 16.0 mm further outboard via rigid metal outrigger!
TURRET_BACK_Y_MM      = -12.0    # 4.5 mm outboard of laptop's outermost beveled edge (-7.0 mm)
TURRET_BOT_Z_MM       = 0.0      # Seats flat on `leg_bay_outrigger` Z = 0.0 mm 12-gauge aluminum floor
TURRET_TOP_Z_MM       = 33.10    # Places `leg_hip_bracket` axle holes coaxially at Z = +22.10 mm!

SERVO_CENTER_X_MM     = 7.30     # Symmetric left/right servo centerlines (+/- 7.30 mm)
SERVO_POCKET_W_X_MM   = 12.6     # Clears 12.2 mm MG90S servo thickness
SERVO_POCKET_H_Z_MM   = 23.2     # Clears 22.8 mm MG90S servo case length (Z = +5.00 .. +28.20)
SERVO_POCKET_Z0_MM    = 5.00     # Leaves solid 5.00 mm floor under servos & mounting ears; shaft at Z = +22.10 mm!
SERVO_POCKET_DEPTH_Y  = 22.8     # Pocket from Y = -37.5 (outboard face) to Y = -14.7

CANOPY_OUT_Y_MM       = -46.5    # Roof overhang supporting `leg_hip_bracket` top flange
CANOPY_IN_Y_MM        = -14.0
CANOPY_WIDTH_X_MM     = 28.0     # Matches `leg_hip_bracket` 28.0 mm width
CANOPY_THICK_Z_MM     = 3.8      # Stiff upper bridge floor (Z = 27.50 .. 31.30)
BRACKET_BOLT_X_MM     = 9.0      # +/- 9.0 mm M2.5 mounting pattern
BRACKET_BOLT_Y1_MM    = -42.0    # Matches `leg_hip_bracket` Y_local = +19.6 when placed at Y = -61.6
BRACKET_BOLT_Y2_MM    = -34.0    # Matches `leg_hip_bracket` Y_local = +27.6 when placed at Y = -61.6

SPINE_BOLT_X_MM       = 7.00     # Matches `leg_bay_outrigger` M2.5 floor hole pattern
SPINE_BOLT_Y_LIST     = (5.00, 19.00, -18.00, -31.00)

PA12_COLOR            = (0.20, 0.21, 0.23)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_bay_housing.log"


def build():
    doc = App.newDocument("leg_bay_housing")

    # 1. Bay slide-in cartridge body: X = -15..+15, Y = 0..+26.5, Z = 0..+6.8
    bay_body = Part.makeBox(
        BAY_WIDTH_X_MM,
        BAY_INSERT_LEN_Y_MM,
        BAY_HEIGHT_Z_MM,
        App.Vector(-BAY_WIDTH_X_MM * 0.5, 0.0, 0.0),
    )

    # Side guide T-rails along both sides of bay cartridge: X = -16..+16, Y = 1.5..25.5, Z = 2.4..4.2
    rails = Part.makeBox(
        RAIL_SPAN_X_MM,
        BAY_INSERT_LEN_Y_MM - 2.5,
        RAIL_HEIGHT_Z_MM,
        App.Vector(-RAIL_SPAN_X_MM * 0.5, 1.5, 2.4),
    )

    # 2. Low-profile recessed-lip wiring neck (Y = -12.0 .. +1.0, Z = 0.0 .. +6.4)
    neck = Part.makeBox(
        NECK_WIDTH_X_MM,
        1.0 - NECK_Y0_MM,
        NECK_HEIGHT_Z_MM,
        App.Vector(-NECK_WIDTH_X_MM * 0.5, NECK_Y0_MM, 0.0),
    )

    # 3. Outboard (-Y) dual-servo saddle seated inside `leg_bay_outrigger` metal U-cradle
    turret = Part.makeBox(
        TURRET_WIDTH_X_MM,
        TURRET_BACK_Y_MM - TURRET_FRONT_Y_MM,
        TURRET_TOP_Z_MM - TURRET_BOT_Z_MM,
        App.Vector(-TURRET_WIDTH_X_MM * 0.5, TURRET_FRONT_Y_MM, TURRET_BOT_Z_MM),
    )

    # Upper roof canopy overhang supporting `leg_hip_bracket`
    canopy = Part.makeBox(
        CANOPY_WIDTH_X_MM,
        CANOPY_IN_Y_MM - CANOPY_OUT_Y_MM,
        CANOPY_THICK_Z_MM,
        App.Vector(-CANOPY_WIDTH_X_MM * 0.5, CANOPY_OUT_Y_MM, TURRET_TOP_Z_MM - CANOPY_THICK_Z_MM),
    )

    solid = (
        bay_body.fuse(rails)
        .fuse(neck)
        .fuse(turret)
        .fuse(canopy)
        .removeSplitter()
    )

    # 4. Carve internal PCB & component cavity inside bay_body (Y = 2.0 .. 26.0, Z = 1.8 .. 5.8)
    pcb_cavity = Part.makeBox(
        PCB_POCKET_W_X_MM,
        PCB_POCKET_L_Y_MM,
        4.0,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 2.0, 1.8),
    )
    usbc_opening = Part.makeBox(10.5, 4.0, 3.6, App.Vector(-5.25, BAY_INSERT_LEN_Y_MM - 2.5, 1.9))
    # Continuous enclosed servo wire harness tunnel from servo pockets (Y = -15.5) into PCB front headers (Y = +3.0)
    harness_tunnel = Part.makeBox(18.0, 18.5, 3.2, App.Vector(-9.0, -15.5, 2.0))
    # Framework bottom latch catch pocket on underside of bay cartridge (Y = 13.5..17.0, Z = 0..0.8)
    latch_pocket = Part.makeBox(8.0, 3.5, 0.8, App.Vector(-4.0, 13.5, 0.0))

    solid = solid.cut(pcb_cavity).cut(usbc_opening).cut(harness_tunnel).cut(latch_pocket)

    # Add two M2 PCB mounting standoffs inside cavity at X = +/-11.30 mm, Y = PCB_BOSS_Y_MM
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        boss = Part.makeCylinder(2.1, 1.0, App.Vector(bx, PCB_BOSS_Y_MM, 1.8), App.Vector(0, 0, 1))
        solid = solid.fuse(boss)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        pilot = Part.makeCylinder(0.85, 2.2, App.Vector(bx, PCB_BOSS_Y_MM, 0.8), App.Vector(0, 0, 1))
        solid = solid.cut(pilot)

    # 5. Carve TWO horizontal MG90S servo pockets opening outboard (-Y) at X = -7.30 and X = +7.30 mm
    for sx in (-SERVO_CENTER_X_MM, SERVO_CENTER_X_MM):
        pocket = Part.makeBox(
            SERVO_POCKET_W_X_MM,
            SERVO_POCKET_DEPTH_Y + 0.5,
            SERVO_POCKET_H_Z_MM,
            App.Vector(sx - SERVO_POCKET_W_X_MM * 0.5, TURRET_FRONT_Y_MM - 0.5, SERVO_POCKET_Z0_MM),
        )
        solid = solid.cut(pocket)
        for sz in (SERVO_POCKET_Z0_MM - 1.0, SERVO_POCKET_Z0_MM + SERVO_POCKET_H_Z_MM + 1.0):
            pilot_y = Part.makeCylinder(0.85, 6.0, App.Vector(sx, TURRET_FRONT_Y_MM - 0.5, sz), App.Vector(0, 1, 0))
            solid = solid.cut(pilot_y)

    # 6. Drill 4x M2.5 mounting holes in top roof canopy for `leg_hip_bracket`
    for bx in (-BRACKET_BOLT_X_MM, BRACKET_BOLT_X_MM):
        for by in (BRACKET_BOLT_Y1_MM, BRACKET_BOLT_Y2_MM):
            mh = Part.makeCylinder(1.05, 5.0, App.Vector(bx, by, TURRET_TOP_Z_MM - 4.0), App.Vector(0, 0, 1))
            solid = solid.cut(mh)

    # 7. Drill 8x M2.5 bottom pilot holes matching `leg_bay_outrigger` floor bolts
    for by in SPINE_BOLT_Y_MM if False else SPINE_BOLT_Y_LIST:
        depth = 1.5 if by > 0 else 2.6
        for bx in (-SPINE_BOLT_X_MM, SPINE_BOLT_X_MM):
            bh = Part.makeCylinder(1.05, depth + 0.4, App.Vector(bx, by, -0.4), App.Vector(0, 0, 1))
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
        "name": "Leg Bay Structural Housing",
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
