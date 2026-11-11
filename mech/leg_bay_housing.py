#!/usr/bin/env python3
"""
MJF PA12 Structural Expansion-Bay Housing with Dual Outboard Servo Saddle & Under-Chassis Load Shelf
(`mech/leg_bay_housing.py`) for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Coordinate convention: mm, Z up, front (outboard leg turret & servo splines) facing -Y,
bay insertion direction (USB-C plug + under-chassis load shelf) facing +Y,
fore/aft walking axis along +/-X.

Mechanical architecture:
  1. Bay cartridge (`30.0 x 26.5 x 6.8 mm`) slides into the Framework Laptop side T-rails
     (`32.0 mm` overall rail span) and engages the underside latch pocket.
  2. Captures the `26.0 x 30.0 x 0.8 mm` Day 06 PCB on two M2 standoffs spaced `22.60 mm` apart
     (`H1`/`H2` at X = +/-11.30 mm), leaving the USB-C plug tongue protruding +Y.
  3. Extends an `18.0 mm` deep, `30.0 mm` wide, `3.2 mm` thick gusseted **under-chassis load shelf**
     underneath the laptop's aluminum bottom cover (`Z = -3.2..0.0 mm`) so vertical ground reaction
     loads transfer directly into the laptop chassis floor rather than the USB-C receptacle.
  4. Houses **two horizontal metal-gear MG90S micro-servos side-by-side** (`X = -7.30 mm` & `+7.30 mm`)
     with **both output splines pointing outboard (`-Y`)** so the femur and knee-pushrod linkage swing
     in the `X-Z` sagittal walking plane, topped by a rigid roof canopy supporting `leg_hip_bracket`.

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
RAIL_SPAN_X_MM        = 32.0     # Side T-rail outer span (1.0 mm rail tongue per side)
RAIL_HEIGHT_Z_MM      = 1.8      # T-rail tongue thickness
BAY_HEIGHT_Z_MM       = 6.8      # Standard Framework Expansion Card height
BAY_INSERT_LEN_Y_MM   = 26.5     # Inserted bay body length along +Y (Y = 0 .. +26.5)

PCB_POCKET_W_X_MM     = 26.4     # Clears 26.0 mm PCB width
PCB_POCKET_L_Y_MM     = 24.0     # Internal PCB pocket (Y = 2.0 .. 26.0)
PCB_BOSS_SPAN_X_MM    = 22.60    # Exact H1..H2 spacing (22.60 mm)
PCB_BOSS_Y_MM         = 9.0      # Boss Y position inside bay pocket

SHELF_LEN_Y_MM        = 18.0     # Under-chassis load reaction lip extending +Y past bay mouth (Y = 0..+18.0)
SHELF_THICK_Z_MM      = 3.2      # Stiff PA12 load shelf thickness under laptop bottom cover

TURRET_WIDTH_X_MM     = 31.6     # Dual-servo saddle width (matches 32.0 mm rail span)
TURRET_LEN_Y_MM       = 21.5     # Outboard (-Y) turret front face at Y = -21.5 mm
TURRET_BOT_Z_MM       = -9.5     # Lower turret floor below servo pockets
TURRET_TOP_Z_MM       = 20.5     # Upper turret roof where `leg_hip_bracket` bolts on

SERVO_CENTER_X_MM     = 7.30     # Symmetric left/right servo centerlines (+/- 7.30 mm)
SERVO_POCKET_W_X_MM   = 12.6     # Clears 12.2 mm MG90S servo thickness
SERVO_POCKET_H_Z_MM   = 23.2     # Clears 22.8 mm MG90S servo case length (Z = -7.6 .. +15.6)
SERVO_POCKET_Z0_MM    = -7.6     # Bottom of horizontal servo pocket (output spline at Z = +9.50 mm)
SERVO_POCKET_DEPTH_Y  = 22.5     # Pocket from Y = -21.5 (outboard face) to Y = +1.0

CANOPY_OUT_Y_MM       = -30.5    # Roof overhang supporting `leg_hip_bracket` top flange
CANOPY_WIDTH_X_MM     = 28.0     # Matches `leg_hip_bracket` 28.0 mm width
CANOPY_THICK_Z_MM     = 3.8      # Stiff upper bridge floor (Z = 16.7 .. 20.5)
BRACKET_BOLT_X_MM     = 9.0      # +/- 9.0 mm M2.5 mounting pattern
BRACKET_BOLT_Y1_MM    = -26.0    # Outboard pair of M2.5 mounting holes
BRACKET_BOLT_Y2_MM    = -18.0    # Inboard pair of M2.5 mounting holes

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

    # 2. Outboard (-Y) dual-servo actuator turret: X = -15.8..+15.8, Y = -21.5..0.0, Z = -9.5..+20.5
    turret = Part.makeBox(
        TURRET_WIDTH_X_MM,
        TURRET_LEN_Y_MM,
        TURRET_TOP_Z_MM - TURRET_BOT_Z_MM,
        App.Vector(-TURRET_WIDTH_X_MM * 0.5, -TURRET_LEN_Y_MM, TURRET_BOT_Z_MM),
    )

    # Upper roof canopy overhang (Y = -30.5 .. 0.0, Z = 16.7 .. 20.5) supporting `leg_hip_bracket`
    canopy = Part.makeBox(
        CANOPY_WIDTH_X_MM,
        -CANOPY_OUT_Y_MM,
        CANOPY_THICK_Z_MM,
        App.Vector(-CANOPY_WIDTH_X_MM * 0.5, CANOPY_OUT_Y_MM, TURRET_TOP_Z_MM - CANOPY_THICK_Z_MM),
    )

    # 3. Under-chassis load shelf under laptop bottom cover (Y = 0 .. +18.0, Z = -3.2 .. 0.0)
    #    plus transition collar tying into turret lower block (Y = -4.0 .. 0.0, Z = -9.5 .. 0.0)
    under_shelf = Part.makeBox(
        BAY_WIDTH_X_MM,
        4.0 + SHELF_LEN_Y_MM,
        SHELF_THICK_Z_MM,
        App.Vector(-BAY_WIDTH_X_MM * 0.5, -4.0, -SHELF_THICK_Z_MM),
    )
    # Gusset ribs under bay bezel (Y = 0 .. +6.0, Z = -6.0 .. -3.2) transferring bending moment into shelf
    gusset_l = Part.makeBox(4.0, 6.0, 2.8, App.Vector(-BAY_WIDTH_X_MM * 0.5, 0.0, -6.0))
    gusset_r = Part.makeBox(4.0, 6.0, 2.8, App.Vector(BAY_WIDTH_X_MM * 0.5 - 4.0, 0.0, -6.0))

    solid = (
        bay_body.fuse(rails)
        .fuse(turret)
        .fuse(canopy)
        .fuse(under_shelf)
        .fuse(gusset_l)
        .fuse(gusset_r)
        .removeSplitter()
    )

    # 4. Carve internal PCB & component cavity inside bay_body (Y = 2.0 .. 26.0, Z = 1.2 .. 5.6)
    pcb_cavity = Part.makeBox(
        PCB_POCKET_W_X_MM,
        PCB_POCKET_L_Y_MM,
        4.4,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 2.0, 1.2),
    )
    # Rear (+Y) aperture for USB-C straddle plug tongue
    usbc_opening = Part.makeBox(10.5, 4.0, 3.6, App.Vector(-5.25, BAY_INSERT_LEN_Y_MM - 2.5, 1.6))
    # Wire harness channel from both servo pockets (Y = +1.0) into PCB front headers J2/J3 (Y = +2.0..+5.0)
    harness_tunnel = Part.makeBox(22.0, 5.0, 4.0, App.Vector(-11.0, -0.5, 1.6))
    # Framework bottom latch catch pocket on underside of bay cartridge (Y = 13.5..18.5, Z = 0..0.9)
    latch_pocket = Part.makeBox(8.0, 5.0, 0.9, App.Vector(-4.0, 13.5, 0.0))

    solid = solid.cut(pcb_cavity).cut(usbc_opening).cut(harness_tunnel).cut(latch_pocket)

    # Add two M2 PCB mounting standoffs inside cavity at X = +/-11.30 mm, Y = PCB_BOSS_Y_MM
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        boss = Part.makeCylinder(2.1, 1.4, App.Vector(bx, PCB_BOSS_Y_MM, 1.2), App.Vector(0, 0, 1))
        solid = solid.fuse(boss)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        pilot = Part.makeCylinder(0.85, 3.0, App.Vector(bx, PCB_BOSS_Y_MM, 0.6), App.Vector(0, 0, 1))
        solid = solid.cut(pilot)

    # 5. Carve TWO horizontal MG90S servo pockets opening outboard (-Y) at X = -7.30 and X = +7.30 mm
    for sx in (-SERVO_CENTER_X_MM, SERVO_CENTER_X_MM):
        pocket = Part.makeBox(
            SERVO_POCKET_W_X_MM,
            SERVO_POCKET_DEPTH_Y + 0.5,
            SERVO_POCKET_H_Z_MM,
            App.Vector(sx - SERVO_POCKET_W_X_MM * 0.5, -TURRET_LEN_Y_MM - 0.5, SERVO_POCKET_Z0_MM),
        )
        solid = solid.cut(pocket)
        # Servo mounting-ear screw pilot holes on front face (-Y) above & below each pocket
        for sz in (SERVO_POCKET_Z0_MM - 1.0, SERVO_POCKET_Z0_MM + SERVO_POCKET_H_Z_MM + 1.0):
            pilot_y = Part.makeCylinder(0.85, 6.0, App.Vector(sx, -TURRET_LEN_Y_MM - 0.5, sz), App.Vector(0, 1, 0))
            solid = solid.cut(pilot_y)

    # 6. Drill 4x M2.5 mounting holes in top roof canopy for `leg_hip_bracket`
    for bx in (-BRACKET_BOLT_X_MM, BRACKET_BOLT_X_MM):
        for by in (BRACKET_BOLT_Y1_MM, BRACKET_BOLT_Y2_MM):
            mh = Part.makeCylinder(1.05, 5.0, App.Vector(bx, by, TURRET_TOP_Z_MM - 4.0), App.Vector(0, 0, 1))
            solid = solid.cut(mh)

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
