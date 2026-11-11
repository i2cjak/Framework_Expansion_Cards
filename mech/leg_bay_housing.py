#!/usr/bin/env python3
"""
MJF PA12 Structural Expansion-Bay Housing with Under-Chassis Load Shelf (`mech/leg_bay_housing.py`)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Coordinate convention: mm, Z up, front (outward leg turret) facing -Y,
bay insertion direction (USB-C plug + under-chassis shelf) facing +Y.

Mechanical lock-in & load path:
  1. Bay cartridge (`30.0 x 26.5 x 6.8 mm`) slides into the Framework Laptop side T-rails
     (`32.0 mm` overall rail span) and engages the spring latch pocket on the bottom face.
  2. Captures the `26.0 x 30.0 x 0.8 mm` Day 06 PCB on two M2 bosses spaced `22.60 mm` apart
     (`H1`/`H2` at X = +/-11.30 mm), leaving the USB-C plug tongue protruding +Y.
  3. Extends an `18.0 mm` deep, `30.0 mm` wide, `3.2 mm` thick gusseted **under-chassis load shelf**
     underneath the laptop's aluminum bottom cover (at Z = -3.2..0.0 mm) so vertical ground
     reaction forces transfer directly into the laptop chassis rather than the USB-C connector!
  4. Projects a `24.0 mm` outboard actuator saddle (-Y) housing two `23.0 x 12.2 mm` metal-gear
     micro servos (hip + knee linkage) and bolting to `leg_hip_bracket` via 2x M2.5 bosses.

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
BAY_WIDTH_X_MM       = 30.0     # Standard Framework Expansion Card body width
RAIL_SPAN_X_MM       = 32.0     # Side T-rail outer span (1.0 mm rail tongue per side)
RAIL_HEIGHT_Z_MM     = 1.8      # T-rail tongue thickness
BAY_HEIGHT_Z_MM      = 6.8      # Standard Framework Expansion Card height
BAY_INSERT_LEN_Y_MM  = 26.5     # Inserted bay body length along +Y (Y = 0 .. +26.5)

PCB_POCKET_W_X_MM    = 26.4     # Clears 26.0 mm PCB width
PCB_POCKET_L_Y_MM    = 24.8     # Clears internal PCB length
PCB_BOSS_SPAN_X_MM   = 22.60    # Exact H1..H2 spacing (159.8 - 137.2 = 22.60 mm!)
PCB_BOSS_Y_MM        = 8.0      # Boss Y position inside bay pocket

SHELF_LEN_Y_MM       = 18.0     # Under-chassis load reaction lip extending +Y past bay mouth
SHELF_THICK_Z_MM     = 3.2      # Stiff PA12 load shelf thickness under laptop bottom cover

TURRET_LEN_Y_MM      = 24.0     # Outboard (-Y) actuator & hip clevis mount block (Y = -24.0 .. 0)
TURRET_WIDTH_X_MM    = 30.0     # Matches flush with expansion card bezel width
TURRET_HEIGHT_Z_MM   = 15.0     # Houses micro-servo stack cleanly above hip bracket mount plane
SERVO_SLOT_W_X_MM    = 23.4     # Clears standard 23.0 mm micro-servo body length
SERVO_SLOT_D_Y_MM    = 12.6     # Clears standard 12.2 mm micro-servo body thickness

CLEVIS_MOUNT_Y_CENTER = -12.0   # Center of `leg_hip_bracket` on bottom of turret (Z = 0)
CLEVIS_HOLE_OFFSET_Y  = 5.50    # Matches `leg_hip_bracket` +/- 5.50 mm M2.5 holes
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

    # 2. Outboard (-Y) actuator turret & clevis mounting saddle: X = -15..+15, Y = -24..0, Z = 0..+15.0
    turret = Part.makeBox(
        TURRET_WIDTH_X_MM,
        TURRET_LEN_Y_MM,
        TURRET_HEIGHT_Z_MM,
        App.Vector(-TURRET_WIDTH_X_MM * 0.5, -TURRET_LEN_Y_MM, 0.0),
    )

    # 3. Under-chassis load shelf + vertical drop step under bay bezel:
    # Step collar at Y = -4.0 .. +2.5, Z = -SHELF_THICK_Z_MM .. 0.0 bridging turret into under-chassis shelf
    under_collar = Part.makeBox(
        BAY_WIDTH_X_MM,
        6.5 + SHELF_LEN_Y_MM,
        SHELF_THICK_Z_MM,
        App.Vector(-BAY_WIDTH_X_MM * 0.5, -4.0, -SHELF_THICK_Z_MM),
    )
    # Two triangular/ribbed gussets on outer sides (-X/+X) tying turret (-Y) into under-chassis shelf
    gusset_l = Part.makeBox(3.2, 12.0, SHELF_THICK_Z_MM, App.Vector(-BAY_WIDTH_X_MM * 0.5, -14.0, -SHELF_THICK_Z_MM))
    gusset_r = Part.makeBox(3.2, 12.0, SHELF_THICK_Z_MM, App.Vector(BAY_WIDTH_X_MM * 0.5 - 3.2, -14.0, -SHELF_THICK_Z_MM))

    solid = bay_body.fuse(rails).fuse(turret).fuse(under_collar).fuse(gusset_l).fuse(gusset_r).removeSplitter()

    # 4. Carve internal PCB & component cavity inside bay_body (leaving 1.2 mm bottom floor & 1.2 mm top roof)
    pcb_cavity = Part.makeBox(
        PCB_POCKET_W_X_MM,
        PCB_POCKET_L_Y_MM,
        4.4,
        App.Vector(-PCB_POCKET_W_X_MM * 0.5, 1.2, 1.2),
    )
    # Rear (+Y) aperture for USB-C straddle plug tongue (10.5 mm wide x 3.6 mm tall)
    usbc_opening = Part.makeBox(10.5, 4.0, 3.6, App.Vector(-5.25, BAY_INSERT_LEN_Y_MM - 2.5, 1.6))
    # Wire pass-through window from PCB front bezel (Y = 0..2) into outboard servo pocket (Y = -2..0)
    harness_tunnel = Part.makeBox(20.0, 6.0, 4.0, App.Vector(-10.0, -2.5, 1.6))
    # Framework bottom latch catch pocket on underside of bay cartridge (Y = 12..17, Z = 0..1.0)
    latch_pocket = Part.makeBox(8.0, 5.0, 0.9, App.Vector(-4.0, 13.5, 0.0))

    solid = solid.cut(pcb_cavity).cut(usbc_opening).cut(harness_tunnel).cut(latch_pocket)

    # Add two M2 PCB mounting standoffs inside cavity at X = +/-11.30 mm, Y = PCB_BOSS_Y_MM
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        boss = Part.makeCylinder(2.1, 1.4, App.Vector(bx, PCB_BOSS_Y_MM, 1.2), App.Vector(0, 0, 1))
        solid = solid.fuse(boss)
    for bx in (-PCB_BOSS_SPAN_X_MM * 0.5, PCB_BOSS_SPAN_X_MM * 0.5):
        pilot = Part.makeCylinder(0.85, 3.0, App.Vector(bx, PCB_BOSS_Y_MM, 0.6), App.Vector(0, 0, 1))
        solid = solid.cut(pilot)

    # 5. Carve micro-servo cradle pocket in top of outboard turret (X = -11.7..+11.7, Y = -18.3..-5.7, Z = 3.5..16)
    servo_pocket = Part.makeBox(
        SERVO_SLOT_W_X_MM,
        SERVO_SLOT_D_Y_MM,
        13.0,
        App.Vector(-SERVO_SLOT_W_X_MM * 0.5, CLEVIS_MOUNT_Y_CENTER - SERVO_SLOT_D_Y_MM * 0.5, 3.2),
    )
    solid = solid.cut(servo_pocket)

    # 6. Drill 2x M2.5 mounting holes through turret floor for `leg_hip_bracket` (X = 0, Y = -12 +/- 5.5)
    for hy in (CLEVIS_MOUNT_Y_CENTER - CLEVIS_HOLE_OFFSET_Y, CLEVIS_MOUNT_Y_CENTER + CLEVIS_HOLE_OFFSET_Y):
        mh = Part.makeCylinder(1.35, 10.0, App.Vector(0.0, hy, -5.0), App.Vector(0, 0, 1))
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
