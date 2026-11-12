#!/usr/bin/env python3
"""
MJF PA12 Modular Outboard Dual-Servo Saddle (`mech/leg_servo_pod.py`, `qty: 4`)
for Day 06 Walking Legs — Bolts anywhere along the `ALU-080` `leg_flank_bracket` Rail.

Coordinate convention: mm, Z up, front (outboard leg turret & servo splines) facing -Y,
laptop flank facing +Y (`Y = -1.20 mm` back face, `1.20 mm` clear of laptop palmrest rim),
fore/aft walking axis along +/-X.

Why this is a separate modular pod from `leg_bay_housing`:
  Because both Expansion Card bays on each side of the Framework Laptop 13 sit in the rear half
  of the chassis (`Y_laptop = +36.0 mm` & `+76.6 mm`), decoupling the dual-MG90S servo saddle from
  the Expansion Card plug lets `leg_flank_bracket` carry the Front `leg_servo_pod` `81.0 mm` forward
  at `Y_laptop = -45.0 mm` and the Rear `leg_servo_pod` at `Y_laptop = +67.0 mm` for a balanced
  `112.0 mm` quadruped wheelbase.

Mechanical features:
  1. `35.6 x 22.3 x 43.5 mm` main saddle (`Y = -23.50 .. -1.20 mm`, `Z = 0.00 .. +43.50 mm`) sits
     100% flat on the outboard top web of `leg_flank_bracket` (`Y = -24.50 .. 0.00 mm`) and bolts
     down via `4x M2.5 x 3.0 mm` (`OD 3.5 mm`) brass heat-set inserts (`Ø3.25 x 3.50 mm` blind bores)
     at `X = +/-7.00 mm`, `Y = -8.50` & `-14.50 mm`.
  2. Houses two horizontal metal-gear `MG90S` servos (`X = +/-9.00 mm`, `Z_shaft = +22.10 mm`) with
     both brass output splines facing outboard (`-Y`) and exact `27.50 mm` mounting-ear pilot spacing
     (`Z = +2.85 mm` & `+30.35 mm`, `Ø1.70 mm` pilot bores for `M2` self-tapping flange screws).
  3. Upper roof canopy (`30.0 x 35.3 x 4.2 mm`, `Y = -36.50 .. -1.20 mm`, `Z = +39.30 .. +43.50 mm`)
     carries `leg_hip_bracket` via `4x M2.5 x 3.0 mm` (`OD 3.5 mm`) brass heat-set inserts (`Ø3.25 x 3.50 mm`
     blind bores) at `X = +/-10.00 mm`, `Y = -31.50` & `-22.50 mm`, placing its outboard M3 double-shear
     axle holes coaxially with the servo shafts at `Z = +22.10 mm`.
  4. Rear cable entry slot (`16.0 x 4.0 mm` at `Y = -1.20 mm`) receives the dual 3-wire servo harnesses
     routed along `leg_flank_bracket` from the rear-set `leg_bay_housing`.

Exports:
  - mech/leg_servo_pod.FCStd
  - mech/leg_servo_pod.step      (colored matte black MJF PA12 nylon)
  - mech/leg_servo_pod.fab.json  (measured 3dp MJF PA12 features)
"""
import json
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Parameters (mm) ---
TURRET_WIDTH_X_MM     = 35.60    # Compact 35.6 mm fore-aft saddle width (18.0 mm servo-to-servo spacing)
TURRET_FRONT_Y_MM     = -23.50   # Sits inside -24.50 mm outer face of `leg_flank_bracket`
TURRET_BACK_Y_MM      = -1.20    # 1.20 mm clear of laptop's +/-148.30 mm upper palmrest edge
TURRET_BOT_Z_MM       = 0.00     # Seats flat on `leg_flank_bracket` Z = 0.00 mm top face
TURRET_TOP_Z_MM       = 43.50    # High-clearance overhead bridge (`17.2 mm` above Z=22.10 servo shafts!)

SERVO_CENTER_X_MM     = 9.00     # Symmetric left/right MG90S centerlines (+/- 9.00 mm -> 18.0 mm spacing)
SERVO_POCKET_W_X_MM   = 12.60    # Clears 12.2 mm MG90S servo thickness
SERVO_POCKET_H_Z_MM   = 23.20    # Clears 22.8 mm MG90S servo case height (Z = +5.00 .. +28.20; center Z = +16.60)
SERVO_POCKET_Z0_MM    = 5.00     # Solid 5.00 mm floor for M2.5 rail inserts & lower servo ear screws
SERVO_POCKET_DEPTH_Y  = 19.60    # Pocket from Y = -23.50 to Y = -3.90 (clears 19.0 mm rear MG90S body)
SERVO_EAR_PITCH_Z_MM  = 27.50    # Exact MG90S mounting-ear hole pitch -> Z = 16.60 +/- 13.75 = +2.85 & +30.35 mm!

CANOPY_OUT_Y_MM       = -36.50   # High-arch roof overhang supporting `leg_hip_bracket` above linkage
CANOPY_IN_Y_MM        = -1.20
CANOPY_WIDTH_X_MM     = 30.00    # Matches `leg_hip_bracket` 30.0 mm width
CANOPY_THICK_Z_MM     = 4.20     # Stiff 4.20 mm roof (Z = 39.30 .. 43.50) for 3.50 mm deep M2.5 heat-set inserts!
BRACKET_BOLT_X_MM     = 10.00    # +/- 10.00 mm M2.5 mounting pattern
BRACKET_BOLT_Y1_MM    = -31.50   # Matches `leg_hip_bracket` Y_local = +24.70 when placed at Y = -56.20
BRACKET_BOLT_Y2_MM    = -22.50   # Matches `leg_hip_bracket` Y_local = +33.70 when placed at Y = -56.20

RAIL_BOLT_X_MM        = 7.00     # +/- 7.00 mm M2.5 floor hole pattern
RAIL_BOLT_Y_LIST      = (-8.50, -14.50)  # Exact match to `leg_flank_bracket` OUTBOARD_BOLT_Y_MM

M25_INSERT_BORE_R_MM  = 1.625    # Ø3.25 mm blind bore for standard M2.5 x 3.0 mm (OD 3.5 mm) brass heat-set insert
M25_INSERT_DEPTH_Z_MM = 3.50     # 3.50 mm blind bore depth in both 4.20 mm canopy & 5.00 mm floor

PA12_COLOR            = (0.20, 0.21, 0.23)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_servo_pod.log"


def build():
    doc = App.newDocument("leg_servo_pod")

    # 1. Main dual-servo saddle block on `leg_flank_bracket` outboard top face
    turret = Part.makeBox(
        TURRET_WIDTH_X_MM,
        TURRET_BACK_Y_MM - TURRET_FRONT_Y_MM,
        TURRET_TOP_Z_MM - TURRET_BOT_Z_MM,
        App.Vector(-TURRET_WIDTH_X_MM * 0.5, TURRET_FRONT_Y_MM, TURRET_BOT_Z_MM),
    )

    # 2. Upper roof canopy overhang supporting `leg_hip_bracket`
    canopy = Part.makeBox(
        CANOPY_WIDTH_X_MM,
        CANOPY_IN_Y_MM - CANOPY_OUT_Y_MM,
        CANOPY_THICK_Z_MM,
        App.Vector(-CANOPY_WIDTH_X_MM * 0.5, CANOPY_OUT_Y_MM, TURRET_TOP_Z_MM - CANOPY_THICK_Z_MM),
    )

    solid = turret.fuse(canopy).removeSplitter()

    # 3. Two horizontal MG90S servo pockets opening outboard (-Y) at X = +/-9.00 mm
    pocket_mid_z = SERVO_POCKET_Z0_MM + 0.5 * SERVO_POCKET_H_Z_MM  # +16.60 mm
    ear_z_low    = pocket_mid_z - 0.5 * SERVO_EAR_PITCH_Z_MM       # +2.85 mm
    ear_z_high   = pocket_mid_z + 0.5 * SERVO_EAR_PITCH_Z_MM       # +30.35 mm
    for sx in (-SERVO_CENTER_X_MM, SERVO_CENTER_X_MM):
        pocket = Part.makeBox(
            SERVO_POCKET_W_X_MM,
            SERVO_POCKET_DEPTH_Y + 0.50,
            SERVO_POCKET_H_Z_MM,
            App.Vector(sx - SERVO_POCKET_W_X_MM * 0.5, TURRET_FRONT_Y_MM - 0.50, SERVO_POCKET_Z0_MM),
        )
        solid = solid.cut(pocket)
        # Servo mounting-ear pilot holes on front face (exact 27.50 mm MG90S ear pitch: Z = +2.85 and +30.35 mm)
        for sz in (ear_z_low, ear_z_high):
            pilot_y = Part.makeCylinder(0.85, 6.00, App.Vector(sx, TURRET_FRONT_Y_MM - 0.50, sz), App.Vector(0, 1, 0))
            solid = solid.cut(pilot_y)

    # 4. Rear wire-entry slot through 2.70 mm rear wall (Y = -4.50 .. -0.50, Z = +5.20 .. +9.20)
    wire_slot = Part.makeBox(16.00, 4.50, 4.00, App.Vector(-8.00, -4.50, 5.20))
    solid = solid.cut(wire_slot)

    # 5. 4x M2.5 blind heat-set insert bores in top roof canopy (`Z = 43.50` down to `40.00 mm`) for `leg_hip_bracket`
    for bx in (-BRACKET_BOLT_X_MM, BRACKET_BOLT_X_MM):
        for by in (BRACKET_BOLT_Y1_MM, BRACKET_BOLT_Y2_MM):
            mh = Part.makeCylinder(
                M25_INSERT_BORE_R_MM,
                M25_INSERT_DEPTH_Z_MM + 0.20,
                App.Vector(bx, by, TURRET_TOP_Z_MM - M25_INSERT_DEPTH_Z_MM),
                App.Vector(0, 0, 1),
            )
            solid = solid.cut(mh)

    # 6. 4x M2.5 blind heat-set insert bores in 5.00 mm floor (`Z = 0.00` up to `3.50 mm`) matching `leg_flank_bracket`
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
    assert solid.isValid(), "leg_servo_pod shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegServoPod")
    part_obj.Shape = solid
    doc.recompute()

    doc.saveAs(str(MECH_DIR / "leg_servo_pod.FCStd"))
    step_path = MECH_DIR / "leg_servo_pod.step"
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
        "name": "Leg Outboard Dual-Servo Pod",
        "source": "leg_servo_pod.step",
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
    (MECH_DIR / "leg_servo_pod.fab.json").write_text(json.dumps(fab, indent=2) + "\n")

    LOG_PATH.write_text(
        f"OK: leg_servo_pod valid={solid.isValid()} "
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
