#!/usr/bin/env python3
"""
Full 3D Single-Leg Assembly (`mech/walking_leg_assembly.py`) for Day 06 Walking Legs.

Assembles all colored STEP parts in standing posture (mm, Z up, front facing -Y):
  1. `mech/leg_bay_housing.step`   — MJF PA12 expansion-bay cartridge + under-chassis load shelf + turret
  2. `Day_06_Walking_Legs/build/board_only.step` — 0.8 mm 4-layer Day 06 PCB seated on M2 standoffs
  3. Reference MG90S metal-gear micro-servo & brass spline horn seated inside turret cradle
  4. `mech/leg_hip_bracket.step`   — SendCutSend ALU-063 CNC-bent U-clevis bolted under turret
  5. 2x `mech/leg_femur_link.step` — Paired SendCutSend ALU-080 36 mm femur links pitched at 30 deg
  6. `mech/leg_tibia_strut.step`   — SendCutSend ALU-063 CNC-bent 58 mm U-channel tibia strut
  7. `mech/leg_foot_pad.step`      — FDM 95A TPU Ø22 mm hemispherical foot pad
  8. Stainless M3 hip, knee & ankle shoulder-axle pins

Exports:
  - mech/walking_leg_assembly.FCStd
  - mech/walking_leg_assembly.step (full multi-color STEP assembly)
"""
import math
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Assembly Parameters (mm, deg) ---
CLEVIS_Y_MM        = -12.0     # Outboard turret centerline Y
CLEVIS_TOP_Z_MM    = -3.2      # Flush underside of side gussets & load-shelf plane
HIP_AXLE_DROP_MM   = 13.5      # 18.5 mm flange - 5.0 mm from tip
FEMUR_PITCH_MM     = 36.0      # Hip-to-knee axle pitch on leg_femur_link
FEMUR_ANGLE_DEG    = 30.0      # Standing femur pitch below -Y horizontal
FEMUR_HALF_SPAN_X  = 9.6       # Paired femur plates straddling 18.8 mm / 22.0 mm clevis cleanly
TIBIA_KNEE_Z_MM    = 52.0      # Knee bore Z in leg_tibia_strut local coords
TIBIA_BORE_Y_MM    = 3.75      # Knee & ankle bore Y in leg_tibia_strut local coords (7.75 - 4.0)
TIBIA_ANKLE_Z_MM   = 6.0       # Ankle bore Z in leg_tibia_strut local coords
FOOT_BOLT_Z_MM     = 17.0      # Ankle bore Z in leg_foot_pad local coords

PA12_COLOR         = (0.22, 0.23, 0.26)
ALU_COLOR          = (0.76, 0.79, 0.83)
TPU_COLOR          = (0.14, 0.14, 0.16)
SERVO_BODY_COLOR   = (0.14, 0.30, 0.58)
SERVO_CAP_COLOR    = (0.78, 0.65, 0.22)
PCB_COLOR          = (0.16, 0.09, 0.24)
STEEL_PIN_COLOR    = (0.82, 0.84, 0.87)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
LOG_PATH  = MECH_DIR / "walking_leg_assembly.log"


def add_step_part(doc, step_path: Path, placement: App.Placement, name: str, color: tuple[float, float, float]):
    """Loads STEP solid via Part.read (dialog-free offscreen) and assigns placement + RGB metadata."""
    shp = Part.read(str(step_path))
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shp
    obj.Placement = placement
    return obj, color


def build():
    doc = App.newDocument("walking_leg_assembly")
    colored_objs: list[tuple[object, tuple[float, float, float]]] = []

    # 1. MJF PA12 Structural Bay Housing + Under-Chassis Load Shelf at origin
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_bay_housing.step", App.Placement(), "BayHousing", PA12_COLOR)
    )

    # 2. 0.8 mm 4-Layer Day 06 PCB seated on internal M2 bosses (X=0, Y=+1.5..+31.5, Z=+2.6)
    pcb_step = REPO_ROOT / "Day_06_Walking_Legs" / "build" / "board_only.step"
    if pcb_step.exists():
        pcb_shp = Part.read(str(pcb_step))
        bb = pcb_shp.BoundBox
        dx = -0.5 * (bb.XMin + bb.XMax)
        dy = 8.0 - (-109.5)
        dz = 2.6 - bb.ZMin
        pcb_pl = App.Placement(App.Vector(dx, dy, dz), App.Rotation())
        pcb_obj = doc.addObject("Part::Feature", "Day06PCB")
        pcb_obj.Shape = pcb_shp
        pcb_obj.Placement = pcb_pl
        colored_objs.append((pcb_obj, PCB_COLOR))

    # 3. Reference Metal-Gear Micro-Servo seated inside turret pocket (X=-11.5..+11.5, Y=-18.1..-5.9, Z=3.4..19.4)
    servo_body = Part.makeBox(22.8, 12.0, 15.5, App.Vector(-11.4, CLEVIS_Y_MM - 6.0, 3.4))
    servo_ears = Part.makeBox(32.0, 12.0, 2.2, App.Vector(-16.0, CLEVIS_Y_MM - 6.0, 14.8))
    servo_boss = Part.makeCylinder(5.8, 3.5, App.Vector(-5.5, CLEVIS_Y_MM, 18.9), App.Vector(0, 0, 1))
    servo_obj = doc.addObject("Part::Feature", "MicroServoActuator")
    servo_obj.Shape = servo_body.fuse(servo_ears).fuse(servo_boss).removeSplitter()
    colored_objs.append((servo_obj, SERVO_BODY_COLOR))

    spline_obj = doc.addObject("Part::Feature", "ServoBrassSpline")
    spline_obj.Shape = Part.makeCylinder(2.4, 2.5, App.Vector(-5.5, CLEVIS_Y_MM, 22.4), App.Vector(0, 0, 1))
    colored_objs.append((spline_obj, SERVO_CAP_COLOR))

    # 4. SendCutSend ALU-063 Hip Clevis Bracket bolted at (0, CLEVIS_Y_MM, CLEVIS_TOP_Z_MM)
    hip_pl = App.Placement(App.Vector(0.0, CLEVIS_Y_MM, CLEVIS_TOP_Z_MM), App.Rotation())
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_hip_bracket.step", hip_pl, "HipBracket", ALU_COLOR)
    )

    hip_axle_y = CLEVIS_Y_MM
    hip_axle_z = CLEVIS_TOP_Z_MM - HIP_AXLE_DROP_MM  # -16.7 mm

    # 5. Paired SendCutSend ALU-080 Femur Links:
    theta_rad = math.radians(FEMUR_ANGLE_DEG)
    rot_y90 = App.Rotation(App.Vector(0, 1, 0), 90.0)
    rot_x = App.Rotation(App.Vector(1, 0, 0), -(90.0 - FEMUR_ANGLE_DEG))
    femur_rot = rot_x.multiply(rot_y90)

    for side_idx, x_pos in enumerate((-FEMUR_HALF_SPAN_X - 2.032, FEMUR_HALF_SPAN_X)):
        f_pl = App.Placement(App.Vector(x_pos, hip_axle_y, hip_axle_z), femur_rot)
        colored_objs.append(
            add_step_part(doc, MECH_DIR / "leg_femur_link.step", f_pl, f"FemurLink_{side_idx+1}", ALU_COLOR)
        )

    knee_y = hip_axle_y - FEMUR_PITCH_MM * math.cos(theta_rad)
    knee_z = hip_axle_z - FEMUR_PITCH_MM * math.sin(theta_rad)

    # 6. SendCutSend ALU-063 Tibia Strut hanging vertically from knee axle (knee_y, knee_z)
    tibia_orig_y = knee_y - TIBIA_BORE_Y_MM
    tibia_orig_z = knee_z - TIBIA_KNEE_Z_MM
    tibia_pl = App.Placement(App.Vector(0.0, tibia_orig_y, tibia_orig_z), App.Rotation())
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_tibia_strut.step", tibia_pl, "TibiaStrut", ALU_COLOR)
    )

    # 7. FDM TPU Compliant Foot Pad on M3 ankle axle
    ankle_y = tibia_orig_y + TIBIA_BORE_Y_MM
    ankle_z = tibia_orig_z + TIBIA_ANKLE_Z_MM
    foot_pl = App.Placement(App.Vector(0.0, ankle_y, ankle_z - FOOT_BOLT_Z_MM), App.Rotation())
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_foot_pad.step", foot_pl, "FootPad", TPU_COLOR)
    )

    # 8. Stainless M3 Shoulder-Axle Pins at Hip, Knee & Ankle
    p1 = Part.makeCylinder(1.5, 30.0, App.Vector(-15.0, hip_axle_y, hip_axle_z), App.Vector(1, 0, 0))
    p2 = Part.makeCylinder(1.5, 25.0, App.Vector(-12.5, knee_y, knee_z), App.Vector(1, 0, 0))
    p3 = Part.makeCylinder(1.5, 24.0, App.Vector(-12.0, ankle_y, ankle_z), App.Vector(1, 0, 0))
    pins_obj = doc.addObject("Part::Feature", "M3AxlePins")
    pins_obj.Shape = p1.fuse(p2).fuse(p3)
    colored_objs.append((pins_obj, STEEL_PIN_COLOR))

    doc.recompute()

    # Validate shapes
    for obj, _ in colored_objs:
        assert obj.Shape.isValid(), f"Invalid shape on {obj.Name}"

    doc.saveAs(str(MECH_DIR / "walking_leg_assembly.FCStd"))
    step_out = MECH_DIR / "walking_leg_assembly.step"
    export_objs = [obj for obj, _ in colored_objs]

    if App.GuiUp:
        import ImportGui
        for obj, col in colored_objs:
            obj.ViewObject.ShapeColor = col
        ImportGui.export(export_objs, str(step_out))
    else:
        Part.export(export_objs, str(step_out))

    LOG_PATH.write_text(
        f"OK: walking_leg_assembly.step exported ({len(doc.Objects)} objects), "
        f"hip=({hip_axle_y:.2f}, {hip_axle_z:.2f}), knee=({knee_y:.2f}, {knee_z:.2f}), "
        f"foot_ground_z={ankle_z - FOOT_BOLT_Z_MM:.2f} mm\n"
    )


exit_code = 0
try:
    build()
except Exception:
    LOG_PATH.write_text("ERROR:\n" + traceback.format_exc())
    exit_code = 1
finally:
    if App.GuiUp:
        os._exit(exit_code)
    elif exit_code != 0:
        raise SystemExit(exit_code)
