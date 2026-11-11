#!/usr/bin/env python3
"""
Full 3D Single-Leg Sagittal 2-DOF Assembly (`mech/walking_leg_assembly.py`)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Coordinate convention: mm, Z up, front/outboard facing -Y, bay insertion facing +Y,
fore/aft walking stride along +/-X.

Key mechanical & kinematic features:
  1. `mech/leg_bay_housing.step` — MJF PA12 expansion-bay cartridge + 18 mm under-chassis load shelf
     + dual horizontal servo saddle.
  2. `Day_06_Walking_Legs/build/board_only.step` — 0.8 mm 4-layer Day 06 controller PCB inside bay.
  3. **TWO** MG90S metal-gear micro-servos (`HipServo` at X=-7.30, `KneeServo` at X=+7.30, Z=+9.50)
     with **both brass output splines pointing outboard (`-Y`)** + machined aluminum servo horns
     (`HipHorn` direct hub + `KneeHorn` 12.0 mm crank arm).
  4. `mech/leg_hip_bracket.step` — SendCutSend ALU-063 overhead double-shear bearing outrigger
     supporting both outboard M3 axle pins at Y=-44.0..-45.6 mm so radial loads bypass the servos.
  5. 2x `mech/leg_femur_link.step` — Paired SendCutSend ALU-080 38.0 mm femur links in double shear.
  6. `mech/leg_knee_pushrod.step` — SendCutSend ALU-080 36.0 mm pushrod coupling `KneeHorn` to
     the tibia's integral 12.0 mm 90-deg bellcrank lobe (~97 deg transmission angle).
  7. `mech/leg_tibia_strut.step` — SendCutSend ALU-063 CNC-bent sagittal bellcrank U-channel tibia.
  8. `mech/leg_foot_pad.step` — FDM 95A TPU paw with through-arch elastomeric suspension slot.
  9. Stainless M3 shoulder-axle pins at Hip, Knee, Bellcrank, Servo Crank, & Ankle.

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

# --- Named Sagittal Linkage Parameters (mm) ---
SERVO_X_OFFSET_MM   = 7.30     # Hip at -7.30 mm, KneeServo at +7.30 mm
SERVO_AXIS_Z_MM     = 9.50     # Both servo output axes at Z = +9.50 mm
TURRET_FRONT_Y_MM   = -21.50   # Front face of PA12 servo turret
BRACKET_OUT_Y_MM    = -45.60   # Outboard outer face of `leg_hip_bracket`
BRACKET_TOP_Z_MM    = 22.1002  # Top outer face of `leg_hip_bracket` (20.5 + 1.6002)

TIBIA_MID_Y_MM      = -33.50   # Centerplane of 20.0 mm tibia channel (spans Y = -43.5 .. -23.5)
INBOARD_LINK_Y0_MM  = -25.30   # Inboard femur & pushrod start Y (extrudes -2.032 to -27.332)
OUTBOARD_LINK_Y0_MM = -39.668  # Outboard femur start Y (extrudes -2.032 to -41.700)

FEMUR_LEN_MM        = 38.0     # Hip-to-Knee pitch
TIBIA_LEN_MM        = 47.0     # Knee-to-Ankle pitch (53.0 - 6.0)
HORN_CRANK_MM       = 12.0     # KneeServo crank arm radius
PUSHROD_LEN_MM      = 36.0     # Knee pushrod center-to-center pitch
FOOT_BOLT_Z_MM      = 19.0     # Ankle bolt Z in `leg_foot_pad` local coords

# Standing neutral foot ankle target (centered fore-aft at X = 0.0 mm!)
ANKLE_TARGET_X_MM   = 0.0
ANKLE_TARGET_Z_MM   = -68.93   # Places ground sole at Z = -87.93 mm (~84.7 mm under laptop bottom!)

PA12_COLOR          = (0.22, 0.23, 0.26)
ALU_COLOR           = (0.76, 0.79, 0.83)
PUSHROD_COLOR       = (0.64, 0.72, 0.82)
HORN_COLOR          = (0.82, 0.30, 0.22)  # Anodized red-orange aluminum servo horns for clear visual read
TPU_COLOR           = (0.14, 0.15, 0.17)
SERVO_BODY_COLOR    = (0.14, 0.30, 0.58)
SERVO_CAP_COLOR     = (0.82, 0.68, 0.22)
PCB_COLOR           = (0.14, 0.42, 0.22)
STEEL_PIN_COLOR     = (0.84, 0.86, 0.89)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
LOG_PATH  = MECH_DIR / "walking_leg_assembly.log"


def rot_from_cols(c1: tuple[float, float, float], c2: tuple[float, float, float], c3: tuple[float, float, float]) -> App.Rotation:
    m = App.Matrix()
    m.A11, m.A21, m.A31 = c1
    m.A12, m.A22, m.A32 = c2
    m.A13, m.A23, m.A33 = c3
    return App.Rotation(m)


def sagittal_link_placement(p1x: float, p1z: float, p2x: float, p2z: float, y_start: float) -> App.Placement:
    """Places a flat XY link (0..L along +X_local, extruded +Z_local) into the X-Z sagittal plane extruding -Y."""
    dx = p2x - p1x
    dz = p2z - p1z
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    rot = rot_from_cols((ux, 0.0, uz), (-uz, 0.0, ux), (0.0, -1.0, 0.0))
    return App.Placement(App.Vector(p1x, y_start, p1z), rot)


def make_horn_solid(x1: float, z1: float, x2: float, z2: float, y_front: float, thick: float, r1: float, r2: float) -> Part.Shape:
    """Creates a clean stadium-shaped machined servo horn in the X-Z plane from (x1,z1) to (x2,z2)."""
    c1 = Part.makeCylinder(r1, thick, App.Vector(x1, y_front, z1), App.Vector(0, -1, 0))
    c2 = Part.makeCylinder(r2, thick, App.Vector(x2, y_front, z2), App.Vector(0, -1, 0))
    dx, dz = x2 - x1, z2 - z1
    dist = math.hypot(dx, dz)
    if dist < 1e-3:
        return c1
    nx, nz = -dz / dist, dx / dist
    rmid = min(r1, r2)
    pts = [
        App.Vector(x1 + nx * rmid, y_front, z1 + nz * rmid),
        App.Vector(x2 + nx * rmid, y_front, z2 + nz * rmid),
        App.Vector(x2 - nx * rmid, y_front, z2 - nz * rmid),
        App.Vector(x1 - nx * rmid, y_front, z1 - nz * rmid),
    ]
    web = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(App.Vector(0, -thick, 0))
    return c1.fuse(c2).fuse(web).removeSplitter()


def solve_leg_kinematics(ankle_x: float, ankle_z: float, s_dir: float = 1.0) -> dict[str, float]:
    """Exact 2-link sagittal inverse kinematics + 4-bar bellcrank pushrod closure."""
    hx = -SERVO_X_OFFSET_MM * s_dir
    hz = SERVO_AXIS_Z_MM
    sx = SERVO_X_OFFSET_MM * s_dir
    sz = SERVO_AXIS_Z_MM

    # 1. Hip -> Knee -> Ankle 2-circle intersection (picking aft knee bend along s_dir * +X)
    dx = ankle_x - hx
    dz = ankle_z - hz
    d = math.hypot(dx, dz)
    assert abs(FEMUR_LEN_MM - TIBIA_LEN_MM) < d < (FEMUR_LEN_MM + TIBIA_LEN_MM), f"Unreachable ankle d={d:.2f}"
    a_proj = (FEMUR_LEN_MM**2 - TIBIA_LEN_MM**2 + d**2) / (2.0 * d)
    h_perp = math.sqrt(max(0.0, FEMUR_LEN_MM**2 - a_proj**2))
    mx = hx + a_proj * (dx / d)
    mz = hz + a_proj * (dz / d)
    kx = mx + s_dir * h_perp * (-dz / d)
    kz = mz + s_dir * h_perp * (dx / d)

    # 2. Tibia basis & Bellcrank pin (12.0 mm perpendicular to tibia along s_dir * +X_local)
    uz_x = (kx - ankle_x) / TIBIA_LEN_MM
    uz_z = (kz - ankle_z) / TIBIA_LEN_MM
    ex_x = s_dir * uz_z
    ex_z = -s_dir * uz_x
    bx = kx + 12.0 * ex_x
    bz = kz + 12.0 * ex_z

    # 3. KneeServo -> CrankPin -> Bellcrank 2-circle intersection (picking aft horn along s_dir * +X)
    ddx = bx - sx
    ddz = bz - sz
    dd = math.hypot(ddx, ddz)
    assert abs(HORN_CRANK_MM - PUSHROD_LEN_MM) < dd < (HORN_CRANK_MM + PUSHROD_LEN_MM), f"Unreachable bellcrank dd={dd:.2f}"
    ca = (HORN_CRANK_MM**2 - PUSHROD_LEN_MM**2 + dd**2) / (2.0 * dd)
    ch = math.sqrt(max(0.0, HORN_CRANK_MM**2 - ca**2))
    cmx = sx + ca * (ddx / dd)
    cmz = sz + ca * (ddz / dd)
    crx = cmx + s_dir * ch * (-ddz / dd)
    crz = cmz + s_dir * ch * (ddx / dd)

    return {
        "hx": hx, "hz": hz,
        "sx": sx, "sz": sz,
        "kx": kx, "kz": kz,
        "ax": ankle_x, "az": ankle_z,
        "bx": bx, "bz": bz,
        "crx": crx, "crz": crz,
        "uz_x": uz_x, "uz_z": uz_z,
        "ex_x": ex_x, "ex_z": ex_z,
        "s_dir": s_dir,
    }


def add_step_part(doc, step_path: Path, placement: App.Placement, name: str, color: tuple[float, float, float]):
    shp = Part.read(str(step_path))
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shp
    obj.Placement = placement
    return obj, color


def build():
    doc = App.newDocument("walking_leg_assembly")
    colored_objs: list[tuple[object, tuple[float, float, float]]] = []

    kin = solve_leg_kinematics(ANKLE_TARGET_X_MM, ANKLE_TARGET_Z_MM, s_dir=1.0)

    # 1. MJF PA12 Structural Bay Housing + Under-Chassis Load Shelf
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_bay_housing.step", App.Placement(), "BayHousing", PA12_COLOR)
    )

    # 2. 0.8 mm 4-Layer Day 06 PCB seated on internal M2 bosses
    pcb_step = REPO_ROOT / "Day_06_Walking_Legs" / "build" / "board_only.step"
    if pcb_step.exists():
        pcb_shp = Part.read(str(pcb_step))
        bb = pcb_shp.BoundBox
        dx = -0.5 * (bb.XMin + bb.XMax)
        dy = 9.0 - (-109.5)
        dz = 2.6 - bb.ZMin
        pcb_obj = doc.addObject("Part::Feature", "Day06PCB")
        pcb_obj.Shape = pcb_shp
        pcb_obj.Placement = App.Placement(App.Vector(dx, dy, dz), App.Rotation())
        colored_objs.append((pcb_obj, PCB_COLOR))

    # 3. TWO Horizontal Metal-Gear MG90S Servos facing OUTBOARD (-Y)!
    servo_solids = []
    spline_solids = []
    for sx in (-SERVO_X_OFFSET_MM, SERVO_X_OFFSET_MM):
        body = Part.makeBox(12.2, 22.2, 22.8, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM, -7.4))
        ears = Part.makeBox(12.2, 2.2, 32.0, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM + 3.0, -12.0))
        boss = Part.makeCylinder(5.5, 2.0, App.Vector(sx, TURRET_FRONT_Y_MM, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
        servo_solids.append(body.fuse(ears).fuse(boss).removeSplitter())
        spline_solids.append(
            Part.makeCylinder(2.4, 3.8, App.Vector(sx, TURRET_FRONT_Y_MM - 2.0, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
        )

    servos_obj = doc.addObject("Part::Feature", "DualMG90SServos")
    servos_obj.Shape = servo_solids[0].fuse(servo_solids[1])
    colored_objs.append((servos_obj, SERVO_BODY_COLOR))

    splines_obj = doc.addObject("Part::Feature", "ServoBrassSplines")
    splines_obj.Shape = spline_solids[0].fuse(spline_solids[1])
    colored_objs.append((splines_obj, SERVO_CAP_COLOR))

    # 3b. Anodized Aluminum Servo Horns (`HipHorn` bolted to Femur + `KneeHorn` 12.0 mm crank arm)
    hip_end_x = kin["hx"] + 7.5 * (kin["kx"] - kin["hx"]) / FEMUR_LEN_MM
    hip_end_z = kin["hz"] + 7.5 * (kin["kz"] - kin["hz"]) / FEMUR_LEN_MM
    hip_horn = make_horn_solid(kin["hx"], kin["hz"], hip_end_x, hip_end_z, -23.1, 2.2, 5.2, 4.2)
    knee_horn = make_horn_solid(kin["sx"], kin["sz"], kin["crx"], kin["crz"], -23.1, 2.2, 5.2, 3.8)
    horns_obj = doc.addObject("Part::Feature", "AluminumServoHorns")
    horns_obj.Shape = hip_horn.fuse(knee_horn)
    colored_objs.append((horns_obj, HORN_COLOR))

    # 4. SendCutSend ALU-063 Overhead Double-Shear Bearing Outrigger (`leg_hip_bracket`)
    hip_br_pl = App.Placement(App.Vector(0.0, BRACKET_OUT_Y_MM, BRACKET_TOP_Z_MM), App.Rotation())
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_hip_bracket.step", hip_br_pl, "HipOutriggerBracket", ALU_COLOR)
    )

    # 5. Paired SendCutSend ALU-080 Femur Links (inboard + outboard double shear)
    for idx, y0 in enumerate((INBOARD_LINK_Y0_MM, OUTBOARD_LINK_Y0_MM)):
        f_pl = sagittal_link_placement(kin["hx"], kin["hz"], kin["kx"], kin["kz"], y0)
        assert (f_pl.multVec(App.Vector(FEMUR_LEN_MM, 0.0, 0.0)) - App.Vector(kin["kx"], y0, kin["kz"])).Length < 1e-3
        colored_objs.append(
            add_step_part(doc, MECH_DIR / "leg_femur_link.step", f_pl, f"FemurLink_{idx+1}", ALU_COLOR)
        )

    # 6. SendCutSend ALU-080 Knee Actuation Pushrod (`KneeHorn` crank pin -> Tibia bellcrank lobe)
    pr_pl = sagittal_link_placement(kin["crx"], kin["crz"], kin["bx"], kin["bz"], INBOARD_LINK_Y0_MM)
    assert (pr_pl.multVec(App.Vector(PUSHROD_LEN_MM, 0.0, 0.0)) - App.Vector(kin["bx"], INBOARD_LINK_Y0_MM, kin["bz"])).Length < 1e-3
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_knee_pushrod.step", pr_pl, "KneePushrod", PUSHROD_COLOR)
    )

    # 7. SendCutSend ALU-063 Sagittal Bellcrank Tibia Strut
    tibia_rot = rot_from_cols(
        (kin["ex_x"], 0.0, kin["ex_z"]),
        (0.0, kin["s_dir"], 0.0),
        (kin["uz_x"], 0.0, kin["uz_z"]),
    )
    knee_world = App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"])
    tibia_pos = knee_world - tibia_rot.multVec(App.Vector(10.5, 0.0, 53.0))
    tibia_pl = App.Placement(tibia_pos, tibia_rot)
    assert (tibia_pl.multVec(App.Vector(10.5, 0.0, 6.0)) - App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"])).Length < 1e-3
    assert (tibia_pl.multVec(App.Vector(22.5, 0.0, 53.0)) - App.Vector(kin["bx"], TIBIA_MID_Y_MM, kin["bz"])).Length < 1e-3
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_tibia_strut.step", tibia_pl, "TibiaStrut", ALU_COLOR)
    )

    # 8. FDM 95A TPU Compliant Arch Foot Pad (level with ground at Ankle)
    foot_pl = App.Placement(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"] - FOOT_BOLT_Z_MM), App.Rotation())
    colored_objs.append(
        add_step_part(doc, MECH_DIR / "leg_foot_pad.step", foot_pl, "CompliantFootPad", TPU_COLOR)
    )

    # 9. Stainless M3 Shoulder-Axle Pins (all along Y!)
    pin_hip = Part.makeCylinder(1.5, 23.5, App.Vector(kin["hx"], -23.0, kin["hz"]), App.Vector(0, -1, 0))
    pin_srv = Part.makeCylinder(1.5, 23.5, App.Vector(kin["sx"], -23.0, kin["sz"]), App.Vector(0, -1, 0))
    pin_crk = Part.makeCylinder(1.5, 5.5, App.Vector(kin["crx"], -22.8, kin["crz"]), App.Vector(0, -1, 0))
    pin_kne = Part.makeCylinder(1.5, 21.6, App.Vector(kin["kx"], -22.7, kin["kz"]), App.Vector(0, -1, 0))
    pin_bel = Part.makeCylinder(1.5, 21.6, App.Vector(kin["bx"], -22.7, kin["bz"]), App.Vector(0, -1, 0))
    pin_ank = Part.makeCylinder(1.5, 21.6, App.Vector(kin["ax"], -22.7, kin["az"]), App.Vector(0, -1, 0))
    pins_obj = doc.addObject("Part::Feature", "M3ShoulderPins")
    pins_obj.Shape = pin_hip.fuse(pin_srv).fuse(pin_crk).fuse(pin_kne).fuse(pin_bel).fuse(pin_ank)
    colored_objs.append((pins_obj, STEEL_PIN_COLOR))

    doc.recompute()

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
        f"hip=({kin['hx']:.2f}, {kin['hz']:.2f}), knee=({kin['kx']:.2f}, {kin['kz']:.2f}), "
        f"bellcrank=({kin['bx']:.2f}, {kin['bz']:.2f}), crank=({kin['crx']:.2f}, {kin['crz']:.2f}), "
        f"foot_ground_z={kin['az'] - FOOT_BOLT_Z_MM:.2f} mm\n"
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
