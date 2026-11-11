#!/usr/bin/env python3
"""
Full 3D Single-Flank Walking Module Assembly (`mech/walking_leg_assembly.py`)
for Day 06 Walking Legs — Rear-Set Expansion-Bay Plugs + Rigid `ALU-080` Flank Stance-Extension Rail.

Coordinate convention: mm, Z up, outboard facing -Y, under-chassis inboard facing +Y,
fore/aft laptop flank axis along +/-X (`X = -83.0 .. +83.0 mm`; `-X` toward laptop front palmrest,
`+X` toward laptop rear hinge when mounted on the Right flank).

Shows one complete laptop flank subassembly (`Right Flank` orientation):
  1. `mech/leg_flank_bracket.step` — SendCutSend `ALU-080` (`2.032 mm` / 12-gauge `5052-H32`)
     `166.0 x 57.0 x 12.5 mm` CNC-bent inverted U-channel stance-extension rail.
  2. 2x `mech/leg_bay_housing.step` + `Day_06_Walking_Legs` PCBs bolted at the **rear-set physical
     port positions** (`X_rail = +25.00 mm` Front Port & `X_rail = +65.60 mm` Rear Port, `40.60 mm`
     exact Framework Laptop 13 port pitch).
  3. Low-profile 4-wire servo ribbon harnesses routed forward along the top deck of `leg_flank_bracket`
     from the rear-set bay plugs into the Front (`X_rail = -56.00 mm`) & Rear (`X_rail = +56.00 mm`)
     servo pods.
  4. 2x `mech/leg_servo_pod.step` + 4x horizontal metal-gear `MG90S` servos (splines facing outboard `-Y`)
     + 2x complete 2-DOF sagittal 5-bar/bellcrank aluminum walking legs (`leg_hip_bracket`, paired
     `leg_femur_link`, `leg_knee_pushrod`, `leg_tibia_strut`, compliant `leg_foot_pad`).

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

# --- Named Sagittal Linkage & Flank Rail Parameters (mm) ---
SERVO_X_OFFSET_MM     = 9.00      # 18.00 mm center-to-center MG90S spacing
SERVO_AXIS_Z_MM       = 22.10     # Coaxial with `leg_hip_bracket` M3 axle holes (45.1002 - 23.00 = 22.1002 mm)
TURRET_FRONT_Y_MM     = -23.50    # Outboard front face of `leg_servo_pod` inside -24.50 mm rail
BRACKET_OUT_Y_MM      = -56.20    # Outboard outer face of `leg_hip_bracket`
BRACKET_TOP_Z_MM      = 45.1002   # Top outer face of `leg_hip_bracket` (43.50 + 1.6002)

TIBIA_MID_Y_MM        = -44.20    # Centerplane of 20.0 mm tibia channel (spans Y = -54.20 .. -34.20; 9.7 mm clear of rail!)
KNEE_HORN_Y0_MM       = -28.60    # Dedicated inboard knee crank plane (Y = -28.60 .. -30.60)
PUSHROD_Y0_MM         = -31.20    # Dedicated inboard pushrod plane (Y = -31.20 .. -33.232; 6.7 mm clear of rail!)
INBOARD_FEMUR_Y0_MM   = -36.40    # Inboard femur inside tibia channel (Y = -36.40 .. -38.432)
HIP_HORN_Y0_MM        = -38.55    # Hip drive hub inside open channel center (Y = -38.55 .. -40.55)
OUTBOARD_FEMUR_Y0_MM  = -49.968   # Outboard femur inside tibia channel (Y = -49.968 .. -52.000)

FEMUR_LEN_MM          = 38.0
TIBIA_LEN_MM          = 47.0
TIBIA_KNEE_Z_MM       = 54.5      # Matches `KNEE_Z_MM` in `leg_tibia_strut.py` (15.0 mm open clevis throat!)
HORN_CRANK_MM         = 12.0
PUSHROD_LEN_MM        = 36.0
FOOT_BOLT_Z_MM        = 22.5
ANKLE_TARGET_Z_MM     = -52.83    # Ground sole at Z = -75.33 mm (with 22.5 mm high-clearance foot tongue!)

FRONT_BAY_X_MM      = +25.00    # Framework 13 Front Port (+36.0 mm in laptop coords)
REAR_BAY_X_MM       = +65.60    # Framework 13 Rear Port (+76.6 mm in laptop coords; 40.60 mm port pitch)
FRONT_POD_X_MM      = -56.00    # Front Leg (-45.0 mm in laptop coords — 81.0 mm forward of Front Port!)
REAR_POD_X_MM       = +56.00    # Rear Leg (+67.0 mm in laptop coords — 112.0 mm balanced wheelbase!)

RAIL_ALU_COLOR      = (0.64, 0.70, 0.77)
PA12_COLOR          = (0.22, 0.23, 0.26)
ALU_COLOR           = (0.76, 0.79, 0.83)
PUSHROD_COLOR       = (0.60, 0.72, 0.84)
HORN_COLOR          = (0.85, 0.31, 0.21)
TPU_COLOR           = (0.14, 0.15, 0.17)
SERVO_BODY_COLOR    = (0.14, 0.30, 0.58)
SERVO_CAP_COLOR     = (0.82, 0.68, 0.22)
PCB_COLOR           = (0.14, 0.42, 0.22)
HARNESS_COLOR       = (0.18, 0.19, 0.21)
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
    dx = p2x - p1x
    dz = p2z - p1z
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    rot = rot_from_cols((ux, 0.0, uz), (-uz, 0.0, ux), (0.0, -1.0, 0.0))
    return App.Placement(App.Vector(p1x, y_start, p1z), rot)


def make_horn_solid(x1: float, z1: float, x2: float, z2: float, y_front: float, thick: float, r1: float, r2: float, bore_r: float = 1.55, hub_bore_r: float | None = None) -> Part.Shape:
    c1 = Part.makeCylinder(r1, thick, App.Vector(x1, y_front, z1), App.Vector(0, -1, 0))
    c2 = Part.makeCylinder(r2, thick, App.Vector(x2, y_front, z2), App.Vector(0, -1, 0))
    dx, dz = x2 - x1, z2 - z1
    dist = math.hypot(dx, dz)
    if dist < 1e-3:
        solid = c1
    else:
        nx, nz = -dz / dist, dx / dist
        rmid = min(r1, r2)
        pts = [
            App.Vector(x1 + nx * rmid, y_front, z1 + nz * rmid),
            App.Vector(x2 + nx * rmid, y_front, z2 + nz * rmid),
            App.Vector(x2 - nx * rmid, y_front, z2 - nz * rmid),
            App.Vector(x1 - nx * rmid, y_front, z1 - nz * rmid),
        ]
        web = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(App.Vector(0, -thick, 0))
        solid = c1.fuse(c2).fuse(web)
    rb1 = bore_r if hub_bore_r is None else hub_bore_r
    b1 = Part.makeCylinder(rb1, thick + 1.0, App.Vector(x1, y_front + 0.5, z1), App.Vector(0, -1, 0))
    b2 = Part.makeCylinder(bore_r, thick + 1.0, App.Vector(x2, y_front + 0.5, z2), App.Vector(0, -1, 0))
    return solid.cut(b1).cut(b2).removeSplitter()


def solve_leg_kinematics(ankle_x: float, ankle_z: float, s_dir: float = 1.0) -> dict[str, float]:
    hx = -SERVO_X_OFFSET_MM * s_dir
    hz = SERVO_AXIS_Z_MM
    sx = SERVO_X_OFFSET_MM * s_dir
    sz = SERVO_AXIS_Z_MM

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

    uz_x = (kx - ankle_x) / TIBIA_LEN_MM
    uz_z = (kz - ankle_z) / TIBIA_LEN_MM
    ex_x = s_dir * uz_z
    ex_z = -s_dir * uz_x
    bx = kx + 12.0 * ex_x
    bz = kz + 12.0 * ex_z

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


def build():
    doc = App.newDocument("walking_leg_assembly")
    colored_objs: list[tuple[object, tuple[float, float, float]]] = []

    shp_rail    = Part.read(str(MECH_DIR / "leg_flank_bracket.step"))
    shp_bay     = Part.read(str(MECH_DIR / "leg_bay_housing.step"))
    shp_pod     = Part.read(str(MECH_DIR / "leg_servo_pod.step"))
    shp_bracket = Part.read(str(MECH_DIR / "leg_hip_bracket.step"))
    shp_femur   = Part.read(str(MECH_DIR / "leg_femur_link.step"))
    shp_pushrod = Part.read(str(MECH_DIR / "leg_knee_pushrod.step"))
    shp_tibia   = Part.read(str(MECH_DIR / "leg_tibia_strut.step"))
    shp_foot    = Part.read(str(MECH_DIR / "leg_foot_pad.step"))
    pcb_step    = REPO_ROOT / "Day_06_Walking_Legs" / "build" / "board_only.step"
    shp_pcb     = Part.read(str(pcb_step)) if pcb_step.exists() else None

    def add_obj(name: str, shp: Part.Shape, pl: App.Placement, col: tuple[float, float, float]):
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        o.Placement = pl
        colored_objs.append((o, col))

    # 1. Rigid SendCutSend ALU-080 (`2.032 mm`) Flank Stance-Extension Rail
    add_obj("FlankStanceBracket", shp_rail, App.Placement(), RAIL_ALU_COLOR)

    # 2. Two Rear-Set `leg_bay_housing` Expansion Card Plugs (`X = +25.00` Front Port & `X = +65.60` Rear Port)
    for btag, bx in (("FrontPort", FRONT_BAY_X_MM), ("RearPort", REAR_BAY_X_MM)):
        bay_pl = App.Placement(App.Vector(bx, 0.0, 0.0), App.Rotation())
        add_obj(f"{btag}_BayHousing", shp_bay, bay_pl, PA12_COLOR)
        if shp_pcb is not None:
            bb = shp_pcb.BoundBox
            pcb_pl = App.Placement(
                App.Vector(bx - 0.5 * (bb.XMin + bb.XMax), 17.8 - 0.5 * (bb.YMin + bb.YMax), 3.22 - bb.ZMin),
                App.Rotation(),
            )
            add_obj(f"{btag}_Day06PCB", shp_pcb, pcb_pl, PCB_COLOR)

    # 3. Low-profile sleeved 4-wire servo harnesses running forward along rail deck (`Z = 0.05 .. 1.65 mm`)
    h_front_run = Part.makeBox(81.0, 3.6, 1.6, App.Vector(FRONT_POD_X_MM, -1.0, 0.05))
    h_front_in  = Part.makeBox(4.8, 4.2, 1.6, App.Vector(FRONT_BAY_X_MM - 2.4, -1.0, 0.05))
    h_rear_run  = Part.makeBox(12.0, 3.6, 1.6, App.Vector(REAR_POD_X_MM, -1.0, 0.05))
    h_rear_in   = Part.makeBox(4.8, 4.2, 1.6, App.Vector(REAR_BAY_X_MM - 2.4, -1.0, 0.05))
    harness_shp = h_front_run.fuse([h_front_in, h_rear_run, h_rear_in]).removeSplitter()
    add_obj("ServoHarnesses", harness_shp, App.Placement(), HARNESS_COLOR)

    # 4. Front (`X = -56.0`) & Rear (`X = +56.0`) Modular Servo Pods + 2-DOF Sagittal Legs
    for ltag, pod_x, stride_dx in (("FrontLeg", FRONT_POD_X_MM, +8.0), ("RearLeg", REAR_POD_X_MM, -8.0)):
        pod_pl = App.Placement(App.Vector(pod_x, 0.0, 0.0), App.Rotation())
        kin = solve_leg_kinematics(stride_dx, ANKLE_TARGET_Z_MM, s_dir=1.0)

        add_obj(f"{ltag}_ServoPod", shp_pod, pod_pl, PA12_COLOR)

        s_list, sp_list = [], []
        for sx in (-SERVO_X_OFFSET_MM, SERVO_X_OFFSET_MM):
            rear_case  = Part.makeBox(12.2, 19.0, 22.8, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM + 0.10, 5.20))
            ears       = Part.makeBox(12.2, 2.2, 31.2, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM - 2.25, 1.00))
            front_case = Part.makeBox(12.2, 2.2, 22.8, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM - 4.45, 5.20))
            boss       = Part.makeCylinder(5.5, 0.6, App.Vector(sx, TURRET_FRONT_Y_MM - 4.45, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            srv_bore   = Part.makeCylinder(1.55, 10.0, App.Vector(sx, TURRET_FRONT_Y_MM - 2.0, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            s_list.append(rear_case.fuse([ears, front_case, boss]).cut(srv_bore).removeSplitter())
            sp_cyl     = Part.makeCylinder(2.5, 2.4, App.Vector(sx, TURRET_FRONT_Y_MM - 5.05, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            sp_bore    = Part.makeCylinder(1.55, 4.0, App.Vector(sx, TURRET_FRONT_Y_MM - 4.5, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            sp_list.append(sp_cyl.cut(sp_bore).removeSplitter())
        add_obj(f"{ltag}_Servos", s_list[0].fuse(s_list[1]), pod_pl, SERVO_BODY_COLOR)
        add_obj(f"{ltag}_Splines", sp_list[0].fuse(sp_list[1]), pod_pl, SERVO_CAP_COLOR)

        hip_end_x = kin["hx"] + 7.5 * (kin["kx"] - kin["hx"]) / FEMUR_LEN_MM
        hip_end_z = kin["hz"] + 7.5 * (kin["kz"] - kin["hz"]) / FEMUR_LEN_MM
        h1 = make_horn_solid(kin["hx"], kin["hz"], hip_end_x, hip_end_z, HIP_HORN_Y0_MM, 2.0, 5.2, 4.2, hub_bore_r=1.55)
        h2 = make_horn_solid(kin["sx"], kin["sz"], kin["crx"], kin["crz"], KNEE_HORN_Y0_MM, 2.0, 5.2, 3.8, hub_bore_r=2.52)
        add_obj(f"{ltag}_Horns", h1.fuse(h2), pod_pl, HORN_COLOR)

        br_pl = pod_pl.multiply(App.Placement(App.Vector(0.0, BRACKET_OUT_Y_MM, BRACKET_TOP_Z_MM), App.Rotation()))
        add_obj(f"{ltag}_HipBracket", shp_bracket, br_pl, ALU_COLOR)

        for idx, y0 in enumerate((INBOARD_FEMUR_Y0_MM, OUTBOARD_FEMUR_Y0_MM)):
            f_pl = pod_pl.multiply(sagittal_link_placement(kin["hx"], kin["hz"], kin["kx"], kin["kz"], y0))
            add_obj(f"{ltag}_Femur_{idx+1}", shp_femur, f_pl, ALU_COLOR)

        pr_pl = pod_pl.multiply(sagittal_link_placement(kin["crx"], kin["crz"], kin["bx"], kin["bz"], PUSHROD_Y0_MM))
        add_obj(f"{ltag}_Pushrod", shp_pushrod, pr_pl, PUSHROD_COLOR)

        tibia_rot = rot_from_cols(
            (kin["ex_x"], 0.0, kin["ex_z"]),
            (0.0, kin["s_dir"], 0.0),
            (kin["uz_x"], 0.0, kin["uz_z"]),
        )
        tibia_pos = App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"]) - tibia_rot.multVec(App.Vector(10.5, 0.0, TIBIA_KNEE_Z_MM))
        add_obj(f"{ltag}_Tibia", shp_tibia, pod_pl.multiply(App.Placement(tibia_pos, tibia_rot)), ALU_COLOR)

        foot_pl = pod_pl.multiply(App.Placement(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"] - FOOT_BOLT_Z_MM), App.Rotation()))
        add_obj(f"{ltag}_FootPad", shp_foot, foot_pl, TPU_COLOR)

        p_hip = Part.makeCylinder(1.48, 28.9, App.Vector(kin["hx"], -27.5, kin["hz"]), App.Vector(0, -1, 0))
        p_srv = Part.makeCylinder(1.48, 3.25, App.Vector(kin["sx"], -27.5, kin["sz"]), App.Vector(0, -1, 0))
        p_crk = Part.makeCylinder(1.48, 4.90, App.Vector(kin["crx"], -28.5, kin["crz"]), App.Vector(0, -1, 0))
        p_kne = Part.makeCylinder(1.48, 20.4, App.Vector(kin["kx"], -34.0, kin["kz"]), App.Vector(0, -1, 0))
        p_bel = Part.makeCylinder(1.48, 23.4, App.Vector(kin["bx"], -31.0, kin["bz"]), App.Vector(0, -1, 0))
        p_ank = Part.makeCylinder(1.48, 20.4, App.Vector(kin["ax"], -34.0, kin["az"]), App.Vector(0, -1, 0))
        add_obj(f"{ltag}_Pins", p_hip.fuse([p_srv, p_crk, p_kne, p_bel, p_ank]), pod_pl, STEEL_PIN_COLOR)

    # M2.5 rail fasteners under active bays (+25, +65.6) & active pods (-56, +56)
    bolts = []
    for cx in (FRONT_BAY_X_MM, REAR_BAY_X_MM):
        for dx in (-7.0, 7.0):
            for hy in (8.5, 14.5):
                bolts.append(Part.makeCylinder(2.2, 1.4, App.Vector(cx + dx, hy, -2.06), App.Vector(0, 0, -1)))
    for cx in (FRONT_POD_X_MM, REAR_POD_X_MM):
        for dx in (-7.0, 7.0):
            for hy in (-8.5, -14.5):
                bolts.append(Part.makeCylinder(2.2, 1.4, App.Vector(cx + dx, hy, -2.06), App.Vector(0, 0, -1)))
    bolt_shp = bolts[0].fuse(bolts[1:])
    add_obj("RailFastenersM25", bolt_shp, App.Placement(), STEEL_PIN_COLOR)

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
        f"bays=({FRONT_BAY_X_MM:.1f}, {REAR_BAY_X_MM:.1f} mm), "
        f"pods=({FRONT_POD_X_MM:.1f}, {REAR_POD_X_MM:.1f} mm), "
        f"foot_ground_z={ANKLE_TARGET_Z_MM - FOOT_BOLT_Z_MM:.2f} mm\n"
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
