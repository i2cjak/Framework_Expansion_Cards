#!/usr/bin/env python3
"""
Full Walk-Ready 4-Leg Framework Laptop 13 Quadruped Assembly (`mech/walking_laptop_assembly.py`)
for Day 06 Walking Legs — featuring Recessed Underside Expansion Bays & Rigid `ALU-080` Metal Outriggers.

Coordinate convention: mm, Z up, laptop front (palmrest edge) facing -Y, laptop rear (hinge) facing +Y,
upper palmrest outer rim at X = +/-148.3 mm, recessed underside bay mouths at X = +/-141.3 mm (inset 7.0 mm
beneath the beveled overhanging chassis skirt, matching the real Framework Laptop 13 bottom cover!),
laptop flat bottom cover at Z = 0.0 mm.

Models:
  1. **Framework Laptop 13** (`296.6 x 228.9 x 14.8 mm` CNC-anodized aluminum chassis with tapered
     lower beveled skirt, 4 underside-open recessed expansion card bays + latch release buttons,
     keyboard island, glass touchpad, hinge barrel, and `106 deg` open `13.5"` `3:2` display).
  2. **Four complete 2-DOF sagittal walking leg modules (`FL`, `FR`, `RL`, `RR`)** anchored in the
     recessed underside bays, with **`ALU-080` (2.032 mm / 12-gauge) 5052-H32 aluminum `leg_bay_outrigger`
     cantilever spines** bearing 17.5 mm under the solid aluminum bottom cover and extending the dual
     servo cradles 16 mm outboard completely clear of the overhanging laptop palmrest lip.
  3. Poised walk-ready diagonal trot stance (`+/-8 mm` stride offset; all 4 compliant TPU feet planted
     on the `Z = -75.33 mm` ground plane).

Exports:
  - mech/walking_laptop_assembly.FCStd
  - mech/walking_laptop_assembly.step (full multi-color walk-ready laptop assembly)
"""
import math
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Framework Laptop 13 Dimensions (mm) ---
LAPTOP_W_X_MM       = 296.6    # Upper palmrest width (-148.3 .. +148.3)
LAPTOP_D_Y_MM       = 228.9    # Depth (-114.45 .. +114.45)
BASE_H_Z_MM         = 14.8     # Total aluminum base thickness (Z = 0.0 .. +14.8)
SKIRT_H_Z_MM        = 6.8      # Underside expansion-card bay & beveled lower-cover height (Z = 0.0 .. +6.8)
BAY_RECESS_X_MM     = 7.0      # Recessed bay mouth inset from +/-148.3 upper rim -> X_bay_mouth = +/-141.3 mm!
LID_THICK_MM        = 4.2
LID_OPEN_DEG        = 106.0
BAY_FRONT_Y_MM      = -58.0
BAY_REAR_Y_MM       = 54.0
BAY_SEAT_Z_MM       = 0.0      # `leg_bay_outrigger` top face (Z_leg = 0) bears flush against laptop bottom Z = 0!

# --- Shared Leg Kinematic Constants (matching `walking_leg_assembly.py`) ---
SERVO_X_OFFSET_MM   = 7.30
SERVO_AXIS_Z_MM     = 22.10
TURRET_FRONT_Y_MM   = -37.50
BRACKET_OUT_Y_MM    = -61.60
BRACKET_TOP_Z_MM    = 34.7002
TIBIA_MID_Y_MM      = -49.50
INBOARD_LINK_Y0_MM  = -41.30
OUTBOARD_LINK_Y0_MM = -55.668
FEMUR_LEN_MM        = 38.0
TIBIA_LEN_MM        = 47.0
HORN_CRANK_MM       = 12.0
PUSHROD_LEN_MM      = 36.0
FOOT_BOLT_Z_MM      = 19.0
ANKLE_LEG_Z_MM      = -56.33   # Ground plane in laptop coords = -56.33 - 19.0 = -75.33 mm!

# Colors
CHASSIS_ALU_COLOR   = (0.79, 0.81, 0.84)
BEZEL_DARK_COLOR    = (0.13, 0.14, 0.16)
SCREEN_GLOW_COLOR   = (0.14, 0.24, 0.36)
TRACKPAD_COLOR      = (0.68, 0.70, 0.73)
ACCENT_ORANGE_COLOR = (0.92, 0.38, 0.14)
SPINE_ALU_COLOR     = (0.64, 0.69, 0.76)
PA12_COLOR          = (0.22, 0.23, 0.26)
ALU_LINK_COLOR      = (0.76, 0.79, 0.83)
PUSHROD_COLOR       = (0.60, 0.72, 0.84)
HORN_COLOR          = (0.85, 0.31, 0.21)
TPU_COLOR           = (0.15, 0.16, 0.18)
SERVO_BODY_COLOR    = (0.14, 0.30, 0.58)
SERVO_CAP_COLOR     = (0.82, 0.68, 0.22)
PCB_COLOR           = (0.14, 0.42, 0.22)
STEEL_PIN_COLOR     = (0.85, 0.87, 0.90)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
LOG_PATH  = MECH_DIR / "walking_laptop_assembly.log"


def rot_from_cols(c1, c2, c3) -> App.Rotation:
    m = App.Matrix()
    m.A11, m.A21, m.A31 = c1
    m.A12, m.A22, m.A32 = c2
    m.A13, m.A23, m.A33 = c3
    return App.Rotation(m)


def sagittal_link_placement(p1x: float, p1z: float, p2x: float, p2z: float, y_start: float) -> App.Placement:
    dx, dz = p2x - p1x, p2z - p1z
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    rot = rot_from_cols((ux, 0.0, uz), (-uz, 0.0, ux), (0.0, -1.0, 0.0))
    return App.Placement(App.Vector(p1x, y_start, p1z), rot)


def make_horn_solid(x1: float, z1: float, x2: float, z2: float, y_front: float, thick: float, r1: float, r2: float) -> Part.Shape:
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


def solve_leg_kinematics(ankle_x: float, ankle_z: float, s_dir: float) -> dict[str, float]:
    hx = -SERVO_X_OFFSET_MM * s_dir
    hz = SERVO_AXIS_Z_MM
    sx = SERVO_X_OFFSET_MM * s_dir
    sz = SERVO_AXIS_Z_MM

    dx, dz = ankle_x - hx, ankle_z - hz
    d = math.hypot(dx, dz)
    a_proj = (FEMUR_LEN_MM**2 - TIBIA_LEN_MM**2 + d**2) / (2.0 * d)
    h_perp = math.sqrt(max(0.0, FEMUR_LEN_MM**2 - a_proj**2))
    mx, mz = hx + a_proj * (dx / d), hz + a_proj * (dz / d)
    kx = mx + s_dir * h_perp * (-dz / d)
    kz = mz + s_dir * h_perp * (dx / d)

    uz_x, uz_z = (kx - ankle_x) / TIBIA_LEN_MM, (kz - ankle_z) / TIBIA_LEN_MM
    ex_x, ex_z = s_dir * uz_z, -s_dir * uz_x
    bx, bz = kx + 12.0 * ex_x, kz + 12.0 * ex_z

    ddx, ddz = bx - sx, bz - sz
    dd = math.hypot(ddx, ddz)
    ca = (HORN_CRANK_MM**2 - PUSHROD_LEN_MM**2 + dd**2) / (2.0 * dd)
    ch = math.sqrt(max(0.0, HORN_CRANK_MM**2 - ca**2))
    cmx, cmz = sx + ca * (ddx / dd), sz + ca * (ddz / dd)
    crx = cmx + s_dir * ch * (-ddz / dd)
    crz = cmz + s_dir * ch * (ddx / dd)

    return {
        "hx": hx, "hz": hz, "sx": sx, "sz": sz,
        "kx": kx, "kz": kz, "ax": ankle_x, "az": ankle_z,
        "bx": bx, "bz": bz, "crx": crx, "crz": crz,
        "uz_x": uz_x, "uz_z": uz_z, "ex_x": ex_x, "ex_z": ex_z,
        "s_dir": s_dir,
    }


def make_rounded_rect_face(w: float, d: float, r: float, x0: float, y0: float, z0: float) -> Part.Wire:
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
    return Part.Wire(edges)


def make_rounded_rect_prism(w: float, d: float, h: float, r: float, x0: float, y0: float, z0: float) -> Part.Shape:
    return Part.Face(make_rounded_rect_face(w, d, r, x0, y0, z0)).extrude(App.Vector(0, 0, h))


def build_framework_laptop(doc, colored_objs: list) -> None:
    """Models a Framework Laptop 13 with authentic beveled lower skirt & underside-open recessed bays."""
    hw = LAPTOP_W_X_MM * 0.5
    hd = LAPTOP_D_Y_MM * 0.5
    bot_hw = hw - BAY_RECESS_X_MM   # +/-141.3 mm lower flat cover width (7.0 mm side bevel inset!)
    bot_hd = hd - 3.5               # Subtle front/rear lower bevel

    # 1. Lofted lower beveled bottom-cover tub (Z = 0.0 .. +6.8 mm) + upper palmrest deck (Z = +6.8 .. +14.8 mm)
    w_bot = make_rounded_rect_face(bot_hw * 2.0, bot_hd * 2.0, 6.0, -bot_hw, -bot_hd, 0.0)
    w_mid = make_rounded_rect_face(LAPTOP_W_X_MM, LAPTOP_D_Y_MM, 7.5, -hw, -hd, SKIRT_H_Z_MM)
    lower_tub = Part.makeLoft([w_bot, w_mid], True, True)
    upper_deck = make_rounded_rect_prism(LAPTOP_W_X_MM, LAPTOP_D_Y_MM, BASE_H_Z_MM - SKIRT_H_Z_MM, 7.5, -hw, -hd, SKIRT_H_Z_MM)
    base = lower_tub.fuse(upper_deck).removeSplitter()

    # Cut 4 UNDERSIDE-OPEN recessed expansion bays (Z = -0.5 .. +6.85 mm, recessed under upper_deck!)
    # Left bays: X = -hw .. -(bot_hw - 26.8) = -148.3 .. -114.5 mm
    # Right bays: X = (bot_hw - 26.8) .. +hw = +114.5 .. +148.3 mm
    for bx_sign in (-1.0, 1.0):
        for by_center in (BAY_FRONT_Y_MM, BAY_REAR_Y_MM):
            if bx_sign < 0:
                x0 = -hw - 1.0
                w_cut = (hw - (bot_hw - 26.8)) + 1.0
            else:
                x0 = bot_hw - 26.8
                w_cut = (hw - (bot_hw - 26.8)) + 1.0
            bay_pocket = Part.makeBox(w_cut, 30.4, SKIRT_H_Z_MM + 0.55, App.Vector(x0, by_center - 15.2, -0.5))
            # T-rail side grooves inside bay pocket (32.4 mm span along Y, Z = 2.2 .. 4.4 mm)
            rail_groove = Part.makeBox(w_cut - 1.5 if bx_sign > 0 else w_cut - 1.5, 32.4, 2.2,
                                       App.Vector(x0 if bx_sign > 0 else x0 + 1.5, by_center - 16.2, 2.2))
            base = base.cut(bay_pocket).cut(rail_groove)

    kb_well = Part.makeBox(262.0, 102.0, 1.4, App.Vector(-131.0, -8.0, BASE_H_Z_MM - 1.3))
    base = base.cut(kb_well).removeSplitter()

    base_obj = doc.addObject("Part::Feature", "Framework13_BaseChassis")
    base_obj.Shape = base
    colored_objs.append((base_obj, CHASSIS_ALU_COLOR))

    # 2. Keyboard Matrix + Hinge + Touchpad + Underside Rubber Feet Strips
    kb_plate = Part.makeBox(258.0, 98.0, 1.3, App.Vector(-129.0, -6.0, BASE_H_Z_MM - 1.1))
    hinge_barrel = Part.makeCylinder(4.8, 236.0, App.Vector(-118.0, hd - 6.0, BASE_H_Z_MM - 1.0), App.Vector(1, 0, 0))
    kb_obj = doc.addObject("Part::Feature", "Framework13_KeyboardAndHinge")
    kb_obj.Shape = kb_plate.fuse(hinge_barrel)
    colored_objs.append((kb_obj, BEZEL_DARK_COLOR))

    tp_obj = doc.addObject("Part::Feature", "Framework13_Touchpad")
    tp_obj.Shape = Part.makeBox(115.0, 76.0, 0.4, App.Vector(-57.5, -96.0, BASE_H_Z_MM - 0.2))
    colored_objs.append((tp_obj, TRACKPAD_COLOR))

    # 3. Open 13.5" 3:2 Display Lid
    hinge_pt = App.Vector(0.0, hd - 6.0, BASE_H_Z_MM + 1.0)
    lid_rot = App.Rotation(App.Vector(1, 0, 0), -LID_OPEN_DEG)
    lid_pl = App.Placement(hinge_pt, lid_rot)

    lid_shell = make_rounded_rect_prism(LAPTOP_W_X_MM, LAPTOP_D_Y_MM - 4.0, LID_THICK_MM, 7.5, -hw, -(LAPTOP_D_Y_MM - 4.0), 0.0)
    lid_obj = doc.addObject("Part::Feature", "Framework13_DisplayLid")
    lid_obj.Shape = lid_shell
    lid_obj.Placement = lid_pl
    colored_objs.append((lid_obj, CHASSIS_ALU_COLOR))

    bezel = Part.makeBox(LAPTOP_W_X_MM - 4.0, LAPTOP_D_Y_MM - 8.0, 0.5, App.Vector(-(hw - 2.0), -(LAPTOP_D_Y_MM - 6.0), -0.4))
    bezel_obj = doc.addObject("Part::Feature", "Framework13_DisplayBezel")
    bezel_obj.Shape = bezel
    bezel_obj.Placement = lid_pl
    colored_objs.append((bezel_obj, BEZEL_DARK_COLOR))

    screen = Part.makeBox(280.0, 188.0, 0.6, App.Vector(-140.0, -(LAPTOP_D_Y_MM - 20.0), -0.55))
    screen_obj = doc.addObject("Part::Feature", "Framework13_ScreenPanel")
    screen_obj.Shape = screen
    screen_obj.Placement = lid_pl
    colored_objs.append((screen_obj, SCREEN_GLOW_COLOR))

    cam_accent = Part.makeBox(8.0, 2.5, 0.7, App.Vector(-4.0, -(LAPTOP_D_Y_MM - 9.0), -0.6))
    accent_obj = doc.addObject("Part::Feature", "Framework13_CameraAccent")
    accent_obj.Shape = cam_accent
    accent_obj.Placement = lid_pl
    colored_objs.append((accent_obj, ACCENT_ORANGE_COLOR))


def build():
    doc = App.newDocument("walking_laptop_assembly")
    colored_objs: list[tuple[object, tuple[float, float, float]]] = []

    shp_spine   = Part.read(str(MECH_DIR / "leg_bay_outrigger.step"))
    shp_housing = Part.read(str(MECH_DIR / "leg_bay_housing.step"))
    shp_bracket = Part.read(str(MECH_DIR / "leg_hip_bracket.step"))
    shp_femur   = Part.read(str(MECH_DIR / "leg_femur_link.step"))
    shp_pushrod = Part.read(str(MECH_DIR / "leg_knee_pushrod.step"))
    shp_tibia   = Part.read(str(MECH_DIR / "leg_tibia_strut.step"))
    shp_foot    = Part.read(str(MECH_DIR / "leg_foot_pad.step"))
    pcb_path    = REPO_ROOT / "Day_06_Walking_Legs" / "build" / "board_only.step"
    shp_pcb     = Part.read(str(pcb_path)) if pcb_path.exists() else None

    build_framework_laptop(doc, colored_objs)

    # Recessed bay mouths sit at X = +/-141.3 mm (7.0 mm inset from +/-148.3 mm upper palmrest edge!)
    bay_mouth_x = LAPTOP_W_X_MM * 0.5 - BAY_RECESS_X_MM   # 141.3 mm
    bays = [
        ("FL", -bay_mouth_x, BAY_FRONT_Y_MM, -90.0, -1.0, -8.0),
        ("RL", -bay_mouth_x, BAY_REAR_Y_MM,  -90.0, -1.0, +8.0),
        ("FR", +bay_mouth_x, BAY_FRONT_Y_MM, +90.0, +1.0, +8.0),
        ("RR", +bay_mouth_x, BAY_REAR_Y_MM,  +90.0, +1.0, -8.0),
    ]

    def add_placed(name: str, shp: Part.Shape, bay_pl: App.Placement, local_pl: App.Placement, col):
        obj = doc.addObject("Part::Feature", name)
        obj.Shape = shp
        obj.Placement = bay_pl.multiply(local_pl)
        colored_objs.append((obj, col))

    for tag, bx_world, by_world, rz_deg, s_dir, world_dy_stride in bays:
        bay_pl = App.Placement(App.Vector(bx_world, by_world, BAY_SEAT_Z_MM), App.Rotation(App.Vector(0, 0, 1), rz_deg))
        ankle_x_local = -world_dy_stride if rz_deg < 0 else world_dy_stride
        kin = solve_leg_kinematics(ankle_x_local, ANKLE_LEG_Z_MM, s_dir=s_dir)

        w_hip = bay_pl.multVec(App.Vector(kin["hx"], TIBIA_MID_Y_MM, kin["hz"]))
        w_kne = bay_pl.multVec(App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"]))
        w_ank = bay_pl.multVec(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"]))
        assert w_hip.y < w_kne.y, f"{tag}: expected Hip ({w_hip.y:.1f}) forward of Knee ({w_kne.y:.1f})!"
        assert abs(w_ank.y - (by_world + world_dy_stride)) < 1e-2

        # Verify outboard servo saddle clears the +/-148.3 mm overhanging upper palmrest rim!
        w_saddle_back = bay_pl.multVec(App.Vector(0.0, -12.0, 20.0))
        assert abs(w_saddle_back.x) > LAPTOP_W_X_MM * 0.5 + 4.0, f"{tag}: saddle not clear of chassis! x={w_saddle_back.x}"

        add_placed(f"{tag}_BayOutrigger", shp_spine, bay_pl, App.Placement(), SPINE_ALU_COLOR)
        add_placed(f"{tag}_BayHousing", shp_housing, bay_pl, App.Placement(), PA12_COLOR)

        if shp_pcb is not None:
            bb = shp_pcb.BoundBox
            pcb_pl = App.Placement(App.Vector(-0.5 * (bb.XMin + bb.XMax), 9.0 - (-109.5), 2.8 - bb.ZMin), App.Rotation())
            add_placed(f"{tag}_Day06PCB", shp_pcb, bay_pl, pcb_pl, PCB_COLOR)

        s_list, sp_list = [], []
        for sx in (-SERVO_X_OFFSET_MM, SERVO_X_OFFSET_MM):
            body = Part.makeBox(12.2, 22.2, 22.8, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM, 5.20))
            ears = Part.makeBox(12.2, 2.2, 32.0, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM + 3.0, 0.60))
            boss = Part.makeCylinder(5.5, 2.0, App.Vector(sx, TURRET_FRONT_Y_MM, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            s_list.append(body.fuse(ears).fuse(boss).removeSplitter())
            sp_list.append(Part.makeCylinder(2.4, 3.8, App.Vector(sx, TURRET_FRONT_Y_MM - 2.0, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0)))

        add_placed(f"{tag}_Servos", s_list[0].fuse(s_list[1]), bay_pl, App.Placement(), SERVO_BODY_COLOR)
        add_placed(f"{tag}_Splines", sp_list[0].fuse(sp_list[1]), bay_pl, App.Placement(), SERVO_CAP_COLOR)

        horn_y = TURRET_FRONT_Y_MM - 1.60
        hip_end_x = kin["hx"] + 7.5 * (kin["kx"] - kin["hx"]) / FEMUR_LEN_MM
        hip_end_z = kin["hz"] + 7.5 * (kin["kz"] - kin["hz"]) / FEMUR_LEN_MM
        h1 = make_horn_solid(kin["hx"], kin["hz"], hip_end_x, hip_end_z, horn_y, 2.2, 5.2, 4.2)
        h2 = make_horn_solid(kin["sx"], kin["sz"], kin["crx"], kin["crz"], horn_y, 2.2, 5.2, 3.8)
        add_placed(f"{tag}_Horns", h1.fuse(h2), bay_pl, App.Placement(), HORN_COLOR)

        br_pl = App.Placement(App.Vector(0.0, BRACKET_OUT_Y_MM, BRACKET_TOP_Z_MM), App.Rotation())
        add_placed(f"{tag}_HipBracket", shp_bracket, bay_pl, br_pl, ALU_LINK_COLOR)

        for idx, y0 in enumerate((INBOARD_LINK_Y0_MM, OUTBOARD_LINK_Y0_MM)):
            f_pl = sagittal_link_placement(kin["hx"], kin["hz"], kin["kx"], kin["kz"], y0)
            add_placed(f"{tag}_Femur_{idx+1}", shp_femur, bay_pl, f_pl, ALU_LINK_COLOR)

        pr_pl = sagittal_link_placement(kin["crx"], kin["crz"], kin["bx"], kin["bz"], INBOARD_LINK_Y0_MM)
        add_placed(f"{tag}_Pushrod", shp_pushrod, bay_pl, pr_pl, PUSHROD_COLOR)

        tibia_rot = rot_from_cols(
            (kin["ex_x"], 0.0, kin["ex_z"]),
            (0.0, kin["s_dir"], 0.0),
            (kin["uz_x"], 0.0, kin["uz_z"]),
        )
        tibia_pos = App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"]) - tibia_rot.multVec(App.Vector(10.5, 0.0, 53.0))
        add_placed(f"{tag}_Tibia", shp_tibia, bay_pl, App.Placement(tibia_pos, tibia_rot), ALU_LINK_COLOR)

        foot_pl = App.Placement(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"] - FOOT_BOLT_Z_MM), App.Rotation())
        add_placed(f"{tag}_FootPad", shp_foot, bay_pl, foot_pl, TPU_COLOR)

        p_hip = Part.makeCylinder(1.5, 23.5, App.Vector(kin["hx"], -39.0, kin["hz"]), App.Vector(0, -1, 0))
        p_srv = Part.makeCylinder(1.5, 23.5, App.Vector(kin["sx"], -39.0, kin["sz"]), App.Vector(0, -1, 0))
        p_crk = Part.makeCylinder(1.5, 5.5, App.Vector(kin["crx"], -38.8, kin["crz"]), App.Vector(0, -1, 0))
        p_kne = Part.makeCylinder(1.5, 21.6, App.Vector(kin["kx"], -38.7, kin["kz"]), App.Vector(0, -1, 0))
        p_bel = Part.makeCylinder(1.5, 21.6, App.Vector(kin["bx"], -38.7, kin["bz"]), App.Vector(0, -1, 0))
        p_ank = Part.makeCylinder(1.5, 21.6, App.Vector(kin["ax"], -38.7, kin["az"]), App.Vector(0, -1, 0))
        hw_shp = p_hip.fuse(p_srv).fuse(p_crk).fuse(p_kne).fuse(p_bel).fuse(p_ank)
        for by in (5.00, 19.00, -18.00, -31.00):
            for bx in (-7.00, 7.00):
                hw_shp = hw_shp.fuse(Part.makeCylinder(2.2, 1.4, App.Vector(bx, by, -2.032), App.Vector(0, 0, -1)))
        add_placed(f"{tag}_Pins", hw_shp, bay_pl, App.Placement(), STEEL_PIN_COLOR)

    doc.recompute()

    for obj, _ in colored_objs:
        assert obj.Shape.isValid(), f"Invalid shape on {obj.Name}"

    doc.saveAs(str(MECH_DIR / "walking_laptop_assembly.FCStd"))
    step_out = MECH_DIR / "walking_laptop_assembly.step"
    export_objs = [obj for obj, _ in colored_objs]

    if App.GuiUp:
        import ImportGui
        for obj, col in colored_objs:
            obj.ViewObject.ShapeColor = col
        ImportGui.export(export_objs, str(step_out))
    else:
        Part.export(export_objs, str(step_out))

    LOG_PATH.write_text(
        f"OK: walking_laptop_assembly.step exported ({len(doc.Objects)} objects), "
        f"ground_z={BAY_SEAT_Z_MM + ANKLE_LEG_Z_MM - FOOT_BOLT_Z_MM:.2f} mm\n"
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
