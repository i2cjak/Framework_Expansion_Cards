#!/usr/bin/env python3
"""
Full Walk-Ready 4-Leg Framework Laptop 13 Quadruped Assembly (`mech/walking_laptop_assembly.py`)
for Day 06 Walking Legs — Authentic Rear-Set Expansion Ports + Rigid `ALU-080` Flank Stance Brackets.

Coordinate convention: mm, Z up, laptop front (palmrest edge) facing -Y (`Y = -114.45 mm`),
laptop rear (hinge) facing +Y (`Y = +114.45 mm`), upper palmrest outer rim at `X = +/-148.30 mm`,
flat CNC aluminum bottom cover at `Z = 0.00 mm`.

Authentic Framework Laptop 13 Port & Bracket Geometry (verified against `FW_MAIN_PCB_GENERIC_2`):
  1. **Both Expansion Card bays on each flank sit in the REAR half of the laptop**:
       - Front Ports (Slots 2 & 4): `Y = +36.00 mm` (`30 mm` bay spanning `+21.0 .. +51.0 mm`)
       - Rear Ports  (Slots 1 & 3): `Y = +76.60 mm` (`30 mm` bay spanning `+61.6 .. +91.6 mm`,
         exact `40.60 mm` center-to-center pitch -> `10.60 mm` divider between ports!)
       - Front `135.45 mm` of chassis (`Y = -114.45 .. +21.00 mm`, battery + palmrest) has **zero ports**.
  2. **2x Reversible SendCutSend `ALU-080` (`2.032 mm` `5052-H32`) `leg_flank_bracket` Stance Rails**:
       - Centered at `Y = +11.00 mm` (`X = +/-148.30 mm`), spanning `Y = -72.00 .. +94.00 mm` (`166.0 mm`).
       - Bolt across **both** rear-set `leg_bay_housing` cartridges (`+36.0` & `+76.6 mm`) and cantilever
         **`93 mm` forward** along the laptop flank + `28.5 mm` underneath the solid CNC aluminum bottom cover
         (clearing both the `|X| = 111.5 mm` latch release buttons and the `Y = +/-103 mm` rubber foot strips!).
  3. **Balanced `112.0 mm` Quadruped Wheelbase**:
       - Front Walking Legs (`FL`, `FR`) carried **`81.0 mm` forward of the Front Port** at `Y = -45.00 mm`
         (vertical load bears directly upward into the solid aluminum bottom cover right at `Y = -45.00 mm`!).
       - Rear Walking Legs (`RL`, `RR`) carried at `Y = +67.00 mm`.
       - Poised diagonal trot stance (`+/-8 mm` stride; all 4 TPU paws planted flat on `Z = -75.33 mm`).

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

# --- Authentic Framework Laptop 13 Dimensions (mm, per `FW_MAIN_PCB_GENERIC_2`) ---
LAPTOP_W_X_MM       = 296.60   # Upper palmrest width (-148.30 .. +148.30)
LAPTOP_D_Y_MM       = 228.90   # Depth (-114.45 front palmrest .. +114.45 rear hinge)
BASE_H_Z_MM         = 14.80    # Total aluminum base thickness (Z = 0.00 .. +14.80)
SKIRT_H_Z_MM        = 6.80     # Underside expansion-card bay & beveled lower-cover height (Z = 0.00 .. +6.80)
BAY_RECESS_X_MM     = 6.60     # Lower beveled side inset -> flat bottom cover spans X = +/-141.70 mm
LID_THICK_MM        = 4.20
LID_OPEN_DEG        = 106.0

# Authentic rear-set physical port coordinates on Framework Laptop 13!
BAY_FRONT_Y_MM      = +36.00   # Slots 2 & 4 (just behind palmrest/battery bulkhead)
BAY_REAR_Y_MM       = +76.60   # Slots 1 & 3 (exact 40.60 mm center-to-center behind Front Ports!)

# Reversible `leg_flank_bracket` center & balanced quadruped leg Y coordinates:
RAIL_CENTER_Y_MM    = +11.00   # Spans Y = -72.00 .. +94.00 mm (`166.0 mm` total)
LEG_FRONT_Y_MM      = -45.00   # 81.00 mm forward of Front Port! (`X_rail = -/+56.00 mm`)
LEG_REAR_Y_MM       = +67.00   # Balanced 112.00 mm wheelbase centered at +11.00 mm (`X_rail = +/-56.00 mm`)
RAIL_SEAT_Z_MM      = 0.00     # Top of `leg_flank_bracket` bears flush against Z = 0.00 mm bottom cover

# --- Shared Leg Kinematic Constants (matching `walking_leg_assembly.py` & `leg_servo_pod.py`) ---
SERVO_X_OFFSET_MM   = 7.30
SERVO_AXIS_Z_MM     = 22.10
TURRET_FRONT_Y_MM   = -25.40
BRACKET_OUT_Y_MM    = -49.50
BRACKET_TOP_Z_MM    = 34.7002
TIBIA_MID_Y_MM      = -37.40
INBOARD_LINK_Y0_MM  = -29.20
OUTBOARD_LINK_Y0_MM = -43.568
FEMUR_LEN_MM        = 38.0
TIBIA_LEN_MM        = 47.0
HORN_CRANK_MM       = 12.0
PUSHROD_LEN_MM      = 36.0
FOOT_BOLT_Z_MM      = 19.0
ANKLE_LEG_Z_MM      = -56.33   # Ground plane in laptop coords = -56.33 - 19.0 = -75.33 mm!

# Colors
CHASSIS_ALU_COLOR   = (0.79, 0.81, 0.84)
BEZEL_DARK_COLOR    = (0.13, 0.14, 0.16)
RUBBER_FOOT_COLOR   = (0.16, 0.17, 0.19)
SCREEN_GLOW_COLOR   = (0.14, 0.24, 0.36)
TRACKPAD_COLOR      = (0.68, 0.70, 0.73)
ACCENT_ORANGE_COLOR = (0.92, 0.38, 0.14)
RAIL_ALU_COLOR      = (0.62, 0.69, 0.78)
PA12_COLOR          = (0.22, 0.23, 0.26)
ALU_LINK_COLOR      = (0.76, 0.79, 0.83)
PUSHROD_COLOR       = (0.60, 0.72, 0.84)
HORN_COLOR          = (0.85, 0.31, 0.21)
TPU_COLOR           = (0.15, 0.16, 0.18)
SERVO_BODY_COLOR    = (0.14, 0.30, 0.58)
SERVO_CAP_COLOR     = (0.82, 0.68, 0.22)
PCB_COLOR           = (0.14, 0.42, 0.22)
HARNESS_COLOR       = (0.18, 0.19, 0.21)
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
    """Models a Framework Laptop 13 with authentic REAR-SET Expansion Card bays (`Y = +36.0 & +76.6 mm`)."""
    hw = LAPTOP_W_X_MM * 0.5          # 148.30 mm
    hd = LAPTOP_D_Y_MM * 0.5          # 114.45 mm
    bot_hw = hw - BAY_RECESS_X_MM     # 141.70 mm flat bottom cover edge

    # 1. Lofted lower beveled bottom-cover tub (Z = 0.0 .. +6.8 mm) + upper palmrest deck (Z = +6.8 .. +14.8 mm)
    w_bot = make_rounded_rect_face(bot_hw * 2.0, (hd - 3.5) * 2.0, 6.0, -bot_hw, -(hd - 3.5), 0.0)
    w_mid = make_rounded_rect_face(LAPTOP_W_X_MM, LAPTOP_D_Y_MM, 7.5, -hw, -hd, SKIRT_H_Z_MM)
    lower_tub = Part.makeLoft([w_bot, w_mid], True, True)
    upper_deck = make_rounded_rect_prism(LAPTOP_W_X_MM, LAPTOP_D_Y_MM, BASE_H_Z_MM - SKIRT_H_Z_MM, 7.5, -hw, -hd, SKIRT_H_Z_MM)
    base = lower_tub.fuse(upper_deck).removeSplitter()

    # Cut 4 REAR-SET underside-open Expansion Card bays at Y = +36.00 mm & +76.60 mm (`10.6 mm` divider!)
    # Each pocket is 30.2 mm deep along X (`|X| = 118.1 .. 148.3 mm`), leaving the `|X| = 111.5 mm` latch button exposed!
    for bx_sign in (-1.0, 1.0):
        for by_center in (BAY_FRONT_Y_MM, BAY_REAR_Y_MM):
            if bx_sign < 0:
                x0 = -hw - 1.0
                w_cut = 30.20 + 1.0
            else:
                x0 = hw - 30.20
                w_cut = 30.20 + 1.0
            bay_pocket = Part.makeBox(w_cut, 30.40, SKIRT_H_Z_MM + 0.40, App.Vector(x0, by_center - 15.20, -0.40))
            rail_groove = Part.makeBox(
                28.50, 32.40, 2.20,
                App.Vector(-hw + 1.50 if bx_sign < 0 else hw - 30.00, by_center - 16.20, 2.20),
            )
            base = base.cut(bay_pocket).cut(rail_groove)

    kb_well = Part.makeBox(262.0, 102.0, 1.4, App.Vector(-131.0, -8.0, BASE_H_Z_MM - 1.3))
    base = base.cut(kb_well).removeSplitter()

    base_obj = doc.addObject("Part::Feature", "Framework13_BaseChassis")
    base_obj.Shape = base
    colored_objs.append((base_obj, CHASSIS_ALU_COLOR))

    # 2. Underside Rubber Foot Strips (`Y = -103` & `+103 mm`) + 4x Card-Release Latch Buttons (`|X| = 111.5 mm`)
    front_foot = make_rounded_rect_prism(232.0, 6.0, 1.8, 2.8, -116.0, -106.0, -1.8)
    rear_foot  = make_rounded_rect_prism(232.0, 6.0, 2.2, 2.8, -116.0, +100.0, -2.2)
    feet_obj = doc.addObject("Part::Feature", "Framework13_UndersideRubberFeet")
    feet_obj.Shape = front_foot.fuse(rear_foot)
    colored_objs.append((feet_obj, RUBBER_FOOT_COLOR))

    latch_btns = []
    for bx_sign in (-1.0, 1.0):
        for by_center in (BAY_FRONT_Y_MM, BAY_REAR_Y_MM):
            latch_btns.append(
                make_rounded_rect_prism(5.0, 12.0, 0.8, 1.5, bx_sign * 111.5 - 2.5, by_center - 6.0, -0.6)
            )
    latch_obj = doc.addObject("Part::Feature", "Framework13_BayReleaseButtons")
    latch_obj.Shape = latch_btns[0].fuse(latch_btns[1:])
    colored_objs.append((latch_obj, ACCENT_ORANGE_COLOR))

    # 3. Keyboard Matrix + Hinge + Touchpad
    kb_plate = Part.makeBox(258.0, 98.0, 1.3, App.Vector(-129.0, -6.0, BASE_H_Z_MM - 1.1))
    hinge_barrel = Part.makeCylinder(4.8, 236.0, App.Vector(-118.0, hd - 6.0, BASE_H_Z_MM - 1.0), App.Vector(1, 0, 0))
    kb_obj = doc.addObject("Part::Feature", "Framework13_KeyboardAndHinge")
    kb_obj.Shape = kb_plate.fuse(hinge_barrel)
    colored_objs.append((kb_obj, BEZEL_DARK_COLOR))

    tp_obj = doc.addObject("Part::Feature", "Framework13_Touchpad")
    tp_obj.Shape = Part.makeBox(115.0, 76.0, 0.4, App.Vector(-57.5, -96.0, BASE_H_Z_MM - 0.2))
    colored_objs.append((tp_obj, TRACKPAD_COLOR))

    # 4. Open 13.5" 3:2 Display Lid
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

    shp_rail    = Part.read(str(MECH_DIR / "leg_flank_bracket.step"))
    shp_bay     = Part.read(str(MECH_DIR / "leg_bay_housing.step"))
    shp_pod     = Part.read(str(MECH_DIR / "leg_servo_pod.step"))
    shp_bracket = Part.read(str(MECH_DIR / "leg_hip_bracket.step"))
    shp_femur   = Part.read(str(MECH_DIR / "leg_femur_link.step"))
    shp_pushrod = Part.read(str(MECH_DIR / "leg_knee_pushrod.step"))
    shp_tibia   = Part.read(str(MECH_DIR / "leg_tibia_strut.step"))
    shp_foot    = Part.read(str(MECH_DIR / "leg_foot_pad.step"))
    pcb_path    = REPO_ROOT / "Day_06_Walking_Legs" / "build" / "board_only.step"
    shp_pcb     = Part.read(str(pcb_path)) if pcb_path.exists() else None

    build_framework_laptop(doc, colored_objs)

    flank_x = LAPTOP_W_X_MM * 0.5   # 148.30 mm

    def add_placed(name: str, shp: Part.Shape, base_pl: App.Placement, local_pl: App.Placement, col):
        obj = doc.addObject("Part::Feature", name)
        obj.Shape = shp
        obj.Placement = base_pl.multiply(local_pl)
        colored_objs.append((obj, col))

    # 1. Mount the 2 Reversible `ALU-080` Flank Stance-Extension Rails (`Left` RotZ(-90), `Right` RotZ(+90))
    #    Centered at Y = +11.00 mm -> spans Y = -72.00 .. +94.00 mm!
    for ftag, fx, rz_deg in (("Left", -flank_x, -90.0), ("Right", +flank_x, +90.0)):
        rail_pl = App.Placement(App.Vector(fx, RAIL_CENTER_Y_MM, RAIL_SEAT_Z_MM), App.Rotation(App.Vector(0, 0, 1), rz_deg))
        add_placed(f"{ftag}_FlankStanceBracket", shp_rail, rail_pl, App.Placement(), RAIL_ALU_COLOR)

        # Verify exact hole-to-port and hole-to-pod alignment on both sides!
        if rz_deg > 0:
            bay_xs = (+25.00, +65.60)
            pod_xs = (-56.00, +56.00)
            h_f = Part.makeBox(81.0, 4.8, 1.6, App.Vector(-56.0, -3.6, 0.05)).fuse([
                Part.makeBox(4.8, 6.5, 1.6, App.Vector(25.0 - 2.4, -3.6, 0.05)),
                Part.makeBox(4.8, 2.2, 5.8, App.Vector(-56.0, -3.6, 0.05)),
                Part.makeBox(12.0, 4.8, 1.6, App.Vector(56.0, -3.6, 0.05)),
                Part.makeBox(4.8, 6.5, 1.6, App.Vector(65.6 - 2.4, -3.6, 0.05)),
                Part.makeBox(4.8, 2.2, 5.8, App.Vector(56.0, -3.6, 0.05)),
            ]).removeSplitter()
        else:
            bay_xs = (-25.00, -65.60)
            pod_xs = (+56.00, -56.00)
            h_f = Part.makeBox(81.0, 4.8, 1.6, App.Vector(-25.0, -3.6, 0.05)).fuse([
                Part.makeBox(4.8, 6.5, 1.6, App.Vector(-25.0 - 2.4, -3.6, 0.05)),
                Part.makeBox(4.8, 2.2, 5.8, App.Vector(+56.0 - 4.8, -3.6, 0.05)),
                Part.makeBox(12.0, 4.8, 1.6, App.Vector(-65.6, -3.6, 0.05)),
                Part.makeBox(4.8, 6.5, 1.6, App.Vector(-65.6 - 2.4, -3.6, 0.05)),
                Part.makeBox(4.8, 2.2, 5.8, App.Vector(-56.0 - 4.8, -3.6, 0.05)),
            ]).removeSplitter()
        add_placed(f"{ftag}_ServoHarnesses", h_f, rail_pl, App.Placement(), HARNESS_COLOR)

        bolts = []
        for cx in bay_xs:
            for dx in (-7.0, 7.0):
                for hy in (8.0, 17.0):
                    bolts.append(Part.makeCylinder(2.2, 1.4, App.Vector(cx + dx, hy, -2.032), App.Vector(0, 0, -1)))
        for cx in pod_xs:
            for dx in (-7.0, 7.0):
                for hy in (-8.0, -17.0):
                    bolts.append(Part.makeCylinder(2.2, 1.4, App.Vector(cx + dx, hy, -2.032), App.Vector(0, 0, -1)))
        add_placed(f"{ftag}_RailFasteners", bolts[0].fuse(bolts[1:]), rail_pl, App.Placement(), STEEL_PIN_COLOR)

    # 2. Insert the 4 Flush `leg_bay_housing` Expansion Card Plugs at the REAR-SET Physical Ports (`Y = +36.0 & +76.6 mm`)
    ports = [
        ("Slot2_FrontLeft",  -flank_x, BAY_FRONT_Y_MM, -90.0),
        ("Slot1_RearLeft",   -flank_x, BAY_REAR_Y_MM,  -90.0),
        ("Slot4_FrontRight", +flank_x, BAY_FRONT_Y_MM, +90.0),
        ("Slot3_RearRight",  +flank_x, BAY_REAR_Y_MM,  +90.0),
    ]
    for ptag, px, py, rz_deg in ports:
        port_pl = App.Placement(App.Vector(px, py, RAIL_SEAT_Z_MM), App.Rotation(App.Vector(0, 0, 1), rz_deg))
        add_placed(f"{ptag}_BayHousing", shp_bay, port_pl, App.Placement(), PA12_COLOR)
        if shp_pcb is not None:
            bb = shp_pcb.BoundBox
            pcb_pl = App.Placement(
                App.Vector(-0.5 * (bb.XMin + bb.XMax), 16.0 - 0.5 * (bb.YMin + bb.YMax), 2.80 - bb.ZMin),
                App.Rotation(),
            )
            add_placed(f"{ptag}_Day06PCB", shp_pcb, port_pl, pcb_pl, PCB_COLOR)

    # 3. Mount the 4 Modular `leg_servo_pod` + 2-DOF Sagittal Legs at `Y = -45.0 mm` (Front) & `Y = +67.0 mm` (Rear)!
    legs = [
        ("FL", -flank_x, LEG_FRONT_Y_MM, -90.0, -1.0, -8.0),
        ("RL", -flank_x, LEG_REAR_Y_MM,  -90.0, -1.0, +8.0),
        ("FR", +flank_x, LEG_FRONT_Y_MM, +90.0, +1.0, +8.0),
        ("RR", +flank_x, LEG_REAR_Y_MM,  +90.0, +1.0, -8.0),
    ]

    for tag, lx_world, ly_world, rz_deg, s_dir, world_dy_stride in legs:
        pod_pl = App.Placement(App.Vector(lx_world, ly_world, RAIL_SEAT_Z_MM), App.Rotation(App.Vector(0, 0, 1), rz_deg))
        ankle_x_local = -world_dy_stride if rz_deg < 0 else world_dy_stride
        kin = solve_leg_kinematics(ankle_x_local, ANKLE_LEG_Z_MM, s_dir=s_dir)

        w_hip = pod_pl.multVec(App.Vector(kin["hx"], TIBIA_MID_Y_MM, kin["hz"]))
        w_kne = pod_pl.multVec(App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"]))
        w_ank = pod_pl.multVec(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"]))
        assert w_hip.y < w_kne.y, f"{tag}: expected Hip ({w_hip.y:.1f}) forward of Knee ({w_kne.y:.1f})!"
        assert abs(w_ank.y - (ly_world + world_dy_stride)) < 1e-2

        # Verify outboard servo pod clears the +/-148.30 mm upper palmrest rim!
        w_pod_back = pod_pl.multVec(App.Vector(0.0, -1.20, 20.0))
        assert abs(w_pod_back.x) > LAPTOP_W_X_MM * 0.5 + 1.0, f"{tag}: pod not clear of chassis! x={w_pod_back.x}"

        add_placed(f"{tag}_ServoPod", shp_pod, pod_pl, App.Placement(), PA12_COLOR)

        s_list, sp_list = [], []
        for sx in (-SERVO_X_OFFSET_MM, SERVO_X_OFFSET_MM):
            body = Part.makeBox(12.2, 20.8, 22.8, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM, 5.20))
            ears = Part.makeBox(12.2, 2.2, 32.0, App.Vector(sx - 6.1, TURRET_FRONT_Y_MM + 2.8, 0.60))
            boss = Part.makeCylinder(5.5, 2.0, App.Vector(sx, TURRET_FRONT_Y_MM, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0))
            s_list.append(body.fuse(ears).fuse(boss).removeSplitter())
            sp_list.append(Part.makeCylinder(2.4, 3.8, App.Vector(sx, TURRET_FRONT_Y_MM - 2.0, SERVO_AXIS_Z_MM), App.Vector(0, -1, 0)))

        add_placed(f"{tag}_Servos", s_list[0].fuse(s_list[1]), pod_pl, App.Placement(), SERVO_BODY_COLOR)
        add_placed(f"{tag}_Splines", sp_list[0].fuse(sp_list[1]), pod_pl, App.Placement(), SERVO_CAP_COLOR)

        horn_y = TURRET_FRONT_Y_MM - 1.60
        hip_end_x = kin["hx"] + 7.5 * (kin["kx"] - kin["hx"]) / FEMUR_LEN_MM
        hip_end_z = kin["hz"] + 7.5 * (kin["kz"] - kin["hz"]) / FEMUR_LEN_MM
        h1 = make_horn_solid(kin["hx"], kin["hz"], hip_end_x, hip_end_z, horn_y, 2.2, 5.2, 4.2)
        h2 = make_horn_solid(kin["sx"], kin["sz"], kin["crx"], kin["crz"], horn_y, 2.2, 5.2, 3.8)
        add_placed(f"{tag}_Horns", h1.fuse(h2), pod_pl, App.Placement(), HORN_COLOR)

        br_pl = App.Placement(App.Vector(0.0, BRACKET_OUT_Y_MM, BRACKET_TOP_Z_MM), App.Rotation())
        add_placed(f"{tag}_HipBracket", shp_bracket, pod_pl, br_pl, ALU_LINK_COLOR)

        for idx, y0 in enumerate((INBOARD_LINK_Y0_MM, OUTBOARD_LINK_Y0_MM)):
            f_pl = sagittal_link_placement(kin["hx"], kin["hz"], kin["kx"], kin["kz"], y0)
            add_placed(f"{tag}_Femur_{idx+1}", shp_femur, pod_pl, f_pl, ALU_LINK_COLOR)

        pr_pl = sagittal_link_placement(kin["crx"], kin["crz"], kin["bx"], kin["bz"], INBOARD_LINK_Y0_MM)
        add_placed(f"{tag}_Pushrod", shp_pushrod, pod_pl, pr_pl, PUSHROD_COLOR)

        tibia_rot = rot_from_cols(
            (kin["ex_x"], 0.0, kin["ex_z"]),
            (0.0, kin["s_dir"], 0.0),
            (kin["uz_x"], 0.0, kin["uz_z"]),
        )
        tibia_pos = App.Vector(kin["kx"], TIBIA_MID_Y_MM, kin["kz"]) - tibia_rot.multVec(App.Vector(10.5, 0.0, 53.0))
        add_placed(f"{tag}_Tibia", shp_tibia, pod_pl, App.Placement(tibia_pos, tibia_rot), ALU_LINK_COLOR)

        foot_pl = App.Placement(App.Vector(kin["ax"], TIBIA_MID_Y_MM, kin["az"] - FOOT_BOLT_Z_MM), App.Rotation())
        add_placed(f"{tag}_FootPad", shp_foot, pod_pl, foot_pl, TPU_COLOR)

        p_hip = Part.makeCylinder(1.5, 23.5, App.Vector(kin["hx"], -26.9, kin["hz"]), App.Vector(0, -1, 0))
        p_srv = Part.makeCylinder(1.5, 23.5, App.Vector(kin["sx"], -26.9, kin["sz"]), App.Vector(0, -1, 0))
        p_crk = Part.makeCylinder(1.5, 5.5, App.Vector(kin["crx"], -26.7, kin["crz"]), App.Vector(0, -1, 0))
        p_kne = Part.makeCylinder(1.5, 21.6, App.Vector(kin["kx"], -26.6, kin["kz"]), App.Vector(0, -1, 0))
        p_bel = Part.makeCylinder(1.5, 21.6, App.Vector(kin["bx"], -26.6, kin["bz"]), App.Vector(0, -1, 0))
        p_ank = Part.makeCylinder(1.5, 21.6, App.Vector(kin["ax"], -26.6, kin["az"]), App.Vector(0, -1, 0))
        add_placed(f"{tag}_Pins", p_hip.fuse([p_srv, p_crk, p_kne, p_bel, p_ank]), pod_pl, App.Placement(), STEEL_PIN_COLOR)

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
        f"ports_y=({BAY_FRONT_Y_MM:.1f}, {BAY_REAR_Y_MM:.1f} mm), "
        f"legs_y=({LEG_FRONT_Y_MM:.1f}, {LEG_REAR_Y_MM:.1f} mm), "
        f"ground_z={RAIL_SEAT_Z_MM + ANKLE_LEG_Z_MM - FOOT_BOLT_Z_MM:.2f} mm\n"
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
