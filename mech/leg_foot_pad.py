#!/usr/bin/env python3
"""
3D-Printed Compliant Leaf-Spring Arch TPU Foot Pad (`mech/leg_foot_pad.py`)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Nests inside the bottom `16.8 mm` U-channel gap of `leg_tibia_strut` on an M3 cross-bolt
along Y (`Z_local = 19.0 mm`) and integrates a **through-arch elastomeric suspension slot**
(`10.0 x 5.2 mm` stadium window flanked by twin `3.2 mm` curved 95A TPU flexure ribs) above a
treaded `Ø22.0 mm` contact dome. Provides ~2.5-3.5 mm of progressive shock-absorbing compliance
on every foot strike so ground impacts do not spike the servo gearboxes.

Coordinate convention: mm, Z up, front/outboard facing -Y, walking stride along +/-X.
Exports:
  - mech/leg_foot_pad.FCStd
  - mech/leg_foot_pad.step      (colored matte dark charcoal TPU)
  - mech/leg_foot_pad.fab.json  (measured 3dp TPU features)
"""
import json
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Parameters (mm) ---
NECK_WIDTH_Y_MM    = 14.6    # Fits inside 16.80 mm tibia U-channel gap & clears 0.89 mm inner bend radii!
NECK_DEPTH_X_MM    = 10.0    # Fore-aft ankle block depth along X (R=5.0 mm < 8.90 mm front web distance!)
ARCH_SPAN_X_MM     = 13.8    # Outer fore-aft width across twin TPU flexure ribs (+/-6.9 mm -> >0.9 mm clear!)
DOME_RADIUS_MM     = 11.0    # Ø22.0 mm compliant hemisphere contact sole
BOLT_Z_ABOVE_BASE  = 22.5    # M3 ankle cross-bolt height above Z=0 ground tip (15.2 mm above dome shoulder!)
BOLT_HOLE_DIA_MM   = 3.20    # M3 clearance bore along Y
FLEX_SLOT_HALF_X   = 1.9     # Stadium suspension window half-span in X (total width = 8.4 mm)
FLEX_SLOT_RAD_Z    = 2.2     # Stadium suspension window radius (height = 4.4 mm -> ~3 mm travel)
FLEX_SLOT_Z_MID    = 10.6    # Centered between contact dome and ankle cross-bore
TPU_COLOR          = (0.16, 0.17, 0.19)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_foot_pad.log"


def build():
    doc = App.newDocument("leg_foot_pad")

    # 1. Bottom contact dome: sphere R=11 centered at (0, 0, 11), cut above Z = 7.3 mm (15.2 mm below bolt > 12.9 mm tibia corner!)
    sphere = Part.makeSphere(DOME_RADIUS_MM, App.Vector(0, 0, DOME_RADIUS_MM))
    upper_cut = Part.makeBox(40.0, 40.0, 30.0, App.Vector(-20.0, -20.0, 7.3))
    dome = sphere.cut(upper_cut)

    # 2. Compliant flexure-arch body (Z = 7.3 .. 13.6 mm)
    arch_block = Part.makeBox(
        ARCH_SPAN_X_MM,
        NECK_WIDTH_Y_MM,
        6.3,
        App.Vector(-ARCH_SPAN_X_MM * 0.5, -NECK_WIDTH_Y_MM * 0.5, 7.3),
    )

    # 3. Upper ankle mounting tongue with rounded R=5.0mm top clevis ear (Z = 13.2 .. 27.5 mm)
    neck_lower = Part.makeBox(
        NECK_DEPTH_X_MM,
        NECK_WIDTH_Y_MM,
        BOLT_Z_ABOVE_BASE - 13.2,
        App.Vector(-NECK_DEPTH_X_MM * 0.5, -NECK_WIDTH_Y_MM * 0.5, 13.2),
    )
    neck_cap = Part.makeCylinder(
        NECK_DEPTH_X_MM * 0.5,
        NECK_WIDTH_Y_MM,
        App.Vector(0.0, -NECK_WIDTH_Y_MM * 0.5, BOLT_Z_ABOVE_BASE),
        App.Vector(0.0, 1.0, 0.0),
    )
    neck = neck_lower.fuse(neck_cap)

    solid = dome.fuse(arch_block).fuse(neck).removeSplitter()

    # 4. Cut through-arch oval suspension window (along Y) creating twin fore/aft TPU leaf springs!
    p1 = App.Vector(-FLEX_SLOT_HALF_X, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID - FLEX_SLOT_RAD_Z)
    p2 = App.Vector(FLEX_SLOT_HALF_X, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID - FLEX_SLOT_RAD_Z)
    p3 = App.Vector(FLEX_SLOT_HALF_X + FLEX_SLOT_RAD_Z, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID)
    p4 = App.Vector(FLEX_SLOT_HALF_X, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID + FLEX_SLOT_RAD_Z)
    p5 = App.Vector(-FLEX_SLOT_HALF_X, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID + FLEX_SLOT_RAD_Z)
    p6 = App.Vector(-FLEX_SLOT_HALF_X - FLEX_SLOT_RAD_Z, -NECK_WIDTH_Y_MM, FLEX_SLOT_Z_MID)
    slot_face = Part.Face(
        Part.Wire(
            [
                Part.LineSegment(p1, p2).toShape(),
                Part.Arc(p2, p3, p4).toShape(),
                Part.LineSegment(p4, p5).toShape(),
                Part.Arc(p5, p6, p1).toShape(),
            ]
        )
    )
    arch_window = slot_face.extrude(App.Vector(0.0, NECK_WIDTH_Y_MM * 2.0, 0.0))
    solid = solid.cut(arch_window)

    # 5. Cross-bore for M3 ankle bolt along Y (matches sagittal tibia flanges!)
    bolt = Part.makeCylinder(
        BOLT_HOLE_DIA_MM * 0.5,
        NECK_WIDTH_Y_MM * 2.0,
        App.Vector(0.0, -NECK_WIDTH_Y_MM, BOLT_Z_ABOVE_BASE),
        App.Vector(0, 1, 0),
    )
    solid = solid.cut(bolt)

    # 6. Anti-slip tread grooves across bottom sole
    for offset in (-3.5, 0.0, 3.5):
        gx = Part.makeBox(1.4, DOME_RADIUS_MM * 3.0, 1.2, App.Vector(offset - 0.7, -DOME_RADIUS_MM * 1.5, -0.1))
        gy = Part.makeBox(DOME_RADIUS_MM * 3.0, 1.4, 1.2, App.Vector(-DOME_RADIUS_MM * 1.5, offset - 0.7, -0.1))
        solid = solid.cut(gx).cut(gy)

    solid = solid.removeSplitter()
    assert solid.isValid(), "leg_foot_pad shape is invalid!"

    part_obj = doc.addObject("Part::Feature", "LegFootPad")
    part_obj.Shape = solid
    doc.recompute()

    doc.saveAs(str(MECH_DIR / "leg_foot_pad.FCStd"))
    step_path = MECH_DIR / "leg_foot_pad.step"
    if App.GuiUp:
        import ImportGui
        part_obj.ViewObject.ShapeColor = TPU_COLOR
        ImportGui.export([part_obj], str(step_path))
    else:
        Part.export([part_obj], str(step_path))

    bb = solid.BoundBox
    vol_cm3 = round(solid.Volume / 1000.0, 2)
    area_cm2 = round(solid.Area / 100.0, 2)

    fab = {
        "kind": "3dp",
        "name": "Leg Compliant TPU Foot Pad",
        "source": "leg_foot_pad.step",
        "qty": 4,
        "options": {
            "process": "fdm",
            "material": "tpu",
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
    (MECH_DIR / "leg_foot_pad.fab.json").write_text(json.dumps(fab, indent=2) + "\n")

    LOG_PATH.write_text(
        f"OK: leg_foot_pad valid={solid.isValid()} "
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
