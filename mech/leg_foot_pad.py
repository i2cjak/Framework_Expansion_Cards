#!/usr/bin/env python3
"""
3D-Printed Compliant High-Friction TPU Foot Pad (`mech/leg_foot_pad.py`)
for Day 06 Walking Legs (Framework Quadruped Expansion Card Set).

Nests cleanly inside the bottom 18.8 mm clevis gap of `leg_tibia_strut` on an M3 cross-bolt
and presents a 22 mm grooved hemispherical contact dome to desk surfaces.

Coordinate convention: mm, Z up, front facing -Y.
Exports:
  - mech/leg_foot_pad.FCStd
  - mech/leg_foot_pad.step      (colored matte rubber dark charcoal)
  - mech/leg_foot_pad.fab.json  (measured 3dp TPU features)
"""
import json
import os
import traceback
from pathlib import Path
import FreeCAD as App
import Part

# --- Named Parameters (mm) ---
NECK_WIDTH_X_MM    = 18.2    # Fits inside 18.8 mm tibia U-channel gap with 0.3 mm per side clearance
NECK_DEPTH_Y_MM    = 13.0    # Fits inside tibia channel depth
NECK_HEIGHT_Z_MM   = 13.0    # Extends up into bottom of tibia strut
DOME_RADIUS_MM     = 11.0    # Ø22.0 mm compliant hemisphere contact foot
BOLT_Z_ABOVE_BASE  = 17.0    # M3 cross-bolt height above ground tip Z=0 (dome=11 + ankle=6)
BOLT_HOLE_DIA_MM   = 3.20    # M3 clearance bore along X
TPU_COLOR          = (0.16, 0.17, 0.19)

REPO_ROOT = Path("/home/h/Documents/i2cjak/Framework_Expansion_Cards")
MECH_DIR  = REPO_ROOT / "mech"
MECH_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH  = MECH_DIR / "leg_foot_pad.log"


def build():
    doc = App.newDocument("leg_foot_pad")

    # Bottom hemisphere: sphere centered at (0, 0, DOME_RADIUS_MM), cut above Z = DOME_RADIUS_MM
    sphere = Part.makeSphere(DOME_RADIUS_MM, App.Vector(0, 0, DOME_RADIUS_MM))
    upper_cut = Part.makeBox(
        DOME_RADIUS_MM * 4.0,
        DOME_RADIUS_MM * 4.0,
        DOME_RADIUS_MM * 2.0,
        App.Vector(-DOME_RADIUS_MM * 2.0, -DOME_RADIUS_MM * 2.0, DOME_RADIUS_MM),
    )
    dome = sphere.cut(upper_cut)

    # Cylindrical skirt collar from Z = DOME_RADIUS_MM to Z = DOME_RADIUS_MM + 2.5 mm
    collar = Part.makeCylinder(DOME_RADIUS_MM, 2.5, App.Vector(0, 0, DOME_RADIUS_MM), App.Vector(0, 0, 1))

    # Rectangular mounting tongue nesting inside tibia U-channel (Z = DOME_RADIUS_MM .. DOME_RADIUS_MM + NECK_HEIGHT_Z_MM)
    neck = Part.makeBox(
        NECK_WIDTH_X_MM,
        NECK_DEPTH_Y_MM,
        NECK_HEIGHT_Z_MM,
        App.Vector(-NECK_WIDTH_X_MM * 0.5, -NECK_DEPTH_Y_MM * 0.5, DOME_RADIUS_MM),
    )

    solid = dome.fuse(collar).fuse(neck).removeSplitter()

    # Cross-bore for M3 ankle bolt along X
    bolt = Part.makeCylinder(
        BOLT_HOLE_DIA_MM * 0.5,
        NECK_WIDTH_X_MM * 2.0,
        App.Vector(-NECK_WIDTH_X_MM, 0.0, BOLT_Z_ABOVE_BASE),
        App.Vector(1, 0, 0),
    )
    solid = solid.cut(bolt)

    # Anti-slip tread grooves across bottom sole (Y-running & X-running shallow channels)
    for offset in (-3.5, 0.0, 3.5):
        gx = Part.makeBox(1.4, DOME_RADIUS_MM * 3.0, 1.2, App.Vector(offset - 0.7, -DOME_RADIUS_MM * 1.5, -0.1))
        gy = Part.makeBox(DOME_RADIUS_MM * 3.0, 1.4, 1.2, App.Vector(-DOME_RADIUS_MM * 1.5, offset - 0.7, -0.1))
        solid = solid.cut(gx).cut(gy)

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
