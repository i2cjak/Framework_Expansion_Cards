# Goal
Design a series of increasingly ambitious and wacky Framework Laptop expansion cards — starting with an easy, genuinely useful lab card, then a four-card quadruped walking-legs set, and escalating wild concepts — ready to order with clean schematics, layouts, mechanical parts, and firmware skeletons.

# Parts
- Day 05 Power & Bus Probe PCB (`Day_05_Power_Bus_Probe/`): 4-layer assembled expansion card with USB-C I2C/Qwiic bridge and 36V/5A rail monitor, to be ordered from the fab.
- Day 06 Walking Leg Controller PCB (`Day_06_Walking_Legs/`): 4-layer assembled dual-servo gait & current-telemetry expansion card (qty 4 for a full laptop), to be ordered from the fab.
- Leg Bay Structural Housing (`mech/leg_bay_housing.step`): MJF PA12 3D-printed expansion-bay lock housing with under-chassis load lip, to be ordered.
- Leg Hip Clevis Bracket (`mech/leg_hip_bracket.step`): SendCutSend bent 5052-H32 aluminum sheet-metal hip mount, to be ordered from SendCutSend.
- Leg Femur Link (`mech/leg_femur_link.step`): SendCutSend laser-cut 5052-H32 aluminum sheet-metal upper link, to be ordered from SendCutSend.
- Leg Tibia Strut (`mech/leg_tibia_strut.step`): SendCutSend bent 5052-H32 aluminum sheet-metal lower leg strut, to be ordered from SendCutSend.
- Leg Foot Pad (`mech/leg_foot_pad.step`): TPU 3D-printed high-friction compliant foot tip, to be ordered.
- Single Walking Leg Assembly (`mech/walking_leg_assembly.step`): Reference 3D fit-check assembly (`mech/`), not ordered.
- Day 07 Chiptune FM Synth & MIDI Card PCB (`Day_07_Chiptune_Synth_MIDI/`): 4-layer assembled crystal-less USB-MIDI polyphonic FM/chiptune synthesizer card with `PCM5102APWR` 32-bit I2S DAC, `3.5 mm` stereo headphone/line-out jack, and locking Type-A hardware MIDI Out port, to be ordered from the fab.

# Notes
- Follow existing repository conventions: `Day_NN_<name>/` folders on `main`, `26.0 x 30.0 mm` 4-layer 0.8 mm card outline with Amphenol `12401981E412A` straddle-mount USB-C plug, `build.py` outputs, card `README.md`, and showcase renders in `img/`.
- Prefer basic and preferred components from the fab's stock with an `MPN` on every BOM line and single-sided SMD placement where possible; sheet-metal leg parts target SendCutSend live DFM rules; printed parts use FreeCAD scripts in `mech/`.
- Commit per card on `main`; do not push without asking Bub first; order nothing directly (keep `.fab.json` measured and current for Buy it).
