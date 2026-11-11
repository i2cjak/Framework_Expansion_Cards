# Goal
Design a series of increasingly ambitious and wacky Framework Laptop expansion cards — starting with an easy, genuinely useful lab card, then a four-card quadruped walking-legs set, and escalating wild concepts — ready to order with clean schematics, layouts, mechanical parts, and firmware skeletons.

# Parts
- Day 05 Power & Bus Probe PCB (`Day_05_Power_Bus_Probe/`): 4-layer assembled expansion card with USB-C I2C/Qwiic bridge and 36V/5A rail monitor, to be ordered from the fab.
- Day 06 Walking Leg Controller PCB (`Day_06_Walking_Legs/`): 4-layer assembled dual-servo gait & current-telemetry expansion card (qty 4 for a full laptop), to be ordered from the fab.
- Leg Bay Structural Housing (`mech/leg_bay_housing.step`): MJF PA12 3D-printed expansion-bay lock housing with dual outboard horizontal servo pockets and under-chassis load lip, to be ordered.
- Leg Hip Outrigger Bracket (`mech/leg_hip_bracket.step`): SendCutSend bent 5052-H32 aluminum sheet-metal outboard double-shear bearing outrigger, to be ordered from SendCutSend.
- Leg Femur Link (`mech/leg_femur_link.step`): SendCutSend laser-cut 5052-H32 aluminum sheet-metal sagittal upper femur link (qty 8), to be ordered from SendCutSend.
- Leg Knee Pushrod (`mech/leg_knee_pushrod.step`): SendCutSend laser-cut 5052-H32 aluminum sheet-metal sagittal 4-bar knee actuation pushrod (qty 4), to be ordered from SendCutSend.
- Leg Bellcrank Tibia Strut (`mech/leg_tibia_strut.step`): SendCutSend bent 5052-H32 aluminum sheet-metal sagittal U-channel lower leg strut with integral 90-deg bellcrank lobe, to be ordered from SendCutSend.
- Leg Compliant TPU Foot Pad (`mech/leg_foot_pad.step`): 95A TPU 3D-printed leaf-spring arch suspension foot tip, to be ordered.
- Single Walking Leg Assembly (`mech/walking_leg_assembly.step`): Reference 3D single-leg sagittal fit-check assembly (`mech/`), not ordered.
- Full Walking Laptop Assembly (`mech/walking_laptop_assembly.step`): Reference 3D walk-ready Framework Laptop 13 four-leg quadruped assembly (`mech/`), not ordered.
- Day 07 Chiptune FM Synth & MIDI Card PCB (`Day_07_Chiptune_Synth_MIDI/`): 4-layer assembled crystal-less USB-MIDI polyphonic FM/chiptune synthesizer card with `PCM5102APWR` 32-bit I2S DAC, `3.5 mm` stereo headphone/line-out jack, and locking Type-A hardware MIDI Out port, to be ordered from the fab.
- Day 08 CAN 2.0B Bus Analyzer PCB (`Day_08_CAN_Bus_Analyzer/`): 4-layer assembled 1 Mbps ISO-11898 CAN 2.0B / DroneCAN analyzer card (`STM32F072CBT6` simultaneous USB + `bxCAN`, `8.000 MHz` quartz crystal, `SN65HVD230DR` transceiver with listen-only mode, `TLP172AM` PhotoMOS `120 Ohm` software termination, `PESD2CAN` surge clamp, and `0–36 V` `V_CAN` sense), to be ordered from the fab.
- Day 09 Solid-State PIN Radiation Dosimeter & Quantum RNG PCB (`Day_09_Radiation_Dosimeter_RNG/`): 4-layer assembled purple-soldermask ENIG expansion card with dual parallel `BPW34S-Z` (`15 mm²`) depleted silicon PIN detector array inside an exposed-ENIG Faraday/optical shield-can frame, `+13.3 V` 4-stage `BAV99` Dickson bias multiplier with `> 140 dB` 2-pole RC ripple filter, dual `TLV9061IDBVR` `0.5 pA` CMOS charge-sensitive pre-amp & CR-RC shaper, and crystal-less `STM32F072CBT6` `/dev/hwrng` Poisson entropy + pulse-height spectroscopy engine, to be ordered from the fab.

# Notes
- Follow existing repository conventions: `Day_NN_<name>/` folders on `main`, `26.0 x 30.0 mm` 4-layer 0.8 mm card outline with Amphenol `12401981E412A` straddle-mount USB-C plug, `build.py` outputs, card `README.md`, and showcase renders in `img/`.
- Prefer basic and preferred components from the fab's stock with an `MPN` on every BOM line and single-sided SMD placement where possible; sheet-metal leg parts target SendCutSend live DFM rules; printed parts use FreeCAD scripts in `mech/`.
- Commit per card on `main`; do not push without asking Bub first; order nothing directly (keep `.fab.json` measured and current for Buy it).
