# Day 11 — Dual `4 m` Laser Time-of-Flight Stereo Gesture Deck & Universal `940 nm` IR Blaster

A contactless mid-air USB HID gesture deck, presence/walk-away lock sensor, and high-power room-filling universal `940 nm` infrared remote blaster + `38 kHz` code learner inside a matte-black ENIG `26.0 × 30.0 mm` Framework Laptop Expansion Card.

## Hardware Architecture

1. **`22.2 mm`-Baseline Stereo `4 m` Class-1 VCSEL Time-of-Flight Array (`U4`, `U5` — `2×` ST `VL53L1CXV0FY/1`):**
   - Two `940 nm` single-photon avalanche diode (`SPAD`) Time-of-Flight ranging sensors sit at opposite corners of the outward bezel (`22.2 mm` stereo baseline), each backed by local `4.7 µF` + `100 nF` `AVDDVCSEL`/`AVDD` bypass (`C10`–`C13`).
   - Independent `XSHUT` reset lines (`PB0`, `PB10` with `10 kΩ` boot pull-downs `R14`, `R16`) allow clean sequential power-up and dynamic `400 kHz` `I2C1` address reassignment (`0x29 → 0x2A` Left, `0x2B` Right), while dedicated `GPIO1` open-drain interrupts (`PB1`, `PB11`) stream synchronized `50 Hz` stereo distance & `4×4` ROI vectors for left/right swipe, push-to-mute, pull-to-zoom, and walk-away screen lock.

2. **Phased-Arc `4×` `940 nm` High-Power `IRTIM` Blaster (`Q1`, `D4`–`D7`, `R12`, `R13`, `C7`, `C8`):**
   - Four side-firing `20°` sniper `940 nm` GaAlAs emitters (`Everlight IR26-21C/L110/CT`, `D4`–`D7`) are arranged in a `-8° / -3° / +3° / +8°` convex bezel fan for wide horizontal room coverage without desk-bounce loss.
   - Driven directly from `5 V` `VBUS` (`20 µF` local `0805` pulse reservoir `C7`+`C8`) in two parallel `2`-series strings (`R12`, `R13` = `33 Ω` `0603` → `~68 mA` per string, `~136 mA` peak optical burst across `4` emitters) switched by an `AO3400A` `5.7 A` low-threshold N-MOSFET (`Q1`) on the `STM32F072CBT6`'s dedicated hardware **`IRTIM`** output (`PB9` = `TIM17` `36–56 kHz` carrier × `TIM16` envelope, zero CPU jitter).

3. **`38 kHz` AGC Universal IR Code Learner (`U6` — `Everlight IRM-H638T/TR2`):**
   - Center-mounted outward-facing `38 kHz` automatic-gain-control receiver (`U6`) isolated by a dedicated `100 Ω` + `4.7 µF` / `100 nF` RC supply filter (`R19`, `C14`, `C15`) feeds `PA8` (`TIM1_CH1` `1 µs` both-edge hardware input capture) to record and replay `NEC`, `RC5`, `RC6`, `Sony SIRC`, and raw `Pronto` HVAC frames over USB `CDC-ACM`.

4. **Crystal-Less USB 2.0 FS Composite Controller (`U3` — `STM32F072CBT6`):**
   - Runs at `48 MHz` from internal `HSI48` + `CRS` `USB SOF` clock recovery (`D3V3XA4B10LP-7` low-capacitance ESD array `U1` + `H5VL10B` `VBUS` TVS `D1`), enumerating simultaneously as a driverless **USB HID Consumer/Keyboard Gesture Deck** and **USB `CDC-ACM` IR Transceiver**.

## Fabrication & Firmware Build

All `48` SMT components are sourced from the fab's stock on a `0.8 mm` 4-layer Matte Black ENIG stackup (`Day_11_ToF_Gesture_IR_Blaster.fab.json`), with all optical sensors, emitters, ICs, and connectors on `F.Cu` and local VCSEL/RC decoupling passives on `B.Cu`.

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL, STEP model, and 3D renders
python3 build.py

# Compile bare-metal STM32F072CBT6 Dual-ToF + Hardware IRTIM firmware
make -C firmware
```
