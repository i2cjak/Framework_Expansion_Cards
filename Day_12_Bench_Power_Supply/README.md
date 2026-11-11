# Day 12 — Pocket Programmable `1.0V–15.0V` SEPIC Micro Bench Supply & Electronic Fuse

A digitally trimmed pocket DC lab rail and `140 µs` hardware electronic fuse powered from a single Framework Laptop USB-C bay (`5 V` `VBUS`), delivering a regulated **`1.00 V – 15.00 V`** (`4.0 W` continuous) output with true **`0.00 V` DC shutoff**, 4-wire Kelvin voltage sensing, `16-bit` live `V/I/P` readback, and SCPI/CLI control inside a blue ENIG `26.0 × 30.0 mm` Expansion Card.

## Hardware Architecture

1. **True-Zero-Disconnect `1.2 MHz` Dual-Inductor SEPIC Stage (`U4` `MT3608`, `L1`/`L2` `SWPA4018S100MT`, `C10`, `D3` `SS34`):**
   - Unlike a conventional boost regulator—which passes `VBUS - V_D ≈ 4.7 V` straight to the output even when disabled—the **SEPIC** topology inserts a `50 V` `2.2 µF` DC-blocking coupling capacitor (`C10`) between shielded `10 µH / 1.6 A` inductors `L1` and `L2` (`4.0 × 4.0 × 1.8 mm`), providing **galvanic DC path interruption (`0.00 V` off-state)** while stepping smoothly both **below and above `5 V` `VBUS`** (`1.00 V` to `15.00 V`) into a `4× 2.2 µF / 50 V` low-ESR ceramic bank (`C11`–`C13`, `C15`) with an `18 V` `BZT52C18` overvoltage Zener clamp (`D6`).

2. **`12-Bit` DAC Feedback Injection & Closed-Loop Millivolt Trim (`U5` `MCP4725A0T-E/CH`):**
   - Injects a rail-to-rail `0–3.30 V` 12-bit DAC output (`0x60`) through `R18` (`22 kΩ 1%`) into the `MT3608` `0.600 V` feedback summing node alongside `R16` (`100 kΩ 1%` to `VOUT_SENSE`) and `R17` (`5.1 kΩ 1%` to `GND`), yielding **`3.66 mV / LSB`** native resolution across `1.00 V – 15.00 V`, continuously trimmed against `INA226` `1.25 mV` bus readback.

3. **Pulsed `25 kHz` PWM Input Soft-Start & `140 µs` Hardware Electronic Fuse (`Q1` `AO3401A`, `Q2` `AO3400A`, `U6` `INA226AIDGSR`, `R19` `50 mΩ`):**
   - Always-on `3.3 V` control power (`U2` `XC6206P332MR-G`) keeps the MCU, DAC, and telemetry alive while `Q1` (`AO3401A` high-side P-FET driven via `Q2` `AO3400A` on `PB0` / `TIM3_CH3`) precharges `C7` (`10 µF`) over `12 ms` (`< 0.15 A` peak inrush, verified via `PA1` `ADC_IN1` host `VBUS` droop sensing) and clamps continuous input draw below `0.95 A` (`4.0 W` output budget, safe on both `1.5 A` and `3.0 A` Framework USB-C ports).
   - High-side `INA226AIDGSR` (`0x40`) across `R19` (`50 mΩ 1%`, `50 µA / LSB`) fires its hardware Shunt-Over-Voltage (`SOL`) `~Alert` into `PA0` (`EXTI0`) in **`140 µs`**, killing `SEPIC_EN` (`PB1`) and opening `Q1` in `< 2 µs` ISR latency while latching the bezel `FLT` LED (`D5`).

4. **4-Wire Kelvin Bezel Output Port & Crystal-Less USB SCPI Controller (`J3` `SM04B-SRSS-TB`, `U3` `STM32F072CBT6`):**
   - Outward 4-pin `1.0 mm` right-angle header (`J3`: `VOUT`, `VOUT_SENSE` with `100 Ω` local Kelvin bridge `R21`, `GND`, `GND`) compensates lead drop right at the load, controlled over crystal-less USB 2.0 FS `CDC-ACM` (`U1` `D3V3XA4B10LP-7` + `D1` `H5VL10B`).

## Fabrication & Firmware Build

All `44` SMT components are sourced from the fab's stock on a `0.8 mm` 4-layer Blue ENIG stackup (`Day_12_Bench_Power_Supply.fab.json`).

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL, STEP model, and 3D renders
python3 build.py

# Compile bare-metal STM32F072CBT6 SEPIC DAC Trim + 140us E-Fuse firmware
make -C firmware
```
