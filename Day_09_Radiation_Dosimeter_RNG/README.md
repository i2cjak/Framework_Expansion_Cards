# Day 09 — Solid-State PIN Radiation Dosimeter & Quantum Decay Hardware RNG

A pocket solid-state ionizing-radiation dosimeter, pulse-height spectrometer, and `/dev/hwrng` true quantum hardware random-number generator built into a `26.0 × 30.0 mm` Framework Laptop Expansion Card.

## Hardware Architecture

1. **Dual Parallel Depleted Silicon PIN Detector (`D3`, `D4` — `2×` OSRAM `BPW34S-Z`, `15.0 mm²` Active Area):**
   - Two `7.5 mm²` intrinsic-layer silicon PIN diodes wired in parallel inside an exposed-ENIG **Faraday & optical shield-can frame** (`10.5 × 10.3 mm` solderable perimeter ring with 12 Faraday stitching vias to the unbroken `In1.Cu` ground plane).
   - Soldering a tinned-steel EMI shield lid (or copper foil tape) over the gold ring blocks visible/IR photons completely while letting X-ray, gamma, and energetic beta particles ionize electron-hole pairs inside the depleted silicon bulk.

2. **`+13.3 V` 4-Stage Dickson Depletion Multiplier & `> 140 dB` Ripple Filter (`D5`, `D6` — `2× BAV99`):**
   - Driven out-of-phase by `TIM1` `100 kHz` push-pull clocks (`PA8` / `PA9`) from `+5.0 V` `VBUS` across three `50 V` flying capacitors (`C11`–`C13`), elevating cathode reverse bias to `+13.3 V` so each `BPW34S-Z` depletes from `72 pF` (`0 V`) down to `~12 pF`.
   - Two cascaded low-pass RC poles (`100 kΩ + 100 nF`, `f_c = 15.9 Hz` followed by `1 MΩ + 100 nF`, `f_c = 1.59 Hz`) attenuate `100 kHz` charge-pump switching ripple below Johnson thermal noise before reaching the PIN cathodes, while an `11:1` divider (`R12`/`R13`) provides closed-loop `HV_RAW` telemetry on `PA0` (`ADC_IN0`).

3. **Femtoamp CMOS Charge-Sensitive Pre-Amplifier & CR-RC Pulse Shaper (`U4`, `U5` — `2×` TI `TLV9061IDBVR`):**
   - Physically separated single-channel `SOT-23-5` CMOS op-amps (`0.5 pA` input bias, `10 MHz` GBW, `10 nV/√Hz` input voltage noise) eliminate dual-op-amp leadframe crosstalk between the high-gain shaper output and the `2.2 pF` charge-integration node.
   - **Stage 1 (`U4` CSA):** `C18 = 2.2 pF C0G` feedback capacitor (`0.45 mV/fC` charge gain) in parallel with `R14 = 10 MΩ` (`τ = 22 µs`) biased at `+0.58 V` (`VBIAS_0V58`) on dedicated RC-isolated analog power (`V33_ANA`).
   - **Stage 2 (`U5` CR-RC Shaper):** `C19 = 100 pF C0G` + `R15 = 2.2 kΩ` differentiator feeding a `-45.5×` active integrator (`R16 = 100 kΩ || C20 = 22 pF C0G`, `τ = 2.2 µs`), converting negative CSA steps into clean positive unipolar pulses (`+0.58 V` baseline to `+3.30 V` full-scale).

4. **Hardware Discriminator, Spectroscopy & Unbiased Quantum Entropy (`U3` — `STM32F072CBT6`):**
   - Crystal-less Full-Speed USB (`HSI48` + `CRS` `1 kHz` USB `SOF` trim) streams conditioned entropy to Linux `/dev/hwrng` and pulse-height histograms over USB CDC.
   - On-chip `DAC1` (`PA4`) sets a software-programmable millivolt threshold on `COMP1_INM`; `PULSE_OUT` on `PA1` triggers `COMP1` (`50 ns`) to timestamp `48 MHz` `TIM2` arrival intervals (`t2 - t1` vs `t4 - t3` unbiased Poisson interval comparison + Von Neumann whitening) and triggers `1 Msps` `ADC_IN1` peak capture for gamma energy spectroscopy.

## Fabrication & Build

All components are sourced from the fab's stock on a `0.8 mm` 4-layer purple-soldermask ENIG stackup (`Day_09_Radiation_Dosimeter_RNG.fab.json`). Regenerate ERC/DRC reports, Gerbers, placement files, STEP model, and 3D renders with:

```bash
python3 build.py
```
