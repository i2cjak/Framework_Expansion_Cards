# Day 13 — `+22 dBm` `868 / 915 MHz` `SX1262` LoRa Mesh Radio & Sub-GHz ISM Spectrum Sweeper

A dual-mode long-range `868 / 915 MHz` (`150–960 MHz` continuous) `+22 dBm` LoRa / FSK packet transceiver (`Meshtastic` / `Reticulum RNode` / `KISS TNC`) and real-time sub-GHz ISM waterfall spectrum analyzer with an outward-facing `50 Ω` gold edge-mount SMA coaxial port inside a crimson-red ENIG `26.0 × 30.0 mm` Framework Laptop Expansion Card.

## Hardware Architecture

1. **`+22 dBm` `150–960 MHz` LoRa & FSK Radio Engine (`U4` — Semtech `SX1262IMLTRT` + `Y1` `32.000 MHz` `±10 ppm` Crystal):**
   - Covers every sub-GHz ISM allocation (`433 MHz`, `868 MHz` EU, `915 MHz` US/AU) from `SF5` to `SF12` (`+22 dBm` high-efficiency PA, `-148 dBm` sensitivity, `170 dB` link budget) plus high-speed `FSK` / `GFSK` packet and continuous `GetRssiInst()` spectrum-sweep modes.
   - Configured in internal synchronous DC-DC buck mode (`DCC_SW` + `15 µH` `L1` + `470 nF` `C12` on `VREG`/`VDD_IN`), cutting active RX (`~4.6 mA`) and `+22 dBm` TX (`~118 mA`) current draw in half while maintaining clean `32.000 MHz` (`X322532MOB4SI`) frequency synthesis trimmed via `SX1262` internal `XTA_TRIM` / `XTB_TRIM` capacitor banks.

2. **`868 / 915 MHz` Matched Front-End, `AS179-92LF` GaAs SPDT Switch & `50 Ω` Edge-Mount SMA (`U5`, `J3`):**
   - **TX Path:** `VR_PA`-fed `47 nH` RF choke (`L2`) and `15 nH` / `4.7 pF C0G` impedance match (`L3`, `C15`) feed `J2` of a Skyworks `AS179-92LF` (`0.02–3.0 GHz`, `0.4 dB` insertion loss) GaAs PHEMT SPDT switch (`U5`), switched hardware-autonomously by `SX1262` `DIO2` (`SetDio2AsRfSwitchCtrl(true)`).
   - **RX Path:** `U5` `J3` feeds a 3-element `15 nH` / `47 pF C0G` single-ended-to-differential LNA balun (`C17`, `L4`, `L5`, `C19`) into `RFI_P` / `RFI_N` for maximum sensitivity across `868 / 915 MHz` LoRa and fast ISM band sweeps.
   - **50 Ω Coaxial Port:** `U5` `J1` passes through a `47 pF C0G` DC block (`C20`) and `2.2 pF / 8.2 nH / 2.2 pF` harmonic low-pass Pi filter (`C21`, `L6`, `C22`) over an unbroken `In1.Cu` ground reference plane to a gold `50 Ω` edge-mount SMA jack (`J3`, `BWSMA-KE-Z001`).

3. **Crystal-Less `STM32F072CBT6` USB `CDC-ACM` TNC & Waterfall Engine (`U3`):**
   - Runs at `48 MHz` (`HSI48` + `CRS` `USB SOF` lock, `D3V3XA4B10LP-7` ESD `U1` + `H5VL10B` `VBUS` TVS `D1`), driving `SX1262` over `12 MHz` `SPI1` (`PA4`–`PA7`) with hardware `BUSY` (`PB1`) and `DIO1` `EXTI` (`PB2`) handshaking to switch seamlessly between zero-configuration `KISS`/`RNode` mesh packet framing and `2,500+ bin/s` terminal/GUI waterfall spectrum sweeps.

## Fabrication & Firmware Build

All `45` SMT/edge-mount components are sourced from the fab's stock on a `0.8 mm` 4-layer Red ENIG stackup (`Day_13_LoRa_Spectrum_Sweeper.fab.json`).

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL, STEP model, and 3D renders
python3 build.py

# Compile bare-metal STM32F072CBT6 SX1262 LoRa TNC + ISM Spectrum Sweeper firmware
make -C firmware
```
