# Day 08 — `1 Mbps` ISO-11898 CAN 2.0B Bus Analyzer (`gs_usb` SocketCAN)

A flush-fitting Framework Laptop Expansion Card implementing a native Linux mainline **`gs_usb` / candleLight SocketCAN** `1 Mbps` ISO-11898-2 CAN 2.0B analyzer with **galvanically isolated solid-state `120 Ω` termination switching**, hardware listen-only mode, `±24 V` automotive surge clamping, and `0–36.3 V` CAN-harness supply rail telemetry on a locking DroneCAN/Cyphal `4`-pin JST-GH bezel port.

---

## Hardware Architecture & Engineering Decisions

### 1. Simultaneous Full-Speed USB + `1 Mbps` `bxCAN` (`STM32F072CBT6` + `±10 ppm` Quartz)
- Unlike `STM32F103x8` devices (which share a single 512 B SRAM block and IRQ vector between USB and `CAN1`), **`U3` (`STM32F072CBT6`)** provides a dedicated `1 KB` USB packet buffer at `0x40006000` (`PA11`/`PA12`) alongside an independent `bxCAN` controller (`PB8`/`PB9`), sustaining **100% `1 Mbps` CAN bus load** over USB without dropped frames.
- **`Y1` (`X32258MOB4SI` `8.000 MHz` `±10 ppm` `3225` quartz crystal)** clocks the `bxCAN` bit-timing engine (`48.000 MHz` via `×6` PLL: `BRP = 3`, `16 TQ`, `TS1 = 13`, `TS2 = 2`, `87.5%` CiA sample point), exceeding ISO-11898-1 oscillator accuracy (`±0.15%`) by **`150×`** while `HSI48` + `CRS` clocks USB Full-Speed.

### 2. Opto-Isolated `120 Ω` PhotoMOS Solid-State Termination (`TLP172AM`)
- Switching a `120 Ω` resistor across a differential CAN bus with a bare GPIO-driven N-MOSFET fails under common-mode excursions (`-2 V` to `+7 V` operational, `±24 V` fault) due to `V_GS` modulation and body-diode conduction.
- **`U5` (Toshiba `TLP172AM` `SOP-4` PhotoMOS solid-state relay, `60 V` blocking, `1.0 Ω` `R_on`, `3.75 kVrms` isolation)** places back-to-back optically driven MOSFETs in series with **`R14` (`120 Ω` `1%` `0805` `125 mW`)**:
  - **Enabled (`PB1 = HIGH`):** `R_term = 120.0 + 1.0 = 121.0 Ω` (within `< 1%` of nominal `120 Ω`, ISO-11898 window `118–130 Ω`), illuminating the bezel `TERM` indicator (`D5`).
  - **Disabled (`PB1 = LOW`):** Bidirectional `±60 V` open circuit with `< 15 pF` symmetric off-capacitance and zero ground-loop or common-mode coupling—toggleable directly via `ip link set can0 type can termination 120`.

### 3. Transceiver, `±24 V` Automotive Protection & `0–36 V` DroneCAN Bus Sense
- **`U4` (`SN65HVD230DR`):** `3.3 V` `1 Mbps` ISO-11898 transceiver with `R9` (`10 kΩ` recessive pull-up on `D` during MCU reset) and `PB0`-controlled `Rs` pin supporting hardware **Listen-Only / Silent** mode (`ip link set can0 type can listen-only on`).
- **`D6` (`PESD2CAN` `SOT-23`) + `C13`/`C14` (`22 pF` `50 V` `C0G`):** Dual bidirectional `24 V` CAN surge clamp (`IEC 61000-4-2 ±30 kV`) placed immediately behind **`J2` (`SM04B-GHS-TB` locking 4-pin `1.25 mm` JST-GH)** wired to the standard **DroneCAN / Cyphal / Pixhawk / VESC** pinout (`1: V_CAN`, `2: CAN_H`, `3: CAN_L`, `4: GND`).
- **`V_CAN` Voltage Sense (`PA0 / ADC_IN0`):** `100 kΩ / 10 kΩ` `1%` divider (`11:1`, `0–36.3 V` full-scale) filtered by `100 nF` (`C15`) and clamped by `D7` (`H5VL10B` TVS), strictly isolated from host `VBUS` so external `12 V / 24 V` vehicle or drone batteries can never back-feed the laptop.

---

## Building PCB & Firmware

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL (from the fab's stock), and 3D renders
cd Day_08_CAN_Bus_Analyzer && python3 build.py

# Compile STM32F072CBT6 gs_usb / bxCAN firmware
make -C firmware
```
