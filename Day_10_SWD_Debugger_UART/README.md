# Day 10 — Autoleveling `1.65V–5.5V` CMSIS-DAPv2 SWD Debugger & Target UART Bridge

A flush-fit ARM Cortex-M / RISC-V `CMSIS-DAPv2` hardware debug probe and `6 Mbps` target `UART` / `SWO` bridge inside a `26.0 × 30.0 mm` Framework Laptop Expansion Card, featuring true `1.65 V – 5.5 V` dual-supply level translation across every target signal.

## Hardware Architecture

1. **Five-Channel `420 Mbps` `1.65V–5.5V` Autoleveling Front-End (`U4`–`U8` — `5×` TI `SN74LVC1T45DBVR`):**
   - Each signal uses an independent dual-supply (`VCCA = +3.3 V`, `VCCB = VTREF`) `SOT-23-6` bus transceiver with `I_off` partial-power-down isolation: when a target board is powered off (`VTREF = 0 V`), all target pins remain high-impedance (`Hi-Z`) with zero back-powering.
   - **`U4` (`SWDIO`):** Hardware direction-controlled turnaround (`PB12` with `10 kΩ` reset pull-down `R17` so `U4` powers up in passive `B → A` input mode).
   - **`U5` (`SWCLK`):** Dedicated low-skew `SPI2_SCK` / GPIO clock translator (`PB13`).
   - **`U6` (`nRESET` Bidirectional Open-Drain + Target Reset Sense):** `PB0` controls `DIR` (`10 kΩ` pull-down `R18` defaults `U6` to `B → A` `Hi-Z` release at boot so `PB1` monitors external target reset button presses without contention; asserting `PB1 = 0` + `PB0 = 1` actively sinks target `nRESET` to `0.0 V` at `1.65 V – 5.5 V`).
   - **`U7` & `U8` (`UART TX` & `UART RX / SWO`):** Independent `USART2` DMA transmit/receive channels (`PA2`/`PA3`) with `22 Ω` series damping resistors (`R12`–`R16`) and weak `100 kΩ` RX idle pull-up (`R19`).

2. **`VTref` Surge Protection & Millivolt ADC Telemetry (`D3`, `R5`–`R7`, `C7`, `C8`):**
   - `H5VL10B` bidirectional TVS (`D3`), `22 Ω` surge limiter (`R5`), and `4.7 µF` + `5× 100 nF` decoupling condition the sensed target supply rail, while a `2:1` (`100 kΩ / 100 kΩ`) precision divider reports `0.00 V – 6.60 V` `VTref` to `PA0` (`ADC_IN0`) for `CMSIS-DAPv2` `DAP_Info` target-power verification.

3. **Crystal-Less USB 2.0 FS Controller & Dual Shrouded `1.0 mm` Bezel Ports (`U3`, `J2`, `J3`):**
   - `STM32F072CBT6` (`U3`, `48 MHz` `HSI48` + `CRS` `SOF` clock recovery) exposes `WinUSB` `CMSIS-DAPv2` bulk endpoints (zero-driver plug-and-play in `OpenOCD`, `pyOCD`, `probe-rs`, and `VS Code Cortex-Debug`) alongside a standard `CDC-ACM` Virtual COM Port.
   - Two side-by-side shrouded `4`-pin `1.0 mm` JST-SH connectors on the outward bezel separate `TGT_SWD` (`J2`: `VTref`, `SWDIO`, `SWCLK`, `GND`) and `TGT_UART` (`J3`: `nRESET`, `TGT_TX`, `TGT_RX`, `GND`) alongside `DAP` and `UART` activity LEDs (`D4`, `D5`).

## Fabrication & Build

All components are sourced from the fab's stock on a `0.8 mm` 4-layer ENIG stackup (`Day_10_SWD_Debugger_UART.fab.json`). Regenerate ERC/DRC reports, Gerbers, placement files, STEP model, and 3D renders with:

```bash
python3 build.py
```
