# BONUS USB Isolator System Spec

## Goal

Framework Expansion Card with isolated USB 2.0 high-speed passthrough, an isolated STM32 debugger, UART bridge, target VREF sensing, and compact isolated power.

## Main Parts

| Function | Part | Notes |
|---|---|---|
| USB isolator | ADuM3165BRSZ-RL | USB 2.0 LS/FS/HS to 480 Mbps, 3.75 kVrms isolation, SSOP-20 |
| Isolated DC/DC | MIE1W0505BGLVH-3R-Z | 5 V in, selectable regulated 5 V or 3.3 V out, 1 W, 2.5 kVrms, LGA-12 4 x 5 mm |
| USB hub | USB2422T-I/MJ | 2-port USB 2.0 high-speed hub, 3.3 V, QFN-24 EP 4 x 4 mm, smallest found 2-port HS hub |
| Debug MCU | STM32L432KCU6 | UFQFPN-32 5 x 5 mm, M4F 80 MHz, 256 KB Flash, 64 KB SRAM, USB FS, ADC, 2x DAC |

## STM32 Result

- Pick: STM32L432KCU6. Best small STM32 with USB FS, ADC, DAC, and usable memory.
- Rejected: STM32G071GBU6. Smaller 4 x 4 mm UFQFPN, but no normal USB data peripheral.
- Alternate: STM32G0B1KEU6N. Same 5 x 5 mm class, 512 KB Flash and 144 KB RAM, but M0+ core and weaker availability.

## USB Topology

- Host USB-C from Framework card to ADuM3165 upstream side.
- ADuM3165 downstream isolated side to USB2422 upstream port.
- USB2422 downstream port 1 to STM32L432 USB FS pins.
- USB2422 downstream port 2 to isolated USB-C receptacle out.
- USB2422 needs a 24 MHz crystal or valid external clock per datasheet.

## STM32 Firmware

- Firmware role: USB debugger plus USB CDC UART bridge.
- Debug protocol: CMSIS-DAP v2 over USB.
- Target debug interface: SWD, with SWO optional.
- Runtime checks: sample UART VREF and debug VTref before enabling translators.

## Power

- Host 5 V feeds MIE1W0505BGLVH-3R-Z VIN and EN on the non-isolated side.
- Set MIE1W VSEL for isolated 5 V output.
- Isolated 5 V feeds USB downstream VBUS switch and an isolated 3.3 V LDO.
- Isolated 3.3 V feeds ADuM3165 side 2 logic, USB2422, STM32, level shifting, and LEDs.
- MIE1W budget is 1 W max. Assume 5 V at about 200 mA usable output.
- MIE1W cannot supply a full 500 mA USB downstream port after hub, STM32, and isolator load.
- Treat USB-C out as low-power/debug-only with about 100 mA current limit, or replace the DC/DC with a larger isolated supply.
- Place 10 uF + 0.1 uF on MIE input and 22 uF + 0.1 uF on output. Follow MPS layout guidance.

## Connectors

- Isolated USB-C receptacle out: USB 2.0 only, no SuperSpeed pairs.
- USB-C out is downstream-facing. Add CC1/CC2 Rp/Rp or controller configuration appropriate for a DFP sourcing VBUS.
- If the USB-C out current limit stays below USB default current, mark the port as low-power/debug-only.
- PicoBlade 4-pin UART target connector: GND_ISO, VREF_UART, TARGET_TX to STM32_RX, STM32_TX to TARGET_RX.
- 10-pin horizontal 1.27 mm Cortex debug header: VTref, SWDIO, SWCLK, SWO optional, nRESET, GND pins. Do not source target power by default.

## VREF Sensing And Leveling

- Sense UART VREF and debug VTref separately with high-impedance ADC dividers into STM32.
- Firmware valid VREF range: 1.2 V to 5.5 V.
- Disable all target-facing drivers when VREF is missing or out of range.
- Target VREF is input only. Do not back-power the target from this card.
- Use dual-supply level translators with A side at isolated 3.3 V and B side at sensed target VREF.
- SWCLK, nRESET, UART TX: unidirectional translators or direction-controlled buffers.
- SWDIO: bidirectional translator suitable for push-pull SWD timing. Avoid weak auto-bidirectional parts at high SWD speed.
- UART RX and SWO: target-VREF referenced inputs into isolated 3.3 V logic.

## LEDs

- UART_TX and UART_RX LEDs driven by STM32 firmware activity indicators.
- DBG_TX and DBG_RX LEDs for CMSIS-DAP/SWD transaction activity.
- Keep LEDs on isolated 3.3 V rail. Use about 1 mA to 2 mA per LED.

## Layout

- Maintain isolation clearance and creepage between host ground and isolated ground.
- No copper crossing the isolation barrier except approved isolator and DC/DC footprints.
- Route USB D+/D- as 90 ohm differential pairs.
- Keep ADuM3165, USB2422, USB-C out, and STM32 USB traces short.
- Place ESD protection at host USB entry and isolated USB-C receptacle.
- Keep hub crystal close to USB2422.
