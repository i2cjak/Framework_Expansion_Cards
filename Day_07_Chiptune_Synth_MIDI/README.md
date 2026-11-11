# Day 07 — Polyphonic FM / Chiptune Hardware Synthesizer & Type-A MIDI Expansion Card

A pocket hardware music workstation inside a `26.0 × 30.0 mm` Framework Expansion Card bay (`IDEAS.md` Concept #10). Enumerates over USB-C as a class-compliant **USB-MIDI 1.0** instrument using crystal-less `HSI48` USB-SOF clock recovery (`CRS`), synthesizes **4-voice polyphonic 2-operator FM + pulse/duty + wavetable/triangle + LFSR noise chiptune audio** in real time on an `STM32F072CBT6`, streams `48 kHz` stereo digital audio over hardware `I2S2` into a **Texas Instruments `PCM5102APWR` 32-bit / 384 kHz DirectPath™ charge-pump DAC** (`112 dB` SNR, ground-centered `2.1 Vrms` stereo output with zero DC-blocking output capacitors), and provides both a **gold-plated `3.5 mm` stereo headphone/line-out jack (`J2`)** and a **locking `1.25 mm` 3-pin MMA Type-A hardware MIDI Out port (`J3`)** plus a **4-channel front-bezel voice activity LED bar (`D3..D6`)**.

---

## Architecture & Key Specifications

| Subsystem | Implementation | Key Rating / Performance |
| :--- | :--- | :--- |
| **USB-MIDI & Synth Engine (`U3`)** | `STM32F072CBT6` (`48 MHz` Cortex-M0, `128 KB` Flash, `16 KB` SRAM) | Crystal-less `HSI48` trimmed to `±0.05%` via `1 kHz` USB `SOF` (`CRS`); hardware `I2S2` master + `USART1` `31.25 kbaud` MIDI |
| **Stereo Audio DAC (`U4`)** | `PCM5102APWR` (`TSSOP-20`, internal negative charge pump + internal PLL) | `32-bit / 384 kHz` max (`48 kHz` native synth engine), `112 dB` dynamic range, `-93 dB` THD+N, `2.1 Vrms` ground-centered DirectPath™ output (`SCK=GND` 3-wire BCK-PLL mode) |
| **Stereo Output (`J2`)** | `PJ-320D-4A` gold-plated `3.5 mm` SMD TRS/TRRS jack + `22 Ω` series isolation (`R5`, `R6`) | Drives studio line inputs and `32 Ω+` headphones directly down to `0 Hz` without bass roll-off |
| **Hardware MIDI Out (`J3`)** | Locking `1.25 mm` 3-pin JST-GH (`SM03B-GHS-TB`) wired per **MMA 3.3 V Type-A TRS MIDI** | Pin 1 (`Tip/Sink`) via `22 Ω` (`R9`) from `PA9/USART1_TX`, Pin 2 (`Ring/Source`) via `22 Ω` (`R10`) from `+3V3`, Pin 3 (`Sleeve`) = `GND` |
| **Front-Bezel Voice Bar (`D3..D6`)** | 4× `0603` SMD LEDs (`PA0..PA3`, alternating Red/White) | Per-voice gate & envelope trigger indicators (Voices 1–2 FM/Pulse, Voice 3 Triangle/Wave, Voice 4 LFSR Noise/Drums) |
| **USB Protection & Power** | `H5VL10B` TVS (`D1`), `D3V3XA4B10LP-7` low-cap ESD (`U1`), `XC6206P332MR-G` `3.3 V` LDO (`U2`) | `5.1 kΩ` `CC1` sink (`R1`), `< 45 mA` total card draw from `5 V` `VBUS` |
| **4-Layer Stackup** | `0.8 mm` 4-Layer Red ENIG (`F.Cu` / `In1.Cu GND` / `In2.Cu +3V3` / `B.Cu GND`) | Unbroken `In1.Cu` ground reference plane isolating digital USB/MCU return currents from analog `OUTL`/`OUTR` |

---

## Pin Map (`STM32F072CBT6`)

| MCU Pin | Signal | Target / Function |
| :--- | :--- | :--- |
| `PA11` / `PA12` | `USB_DM` / `USB_DP` | USB 2.0 Full-Speed Device (`USB-MIDI 1.0` + `CRS` `1 kHz` SOF trim, via `22 Ω` `R2`/`R3` + `U1` ESD) |
| `PB12` (`I2S2_WS`) | `I2S_LRCK` | `PCM5102A` Pin 15 (`LRCK`, `48 kHz` stereo frame clock) |
| `PB13` (`I2S2_CK`) | `I2S_BCK` | `PCM5102A` Pin 13 (`BCK`, `1.536 MHz` bit clock; drives `PCM5102A` internal PLL with `SCK=GND`) |
| `PB15` (`I2S2_SD`) | `I2S_DIN` | `PCM5102A` Pin 14 (`DIN`, 16-bit/32-bit stereo I2S PCM stream) |
| `PB14` | `DAC_XSMT` | `PCM5102A` Pin 17 (`XSMT` soft-mute control; `10 kΩ` pull-down `R15` keeps DAC pop-free during reset) |
| `PA9` (`USART1_TX`) | `MIDI_TX` | `31,250 baud` 8N1 Hardware MIDI Out (`R9 = 22 Ω` sink, `R10 = 22 Ω` `+3V3` source on `J3`) |
| `PA0 .. PA3` | `VOICE_LED1..4` | Front-bezel 4-channel polyphonic voice gate LEDs (`D3..D6`) |
| `PA13` / `PA14` | `SWDIO` / `SWCLK` | `J4` 4-pin `1.0 mm` JST-SH SWD debug & flash header |

---

## Building & Verifying

### Hardware (KiCad 10 ERC, DRC, Gerbers, BOM, 3D STEP & Renders)
```bash
cd Day_07_Chiptune_Synth_MIDI
python3 build.py
```
- **ERC:** `0` errors, `0` warnings
- **DRC:** `0` violations, `0` unconnected items, `0` schematic parity issues
- **Fabrication Spec:** [`Day_07_Chiptune_Synth_MIDI.fab.json`](./Day_07_Chiptune_Synth_MIDI.fab.json) (`4` layers, `0.8 mm` thickness, Red solder mask, ENIG finish, all SMD components verified in stock at the fab)

### Firmware (`arm-none-eabi-gcc`)
```bash
cd Day_07_Chiptune_Synth_MIDI/firmware
make
```
Builds `build/chiptune_synth_midi.elf` and `.bin` with `0` warnings — configuring `HSI48` + `CRS` USB-SOF auto-trimming, `SPI2`/`I2S2` `48 kHz` master audio streaming to `PCM5102APWR` (`PB14` pop-free unmute sequencing), `31,250 baud` `USART1` hardware MIDI Out mirroring, 4-voice DDS phase-accumulated FM / pulse / triangle / 15-bit LFSR percussion synthesis, and `PA0..PA3` voice activity LEDs.
