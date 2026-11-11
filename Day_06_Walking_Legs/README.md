# Day 06 — Walking Legs (Quadruped 2-DOF Expansion Card Set)

Four identical USB Type-C Expansion Cards plug flush into the four rear-set expansion bays (`Y = +36.0 mm` & `+76.6 mm`) of a Framework Laptop 13, anchoring a pair of reversible **12-gauge (`2.03 mm`) `5052-H32` aluminum Flank Stance-Extension Rails (`mech/leg_flank_bracket.step`)** that bolt across both bays per side and cantilever `93 mm` forward along/under the solid CNC aluminum bottom cover (`Y = -72.0 .. +94.0 mm`) to carry the front walking legs `81.0 mm` forward of the front ports (`Y = -45.0 mm` & `+67.0 mm`, `112.0 mm` balanced wheelbase) so the ~`1.3 kg` laptop stands walk-ready and executes static crawl and dynamic trot gaits.

---

## System Architecture & Engineering Calculations

### 1. Laptop Mass, Leg Kinematics & Torque Budget
- **Laptop + 4-Leg Mass:** $m_{\text{total}} \approx 1.30\text{ kg} + 4 \times 0.065\text{ kg} = 1.56\text{ kg}$ ($W = 15.3\text{ N}$).
- **Static & Dynamic Ground Reaction per Leg:**
  - Standing on 4 legs: $F_{z,\text{4-leg}} = 3.83\text{ N}$.
  - Static 3-leg crawl stance (worst-case CoM shift): $F_{z,\text{crawl}} \approx 6.2\text{ N}$.
  - Dynamic diagonal trot peak ($1.5\times$ impact factor across 2 diagonal legs): $F_{z,\text{trot}} \approx 11.5\text{ N}$.
- **Sagittal 2-DOF Linkage & Bellcrank Pushrod Geometry (`mech/`, `0 / 85` Full-ROM 3D Collisions Verified via `mech/simulate_walking_rom.py`):**
  - **Both MG90S metal-gear micro-servos** mount horizontally side-by-side (`X = ±9.00 mm, Z = +22.10 mm`) inside a high-arch MJF PA12 saddle (`mech/leg_servo_pod.step`, `17.6 mm` overhead crank headroom) with **both brass output splines pointing outboard (`-Y`)**, so all pivots act in the **`X–Z` sagittal walking plane** across 6 dedicated non-overlapping `Y` planes (`9.70 mm` tibia-to-rail clearance, `6.70 mm` pushrod-to-rail clearance, `3.17 mm` inboard-femur-to-pushrod clearance).
  - **Paired Upper Femur Links (`mech/leg_femur_link.step`, `qty: 8`):** $L_1 = 38.0\text{ mm}$ (`2.03 mm` `5052-H32` aluminum) driven directly by the Hip servo horn (`J2`) in symmetric double shear, supported outboard by `mech/leg_hip_bracket.step` (`ALU-063` overhead outrigger bridge) so radial ground loads bypass the servo shafts.
  - **Non-Crossing 4-Bar Knee Drive (`mech/leg_knee_pushrod.step`, `qty: 4` + `mech/leg_tibia_strut.step`, `qty: 4`):** The Knee servo (`J3`) drives a $12.0\text{ mm}$ crank arm coupled via a $36.0\text{ mm}$ `5052-H32` pushrod ($31.1^\circ\text{–}93.7^\circ$ transmission angle across the entire $48\times 26\text{ mm}$ foot workspace — zero dead-center singularity) to an integral $12.0\text{ mm}$ $90^\circ$ bellcrank lobe on the CNC-bent $61.5\text{ mm}$ ($L_2 = 47.0\text{ mm}$) sagittal U-channel tibia strut with an **open $15.0\text{ mm}$ upper clevis throat** (`Z = 39.5..61.5 mm`) so the twin femur links flex freely without front-web interference.
  - **Series-Elastic Compliant Arch Foot (`mech/leg_foot_pad.step`, `qty: 4`):** Printed in `95A` TPU (`22.5 mm` high-clearance ankle clevis tongue) with a through-arch oval suspension window flanked by twin elastomeric leaf-spring ribs ($\approx 3\text{ mm}$ progressive shock travel on foot strike) to cushion desk impacts and protect the servo gear trains.
  - Closed-loop 5-bar Jacobian virtual-work simulation (`m = 1.58 kg`):
    - 4-leg stand: $\tau_{\text{hip}} = 0.15\text{ N}\cdot\text{m}$ ($35\%$ of stall).
    - Static crawl peak joint torque: $\tau_{\text{crawl}} = \mathbf{0.30\text{ N}\cdot\text{m}}$ ($68\%$ of stall).
    - Paced diagonal trot ($0.85\text{ Hz}$): $\tau_{\text{trot}} = \mathbf{0.32\text{ N}\cdot\text{m}}$ ($73\%$ of the $\mathbf{0.44\text{ N}\cdot\text{m}}$ / $4.5\text{ kg}\cdot\text{cm}$ stall rating at $6.04\text{ V}$).

### 2. Framework Laptop 13 USB-C Port Power Validation (`2× 5 V @ 3.0 A` + `2× 5 V @ 1.5 A` = `45.0 W` PD Host Budget)
- **Host Port Specification:** With USB-PD wall power attached, two Framework Laptop 13 expansion bays source up to **`5.0 V @ 3.0 A` (`15.0 W` each = `30.0 W`)** and the other two bays source up to **`5.0 V @ 1.5 A` (`7.5 W` each = `15.0 W`)**, providing **`45.0 W` (`9.0 A @ 5.0 V`)** total across all four legs. Because two legs always land on **`1.5 A` (`7.5 W`)** bays, `1.50 A` on `5 V` `VBUS` is the binding per-leg design ceiling.
- **Simulated Per-Port & 4-Port `VBUS` Load (`mech/simulate_walking_rom.py`, `MT3608` `6.04 V` boost @ $\eta = 89\%$ + `32 mA` MCU/telemetry):**
  - **4-Leg Stand:** `0.57 A` avg / **`0.58 A` peak per port (`2.88 W`)** — `38.5%` of `1.5 A` port, `11.5 W` / `45.0 W` total.
  - **Belly Push-Up (`14 mm` stroke):** `0.48 A` avg / **`1.26 A` peak per port (`6.27 W`)** — `83.7%` of `1.5 A` port, `25.1 W` / `45.0 W` total.
  - **Static Crawl (`28 mm` stride):** `0.72 A` avg / **`1.22 A` peak per port (`6.08 W`)** — `81.0%` of `1.5 A` port, `16.3 W` / `45.0 W` total.
  - **Paced Diagonal Trot (`0.85 Hz`):** `0.67 A` avg / **`1.51 A` raw impact peak** (`17.7 W` / `45.0 W` 4-port total) — supplied locally during `~20 ms` foot-strike peaks by the `88 µF` (`+6V0_SERVO`) ceramic reservoir and capped below `1.50 A` at **`1.38 A`** on `1.5 A` ports.
  - **Sprint Trot (`1.05 Hz`) / Dual-Servo Hard Stall (`2.53 A` raw):** `3.0 A` (`15 W`) ports carry full dual-servo stall continuously (`84%`), whereas `1.5 A` (`7.5 W`) ports are protected by **`0.14 ms` hardware overcurrent foldback** (`INA226` `140 µs` `SOL` alert at **`1.38 A` / `27.6 mV`** across `R8` wired to `PB12` `EXTI12` ISR — $>30\times$ faster than the motherboard port switch OCP!).
- **Pulsed `12 ms` Precharge Soft-Start (`0.22 A` Peak vs `32.4 A` Hard Step):**
  1. Snapping `Q1` (`AO3401A`) hard ON into `110 µF` total capacitance (`C12 = 22 µF` + `4× 22 µF = 88 µF` boost reservoir through `L1`/`D4`) would draw a `32.4 A` microsecond spike and trip a `1.5 A` port switch.
  2. `firmware/main.c` (`set_servo_power`) staggers each bay by `bay_id × 40 ms` and pulses `PB4` at `25 kHz` (`1 → 8 µs` duty over `12 ms`) while `BOOST_EN = LOW`, using `L1`'s `di/dt` limit to precharge the `110 µF` bank to `4.70 V` at **`0.22 A` peak port current** before latching `Q1` ON and enabling `U5` (`MT3608` `6.04 V`).

### 3. Sub-Microsecond Multi-Card Gait Coordination (`1 kHz` USB `SOF`)
- All four Expansion Cards enumerate on the laptop's internal xHCI root hub, which broadcasts the USB Full-Speed **Start-Of-Frame (`SOF`)** packet every `1.000 ms` with `< 500 ns` skew across all four ports.
- `software/walk_controller.py` streams trajectory packets tagged with a target 11-bit `SOF` frame number `(FNR + 5) & 0x7FF`.
- On each card, `USB_LP_CAN1_RX0_IRQHandler` phase-locks `TIM2`'s `200 Hz` (`5000 µs`, `1 µs` resolution) hardware PWM counter to `FNR % 5 == 0` and commits `HIP_PWM` (`PA0`) and `KNEE_PWM` (`PA1`) simultaneously on the target `SOF` edge—achieving deterministic 4-leg synchronization with zero external sync wires.

### 4. Rear-Set Port Geometry, Reversible `ALU-080` Flank Stance-Extension Rail & SendCutSend DFM
- **Why Direct Port Mounting Fails Fore-Aft Balance (`FW_MAIN_PCB_GENERIC_2` Port Coordinates):** On a Framework Laptop 13 (`228.9 mm` front-to-back, `Y = -114.45 .. +114.45 mm`), **both** physical Expansion Card bays on each flank sit in the **rear half** of the laptop (`Y = +36.0 mm` Front Port and `Y = +76.6 mm` Rear Port, `40.60 mm` center-to-center pitch with a `10.6 mm` aluminum divider), while the entire front `135.45 mm` (`Y = -114.45 .. +21.0 mm`, battery + palmrest) has zero expansion bays. Mounting legs directly at the ports would leave `150 mm` of unsupported front laptop mass ahead of the front legs.
- **Reversible Flank Stance-Extension Rail (`mech/leg_flank_bracket.step`, `ALU-080` `2.03 mm` `12-gauge` `5052-H32`, `qty: 2`):** Each `166.0 × 49.0 × 12.5 mm` CNC-bent inverted U-channel rail (`Y = -24.5 .. +24.5 mm`, giving `9.70 mm` outboard clearance to the `leg_tibia_strut` U-channel and `6.70 mm` clearance to `leg_knee_pushrod`) bolts across **both** rear-set `leg_bay_housing` plugs on a flank (`8× M2.5` across the `40.6 mm` two-bay baseline, locking out yaw twist) and extends **`93 mm` forward** along the laptop flank and `24.5 mm` underneath the solid CNC aluminum bottom cover (`Y = -72.0 .. +94.0 mm`, clearing both the `|X| = 111.5 mm` card-release latch buttons and the `Y = ±103 mm` rubber foot strips). It carries the **Front `leg_servo_pod` at `Y = -45.0 mm`** (`81.0 mm` forward of the Front Port — reacting front-leg vertical loads straight upward into the solid aluminum bottom cover right above `Y = -45.0 mm` with zero port prying!) and the **Rear `leg_servo_pod` at `Y = +67.0 mm`** (`112.0 mm` balanced quadruped wheelbase). A mirror-symmetric 24-hole layout lets **one single reversible SKU (`qty: 2`)** fit both Left and Right flanks (`37 pass · 0 fail · 0 warn` SendCutSend live DFM).
- **Modular Printed Plugs & Saddles (`MJF PA12 Nylon`):**
  - `mech/leg_bay_housing.step` (`qty: 4`) — Flush `30 × 32 × 6.8 mm` Expansion Card bay cartridge housing the `Day_06` PCB at `Y = +36.0` & `+76.6 mm`, with a front 4-wire servo harness exit onto the rail deck (`4× M2.5` floor inserts).
  - `mech/leg_servo_pod.step` (`qty: 4`) — Modular outboard dual-`MG90S` saddle bolted at `Y = -45.0` & `+67.0 mm` on the outboard deck of `leg_flank_bracket` (`4× M2.5` floor inserts + `4× M2.5` roof inserts for `leg_hip_bracket`).
- **SendCutSend `5052-H32` Aluminum Sheet-Metal Set (`0 fail · 0 warn` live DFM verified across all 5 sheet-metal parts):**
  - `mech/leg_flank_bracket.flat.dxf` (`ALU-080` `2.03 mm` CNC-bent `166.0 mm` reversible flank stance-extension U-channel rail, `qty: 2` — `37 pass · 0 fail · 0 warn`).
  - `mech/leg_hip_bracket.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent 90° overhead double-shear bearing outrigger, `qty: 4` — `25 pass · 0 fail · 0 warn`).
  - `mech/leg_femur_link.dxf` (`ALU-080` `2.03 mm` laser-cut `38.0 mm` sagittal femur link, `qty: 8` — `13 pass · 0 fail · 0 warn`).
  - `mech/leg_knee_pushrod.dxf` (`ALU-080` `2.03 mm` laser-cut `36.0 mm` knee actuation pushrod, `qty: 4` — `13 pass · 0 fail · 0 warn`).
  - `mech/leg_tibia_strut.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent `61.5 mm` sagittal bellcrank U-channel tibia strut with open `15.0 mm` knee clevis throat, `qty: 4` — `34 pass · 0 fail · 0 warn`).
- **`mech/leg_foot_pad.step` (FDM 95A TPU, `qty: 4`):** Ø`22 mm` compliant leaf-spring arch paw with through-arch suspension slot bolted inside the tibia clevis on an M3 cross-axle.
- **Reference 3D Fit-Check Assemblies (`mech/`):**
  - `mech/walking_leg_assembly.step` — Complete single-flank module (`1× leg_flank_bracket` + `2×` rear-set `leg_bay_housing` + `2×` `leg_servo_pod` & 2-DOF sagittal legs + routed servo harnesses).
  - `mech/walking_laptop_assembly.step` — Full walk-ready **Framework Laptop 13** with authentic rear-set ports (`Y = +36.0` & `+76.6 mm`), `2×` `ALU-080` flank stance-extension brackets, and balanced 4-leg stance (`Y = -45.0` & `+67.0 mm`).

---

## Building PCB, Firmware & Host Controller

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL (from the fab's stock), and 3D renders
cd Day_06_Walking_Legs && python3 build.py

# Compile STM32F103C8T6 SOF-synced servo firmware
make -C firmware

# Preview 4-leg CRAWL/TROT inverse kinematics & SOF packet schedule
python3 software/walk_controller.py --demo

# Run full-ROM 3D boolean collision sweep + multi-gait USB-C port power simulation
/home/h/.local/bin/freecadcmd ../mech/simulate_walking_rom.py
```
