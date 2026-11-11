# Day 06 — Walking Legs (Quadruped 2-DOF Expansion Card Set)

Four identical USB Type-C Expansion Cards plug into the four expansion bays of a Framework Laptop 13, locking mechanically into the side T-rails and extending a gusseted MJF PA12 **under-chassis load shelf** beneath the aluminum bottom cover so the ~`1.3 kg` laptop stands `95 mm` off the desk and walks with static crawl and dynamic trot gaits.

---

## System Architecture & Engineering Calculations

### 1. Laptop Mass, Leg Kinematics & Torque Budget
- **Laptop + 4-Leg Mass:** $m_{\text{total}} \approx 1.30\text{ kg} + 4 \times 0.065\text{ kg} = 1.56\text{ kg}$ ($W = 15.3\text{ N}$).
- **Static & Dynamic Ground Reaction per Leg:**
  - Standing on 4 legs: $F_{z,\text{4-leg}} = 3.83\text{ N}$.
  - Static 3-leg crawl stance (worst-case CoM shift): $F_{z,\text{crawl}} \approx 6.2\text{ N}$.
  - Dynamic diagonal trot peak ($1.5\times$ impact factor across 2 diagonal legs): $F_{z,\text{trot}} \approx 11.5\text{ N}$.
- **Sagittal 2-DOF Linkage & Bellcrank Pushrod Geometry (`mech/`):**
  - **Both MG90S metal-gear micro-servos** mount horizontally side-by-side (`X = ±7.30 mm, Z = +9.50 mm`) with **both brass output splines pointing outboard (`-Y`)**, so all pivots act in the **`X–Z` sagittal walking plane** (stepping fore-and-aft along the laptop sides).
  - **Paired Upper Femur Links (`mech/leg_femur_link.step`, `qty: 8`):** $L_1 = 38.0\text{ mm}$ (`2.03 mm` `5052-H32` aluminum) driven directly by the Hip servo horn (`J2`) in symmetric double shear, supported outboard by `mech/leg_hip_bracket.step` (`ALU-063` overhead outrigger bridge) so radial ground loads bypass the servo shafts.
  - **Non-Crossing 4-Bar Knee Drive (`mech/leg_knee_pushrod.step`, `qty: 4` + `mech/leg_tibia_strut.step`, `qty: 4`):** The Knee servo (`J3`) drives a $12.0\text{ mm}$ aluminum crank arm coupled via a $36.0\text{ mm}$ `5052-H32` pushrod ($\approx 97^\circ$ transmission angle) to an integral $12.0\text{ mm}$ $90^\circ$ bellcrank lobe on the CNC-bent $47.0\text{ mm}$ sagittal U-channel tibia strut—giving active, determinate 2-DOF positioning at both Hip and Knee with zero dead-center singularity.
  - **Series-Elastic Compliant Arch Foot (`mech/leg_foot_pad.step`, `qty: 4`):** Printed in `95A` TPU with a through-arch oval suspension window flanked by twin $3.2\text{ mm}$ elastomeric leaf-spring ribs ($\approx 2.5\text{–}3.5\text{ mm}$ progressive shock travel on foot strike) to cushion desk impacts and protect the servo gear trains.
  - At neutral standing posture ($r_\perp = L_1 \sin 30^\circ = 19.0\text{ mm}$ at hip, $84.7\text{ mm}$ ground clearance under laptop floor):
    - Static crawl peak joint torque: $\tau_{\text{crawl}} \approx \mathbf{0.18\text{ N}\cdot\text{m}}$ ($1.84\text{ kg}\cdot\text{cm}$).
    - Dynamic trot peak joint torque: $\tau_{\text{trot}} \approx \mathbf{0.34\text{ N}\cdot\text{m}}$ ($3.47\text{ kg}\cdot\text{cm}$), comfortably within the $\mathbf{0.44\text{ N}\cdot\text{m}}$ ($4.5\text{ kg}\cdot\text{cm}$ at $6.0\text{ V}$) stall rating of metal-gear MG90S / Emax ES08MDII digital micro-servos.

### 2. Per-Bay USB Type-C Power Output & Servo Budget (`7.5 W–15 W` Bay Source → `6.0 W` Continuous / `11 W` Peak per Leg)
- **Framework Expansion Bay VBUS Capability:** Each internal Framework Expansion Card slot sources fixed **`5.0 V` VBUS** with dynamic EC current allocation (**`5 V @ 3.0 A = 15.0 W`** on up to two slots first-come/first-served, falling back to **`5 V @ 1.5 A = 7.5 W`** on the remaining slots — **`30.0 W` guaranteed simultaneous 4-bay host budget**, up to **`45.0 W`** asymmetric).
- **Zero-Renegotiation Enumeration:** `R1` (`5.1 kΩ` on `CC1`) requests standard Type-C `5.0 V` without USB-PD contract resets when multiple legs plug in or cycle load.
- **Closed-Loop `6.0 W` Continuous / `11 W` Peak Servo Output per Leg:**
  1. `Q1` (`AO3401A` P-MOSFET) isolates the boost power stage during USB enumeration (`PB4 = HIGH`, `BOOST_EN = LOW`).
  2. Each card reads its hardware `BAY_ID0`/`BAY_ID1` solder-jumper strap (`JP1`/`JP2`, `00`=FL, `01`=FR, `10`=RL, `11`=RR) and staggers soft-start by `bay_id × 35 ms` so four legs never draw simultaneous charging inrush.
  3. `U5` (`MT3608` `1.2 MHz` step-up converter + `L1` `4.7 µH` `5 A` shielded inductor + `D4` `SS34` Schottky + `68 kΩ / 7.5 kΩ` 1% divider) regulates `+6V0_SERVO` to **`6.04 V`** (`~90%` efficiency → **`6.0 W` (`1.0 A @ 6.04 V`) continuous output per leg**, **`24.0 W` total continuous across all 4 legs**; average walking gait draw is `~1.8–2.5 W` per leg).
- **Local `88 µF` Ceramic Transient Reservoir:** Four `22 µF` `25 V` `1206` X5R MLCCs (`C9`, `C13`, `C14`, `C15`) on a dedicated `In2.Cu` `+6V0_SERVO` copper plane supply brief `~1.8 A` (`~11 W`) `200 Hz` servo commutation bursts locally, while `U4` (`INA226AIDGSR` across `R8` `20 mΩ` `1206` Kelvin shunt) enforces closed-loop `1.35 A` (`6.75 W`) per-bay input foldback so even the `1.5 A` (`7.5 W`) fallback bays never trip their motherboard port switch.

### 3. Sub-Microsecond Multi-Card Gait Coordination (`1 kHz` USB `SOF`)
- All four Expansion Cards enumerate on the laptop's internal xHCI root hub, which broadcasts the USB Full-Speed **Start-Of-Frame (`SOF`)** packet every `1.000 ms` with `< 500 ns` skew across all four ports.
- `software/walk_controller.py` streams trajectory packets tagged with a target 11-bit `SOF` frame number `(FNR + 5) & 0x7FF`.
- On each card, `USB_LP_CAN1_RX0_IRQHandler` phase-locks `TIM2`'s `200 Hz` (`5000 µs`, `1 µs` resolution) hardware PWM counter to `FNR % 5 == 0` and commits `HIP_PWM` (`PA0`) and `KNEE_PWM` (`PA1`) simultaneously on the target `SOF` edge—achieving deterministic 4-leg synchronization with zero external sync wires.

### 4. Recessed-Bay Rigid Metal Outrigger & SendCutSend Sheet-Metal Fabrication
- **Clearing the Recessed Underside Bay (`mech/leg_bay_outrigger.step`, `ALU-080` `2.03 mm` `12-gauge` `5052-H32`, `qty: 4`):** Real Framework Laptop expansion bays sit recessed `~7 mm` inward inside a `6.8 mm`-tall underside pocket beneath the beveled overhanging palmrest deck. Rather than cantilevering plastic out of that recessed slot, a **12-gauge (`2.03 mm`) CNC-bent `5052-H32` aluminum U-channel cantilever spine** bolts across the underside of the locked-in bay cartridge (`4× M2.5`), extends **`17.5 mm` inboard underneath the laptop's solid CNC aluminum bottom cover** (`Y = +26.5..+44.0 mm`, with a `10 × 7 mm` access window over the chassis release button) to react vertical loads and the `0.57 N·m` roll moment directly into the chassis floor (`> 6×` yield safety factor at dynamic trot), projects horizontally clear past the beveled chassis overhang, and rises into `18.5 mm` twin vertical cradle walls carrying the dual-servo turret `16.0 mm` further outboard (`34 pass · 0 fail · 0 warn` SendCutSend live DFM).
- **`mech/leg_bay_housing.step` (MJF PA12 Nylon, `qty: 4`):** Low-profile `6.8 mm` recessed bay T-rail cartridge + `6.4 mm`-tall enclosed servo-harness neck clearing the overhanging palmrest rim + outboard dual-servo saddle bolted inside the `ALU-080` metal U-cradle (`4× M2.5`).
- **SendCutSend `5052-H32` Aluminum Linkage (`0 fail · 0 warn` live DFM verified across all 5 sheet-metal parts):**
  - `mech/leg_bay_outrigger.flat.dxf` (`ALU-080` `2.03 mm` CNC-bent 12-gauge bay-extension cantilever spine & under-chassis anchor plate, `qty: 4` — `34 pass · 0 fail · 0 warn`).
  - `mech/leg_hip_bracket.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent 90° overhead double-shear bearing outrigger, `qty: 4` — `25 pass · 0 fail · 0 warn`).
  - `mech/leg_femur_link.dxf` (`ALU-080` `2.03 mm` laser-cut `38.0 mm` sagittal femur link, `qty: 8` — `13 pass · 0 fail · 0 warn`).
  - `mech/leg_knee_pushrod.dxf` (`ALU-080` `2.03 mm` laser-cut `36.0 mm` knee actuation pushrod, `qty: 4` — `13 pass · 0 fail · 0 warn`).
  - `mech/leg_tibia_strut.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent `60.0 mm` sagittal bellcrank U-channel tibia strut, `qty: 4` — `34 pass · 0 fail · 0 warn`).
- **`mech/leg_foot_pad.step` (FDM 95A TPU, `qty: 4`):** Ø`22 mm` compliant leaf-spring arch paw with through-arch suspension slot bolted inside the tibia clevis on an M3 cross-axle.
- **Reference 3D Fit-Check Assemblies (`mech/`):**
  - `mech/walking_leg_assembly.step` — Complete colored single-leg sagittal 2-DOF assembly.
  - `mech/walking_laptop_assembly.step` — Full walk-ready **Framework Laptop 13** on all four 2-DOF sagittal legs (`FL`, `FR`, `RL`, `RR`).

---

## Building PCB, Firmware & Host Controller

```bash
# Regenerate ERC/DRC reports, Gerbers, BOM/CPL (from the fab's stock), and 3D renders
cd Day_06_Walking_Legs && python3 build.py

# Compile STM32F103C8T6 SOF-synced servo firmware
make -C firmware

# Preview 4-leg CRAWL/TROT inverse kinematics & SOF packet schedule
python3 software/walk_controller.py --demo
```
