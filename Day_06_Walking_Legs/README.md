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
- **Linkage Geometry:**
  - Upper femur link (`mech/leg_femur_link.step`): $L_1 = 36.0\text{ mm}$ (`2.03 mm` 5052-H32 aluminum).
  - Lower tibia U-channel strut (`mech/leg_tibia_strut.step`): $L_2 = 46.0\text{ mm}$ axle-to-ankle + $17.0\text{ mm}$ compliant TPU foot dome = $63.0\text{ mm}$ effective lower segment (`1.60 mm` CNC-bent 5052-H32 aluminum).
  - At neutral standing posture ($\theta_{\text{hip}} = 30^\circ$, horizontal moment arm $r_\perp = L_1 \cos 30^\circ = 31.2\text{ mm}$):
    - Static crawl hip torque: $\tau_{\text{crawl}} = 6.2\text{ N} \times 0.0312\text{ m} = \mathbf{0.193\text{ N}\cdot\text{m}}$ ($1.97\text{ kg}\cdot\text{cm}$).
    - Dynamic trot peak torque: $\tau_{\text{trot}} = 11.5\text{ N} \times 0.0312\text{ m} = \mathbf{0.359\text{ N}\cdot\text{m}}$ ($3.66\text{ kg}\cdot\text{cm}$), comfortably within the $\mathbf{0.44\text{ N}\cdot\text{m}}$ ($4.5\text{ kg}\cdot\text{cm}$ at $6.0\text{ V}$) stall rating of metal-gear MG90S / Emax ES08MDII digital micro-servos.

### 2. Per-Bay USB Type-C Power Budget (`5.0 V @ 1.5 A = 7.5 W`)
- **Zero-Renegotiation Enumeration:** `R1` (`5.1 kΩ` on `CC1`) requests standard USB Type-C `5.0 V @ 1.5 A` (`7.5 W` per bay, **`30.0 W` total** across all four bays) without USB-PD contract resets when multiple legs plug in or cycle load.
- **Sequenced Soft-Start & `0 µA` Limp Disconnect:**
  1. `Q1` (`AO3401A` P-MOSFET) isolates the boost power stage during USB enumeration (`PB4 = HIGH`, `BOOST_EN = LOW`).
  2. Each card reads its hardware `BAY_ID0`/`BAY_ID1` solder-jumper strap (`JP1`/`JP2`, `00`=FL, `01`=FR, `10`=RL, `11`=RR) and staggers soft-start by `bay_id × 35 ms` so four legs never draw simultaneous charging inrush.
  3. `U5` (`MT3608` `1.2 MHz` step-up converter + `L1` `4.7 µH` `5 A` shielded inductor + `D4` `SS34` Schottky + `68 kΩ / 7.5 kΩ` 1% divider) regulates `+6V0_SERVO` to **`6.04 V`**.
- **Local `88 µF` Ceramic Transient Reservoir:** Four `22 µF` `25 V` `1206` X5R MLCCs (`C9`, `C13`, `C14`, `C15`) on a dedicated `In2.Cu` `+6V0_SERVO` copper plane absorb `200 Hz` servo commutation current spikes locally, while `U4` (`INA226AIDGSR` across `R8` `20 mΩ` `1206` Kelvin shunt) enforces closed-loop `1.35 A` per-bay current foldback before reaching the `1.50 A` host port ceiling.

### 3. Sub-Microsecond Multi-Card Gait Coordination (`1 kHz` USB `SOF`)
- All four Expansion Cards enumerate on the laptop's internal xHCI root hub, which broadcasts the USB Full-Speed **Start-Of-Frame (`SOF`)** packet every `1.000 ms` with `< 500 ns` skew across all four ports.
- `software/walk_controller.py` streams trajectory packets tagged with a target 11-bit `SOF` frame number `(FNR + 5) & 0x7FF`.
- On each card, `USB_LP_CAN1_RX0_IRQHandler` phase-locks `TIM2`'s `200 Hz` (`5000 µs`, `1 µs` resolution) hardware PWM counter to `FNR % 5 == 0` and commits `HIP_PWM` (`PA0`) and `KNEE_PWM` (`PA1`) simultaneously on the target `SOF` edge—achieving deterministic 4-leg synchronization with zero external sync wires.

### 4. Mechanical Lock-In & SendCutSend Sheet-Metal Fabrication
- **`mech/leg_bay_housing.step` (MJF PA12 Nylon, `qty: 4`):** Slides into the Framework expansion bay T-rails (`32.0 mm` span), captures the bottom spring latch, mounts the `0.8 mm` 4-layer PCB on `22.60 mm` M2 bosses, and extends an **`18.0 mm × 30.0 mm × 3.2 mm` gusseted under-chassis load shelf** beneath the laptop bottom cover so vertical ground reaction loads bypass the straddle-mount USB-C plug completely.
- **SendCutSend `5052-H32` Aluminum Linkage (`0 fail · 0 warn` DFM verified):**
  - `mech/leg_hip_bracket.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent U-clevis, `qty: 4`).
  - `mech/leg_femur_link.dxf` (`ALU-080` `2.03 mm` laser-cut upper femur link, `qty: 8`).
  - `mech/leg_tibia_strut.flat.dxf` (`ALU-063` `1.60 mm` CNC-bent `58 mm` U-channel tibia strut, `qty: 4`).
- **`mech/leg_foot_pad.step` (FDM 95A TPU, `qty: 4`):** Ø`22 mm` compliant hemispherical foot pad bolted inside the tibia clevis on an M3 cross-axle.

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
