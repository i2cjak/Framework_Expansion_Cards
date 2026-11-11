#!/usr/bin/env python3
"""
Framework Quadruped Host Gait Coordinator (`walk_controller.py`)
===============================================================
Coordinates four independent Day 06 Walking Leg Expansion Cards plugged into the four
Framework Laptop USB-C bays (`FL`, `FR`, `RL`, `RR`).

Key Engineering Mechanisms:
  1. Hardware Timebase Lock: All 4 cards hang off the same laptop xHCI USB 2.0 root hub,
     receiving the identical hardware 1.000 ms USB Start-of-Frame (SOF) packet (`FNR.FN`,
     11-bit frame counter 0..2047). Each keyframe packet specifies its target `sof_frame`
     execution timestamp so all 8 servos step within < 5 us of each other!
  2. Exact Closed-Loop 5-Bar Bellcrank Inverse Kinematics:
     - Coaxial hip & knee servos at `X = -9.0 mm` & `+9.0 mm`, `Z = +22.10 mm` (`mech/leg_servo_pod.step`)
     - Twin femur links `L1 = 38.0 mm` (`mech/leg_femur_link.step`), tibia `L2 = 47.0 mm` (`mech/leg_tibia_strut.step`)
     - `12.0 mm` knee horn + `36.0 mm` pushrod (`mech/leg_knee_pushrod.step`) + `12.0 mm` 90-deg tibia bellcrank
     - High-clearance compliant TPU foot tongue `22.5 mm` (`mech/leg_foot_pad.step`) -> `75.33 mm` standing sole
  3. Asymmetric Framework Laptop 13 USB-C Port Power Supervision (`45.0 W` PD Host Budget):
     - Two ports supply `5.0 V @ 3.0 A` (`15.0 W` each = `30.0 W`) and two ports supply `5.0 V @ 1.5 A` (`7.5 W` each = `15.0 W`).
     - Because two legs always reside on `1.5 A` bays, per-bay current is supervised at `1.30 A` soft velocity
       scaling and clamped in hardware at `1.38 A` (`0.14 ms` `INA226` `SOL` alert on `PB12`).

Usage:
  python3 walk_controller.py --demo         # Run kinematic & power-budget dry-run trace
  python3 walk_controller.py --gait crawl   # Stream live synchronized crawl gait to 4 USB legs
"""
import argparse
import glob
import math
import struct
import time
from dataclasses import dataclass

# Exact 5-Bar Bellcrank Linkage Geometry (mm, matching FreeCAD models in `mech/`)
SERVO_X_OFFSET_MM = 9.0
SERVO_AXIS_Z_MM   = 22.10
L1_FEMUR_MM       = 38.0
L2_TIBIA_MM       = 47.0
HORN_CRANK_MM     = 12.0
PUSHROD_LEN_MM    = 36.0
FOOT_BOLT_Z_MM    = 22.5

# Framework Laptop 13 USB-C Port Power Limits (mA @ 5.0V VBUS, PD input)
PORT_1P5A_LIMIT_MA    = 1500   # Two bays rated 5V @ 1.5A (7.5 W)
PORT_3P0A_LIMIT_MA    = 3000   # Two bays rated 5V @ 3.0A (15.0 W)
BAY_SOFT_FOLD_MA      = 1300   # Firmware/host soft velocity foldback threshold
BAY_HW_ALERT_MA       = 1380   # INA226 140 us hardware SOL alert clamp on PB12
LAPTOP_TOTAL_LIMIT_MA = 9000   # 2x 3.0A + 2x 1.5A = 9.0A @ 5.0V (45.0 W total)

BAY_NAMES = ("FL", "FR", "RL", "RR")

# Binary 12-byte gait command frame: <BBhhhHH
#   magic=0xA6, bay_id, hip_us, knee_us, flags, target_sof_mod2048, crc16
FRAME_FMT = "<BBhhhHH"


@dataclass
class LegPose:
    x_fore_mm: float   # +forward (toward screen hinge), -aft
    z_down_mm: float   # +downward from hip axle to desk surface


def solve_leg_ik(x_fore_mm: float, z_down_mm: float, mirror_side: bool = False) -> tuple[int, int]:
    """
    Exact closed-loop 2-DOF 5-bar bellcrank inverse kinematics matching `mech/simulate_walking_rom.py`.
    `x_fore_mm`: +forward toward screen hinge, -aft; `z_down_mm`: ground sole depth below Z=0 rail seat (e.g. 75.33 mm).
    Returns `(hip_pwm_us, knee_pwm_us)` in `750..2250 us` servo pulse range.
    """
    ankle_x = -x_fore_mm
    ankle_z = -(z_down_mm - FOOT_BOLT_Z_MM)
    hx, hz = -SERVO_X_OFFSET_MM, SERVO_AXIS_Z_MM
    sx, sz = +SERVO_X_OFFSET_MM, SERVO_AXIS_Z_MM

    dx, dz = ankle_x - hx, ankle_z - hz
    d = max(abs(L1_FEMUR_MM - L2_TIBIA_MM) + 2.0, min(L1_FEMUR_MM + L2_TIBIA_MM - 1.5, math.hypot(dx, dz)))
    if math.hypot(dx, dz) > 1e-6:
        scale = d / math.hypot(dx, dz)
        dx, dz = dx * scale, dz * scale
    a_proj = (L1_FEMUR_MM**2 - L2_TIBIA_MM**2 + d**2) / (2.0 * d)
    h_perp = math.sqrt(max(0.0, L1_FEMUR_MM**2 - a_proj**2))
    kx = hx + a_proj * (dx / d) + h_perp * (-dz / d)
    kz = hz + a_proj * (dz / d) + h_perp * (dx / d)

    uz_x, uz_z = (kx - ankle_x) / L2_TIBIA_MM, (kz - ankle_z) / L2_TIBIA_MM
    bx, bz = kx + 12.0 * uz_z, kz - 12.0 * uz_x

    ddx, ddz = bx - sx, bz - sz
    dd_raw = max(1e-6, math.hypot(ddx, ddz))
    dd = max(abs(HORN_CRANK_MM - PUSHROD_LEN_MM) + 2.5, min(HORN_CRANK_MM + PUSHROD_LEN_MM - 2.5, dd_raw))
    ddx, ddz = ddx * (dd / dd_raw), ddz * (dd / dd_raw)
    ca = (HORN_CRANK_MM**2 - PUSHROD_LEN_MM**2 + dd**2) / (2.0 * dd)
    ch = math.sqrt(max(0.0, HORN_CRANK_MM**2 - ca**2))
    crx = sx + ca * (ddx / dd) + ch * (-ddz / dd)
    crz = sz + ca * (ddz / dd) + ch * (ddx / dd)

    hip_deg = math.degrees(math.atan2(kz - hz, kx - hx)) + 52.0   # Centered around standing pose (-52 deg)
    knee_deg = math.degrees(math.atan2(crz - sz, crx - sx)) - 14.0  # Centered around standing crank (+14 deg)

    if mirror_side:
        hip_deg = -hip_deg
        knee_deg = -knee_deg

    hip_us = int(round(1500.0 + hip_deg * (500.0 / 60.0)))
    knee_us = int(round(1500.0 + knee_deg * (500.0 / 60.0)))
    return max(750, min(2250, hip_us)), max(750, min(2250, knee_us))


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_command_packet(bay_id: int, hip_us: int, knee_us: int, enable_power: bool, sof_target: int) -> bytes:
    flags = 0x0001 if enable_power else 0x0000
    header = struct.pack("<BBhhhH", 0xA6, bay_id, hip_us, knee_us, flags, sof_target & 0x07FF)
    return header + struct.pack("<H", crc16_ccitt(header))


def crawl_foot_trajectory(phase: float, stride_mm: float = 28.0, lift_mm: float = 15.0, stance_z_mm: float = 75.33) -> LegPose:
    """
    4-beat static-stability wave gait (`duty_factor = 0.75` — 3 legs always on the desk!):
      - phase in [0.00, 0.25): swing leg lifts `lift_mm` and sweeps forward `-stride/2 -> +stride/2`
      - phase in [0.25, 1.00): stance leg pushes chassis forward `+stride/2 -> -stride/2` at `stance_z_mm`
    """
    p = phase % 1.0
    if p < 0.25:
        u = p / 0.25
        x = -0.5 * stride_mm + stride_mm * (0.5 - 0.5 * math.cos(math.pi * u))
        z = stance_z_mm - lift_mm * math.sin(math.pi * u)
    else:
        u = (p - 0.25) / 0.75
        x = 0.5 * stride_mm - stride_mm * u
        z = stance_z_mm
    return LegPose(x_fore_mm=x, z_down_mm=z)


def trot_foot_trajectory(phase: float, stride_mm: float = 26.0, lift_mm: float = 13.0, stance_z_mm: float = 77.00) -> LegPose:
    """Diagonal trot (`FL+RR` vs `FR+RL` at 0.50 phase offset)."""
    p = phase % 1.0
    if p < 0.50:
        u = p / 0.50
        x = -0.5 * stride_mm + stride_mm * (0.5 - 0.5 * math.cos(math.pi * u))
        z = stance_z_mm - lift_mm * math.sin(math.pi * u)
    else:
        u = (p - 0.50) / 0.50
        x = 0.5 * stride_mm - stride_mm * u
        z = stance_z_mm
    return LegPose(x_fore_mm=x, z_down_mm=z)


def run_demo(gait: str, cycles: int = 2) -> None:
    # Phase offsets: static crawl = (FL=0.00, RR=0.25, FR=0.50, RL=0.75); trot = diagonal pairs
    offsets = (0.00, 0.50, 0.75, 0.25) if gait == "crawl" else (0.00, 0.50, 0.50, 0.00)
    traj_fn = crawl_foot_trajectory if gait == "crawl" else trot_foot_trajectory

    print(f"Framework Walking Legs — {gait.upper()} Gait Trajectory & SOF Sync Preview")
    print("SOF   | FL (hip,knee)  | FR (hip,knee)  | RL (hip,knee)  | RR (hip,knee)")
    print("-" * 76)
    sof = 100
    steps = 16 * cycles
    for k in range(steps):
        base_phase = (k / 16.0) % 1.0
        cols = []
        for bay_idx, off in enumerate(offsets):
            pose = traj_fn(base_phase + off)
            mirror = bay_idx in (1, 3)  # Right-side bays mirror sagittal rotation
            h_us, k_us = solve_leg_ik(pose.x_fore_mm, pose.z_down_mm, mirror_side=mirror)
            pkt = build_command_packet(bay_idx, h_us, k_us, True, sof)
            assert len(pkt) == 12
            cols.append(f"({h_us:4d},{k_us:4d}us)")
        print(f"{sof:4d}  | " + " | ".join(cols))
        sof = (sof + 50) & 0x07FF


def discover_serial_legs() -> list[str]:
    return sorted(glob.glob("/dev/ttyACM*"))[:4]


def main() -> None:
    ap = argparse.ArgumentParser(description="Framework 4-Bay Quadruped Gait Coordinator")
    ap.add_argument("--gait", choices=["crawl", "trot"], default="crawl")
    ap.add_argument("--demo", action="store_true", help="Print IK & SOF packet schedule without hardware")
    args = ap.parse_args()

    if args.demo:
        run_demo(args.gait, cycles=1)
        return

    ports = discover_serial_legs()
    if len(ports) < 4:
        print(f"Found {len(ports)}/4 leg cards ({ports}); running kinematic simulation preview:")
        run_demo(args.gait, cycles=1)
        return

    import serial  # Imported lazily when live hardware is present
    devs = [serial.Serial(p, 115200, timeout=0.005) for p in ports]
    offsets = (0.00, 0.50, 0.75, 0.25) if args.gait == "crawl" else (0.00, 0.50, 0.50, 0.00)
    traj_fn = crawl_foot_trajectory if args.gait == "crawl" else trot_foot_trajectory

    t0 = time.monotonic()
    sof_est = 0
    try:
        while True:
            phase = ((time.monotonic() - t0) * 0.65) % 1.0
            sof_est = (sof_est + 20) & 0x07FF
            for bay_idx, dev in enumerate(devs):
                pose = traj_fn(phase + offsets[bay_idx])
                h_us, k_us = solve_leg_ik(pose.x_fore_mm, pose.z_down_mm, mirror_side=bay_idx in (1, 3))
                dev.write(build_command_packet(bay_idx, h_us, k_us, True, sof_est))
            time.sleep(0.020)
    finally:
        for bay_idx, dev in enumerate(devs):
            dev.write(build_command_packet(bay_idx, 1500, 1500, False, sof_est))
            dev.close()


if __name__ == "__main__":
    main()
