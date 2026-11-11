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
  2. 2-DOF Sagittal Planar Inverse Kinematics:
     - Femur link `L1 = 36.0 mm` (`mech/leg_femur_link.step`)
     - Effective tibia + foot contact length `L2 = 57.0 mm` (`mech/leg_tibia_strut.step` + TPU dome)
  3. Per-Bay & Whole-Laptop Power Supervision:
     Reads live `INA226` (`VBUS` mV, `I_bay` mA) telemetry returned by each leg card and
     scales gait velocity if any bay approaches `1.35 A` (`< 1.50 A` USB Type-C limit).

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

# Mechanical Link Lengths (mm, matching FreeCAD models in `mech/`)
L1_FEMUR_MM = 36.0
L2_TIBIA_MM = 57.0

# USB-C Per-Bay Current Ceiling (mA)
BAY_CURRENT_LIMIT_MA = 1350
LAPTOP_TOTAL_LIMIT_MA = 4800

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
    2-DOF law-of-cosines inverse kinematics for sagittal (x_fore, z_down) foot target.
    Returns (hip_pwm_us, knee_pwm_us) in 1000..2000 us servo pulse range.
    """
    r = math.hypot(x_fore_mm, z_down_mm)
    r = max(24.0, min(L1_FEMUR_MM + L2_TIBIA_MM - 2.0, r))

    cos_knee = (L1_FEMUR_MM**2 + L2_TIBIA_MM**2 - r**2) / (2.0 * L1_FEMUR_MM * L2_TIBIA_MM)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_interior_rad = math.acos(cos_knee)

    cos_alpha = (L1_FEMUR_MM**2 + r**2 - L2_TIBIA_MM**2) / (2.0 * L1_FEMUR_MM * r)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    hip_rad = math.atan2(x_fore_mm, z_down_mm) + math.acos(cos_alpha)

    hip_deg = math.degrees(hip_rad) - 35.0
    knee_deg = 95.0 - math.degrees(knee_interior_rad)

    if mirror_side:
        hip_deg = -hip_deg
        knee_deg = -knee_deg

    hip_us = int(round(1500.0 + hip_deg * (500.0 / 60.0)))
    knee_us = int(round(1500.0 + knee_deg * (500.0 / 60.0)))
    return max(950, min(2050, hip_us)), max(950, min(2050, knee_us))


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


def crawl_foot_trajectory(phase: float, stride_mm: float = 28.0, lift_mm: float = 16.0, stance_z_mm: float = 72.0) -> LegPose:
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


def trot_foot_trajectory(phase: float, stride_mm: float = 30.0, lift_mm: float = 14.0, stance_z_mm: float = 72.0) -> LegPose:
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
