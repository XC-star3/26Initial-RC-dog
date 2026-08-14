from __future__ import annotations

import argparse
import sys
import time

try:
    from .quadruped_link import QuadrupedSerialLink
    from .xrusb_codec import ControlCommand, RobotMode, RobotStatus
except ImportError:  # Direct execution: python host/quadruped_control_cli.py ...
    from quadruped_link import QuadrupedSerialLink
    from xrusb_codec import ControlCommand, RobotMode, RobotStatus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RC-dog USB control and diagnostics")
    parser.add_argument("port", help="USB CDC port, for example COM7 or /dev/ttyACM0")
    parser.add_argument("--mode", choices=[mode.name for mode in RobotMode], default="MOTOR_CHECK")
    parser.add_argument("--forward", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--enable", action="store_true", help="set MOTION_ENABLE")
    parser.add_argument("--deadman", action="store_true", help="set DEADMAN_HELD")
    parser.add_argument("--smooth-stop", action="store_true", help="set SMOOTH_STOP")
    parser.add_argument("--status", action="store_true", help="print the latest rcdog.status.v1 packet")
    return parser.parse_args()


def print_status(status: RobotStatus) -> None:
    print(
        f"source={status.control_source} requested={status.requested_mode} "
        f"active={status.active_mode} entry={status.entry_state} "
        f"block={status.block_reason} safety={status.safety_latched} "
        f"obstacle={status.obstacle_state}/{status.obstacle_fault} "
        f"online=leg:0x{status.leg_online_mask:02X},wheel:0x{status.wheel_online_mask:02X} "
        f"faults=0x{status.fault_bits:08X} counter={status.last_command_counter} "
        f"uptime_ms={status.uptime_ms}"
    )


def main() -> int:
    args = parse_args()
    link = QuadrupedSerialLink(args.port)
    link.set_error_callback(lambda exc: print(f"serial error: {exc}", file=sys.stderr))
    try:
        link.open()
        time.sleep(0.08)  # At least three 50 Hz safe-zero frames.
        command = ControlCommand(
            mode=RobotMode[args.mode],
            forward=args.forward,
            yaw=args.yaw,
            speed=args.speed,
            motion_enable=args.enable,
            deadman=args.deadman,
            smooth_stop=args.smooth_stop,
        )
        link.set_command(command)
        time.sleep(max(0.0, args.duration))
        if args.status:
            status = link.latest_status
            if status is None:
                print("no valid rcdog.status.v1 packet received", file=sys.stderr)
                return 2
            print_status(status)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
