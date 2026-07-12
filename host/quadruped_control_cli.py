from __future__ import annotations

import argparse
import sys
import time

try:
    from .quadruped_link import QuadrupedSerialLink
    from .quadruped_protocol import RobotMode, VirtualRemoteCommand
except ImportError:  # Direct execution: python host/quadruped_control_cli.py ...
    from quadruped_link import QuadrupedSerialLink
    from quadruped_protocol import RobotMode, VirtualRemoteCommand


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RC-dog USB control and diagnostics")
    parser.add_argument("port", help="USB CDC port, for example COM7 or /dev/ttyACM0")
    parser.add_argument("--diag", choices=("p", "Y"), help="request one read-only diagnostic")
    parser.add_argument("--mode", choices=[mode.name for mode in RobotMode], default="IDLE")
    parser.add_argument("--forward", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=-1.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--enable", action="store_true", help="set MOTION_ENABLE")
    parser.add_argument("--deadman", action="store_true", help="set DEADMAN_HELD")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    link = QuadrupedSerialLink(args.port)
    link.set_error_callback(lambda exc: print(f"serial error: {exc}", file=sys.stderr))
    try:
        link.open()
        time.sleep(0.08)  # At least three 50 Hz safe-zero frames.
        if args.diag:
            print(link.request_diagnostic(args.diag), end="")
            return 0

        command = VirtualRemoteCommand(
            mode=RobotMode[args.mode],
            forward=args.forward,
            yaw=args.yaw,
            speed_axis=args.speed,
            motion_enable=args.enable,
            deadman=args.deadman,
        )
        link.set_command(command)
        time.sleep(max(0.0, args.duration))
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
