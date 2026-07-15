import struct
import unittest

from host.quadruped_protocol import (
    CHANNEL_VALID_MASK,
    ControlStream,
    RobotMode,
    VirtualRemoteCommand,
    crc16_ccitt_false,
)


class QuadrupedProtocolTest(unittest.TestCase):
    def test_fixed_vector(self) -> None:
        stream = ControlStream(session_id=0x11223344)
        stream.frame_seq = 0x1234
        stream.command_counter = 10
        frame = stream.encode(
            VirtualRemoteCommand(
                mode=RobotMode.GAIT_ONLY,
                yaw=-0.25,
                forward=0.5,
                speed_axis=-0.8,
                motion_enable=True,
                deadman=True,
            ),
            host_time_ms=0x01020304,
        )
        expected = bytes.fromhex(
            "A5 5A 01 10 00 00 34 12 1C 00 44 33 22 11 04 03 02 01 "
            "02 00 03 00 06 FF F4 01 E0 FC 00 00 00 00 67 00 0A 00 "
            "00 00 CF 96"
        )
        self.assertEqual(frame, expected)
        self.assertEqual(crc16_ccitt_false(frame[2:38]), 0x96CF)

    def test_disabled_command_becomes_safe_zero(self) -> None:
        frame = ControlStream(session_id=7).encode(
            VirtualRemoteCommand(
                mode=RobotMode.STAND_WHEEL,
                forward=1.0,
                yaw=1.0,
                speed_axis=1.0,
                deadman=True,
                motion_enable=False,
            ),
            host_time_ms=1,
        )
        payload = struct.unpack("<IIBBHhhhhhHI", frame[10:38])
        self.assertEqual(payload[2:5], (0, 0, 0))
        self.assertEqual(payload[5:10], (0, 0, -1000, 0, 0))
        self.assertEqual(payload[10], CHANNEL_VALID_MASK)

    def test_deadman_release_clears_continuous_axes(self) -> None:
        frame = ControlStream(session_id=8).encode(
            VirtualRemoteCommand(
                mode=RobotMode.STAND_WHEEL,
                forward=0.8,
                yaw=-0.4,
                speed_axis=0.5,
                motion_enable=True,
                deadman=False,
            ),
            host_time_ms=2,
        )
        payload = struct.unpack("<IIBBHhhhhhHI", frame[10:38])
        self.assertEqual(payload[5:7], (0, 0))
        self.assertEqual(payload[7], 500)
        self.assertEqual(payload[8:10], (0, 0))


if __name__ == "__main__":
    unittest.main()
