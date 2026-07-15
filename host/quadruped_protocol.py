from __future__ import annotations

import secrets
import struct
from dataclasses import dataclass, replace
from enum import IntEnum

MAGIC = b"\xA5\x5A"
VERSION = 0x01
VIRTUAL_RC_SETPOINT = 0x10
VIRTUAL_IMU = 0x11
PAYLOAD_SIZE = 28
FRAME_SIZE = 40

DEADMAN_HELD = 1 << 0
MOTION_ENABLE = 1 << 1
SMOOTH_STOP = 1 << 2
CHANNEL_VALID_MASK = 0x0067

_HEADER = struct.Struct("<BBHHH")
_RC_PAYLOAD = struct.Struct("<IIBBHhhhhhHI")


class RobotMode(IntEnum):
    IDLE = 0
    LOW_WHEEL = 1
    SAFE_HOLD = 2
    STAND_HOLD = 3
    STAND_WHEEL = 4
    RESERVED_STAND = 5
    GAIT_ONLY = 6
    GAIT_WHEEL = 7
    RESERVED = 8


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, frame_seq: int, payload: bytes) -> bytes:
    if msg_type not in (VIRTUAL_RC_SETPOINT, VIRTUAL_IMU):
        raise ValueError(f"unsupported message type: 0x{msg_type:02X}")
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError(f"payload must be {PAYLOAD_SIZE} bytes")
    body = _HEADER.pack(VERSION, msg_type, 0, frame_seq & 0xFFFF, len(payload)) + payload
    crc = crc16_ccitt_false(body)
    return MAGIC + body + struct.pack("<H", crc)


def _permille(value: float) -> int:
    return round(max(-1.0, min(1.0, value)) * 1000.0)


@dataclass(frozen=True)
class VirtualRemoteCommand:
    mode: RobotMode = RobotMode.IDLE
    yaw: float = 0.0
    forward: float = 0.0
    speed_axis: float = -1.0
    motion_enable: bool = False
    deadman: bool = False
    smooth_stop: bool = False

    @staticmethod
    def safe_zero() -> "VirtualRemoteCommand":
        return VirtualRemoteCommand()

    def sanitized(self) -> "VirtualRemoteCommand":
        if not self.motion_enable:
            return VirtualRemoteCommand.safe_zero()
        if not self.deadman or self.smooth_stop:
            return replace(self, yaw=0.0, forward=0.0)
        return self


class ControlStream:
    def __init__(self, session_id: int | None = None) -> None:
        generated = secrets.randbits(32) if session_id is None else session_id
        self.session_id = generated & 0xFFFFFFFF
        if self.session_id == 0:
            self.session_id = 1
        self.frame_seq = 0
        self.command_counter = 0

    def encode(self, command: VirtualRemoteCommand, host_time_ms: int) -> bytes:
        command = command.sanitized()
        mode = int(command.mode)
        if not 0 <= mode <= 8:
            raise ValueError(f"mode out of range: {mode}")

        flags = 0
        if command.deadman:
            flags |= DEADMAN_HELD
        if command.motion_enable:
            flags |= MOTION_ENABLE
        if command.smooth_stop:
            flags |= SMOOTH_STOP

        payload = _RC_PAYLOAD.pack(
            self.session_id,
            host_time_ms & 0xFFFFFFFF,
            mode // 3,
            mode % 3,
            flags,
            _permille(command.yaw),
            _permille(command.forward),
            _permille(command.speed_axis),
            0,
            0,
            CHANNEL_VALID_MASK,
            self.command_counter,
        )
        frame = encode_frame(VIRTUAL_RC_SETPOINT, self.frame_seq, payload)
        self.frame_seq = (self.frame_seq + 1) & 0xFFFF
        self.command_counter = (self.command_counter + 1) & 0xFFFFFFFF
        return frame
