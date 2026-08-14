from __future__ import annotations

import secrets
import struct
from dataclasses import dataclass, replace
from enum import IntEnum

PACKET_PREFIX = 0x5A
PACKET_VERSION = 0x01
PACKET_HEADER_SIZE = 16
PACKET_BASE_SIZE = 17
CONTROL_TOPIC = "rcdog.control.command.v1"
STATUS_TOPIC = "rcdog.status.v1"
SCHEMA_VERSION = 1

DEADMAN = 1 << 0
MOTION_ENABLE = 1 << 1
SMOOTH_STOP = 1 << 2

_CONTROL = struct.Struct("<BBHhhhHIII")
_STATUS = struct.Struct("<12BIII")


class RobotMode(IntEnum):
    MOTOR_CHECK = 0
    LOW_WHEEL = 1
    LOW_WHEEL_REVERSE = 2
    STAND_HOLD = 3
    STAND_WHEEL = 4
    STAND_HOLD_ALT = 5
    GAIT_ONLY = 6
    GAIT_WHEEL = 7
    RESERVED_STAND = 8


def crc8(data: bytes) -> int:
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc


def topic_crc(topic: str) -> int:
    return crc32(topic.encode("ascii"))


def encode_topic_packet(topic: str, payload: bytes, timestamp_us: int) -> bytes:
    if len(payload) > 0xFFFFFF:
        raise ValueError("XRUSB Topic payload exceeds u24 length")
    if not 0 <= timestamp_us < 1 << 48:
        raise ValueError("XRUSB Topic timestamp exceeds u48 range")
    header = bytearray(PACKET_HEADER_SIZE)
    header[0] = PACKET_PREFIX
    header[1:4] = len(payload).to_bytes(3, "little")
    header[4:8] = topic_crc(topic).to_bytes(4, "little")
    header[8:14] = timestamp_us.to_bytes(6, "little")
    header[14] = PACKET_VERSION
    header[15] = crc8(header[:15])
    packet = bytes(header) + payload
    return packet + bytes((crc8(packet),))


@dataclass(frozen=True)
class ControlCommand:
    mode: RobotMode = RobotMode.MOTOR_CHECK
    yaw: float = 0.0
    forward: float = 0.0
    speed: float = 0.0
    motion_enable: bool = False
    deadman: bool = False
    smooth_stop: bool = False

    @staticmethod
    def safe_zero() -> "ControlCommand":
        return ControlCommand()

    def sanitized(self) -> "ControlCommand":
        speed = max(0.0, min(1.0, self.speed))
        command = replace(
            self,
            yaw=max(-1.0, min(1.0, self.yaw)),
            forward=max(-1.0, min(1.0, self.forward)),
            speed=speed,
        )
        if not command.motion_enable:
            return ControlCommand.safe_zero()
        if not command.deadman or command.smooth_stop:
            return replace(command, yaw=0.0, forward=0.0)
        return command


class ControlStream:
    def __init__(self, session_id: int | None = None) -> None:
        self.session_id = (secrets.randbits(32) if session_id is None else session_id) & 0xFFFFFFFF
        if self.session_id == 0:
            self.session_id = 1
        self.command_counter = 0

    def encode(self, command: ControlCommand, host_time_ms: int, timestamp_us: int) -> bytes:
        command = command.sanitized()
        flags = 0
        if command.deadman:
            flags |= DEADMAN
        if command.motion_enable:
            flags |= MOTION_ENABLE
        if command.smooth_stop:
            flags |= SMOOTH_STOP
        payload = _CONTROL.pack(
            SCHEMA_VERSION,
            int(command.mode),
            flags,
            round(command.yaw * 1000.0),
            round(command.forward * 1000.0),
            round(command.speed * 1000.0),
            0,
            self.session_id,
            self.command_counter,
            host_time_ms & 0xFFFFFFFF,
        )
        self.command_counter = (self.command_counter + 1) & 0xFFFFFFFF
        return encode_topic_packet(CONTROL_TOPIC, payload, timestamp_us)


@dataclass(frozen=True)
class RobotStatus:
    schema_version: int
    control_source: int
    requested_mode: int
    active_mode: int
    entry_state: int
    block_reason: int
    safety_latched: int
    obstacle_state: int
    obstacle_fault: int
    leg_online_mask: int
    wheel_online_mask: int
    fault_bits: int
    last_command_counter: int
    uptime_ms: int

    @classmethod
    def decode(cls, payload: bytes) -> "RobotStatus":
        if len(payload) != _STATUS.size:
            raise ValueError("RobotStatusV1 payload must be exactly 24 bytes")
        values = _STATUS.unpack(payload)
        if values[0] != SCHEMA_VERSION or values[11] != 0:
            raise ValueError("invalid RobotStatusV1 version or reserved byte")
        return cls(*values[:11], *values[12:])


class TopicPacketParser:
    def __init__(self, topic: str, payload_size: int) -> None:
        self._topic_crc = topic_crc(topic)
        self._payload_size = payload_size
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buffer.extend(data)
        payloads: list[bytes] = []
        while True:
            prefix = self._buffer.find(bytes((PACKET_PREFIX,)))
            if prefix < 0:
                self._buffer.clear()
                break
            if prefix:
                del self._buffer[:prefix]
            if len(self._buffer) < PACKET_HEADER_SIZE:
                break
            header = self._buffer[:PACKET_HEADER_SIZE]
            length = int.from_bytes(header[1:4], "little")
            valid_header = (
                header[14] == PACKET_VERSION
                and crc8(header[:15]) == header[15]
                and int.from_bytes(header[4:8], "little") == self._topic_crc
                and length == self._payload_size
            )
            if not valid_header:
                del self._buffer[0]
                continue
            packet_size = PACKET_BASE_SIZE + length
            if len(self._buffer) < packet_size:
                break
            packet = bytes(self._buffer[:packet_size])
            del self._buffer[:packet_size]
            if crc8(packet[:-1]) != packet[-1]:
                continue
            payloads.append(packet[PACKET_HEADER_SIZE:-1])
        return payloads


assert _CONTROL.size == 24
assert _STATUS.size == 24
