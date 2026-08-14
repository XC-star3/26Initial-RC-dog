from __future__ import annotations

import threading
import time
from collections.abc import Callable

import serial

if __package__:
    from .xrusb_codec import (
        ControlCommand,
        ControlStream,
        RobotStatus,
        STATUS_PAYLOAD_SIZE,
        STATUS_TOPIC,
        TopicPacketParser,
    )
else:  # Direct script/module execution from the host directory.
    from xrusb_codec import (
        ControlCommand,
        ControlStream,
        RobotStatus,
        STATUS_PAYLOAD_SIZE,
        STATUS_TOPIC,
        TopicPacketParser,
    )


class QuadrupedSerialLink:
    PERIOD_S = 0.020

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._port = port
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._stream = ControlStream()
        self._command = ControlCommand.safe_zero()
        self._parser = TopicPacketParser(STATUS_TOPIC, STATUS_PAYLOAD_SIZE)
        self._status: RobotStatus | None = None
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error_callback: Callable[[Exception], None] | None = None
        self._status_callback: Callable[[RobotStatus], None] | None = None

    def set_error_callback(self, callback: Callable[[Exception], None]) -> None:
        self._error_callback = callback

    def set_status_callback(self, callback: Callable[[RobotStatus], None]) -> None:
        self._status_callback = callback

    @property
    def latest_status(self) -> RobotStatus | None:
        with self._state_lock:
            return self._status

    def open(self) -> None:
        if self._serial is not None:
            return
        self._serial = serial.Serial(
            self._port,
            self._baudrate,
            timeout=0.005,
            write_timeout=0.05,
        )
        self._stream = ControlStream()
        self._parser = TopicPacketParser(STATUS_TOPIC, STATUS_PAYLOAD_SIZE)
        self._stop.clear()
        with self._state_lock:
            self._command = ControlCommand.safe_zero()
            self._status = None
        self._thread = threading.Thread(target=self._run, name="quadruped-50hz", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self.stop_motion()
        deadline = time.monotonic() + 0.08
        try:
            while self._serial is not None and self._serial.is_open and time.monotonic() < deadline:
                self._write_command(ControlCommand.safe_zero())
                time.sleep(self.PERIOD_S)
        except serial.SerialException:
            pass
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
            self._serial = None

    def set_command(self, command: ControlCommand) -> None:
        with self._state_lock:
            self._command = command.sanitized()

    def stop_motion(self) -> None:
        with self._state_lock:
            self._command = ControlCommand.safe_zero()

    def _require_open(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial link is not open")
        return self._serial

    def _write_command(self, command: ControlCommand) -> None:
        serial_port = self._require_open()
        monotonic_ns = time.monotonic_ns()
        frame = self._stream.encode(
            command,
            monotonic_ns // 1_000_000,
            (monotonic_ns // 1_000) & ((1 << 48) - 1),
        )
        with self._write_lock:
            serial_port.write(frame)

    def _run(self) -> None:
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                with self._state_lock:
                    command = self._command
                self._write_command(command)
                serial_port = self._require_open()
                incoming = serial_port.read(serial_port.in_waiting or 1)
                for payload in self._parser.feed(incoming):
                    status = RobotStatus.decode(payload)
                    with self._state_lock:
                        self._status = status
                    if self._status_callback is not None:
                        self._status_callback(status)
                deadline += self.PERIOD_S
                delay = deadline - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    deadline = time.monotonic()
        except Exception as exc:  # Serial failure must atomically stop the producer.
            self._stop.set()
            with self._state_lock:
                self._command = ControlCommand.safe_zero()
            if self._error_callback is not None:
                self._error_callback(exc)
