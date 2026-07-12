from __future__ import annotations

import threading
import time
from collections.abc import Callable

import serial

try:
    from .quadruped_protocol import ControlStream, VirtualRemoteCommand
except ImportError:  # Direct script/module execution from the host directory.
    from quadruped_protocol import ControlStream, VirtualRemoteCommand


class QuadrupedSerialLink:
    PERIOD_S = 0.020

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._port = port
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._stream = ControlStream()
        self._command = VirtualRemoteCommand.safe_zero()
        self._command_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error_callback: Callable[[Exception], None] | None = None

    def set_error_callback(self, callback: Callable[[Exception], None]) -> None:
        self._error_callback = callback

    def open(self) -> None:
        if self._serial is not None:
            return
        self._serial = serial.Serial(
            self._port,
            self._baudrate,
            timeout=0.05,
            write_timeout=0.05,
        )
        self._stream = ControlStream()
        self._stop.clear()
        with self._command_lock:
            self._command = VirtualRemoteCommand.safe_zero()
        self._thread = threading.Thread(target=self._run, name="quadruped-50hz", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self.stop_motion()
        deadline = time.monotonic() + 0.08
        try:
            while self._serial is not None and self._serial.is_open and time.monotonic() < deadline:
                self._write_command(VirtualRemoteCommand.safe_zero())
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

    def set_command(self, command: VirtualRemoteCommand) -> None:
        with self._command_lock:
            self._command = command.sanitized()

    def stop_motion(self) -> None:
        with self._command_lock:
            self._command = VirtualRemoteCommand.safe_zero()

    def request_diagnostic(self, command: str, read_window_s: float = 0.25) -> str:
        if command not in ("p", "Y"):
            raise ValueError("diagnostic command must be 'p' or 'Y'")
        serial_port = self._require_open()
        with self._write_lock:
            serial_port.write(command.encode("ascii"))
            serial_port.flush()
        deadline = time.monotonic() + read_window_s
        chunks: list[bytes] = []
        while time.monotonic() < deadline:
            waiting = serial_port.in_waiting
            if waiting:
                chunks.append(serial_port.read(waiting))
            else:
                time.sleep(0.01)
        return b"".join(chunks).decode("utf-8", errors="replace")

    def _require_open(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial link is not open")
        return self._serial

    def _write_command(self, command: VirtualRemoteCommand) -> None:
        serial_port = self._require_open()
        host_time_ms = int(time.monotonic() * 1000.0)
        frame = self._stream.encode(command, host_time_ms)
        with self._write_lock:
            serial_port.write(frame)

    def _run(self) -> None:
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                with self._command_lock:
                    command = self._command
                self._write_command(command)
                deadline += self.PERIOD_S
                delay = deadline - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    deadline = time.monotonic()
        except Exception as exc:  # Serial failure must atomically stop the producer.
            self._stop.set()
            with self._command_lock:
                self._command = VirtualRemoteCommand.safe_zero()
            if self._error_callback is not None:
                self._error_callback(exc)
