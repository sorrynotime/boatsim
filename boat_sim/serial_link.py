from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from .protocol import FrameParser, MotorCommand

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # The simulator remains usable in manual mode.
    serial = None


@dataclass
class LinkStats:
    received_frames: int = 0
    last_motor_monotonic: float = 0.0
    error: str = ""


class SerialLink:
    def __init__(self) -> None:
        self.port = None
        self.parser = FrameParser()
        self.stats = LinkStats()
        self.latest_motor: MotorCommand | None = None
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.running = False

    @staticmethod
    def available_ports() -> list[str]:
        if serial is None:
            return []
        return [item.device for item in serial.tools.list_ports.comports()]

    @property
    def connected(self) -> bool:
        return self.port is not None and self.port.is_open

    def connect(self, port_name: str) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; run: pip install -r requirements.txt")
        self.disconnect()
        self.port = serial.Serial(port_name, 921600, timeout=0.01, write_timeout=0.1)
        self.running = True
        self.stats = LinkStats()
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.running = False
        port, self.port = self.port, None
        if port is not None:
            port.close()

    def send(self, frame: bytes) -> None:
        port = self.port
        if port is not None and port.is_open:
            port.write(frame)

    def motor_command(self, timeout_s: float = 0.2) -> MotorCommand | None:
        with self.lock:
            if time.monotonic() - self.stats.last_motor_monotonic > timeout_s:
                return None
            return self.latest_motor

    def _read_loop(self) -> None:
        while self.running:
            try:
                port = self.port
                if port is None:
                    return
                data = port.read(max(1, port.in_waiting))
                for command in self.parser.feed_motor_commands(data):
                    with self.lock:
                        self.latest_motor = command
                        self.stats.received_frames += 1
                        self.stats.last_motor_monotonic = time.monotonic()
            except Exception as exc:
                self.stats.error = str(exc)
                self.running = False
                return
