"""Serial transport. Opens the port, writes control frames, reads back lines.

Writes happen on the GUI thread from :class:`controls.engine.OutputEngine` and
are non-blocking by construction: the write timeout is short and a timeout is
counted as a dropped frame rather than stalling the UI. Reads run on a daemon
thread and reach Qt through a queued signal.
"""
from __future__ import annotations

import threading

from PySide6.QtCore import QObject, Signal

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Surfaced by the launcher; keep import of this module safe.
    serial = None
    list_ports = None

BAUD_RATES = (115200, 230400, 460800, 921600, 1000000, 2000000)
DEFAULT_BAUD = 921600
WRITE_TIMEOUT_S = 0.02


def available_ports() -> list[tuple[str, str]]:
    """Return ``(device, description)`` for every serial port on the machine."""
    if list_ports is None:
        return []
    return [(p.device, p.description or p.device) for p in list_ports.comports()]


class SerialLink(QObject):
    """A single serial port, with frame and byte counters for the status bar."""

    opened = Signal(str)
    closed = Signal(str)
    failed = Signal(str)
    received = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._port = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.bytes_sent = 0
        self.frames_sent = 0
        self.frames_dropped = 0

    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    @property
    def port_name(self) -> str:
        return self._port.port if self._port is not None else ""

    def open(self, device: str, baud: int = DEFAULT_BAUD) -> bool:
        """Open ``device``; emits :attr:`failed` and returns False on error."""
        if serial is None:
            self.failed.emit("pyserial is not installed")
            return False
        self.close("reopening")
        try:
            self._port = serial.Serial(
                device, baudrate=int(baud), timeout=0.1,
                write_timeout=WRITE_TIMEOUT_S,
            )
        except Exception as exc:  # pyserial raises several unrelated types.
            self._port = None
            self.failed.emit(f"{device}: {exc}")
            return False
        self.bytes_sent = self.frames_sent = self.frames_dropped = 0
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.opened.emit(device)
        return True

    def close(self, reason: str = "closed") -> None:
        """Stop the reader and release the port. Safe to call when not open."""
        self._stop.set()
        reader, self._reader = self._reader, None
        port, self._port = self._port, None
        if port is None:
            return
        try:
            port.close()
        except Exception:
            pass
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=0.5)
        self.closed.emit(reason)

    def write(self, payload: bytes) -> bool:
        """Write one frame. A timeout counts as a drop and never blocks the UI."""
        if not self.is_open:
            return False
        try:
            with self._lock:
                self._port.write(payload)
        except Exception:
            self.frames_dropped += 1
            return False
        self.bytes_sent += len(payload)
        self.frames_sent += 1
        return True

    def _read_loop(self) -> None:
        port = self._port
        buffer = bytearray()
        while not self._stop.is_set() and port is not None and port.is_open:
            try:
                chunk = port.read(port.in_waiting or 1)
            except Exception:
                break
            if not chunk:
                continue
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, rest = bytes(buffer).partition(b"\n")
                buffer = bytearray(rest)
                text = raw.decode("utf-8", "replace").rstrip("\r")
                if text:
                    self.received.emit(text)
