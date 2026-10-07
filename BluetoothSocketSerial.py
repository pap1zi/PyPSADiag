"""
BluetoothSocketSerial.py

Provides a drop-in replacement for pyserial's Serial class using a native Linux
Bluetooth RFCOMM socket (socket.AF_BLUETOOTH, socket.BTPROTO_RFCOMM).

Enables connecting to ELM327 / OBD2 Bluetooth adapters directly by MAC address
without needing `rfcomm bind` or root privileges.
"""
from __future__ import annotations

import array
import fcntl
import os
import re
import select
import socket
import subprocess
import termios
import time
from typing import List, Optional, Tuple

try:
    import serial
    SerialException = serial.SerialException
    SerialTimeoutException = serial.SerialTimeoutException
except ImportError:
    class SerialException(Exception):
        """Serial port exception fallback."""
        pass

    class SerialTimeoutException(SerialException):
        """Serial port timeout exception fallback."""
        pass


MAC_REGEX = re.compile(r"([0-9A-Fa-f]{2}[:-][0-9A-Fa-f]{2}[:-][0-9A-Fa-f]{2}[:-][0-9A-Fa-f]{2}[:-][0-9A-Fa-f]{2}[:-][0-9A-Fa-f]{2})")
OBD_KEYWORDS = (
    "obd", "elm", "vlinker", "vgate", "vieocar", "psa", "diag", "car", "scanner", "ecu"
)


def parse_mac_and_channel(text: str) -> Tuple[Optional[str], int]:
    """Extract and normalize MAC address and RFCOMM channel from an input string.

    Examples:
        '00:1D:A5:68:98:8B' -> ('00:1D:A5:68:98:8B', 1)
        '00:1D:A5:68:98:8B:2' -> ('00:1D:A5:68:98:8B', 2)
        'OBDII [00-1D-A5-68-98-8B]' -> ('00:1D:A5:68:98:8B', 1)
    """
    if not text:
        return None, 1

    clean_text = str(text).strip()
    match = MAC_REGEX.search(clean_text)
    if not match:
        return None, 1

    raw_mac = match.group(1).replace("-", ":").upper()

    channel = 1
    # Check if a channel suffix was specified after the MAC (e.g., MAC:channel)
    suffix = clean_text[match.end():]
    channel_match = re.search(r"^:(\d+)", suffix)
    if channel_match:
        try:
            channel = int(channel_match.group(1))
        except ValueError:
            channel = 1

    return raw_mac, channel


def discover_bluetooth_devices(timeout: float = 3.0) -> List[Tuple[str, str]]:
    """Scan and return discovered Bluetooth devices on Linux via bluetoothctl.

    Returns:
        List of tuples: (mac_address, device_name), prioritized so OBD adapters appear first.
    """
    devices_dict: dict[str, str] = {}

    def _query_bluetoothctl(args: List[str]) -> str:
        try:
            res = subprocess.run(
                ["bluetoothctl", *args],
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False
            )
            return res.stdout or ""
        except (subprocess.SubprocessError, FileNotFoundError, OSError):
            return ""

    device_pattern = re.compile(
        r"Device\s+([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5})\s+(.*)"
    )

    # 1. Query paired devices first
    out_paired = _query_bluetoothctl(["devices", "Paired"])
    for line in out_paired.splitlines():
        m = device_pattern.search(line.strip())
        if m:
            mac, name = m.group(1).upper(), m.group(2).strip()
            devices_dict[mac] = name

    # 2. Query all known/cached devices
    out_all = _query_bluetoothctl(["devices"])
    for line in out_all.splitlines():
        m = device_pattern.search(line.strip())
        if m:
            mac, name = m.group(1).upper(), m.group(2).strip()
            if mac not in devices_dict:
                devices_dict[mac] = name

    # 3. Sort: devices with OBD/ELM keywords first, then alphabetically
    def _sort_key(item: Tuple[str, str]) -> Tuple[int, str]:
        mac, name = item
        name_lower = name.lower()
        is_obd = any(kw in name_lower for kw in OBD_KEYWORDS)
        return (0 if is_obd else 1, name_lower)

    return sorted(devices_dict.items(), key=_sort_key)


class BluetoothSocketSerial:
    """Emulates pyserial's Serial interface using a Linux Bluetooth RFCOMM socket.

    Provides compatibility with serial.Serial so higher-level diagnostic and
    sniffer tools (Elm327Sniffer, CanFrameSniffDialog, PyPSADiag) operate transparently.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: Optional[float] = 3.0,
        write_timeout: Optional[float] = 5.0,
        channel: int = 1,
        sock: Optional[socket.socket] = None,
    ):
        self.port: Optional[str] = port
        self.baudrate: int = baudrate
        self.timeout: Optional[float] = timeout
        self.write_timeout: Optional[float] = write_timeout
        self.channel: int = channel

        self._sock: Optional[socket.socket] = sock
        self._is_open: bool = sock is not None
        if self._sock is not None:
            try:
                self._sock.setblocking(False)
            except Exception:
                pass

    @property
    def is_open(self) -> bool:
        return self._is_open and self._sock is not None

    def isOpen(self) -> bool:
        return self.is_open

    @property
    def in_waiting(self) -> int:
        """Returns the number of bytes available in the receive buffer."""
        if not self.is_open or self._sock is None:
            return 0
        try:
            buf = array.array("i", [0])
            fionread = getattr(termios, "FIONREAD", 0x541B)
            fcntl.ioctl(self._sock.fileno(), fionread, buf)
            return max(0, buf[0])
        except Exception:
            try:
                r, _, _ = select.select([self._sock], [], [], 0)
                return 1 if r else 0
            except Exception:
                return 0

    def open(self) -> None:
        """Open the Bluetooth RFCOMM socket connection."""
        if self.is_open:
            return

        if not hasattr(socket, "AF_BLUETOOTH") or not hasattr(socket, "BTPROTO_RFCOMM"):
            raise SerialException(
                "Native Bluetooth RFCOMM sockets (socket.AF_BLUETOOTH) are not supported "
                "by this Python build / OS. A Linux system with BlueZ is required."
            )

        if not self.port:
            raise SerialException("No Bluetooth device / MAC address specified.")

        mac_addr, ch = parse_mac_and_channel(self.port)
        if not mac_addr:
            raise SerialException(f"Invalid Bluetooth MAC address: {self.port!r}")

        channel = ch if ch > 0 else (self.channel or 1)

        try:
            self._sock = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
            )
            # Socket connection deadline
            conn_timeout = self.timeout or 5.0
            self._sock.settimeout(conn_timeout)
            self._sock.connect((mac_addr, channel))

            # Set non-blocking mode for select-driven timeout management
            self._sock.setblocking(False)
            self._is_open = True
        except (socket.error, OSError) as e:
            self.close()
            raise SerialException(
                f"Failed to connect to Bluetooth device {mac_addr} on channel {channel}: {e}"
            )

    def close(self) -> None:
        """Close the socket connection."""
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._is_open = False

    def write(self, data: bytes) -> int:
        """Write bytes to the Bluetooth socket."""
        if not self.is_open or self._sock is None:
            raise SerialException("Socket is not open")

        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)

        deadline = time.monotonic() + (self.write_timeout or 5.0)
        total_sent = 0
        while total_sent < len(data):
            if self.write_timeout is not None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    raise SerialTimeoutException("Socket write timeout")
                _, w, _ = select.select([], [self._sock], [], max(0.0, rem))
                if not w:
                    raise SerialTimeoutException("Socket write timeout")
            else:
                _, w, _ = select.select([], [self._sock], [], None)
                if not w:
                    raise SerialTimeoutException("Socket write timeout")

            try:
                sent = self._sock.send(data[total_sent:])
                if sent == 0:
                    self.close()
                    raise SerialException("Socket closed by peer during write")
                total_sent += sent
            except (BlockingIOError, socket.timeout):
                continue
            except (socket.error, OSError) as e:
                self.close()
                raise SerialException(f"Socket write error: {e}")

        return total_sent

    def read(self, size: int = 1) -> bytes:
        """Read up to `size` bytes from the Bluetooth socket respecting `self.timeout`."""
        if not self.is_open or self._sock is None:
            raise SerialException("Socket is not open")

        if size <= 0:
            return b""

        deadline = time.monotonic() + (self.timeout if self.timeout is not None else 0.0)
        while True:
            if self.timeout is not None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return b""
                r, _, _ = select.select([self._sock], [], [], max(0.0, rem))
                if not r:
                    return b""
            else:
                r, _, _ = select.select([self._sock], [], [], None)
                if not r:
                    return b""

            try:
                chunk = self._sock.recv(size)
                if not chunk:
                    # Peer closed connection
                    self.close()
                    return b""
                return chunk
            except (BlockingIOError, socket.timeout):
                continue
            except (socket.error, OSError) as e:
                self.close()
                raise SerialException(f"Socket read error: {e}")

    def reset_input_buffer(self) -> None:
        """Flush/drain all unread incoming data from the socket."""
        if not self.is_open or self._sock is None:
            return
        try:
            while True:
                r, _, _ = select.select([self._sock], [], [], 0)
                if not r:
                    break
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
        except Exception:
            pass

    def reset_output_buffer(self) -> None:
        """No-op for TCP/RFCOMM socket."""
        pass

    def flush(self) -> None:
        """Flush output buffer (no-op on socket)."""
        pass
