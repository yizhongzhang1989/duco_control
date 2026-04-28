"""Low-level driver for the Duco 6-DoF force/torque sensor.

Wire protocol (matches the manufacturer's C# sample, NOT the 12-byte int format
written in section 4.3 of the Chinese OCR'd manual which appears to apply to a
different firmware variant):

    Serial:  460800 baud, 8N1, no flow control
    Frame:   28 bytes
               [0]    = 0x48           header byte 1
               [1]    = 0xAA           header byte 2
               [2:26] = 6 x float32 LE (Fx, Fy, Fz, Mx, My, Mz)
               [26]   = 0x0D           tail
               [27]   = 0x0A           tail
             Final values are float * 10  ->  Fx/Fy/Fz in N, Mx/My/Mz in N*m.

Host -> sensor commands (all 4 bytes, sent as-is over the serial line):

    0x43 0xAA 0x0D 0x0A   stop streaming
    0x47 0xAA 0x0D 0x0A   tare (zero) then stream @ 960 Hz
    0x48 0xAA 0x0D 0x0A   stream @ 960 Hz (no tare)
    0x49 0xAA 0x0D 0x0A   one shot
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


# --- protocol constants ----------------------------------------------------
FRAME_LEN = 28
HEADER0 = 0x48
HEADER1 = 0xAA
TAIL0 = 0x0D
TAIL1 = 0x0A

CMD_STOP = b"\x43\xAA\x0D\x0A"
CMD_TARE_STREAM = b"\x47\xAA\x0D\x0A"
CMD_STREAM = b"\x48\xAA\x0D\x0A"
CMD_ONESHOT = b"\x49\xAA\x0D\x0A"

DEFAULT_BAUD = 460800

# Manufacturer's C# sample multiplies each float by 10. Empirically this yields
# Newtons for Fx/Fy/Fz and N*m for Mx/My/Mz.
SCALE = 10.0

_FLOATS = struct.Struct("<6f")


@dataclass
class Wrench:
    """Six-axis wrench in SI units (N and N*m)."""
    fx: float
    fy: float
    fz: float
    tx: float
    ty: float
    tz: float


def parse_frame(frame: bytes) -> Wrench:
    """Decode a 28-byte frame into a Wrench in N / N*m."""
    if len(frame) != FRAME_LEN:
        raise ValueError(f"frame must be {FRAME_LEN} bytes, got {len(frame)}")
    if frame[0] != HEADER0 or frame[1] != HEADER1 or frame[26] != TAIL0 or frame[27] != TAIL1:
        raise ValueError("frame does not match 0x48 0xAA ... 0x0D 0x0A")
    fx, fy, fz, tx, ty, tz = _FLOATS.unpack_from(frame, 2)
    return Wrench(fx * SCALE, fy * SCALE, fz * SCALE,
                  tx * SCALE, ty * SCALE, tz * SCALE)


class FTSensor:
    """Blocking serial driver. Open, send a streaming command, then call
    :meth:`read_wrench` in a loop. Use :meth:`close` (or a context manager)
    to stop the stream and release the port."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = DEFAULT_BAUD,
                 timeout: float = 0.1):
        self._ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=0.5,
        )
        self._buf = bytearray()
        self.dropped_bytes = 0
        # Stop any prior stream so we start in a known state.
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._ser.write(CMD_STOP)
        self._ser.flush()
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    # --- context manager ---------------------------------------------------
    def __enter__(self) -> "FTSensor":
        return self

    def __exit__(self, *_):
        self.close()

    # --- commands ----------------------------------------------------------
    def start_stream(self, tare: bool = False) -> None:
        """Start the 960 Hz stream. If ``tare`` is true, the sensor zeroes first."""
        self._ser.reset_input_buffer()
        self._buf.clear()
        self._ser.write(CMD_TARE_STREAM if tare else CMD_STREAM)
        self._ser.flush()

    def stop_stream(self) -> None:
        """Tell the sensor to stop sending data."""
        try:
            self._ser.write(CMD_STOP)
            self._ser.flush()
        except serial.SerialException:
            pass

    def tare(self) -> None:
        """Zero the sensor (and continue streaming)."""
        self.start_stream(tare=True)

    def close(self) -> None:
        """Stop the stream and close the serial port."""
        self.stop_stream()
        try:
            self._ser.close()
        except serial.SerialException:
            pass

    # --- reading -----------------------------------------------------------
    def read_wrench(self) -> Optional[Wrench]:
        """Return the next decoded :class:`Wrench`.

        Drains the internal buffer one frame at a time (so calling this in a
        tight loop yields every frame the sensor sent — no dropping). When the
        buffer doesn't yet contain a full frame, performs a single bounded
        serial read; returns ``None`` if no complete frame is available within
        the underlying serial timeout. Resyncs automatically on misaligned
        data.
        """
        # 1) Try to extract one complete frame already sitting in the buffer.
        frame = self._pop_frame()
        if frame is not None:
            return parse_frame(frame)

        # 2) Otherwise pull more bytes from the serial port (one read).
        chunk = self._ser.read(1024)
        if chunk:
            self._buf.extend(chunk)
        frame = self._pop_frame()
        if frame is None:
            return None
        return parse_frame(frame)

    def _pop_frame(self) -> Optional[bytes]:
        """Remove and return the first complete 28-byte frame from the buffer,
        skipping any out-of-sync bytes (counted in :attr:`dropped_bytes`).
        Returns ``None`` if the buffer doesn't yet contain a full frame."""
        buf = self._buf
        while len(buf) >= FRAME_LEN:
            if (buf[0] == HEADER0 and buf[1] == HEADER1
                    and buf[26] == TAIL0 and buf[27] == TAIL1):
                frame = bytes(buf[:FRAME_LEN])
                del buf[:FRAME_LEN]
                return frame
            del buf[0]
            self.dropped_bytes += 1
        return None
