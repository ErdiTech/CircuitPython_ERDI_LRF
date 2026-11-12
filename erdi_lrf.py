"""`erdi_lrf` - ERDI laser rangefinder driver for CircuitPython (busio.UART) and CPython.

Two UART protocol families, taken from the ERDI module manuals:

* Family B ("5C" stream): the module free-runs at 460800 baud.
    4-byte frame  5C <lo> <hi> <chk>            (all models except LRF1500H)
    5-byte frame  5C <lo> <mid> <hi> <chk>      (LRF1500H)
    chk = ~(sum of distance bytes) & 0xFF, distance little-endian.
    Unit: centimetres on H models, millimetres on VB models.
* Family A ("55 AA" command/response): 115200 baud, 8-byte frames.
    request  55 AA 88 FF FF FF FF 84            (single shot)
    reply    55 AA 88 STA FF DH DL SUM          (raw = DH<<8|DL, value x 10 m)

Pure parsing functions have no hardware dependency and run under plain CPython.
"""

__version__ = "0.1.0"
__repo__ = "https://github.com/ErdiTech/CircuitPython_ERDI_LRF.git"

import time

try:
    from time import monotonic_ns
except ImportError:  # pragma: no cover
    monotonic_ns = None

HDR_B = 0x5C
FN_SINGLE = 0x88

# model: (family, baud, unit_m_per_count, frame_len, out_of_range_raw, min_m, max_m)
# Family A raw counts are metres x10, so unit = 0.1 m.
MODELS = {
    "LR1000E2": ("A", 115200, 0.1, 8, None, 4.0, 1000.0),
    "LR1000E4": ("A", 115200, 0.1, 8, None, 4.0, 1000.0),
    "LR1200E2": ("A", 115200, 0.1, 8, None, 4.0, 1200.0),
    "LR1200E4": ("A", 115200, 0.1, 8, None, 4.0, 1200.0),
    "LR1500E2": ("A", 115200, 0.1, 8, None, 4.0, 1500.0),
    "LR1500E4": ("A", 115200, 0.1, 8, None, 4.0, 1500.0),
    "LR2000E2": ("A", 115200, 0.1, 8, None, 4.0, 2000.0),
    "LR2000E4": ("A", 115200, 0.1, 8, None, 4.0, 2000.0),
    "LRF1200A1": ("A", 115200, 0.1, 8, None, 5.0, 1200.0),
    "LRF3000A1": ("A", 115200, 0.1, 8, None, 1.0, 3000.0),
    "SPD1200N0": ("A", 115200, 0.1, 8, None, 0.2, 1200.0),
    "SPD1200N2": ("A", 115200, 0.1, 8, None, 0.2, 1200.0),
    "SPD1200N4": ("A", 115200, 0.1, 8, None, 0.2, 1200.0),
    "LRF50H": ("B", 460800, 0.01, 4, 65535, 0.05, 50.0),
    "LRF100H": ("B", 460800, 0.01, 4, 65535, 0.05, 100.0),
    "LRF200H": ("B", 460800, 0.01, 4, 65535, 0.05, 200.0),
    "LRF300H": ("B", 460800, 0.01, 4, 65535, 0.05, 300.0),
    "LRF600H": ("B", 460800, 0.01, 4, 65535, 0.05, 600.0),
    "LRF1500H": ("B", 460800, 0.01, 5, 16777215, 0.05, 1500.0),
    "LRF22VB": ("B", 460800, 0.001, 4, 20000, 0.05, 20.0),
    "LRF50VB": ("B", 460800, 0.001, 4, 50000, 0.05, 50.0),
}


class FrameError(Exception):
    pass


def checksum_b(payload):
    """Family B checksum: bitwise inverse of the byte sum, low 8 bits."""
    return (~sum(payload)) & 0xFF


def checksum_a_rx(frame_without_sum):
    """Family A reply checksum: low 8 bits of the sum including both header bytes."""
    return sum(frame_without_sum) & 0xFF


def checksum_a_tx(function, data):
    """Family A request checksum: low 8 bits of function + D1..D4 (header excluded)."""
    return (function + sum(data)) & 0xFF


def spec(model):
    try:
        return MODELS[model.upper()]
    except KeyError:
        raise ValueError("unknown ERDI model %r; known: %s" % (model, ", ".join(sorted(MODELS))))


def parse_b(frame, model):
    """Parse one family-B data frame. Returns metres, or None when out of range."""
    family, _baud, unit, flen, oor, _mn, _mx = spec(model)
    if family != "B":
        raise ValueError("%s is not a family B model" % model)
    if len(frame) != flen or frame[0] != HDR_B:
        raise FrameError("not a %d-byte 5C frame" % flen)
    if checksum_b(frame[1:-1]) != frame[-1]:
        raise FrameError("bad checksum")
    raw = int.from_bytes(bytes(frame[1:-1]), "little")
    if raw >= oor:
        return None
    return raw * unit


def single_shot_request():
    return bytes([0x55, 0xAA, FN_SINGLE, 0xFF, 0xFF, 0xFF, 0xFF, checksum_a_tx(FN_SINGLE, b"\xff\xff\xff\xff")])


def parse_a(frame, model):
    """Parse a family-A reply. Returns metres, or None if STA != 1 (no valid echo)."""
    family, _baud, unit, flen, _oor, _mn, _mx = spec(model)
    if family != "A":
        raise ValueError("%s is not a family A model" % model)
    if len(frame) != 8 or frame[0] != 0x55 or frame[1] != 0xAA:
        raise FrameError("not a 55 AA reply")
    if checksum_a_rx(frame[:-1]) != frame[-1]:
        raise FrameError("bad checksum")
    if frame[3] != 0x01:
        return None
    return ((frame[5] << 8) | frame[6]) * unit


def iter_b_frames(buf, model):
    """Yield (offset, metres_or_None) for each valid family-B frame in buf; resyncs on bad bytes."""
    flen = spec(model)[3]
    i = 0
    n = len(buf)
    while i <= n - flen:
        if buf[i] == HDR_B and checksum_b(buf[i + 1:i + flen - 1]) == buf[i + flen - 1]:
            yield i, parse_b(buf[i:i + flen], model)
            i += flen
        else:
            i += 1


def _ticks_ms():
    if monotonic_ns is not None:
        return monotonic_ns() // 1000000
    return int(time.monotonic() * 1000)  # pragma: no cover


class ErdiLRF:
    """Reader bound to a busio.UART (or any object with read(), write() and in_waiting).

    Create the UART with the model's documented baud rate and ``timeout=0``::

        uart = busio.UART(board.TX, board.RX, baudrate=460800, timeout=0)
        lrf = ErdiLRF(uart, "LRF22VB")
    """

    def __init__(self, uart, model="LRF22VB"):
        self.uart = uart
        self.model = model.upper()
        self.family, self.baud, self.unit, self.flen = spec(self.model)[:4]
        self._buf = bytearray()

    def _pull(self):
        n = self.uart.in_waiting
        if n:
            data = self.uart.read(n)
            if data:
                self._buf.extend(data)
                if len(self._buf) > 256:
                    del self._buf[:-self.flen * 4]

    def read_latest(self):
        """Family B: newest distance in metres (None = out of range or nothing yet)."""
        if self.family != "B":
            raise ValueError("read_latest() is for family B; use measure()")
        self._pull()
        latest = None
        end = 0
        for off, m in iter_b_frames(self._buf, self.model):
            latest = m
            end = off + self.flen
        if end:
            del self._buf[:end]
        return latest

    def measure(self, timeout_ms=500):
        """Family A: send a single-shot request and wait for the 8-byte reply."""
        if self.family != "A":
            raise ValueError("measure() is for family A; use read_latest()")
        self._buf = bytearray()
        self.uart.write(single_shot_request())
        t0 = _ticks_ms()
        while True:
            self._pull()
            for i in range(len(self._buf) - 7):
                if self._buf[i] == 0x55 and self._buf[i + 1] == 0xAA:
                    return parse_a(bytes(self._buf[i:i + 8]), self.model)
            if _ticks_ms() - t0 > timeout_ms:
                raise OSError("ERDI LRF: no reply within %d ms" % timeout_ms)
