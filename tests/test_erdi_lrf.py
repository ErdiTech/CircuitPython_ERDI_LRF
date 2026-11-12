import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest

import erdi_lrf as e


def h(s):
    return bytes.fromhex(s.replace(" ", ""))


def test_manual_vector_h_series_is_centimetres():
    assert e.parse_b(h("5C 02 11 EC"), "LRF100H") == pytest.approx(43.54)


def test_same_bytes_on_vb_series_are_millimetres():
    assert e.parse_b(h("5C 02 11 EC"), "LRF22VB") == pytest.approx(4.354)


def test_lrf1500h_five_byte_frame():
    raw = 0x031102
    body = raw.to_bytes(3, "little")
    frame = bytes([0x5C]) + body + bytes([e.checksum_b(body)])
    assert e.parse_b(frame, "LRF1500H") == pytest.approx(raw * 0.01)


def test_out_of_range_marker():
    frame = bytes([0x5C, 0x20, 0x4E, e.checksum_b(b"\x20\x4e")])  # 20000 mm
    assert e.parse_b(frame, "LRF22VB") is None


def test_bad_checksum_rejected():
    with pytest.raises(e.FrameError):
        e.parse_b(h("5C 02 11 EB"), "LRF100H")


def test_family_a_request_matches_manual():
    assert e.single_shot_request() == h("55 AA 88 FF FF FF FF 84")


def test_family_a_reply_scales_by_ten():
    assert e.parse_a(h("55 AA 88 01 FF 00 2A B1"), "LRF3000A1") == pytest.approx(4.2)


def test_family_a_no_echo_returns_none():
    frame = bytearray(h("55 AA 88 00 FF 00 00 00"))
    frame[-1] = e.checksum_a_rx(frame[:-1])
    assert e.parse_a(bytes(frame), "LRF3000A1") is None


def test_stream_resync_after_garbage():
    good = h("5C 02 11 EC")
    buf = b"\x00\x5c\x01" + good + good
    out = list(e.iter_b_frames(buf, "LRF100H"))
    assert [round(m, 2) for _, m in out] == [43.54, 43.54]


class FakeUart:
    def __init__(self, data):
        self.data = bytearray(data)
        self.sent = b""

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, n):
        out = bytes(self.data[:n])
        del self.data[:n]
        return out

    def write(self, b):
        self.sent += bytes(b)


def test_reader_returns_newest_value():
    f1 = bytes([0x5C, 0x10, 0x00, e.checksum_b(b"\x10\x00")])
    f2 = bytes([0x5C, 0x20, 0x00, e.checksum_b(b"\x20\x00")])
    lrf = e.ErdiLRF(FakeUart(f1 + f2), "LRF100H")
    assert lrf.read_latest() == pytest.approx(0.32)


def test_single_shot_over_fake_uart():
    reply = h("55 AA 88 01 FF 00 2A B1")
    u = FakeUart(b"")
    u.write = lambda b: u.data.extend(reply)
    lrf = e.ErdiLRF(u, "LRF3000A1")
    assert lrf.measure() == pytest.approx(4.2)


def test_unknown_model():
    with pytest.raises(ValueError):
        e.spec("NOPE")


def test_measure_times_out_without_reply():
    lrf = e.ErdiLRF(FakeUart(b""), "LRF3000A1")
    with pytest.raises(OSError):
        lrf.measure(timeout_ms=30)
