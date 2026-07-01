"""
Hardware-free regression test for _read_metadata() dropping split calibration
chunks (IRNAS issue #61).

The PPK2 sends its calibration metadata as a multi-line ASCII string ending in
'END', which the USB serial driver may split across several reads (observed on
macOS). We feed a split stream through a fake serial port and assert the full
metadata survives and the parsed ADC zero-point offset is preserved.

Run:  ./.venv/bin/python -m pytest tests/ -q
"""
from ppk2_api.ppk2_api import PPK2_API


def _make_api():
    """A PPK2_API instance without opening a serial port."""
    api = PPK2_API.__new__(PPK2_API)
    api.remainder = {"sequence": b"", "len": 0}
    return api


class _FakeSerial:
    """Yields pre-canned read chunks to simulate a split metadata stream."""
    def __init__(self, chunks):
        self._chunks = list(chunks)

    @property
    def in_waiting(self):
        return len(self._chunks[0]) if self._chunks else 0

    def read(self, _n):
        return self._chunks.pop(0) if self._chunks else b""


def _no_sleep(monkeypatch):
    monkeypatch.setattr("ppk2_api.ppk2_api.time.sleep", lambda *a, **k: None)


def test_read_metadata_accumulates_split_chunks(monkeypatch):
    _no_sleep(monkeypatch)
    api = _make_api()
    # 'END' arrives in the second read; the first read's fields must survive.
    api.ser = _FakeSerial([b"Calibrated: 0\nR0: 1000\nO0: 166\n", b"S0: 2\nHW: 1\nEND\n"])
    meta = api._read_metadata()
    for field in ("Calibrated", "R0", "O0", "S0", "HW", "END"):
        assert field in meta, f"{field} dropped from split metadata"


def test_split_metadata_preserves_offset(monkeypatch):
    _no_sleep(monkeypatch)
    api = _make_api()
    # the modifiers dict _parse_metadata writes into (defaults are 0)
    api.modifiers = {
        "Calibrated": None,
        "R": {str(i): 0.0 for i in range(5)},
        "GS": {str(i): 0.0 for i in range(5)},
        "GI": {str(i): 0.0 for i in range(5)},
        "O": {str(i): 0.0 for i in range(5)},
        "S": {str(i): 0.0 for i in range(5)},
        "I": {str(i): 0.0 for i in range(5)},
        "UG": {str(i): 0.0 for i in range(5)},
        "HW": None,
        "IA": None,
    }
    # O0 (the ADC zero-point offset) arrives in the FIRST read, END in the second.
    api.ser = _FakeSerial([b"Calibrated: 1\nO0: 166.0\n", b"R0: 1000.0\nEND\n"])
    api._parse_metadata(api._read_metadata())
    # Under the old drop-chunks bug this stayed 0.0 -> ~1.8 uA low-range offset.
    assert api.modifiers["O"]["0"] == 166.0
    assert api.modifiers["R"]["0"] == 1000.0
