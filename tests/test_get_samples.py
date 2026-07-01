"""
Hardware-free regression test for the PPK2 byte-framing desync (IRNAS issue #7,
also the desync symptoms in #45 / #53).

The framing logic is pure (bytes in, samples out), so it can be proven without a
device: decode the same synthetic stream as one big aligned read (the reference)
and as ragged 1-3 byte reads, and assert they are identical and that the
carried-over remainder length never goes negative.

Run:  ./.venv/bin/python -m pytest tests/ -q
"""
import random
import struct

from ppk2_api.ppk2_api import PPK2_API


def _make_api():
    """A PPK2_API instance without opening a serial port."""
    api = PPK2_API.__new__(PPK2_API)
    api.remainder = {"sequence": b"", "len": 0}
    return api


def _deterministic_decode(api):
    """Stub the per-word decode so the test exercises FRAMING, not calibration.

    Bound as instance attributes (not methods), so they receive only the value.
    Each 4-byte little-endian word -> (low 14-bit adc field, logic byte).
    """
    api._digital_to_analog = lambda b: int.from_bytes(b, "little", signed=False)
    api._handle_raw_data = lambda adc: (adc & 0x3FFF, (adc >> 24) & 0xFF)


def _encode(n):
    """n synthetic 4-byte sample words, each with a distinguishable adc + logic."""
    return b"".join(
        struct.pack("<I", ((i & 0xFF) << 24) | (i & 0x3FFF)) for i in range(n)
    )


def _ragged_sizes(total_len, lo, hi, rng):
    """Chunk sizes in [lo, hi] that sum exactly to total_len (last one clamped)."""
    sizes, remaining = [], total_len
    while remaining > 0:
        n = min(rng.randint(lo, hi), remaining)
        sizes.append(n)
        remaining -= n
    return sizes


def _decode_in_chunks(stream, chunk_sizes):
    api = _make_api()
    _deterministic_decode(api)
    samples, logic, min_len, max_len = [], [], 0, 0
    i = 0
    for n in chunk_sizes:
        s, b = api.get_samples(stream[i:i + n])
        i += n
        samples += s
        logic += b
        min_len = min(min_len, api.remainder["len"])
        max_len = max(max_len, api.remainder["len"])
    return samples, logic, min_len, max_len


def _reference(stream):
    """Ground truth: the whole stream decoded in one 4-aligned read."""
    api = _make_api()
    _deterministic_decode(api)
    s, b = api.get_samples(stream)
    assert api.remainder["len"] == 0
    return s, b


def test_aligned_read_no_regression():
    stream = _encode(2000)
    samples, logic, mn, mx = _decode_in_chunks(stream, [len(stream)])
    ref_s, ref_b = _reference(stream)
    assert samples == ref_s and logic == ref_b
    assert len(samples) == 2000
    assert 0 <= mn and mx <= 3


def test_ragged_small_reads_match_reference():
    stream = _encode(2000)
    sizes = _ragged_sizes(len(stream), 1, 3, random.Random(1))
    samples, logic, mn, mx = _decode_in_chunks(stream, sizes)
    ref_s, ref_b = _reference(stream)
    assert samples == ref_s, "ragged decode diverged from reference (desync)"
    assert logic == ref_b
    assert len(samples) == 2000
    assert mn >= 0, f"remainder len went negative ({mn}) -> desync bug present"
    assert mx <= 3


def test_single_subword_first_read_does_not_desync():
    # A 3-byte first read is the exact trigger that used to tip remainder["len"]
    # negative and corrupt every subsequent (even clean) read.
    stream = _encode(500)
    sizes = [3] + _ragged_sizes(len(stream) - 3, 400, 400, random.Random(0))
    samples, _logic, mn, mx = _decode_in_chunks(stream, sizes)
    ref_s, _ref_b = _reference(stream)
    assert samples == ref_s
    assert len(samples) == 500
    assert mn >= 0 and mx <= 3


def test_remainder_invariant_fuzz():
    for seed in range(50):
        stream = _encode(1000)
        sizes = _ragged_sizes(len(stream), 1, 7, random.Random(seed))
        samples, _logic, mn, mx = _decode_in_chunks(stream, sizes)
        ref_s, _ref_b = _reference(stream)
        assert samples == ref_s, f"seed {seed}: ragged decode diverged from reference"
        assert 0 <= mn and mx <= 3, f"seed {seed}: remainder invariant broken (min={mn}, max={mx})"
