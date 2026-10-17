"""The wire format is the contract with the firmware; pin it down."""
import pytest

from controls import protocol


def test_decimal_line():
    # Frames for DShot values 1548 and 548: half forward and half reverse in 3-D.
    assert protocol.encode(49541, 17544) == b"49541 17544\n"


def test_hex_line():
    assert protocol.encode(0xC185, 0x4488, hex_mode=True) == b"0xC185 0x4488\n"


def test_round_trip_both_radixes():
    assert protocol.decode("49541 17544") == (49541, 17544)
    assert protocol.decode("0xC185 0x4488\r\n") == (0xC185, 0x4488)


def test_rejects_out_of_range_and_wrong_arity():
    with pytest.raises(ValueError):
        protocol.encode(65536, 0)
    with pytest.raises(ValueError):
        protocol.decode("1 2 3")
    with pytest.raises(ValueError):
        protocol.decode("1")


def test_firmware_reference_reads_what_the_app_writes():
    """The receiver in tools/ must decode a line the app just produced."""
    from firmware_reference import Hold

    from controls import dshot

    line = protocol.encode(dshot.frame(1548), dshot.frame(548)).decode()
    hold = Hold()
    row = hold.receive(line, 0.0)
    assert "throttle 1548" in row and "throttle 548" in row
    assert "BAD" not in row
    assert hold.values == (1548, 548)
    assert hold.receive("not a frame", 0.1).startswith("REJECT")


def test_firmware_reference_rejects_a_corrupt_checksum():
    """A flipped bit must not reach the ESCs."""
    from firmware_reference import Hold

    from controls import dshot

    hold = Hold()
    hold.receive(protocol.encode(dshot.frame(1548), dshot.frame(548)).decode(), 0.0)
    bad = protocol.encode(dshot.frame(1548) ^ 0x1, dshot.frame(548)).decode()
    assert "REJECT" in hold.receive(bad, 0.1)
    assert hold.values == (1548, 548)   # the previous packet is still held


def test_firmware_reference_holds_then_fails_safe():
    """Silence past the timeout forces the outputs to zero."""
    from firmware_reference import Hold

    from controls import dshot

    hold = Hold(timeout_s=2.0)
    hold.receive(protocol.encode(dshot.frame(1548), dshot.frame(548)).decode(), 0.0)
    assert hold.tick(1.9) is None           # still inside the hold
    assert hold.values == (1548, 548)
    assert "FAILSAFE" in hold.tick(2.0)
    assert hold.values == (0, 0)
    assert hold.tick(5.0) is None           # trips once, not every check
    assert hold.failsafe_trips == 1


def test_firmware_reference_resumes_after_a_failsafe():
    from firmware_reference import Hold

    from controls import dshot

    hold = Hold(timeout_s=1.0)
    line = protocol.encode(dshot.frame(1548), dshot.frame(548)).decode()
    hold.receive(line, 0.0)
    hold.tick(2.0)
    assert "RESUMED" in hold.receive(line, 2.1)
    assert hold.failsafe is False
    assert hold.values == (1548, 548)
