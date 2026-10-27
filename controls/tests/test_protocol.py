"""The wire format is the contract with the sender ESP32; pin it down."""
import pytest

from controls import protocol


def test_decimal_line_of_bare_values():
    # ramp_test.py's 50% forward and 50% reverse.
    assert protocol.encode(1548, 548) == b"1548 548\n"
    assert protocol.encode(0, 2047) == b"0 2047\n"


def test_round_trip():
    assert protocol.decode("1548 548") == (1548, 548)
    assert protocol.decode("1048 1048\r\n") == (1048, 1048)


def test_rejects_what_the_sender_rejects():
    with pytest.raises(ValueError):
        protocol.encode(2048, 0)            # over 11 bits
    with pytest.raises(ValueError):
        protocol.encode(-1, 0)
    for bad in ("1 2 3", "1", "0xC185 0x4488", "1048  1048", "-5 5", "49541 17544"):
        with pytest.raises(ValueError):
            protocol.decode(bad)


def test_reference_receiver_forwards_what_the_app_writes():
    from firmware_reference import ACTIVE, Receiver

    receiver = Receiver()
    row = receiver.receive(protocol.encode(1548, 548).decode(), 0.0)
    assert "OUTPUT STARTED" in row
    assert receiver.state == ACTIVE
    assert receiver.output == (1548, 548)
    assert receiver.receive("not a line", 0.05).startswith("REJECT")
    assert receiver.output == (1548, 548)


def test_reference_receiver_fails_safe_then_locks_out():
    """A gap over 500 ms stops output, then every packet is ignored for 10 s."""
    from firmware_reference import IDLE, LOCKOUT, Receiver

    receiver = Receiver()
    line = protocol.encode(1548, 548).decode()
    receiver.receive(line, 0.0)
    assert receiver.tick(0.45) is None
    assert "FAILSAFE" in receiver.tick(0.55)
    assert receiver.state == LOCKOUT and receiver.output is None
    assert receiver.receive(line, 5.0).startswith("IGNORE")
    assert receiver.output is None
    assert "lockout over" in receiver.tick(10.6)
    assert receiver.state == IDLE
    assert "OUTPUT STARTED" in receiver.receive(line, 10.7)
    assert receiver.failsafe_trips == 1


def test_reference_receiver_measures_a_late_packet_by_its_arrival():
    from firmware_reference import LOCKOUT, Receiver

    receiver = Receiver()
    line = protocol.encode(1048, 1048).decode()
    receiver.receive(line, 0.0)
    row = receiver.receive(line, 0.6)
    assert "FAILSAFE" in row and "IGNORE" in row
    assert receiver.state == LOCKOUT


def test_app_keepalive_never_trips_the_reference_receiver(qt_app, monkeypatch):
    """Drive the real engine for 30 s and feed every line to the receiver model."""
    from firmware_reference import ACTIVE, Receiver

    from controls import engine as engine_module
    from test_engine import Clock, LoopbackLink

    clock = Clock()
    monkeypatch.setattr(engine_module.time, "perf_counter", clock)
    link = LoopbackLink()
    engine = engine_module.OutputEngine(link)
    engine.stop()
    engine.set_source(lambda t: (0.3, -0.3), "test")
    receiver = Receiver()
    armed_once = False
    for step in range(6000):                # 30 s at 200 Hz
        clock.advance(0.005)
        if step == 200:
            engine.arm()
        engine._tick()
        for line in link.lines:
            receiver.receive(line.decode(), clock.now)
        link.lines.clear()
        receiver.tick(clock.now)
        armed_once = armed_once or engine.state == "armed"
    assert armed_once and engine.state == "armed"
    assert receiver.failsafe_trips == 0
    assert receiver.state == ACTIVE
    assert receiver.output == (1049 + round(0.3 * 998), 48 + round(0.3 * 999))
