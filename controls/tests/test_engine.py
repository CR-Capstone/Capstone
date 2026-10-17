"""Engine behaviour: arming gates output, and every tick writes one line."""
import pytest

from controls import dshot, protocol
from controls.engine import OutputEngine
from controls.link import SerialLink


class LoopbackLink(SerialLink):
    """A link that is always open and keeps what was written."""

    def __init__(self):
        super().__init__()
        self.lines = []
        self.open_flag = True

    @property
    def is_open(self):
        return self.open_flag

    def write(self, payload):
        self.lines.append(payload)
        self.frames_sent += 1
        return True


class Clock:
    """Hand-wound replacement for perf_counter, so keepalives are testable."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        # Rounded so a hundred 5 ms steps land exactly on 0.5 s, not 0.4999…
        self.now = round(self.now + seconds, 9)


@pytest.fixture
def clock(monkeypatch):
    from controls import engine as engine_module

    fake = Clock()
    monkeypatch.setattr(engine_module.time, "perf_counter", fake)
    return fake


@pytest.fixture
def engine(qt_app, clock):
    link = LoopbackLink()
    engine = OutputEngine(link)
    engine.stop()
    engine.clock = clock
    return engine


def last_values(engine):
    return protocol.decode(engine.link.lines[-1].decode())


def test_disarmed_ticks_send_zero(engine):
    engine.set_source(lambda t: (1.0, -1.0), "test")
    engine._tick()
    assert last_values(engine) == (dshot.frame(0), dshot.frame(0))


def test_disarmed_link_stays_alive_on_the_keepalive(engine):
    engine.config.keepalive_s = 0.5
    engine.set_source(None, "")
    engine.link.lines.clear()
    for _ in range(200):           # one second at 200 Hz
        engine.clock.advance(0.005)
        engine._tick()
    assert len(engine.link.lines) == 2
    assert all(
        protocol.decode(line.decode()) == (0, 0) for line in engine.link.lines
    )


def test_arm_requires_a_source_and_an_open_port(engine):
    assert engine.arm() is False
    engine.set_source(lambda t: (0.5, 0.5), "test")
    assert engine.arm() is True
    engine.link.open_flag = False
    engine.disarm("test")
    assert engine.arm() is False


def test_armed_tick_sends_mapped_values(engine):
    engine.config.maps[0] = dshot.MotorMap(three_d=True, deadband=0.0)
    engine.config.maps[1] = dshot.MotorMap(three_d=True, deadband=0.0)
    engine.set_source(lambda t: (1.0, -1.0), "test")
    engine.arm()
    engine._tick()
    assert last_values(engine) == (
        dshot.frame(dshot.FORWARD_MAX),
        dshot.frame(dshot.REVERSE_MAX),
    )


def test_source_exception_disarms(engine):
    def broken(t):
        raise RuntimeError("bad page")

    engine.set_source(broken, "test")
    engine.arm()
    engine._tick()
    assert engine.armed is False
    assert last_values(engine) == (dshot.frame(0), dshot.frame(0))


def test_changing_source_disarms(engine):
    engine.set_source(lambda t: (0.2, 0.2), "a")
    engine.arm()
    engine.set_source(lambda t: (0.3, 0.3), "b")
    assert engine.armed is False


def test_commands_repeat_then_stop(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.queue_command(dshot.Command.BEEP1, repeats=3)
    for _ in range(3):
        engine._tick()
        assert last_values(engine) == (dshot.frame(1), dshot.frame(1))
    engine._tick()
    assert last_values(engine) == (dshot.frame(0), dshot.frame(0))


def test_commands_are_refused_while_armed(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine.queue_command(dshot.Command.SAVE_SETTINGS)
    engine._tick()
    assert last_values(engine)[0] != dshot.frame(12)


def test_unchanged_values_are_held_not_resent(engine):
    """The firmware holds the last packet, so an identical frame is pointless."""
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    held_before = engine.frames_held
    for _ in range(20):
        engine.clock.advance(0.005)
        engine._tick()
    assert len(engine.link.lines) == before
    assert engine.frames_held == held_before + 20


def test_keepalive_resends_the_held_frame(engine):
    engine.config.keepalive_s = 0.5
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    for _ in range(100):           # 0.5 s at 200 Hz
        engine.clock.advance(0.005)
        engine._tick()
    assert len(engine.link.lines) == before + 1
    assert engine.last_frame.reason == "keepalive"
    assert engine.link.lines[-1] == engine.link.lines[before - 1]


def test_a_change_sends_on_the_next_evaluation(engine):
    demand = [0.5]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    demand[0] = 0.9
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1
    assert engine.last_frame.reason == "change"


def test_change_threshold_suppresses_small_moves(engine):
    engine.config.change_counts = 50
    demand = [0.50]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    demand[0] = 0.52               # ~20 counts in the 3-D range
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before
    demand[0] = 0.60               # ~100 counts, over the threshold
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1


def test_starting_or_stopping_a_motor_ignores_the_threshold(engine):
    engine.config.change_counts = 200
    engine.config.maps[0] = dshot.MotorMap(three_d=True, deadband=0.02)
    demand = [0.0]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    demand[0] = 0.03               # value 48-ish: tiny, but no longer zero
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1
    demand[0] = 0.0
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 2


def test_opening_the_port_forces_a_fresh_frame(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    engine.link.opened.emit("/dev/fake")
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1


def test_disarm_sends_immediately_and_repeats(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine._tick()
    before = len(engine.link.lines)
    engine.disarm("test")
    assert len(engine.link.lines) - before == 8
    assert last_values(engine) == (dshot.frame(0), dshot.frame(0))
    assert engine.last_frame.reason == "disarm"


def test_commands_bypass_change_detection(engine):
    """Ten identical command frames must all go out, not be held as duplicates."""
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.queue_command(dshot.Command.BEEP1, repeats=10)
    before = len(engine.link.lines)
    for _ in range(10):
        engine.clock.advance(0.005)
        engine._tick()
    assert len(engine.link.lines) == before + 10
    assert engine.last_frame.reason == "command"


def test_hex_mode_changes_the_wire_bytes(engine):
    engine.config.hex_mode = True
    engine.set_source(None, "")
    engine._tick()
    assert engine.link.lines[-1].startswith(b"0x")


def test_keepalive_margin_passes_with_room():
    from controls.engine import EngineConfig, keepalive_margin

    assert keepalive_margin(EngineConfig()) == ""
    assert keepalive_margin(EngineConfig(keepalive_s=0.66, hold_timeout_s=2.0)) == ""


def test_keepalive_margin_warns_when_one_loss_trips_the_failsafe():
    from controls.engine import EngineConfig, keepalive_margin

    warning = keepalive_margin(EngineConfig(keepalive_s=1.5, hold_timeout_s=2.0))
    assert "0 lost packets" in warning
    assert "0.67 s" in warning
    assert "1 lost packet " in keepalive_margin(
        EngineConfig(keepalive_s=0.9, hold_timeout_s=2.0)
    )
