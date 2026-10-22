"""Engine behaviour: the arming sequence, change-driven sends, the receiver's rules."""
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
        if not self.open_flag:
            return False
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


def run(engine, seconds, step=0.005):
    for _ in range(round(seconds / step)):
        engine.clock.advance(step)
        engine._tick()


def warm_link(engine):
    """Stream long enough that the receiver is certainly out of any lockout."""
    run(engine, engine.config.lockout_s + 0.1)


def arm_fully(engine):
    warm_link(engine)
    assert engine.arm() is True
    run(engine, engine.config.arm_zero_s + engine.config.arm_neutral_s + 0.02)
    assert engine.state == "armed"


def test_disarmed_ticks_send_zero(engine):
    engine.set_source(lambda t: (1.0, -1.0), "test")
    engine._tick()
    assert last_values(engine) == (0, 0)


def test_lines_are_bare_dshot_values(engine):
    engine.set_source(lambda t: (0.5, -0.5), "test")
    arm_fully(engine)
    engine.clock.advance(0.005)
    engine._tick()
    assert engine.link.lines[-1] == b"1548 548\n"


def test_disarmed_link_stays_alive_on_the_keepalive(engine):
    engine.config.keepalive_s = 0.05
    engine.set_source(None, "")
    engine._tick()
    engine.link.lines.clear()
    run(engine, 1.0)
    assert len(engine.link.lines) == 20
    assert all(protocol.decode(line.decode()) == (0, 0) for line in engine.link.lines)


def test_default_keepalive_has_margin_against_the_receiver_failsafe(engine):
    from controls.engine import keepalive_margin

    assert engine.config.keepalive_s == pytest.approx(0.05)
    assert engine.config.hold_timeout_s == pytest.approx(0.5)
    assert keepalive_margin(engine.config) == ""


def test_arm_requires_a_source_and_an_open_port(engine):
    assert engine.arm() is False
    engine.set_source(lambda t: (0.5, 0.5), "test")
    assert engine.arm() is True
    engine.link.open_flag = False
    engine.disarm("test")
    assert engine.arm() is False


def test_arming_sends_zero_then_neutral_then_hands_over(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    warm_link(engine)
    engine.arm()
    assert engine.state == "arming"
    run(engine, 1.55)
    assert last_values(engine) == (0, 0)
    assert engine.last_frame.reason in ("arm", "keepalive")
    run(engine, 0.1)                        # 1.65 s: into the neutral phase
    assert last_values(engine) == (dshot.NEUTRAL_3D, dshot.NEUTRAL_3D)
    assert engine.state == "arming"
    run(engine, 0.4)                        # 2.05 s: the page has the output
    assert engine.state == "armed"
    assert last_values(engine) == (1548, 1548)


def test_arming_waits_out_a_possible_receiver_lockout(engine):
    """Nothing reached the receiver yet, so it may be locked out for 10 s."""
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    run(engine, 9.9)
    assert engine.state == "arming"
    assert engine.arming_status().startswith("lockout")
    assert {protocol.decode(l.decode()) for l in engine.link.lines} == {(0, 0)}
    run(engine, 0.2 + 1.6)                  # lockout over, then the full zero hold
    assert engine.arming_status().startswith("neutral")
    run(engine, 0.45)
    assert engine.state == "armed"


def test_page_clock_starts_when_arming_finishes(engine):
    seen = []

    def source(t):
        seen.append(t)
        return 0.2, 0.2

    engine.set_source(source, "test")
    arm_fully(engine)
    assert seen[0] == pytest.approx(0.0, abs=0.01)


def test_zero_demand_while_armed_is_neutral_not_disarm(engine):
    engine.set_source(lambda t: (0.0, 0.0), "test")
    arm_fully(engine)
    assert last_values(engine) == (dshot.NEUTRAL_3D, dshot.NEUTRAL_3D)


def test_armed_tick_sends_mapped_values(engine):
    engine.config.maps[0] = dshot.MotorMap(three_d=True, deadband=0.0)
    engine.config.maps[1] = dshot.MotorMap(three_d=True, deadband=0.0)
    engine.set_source(lambda t: (1.0, -1.0), "test")
    arm_fully(engine)
    assert last_values(engine) == (dshot.FORWARD_MAX, dshot.REVERSE_MAX)


def test_a_quiet_link_while_armed_disarms(engine):
    """The receiver has cut the ESCs; resuming would skip the arming sequence."""
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    engine.link.open_flag = False           # writes fail, nothing reaches it
    run(engine, 0.6)
    engine.link.open_flag = True
    run(engine, 0.06)                       # the next keepalive reaches it
    assert engine.armed is False
    assert last_values(engine) == (0, 0)


def test_rearming_after_a_gap_waits_for_the_lockout_again(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    engine.disarm("test")
    engine.link.open_flag = False
    run(engine, 1.0)                        # longer than the 0.5 s failsafe
    engine.link.open_flag = True
    engine.arm()
    run(engine, 5.0)
    assert engine.arming_status().startswith("lockout")


def test_source_exception_disarms(engine):
    def broken(t):
        raise RuntimeError("bad page")

    engine.set_source(broken, "test")
    warm_link(engine)
    engine.arm()
    run(engine, 2.1)                        # the source is first called here
    assert engine.armed is False
    assert last_values(engine) == (0, 0)


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
        assert last_values(engine) == (1, 1)
    engine._tick()
    assert last_values(engine) == (0, 0)


def test_commands_are_refused_while_armed(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.arm()
    engine.queue_command(dshot.Command.SAVE_SETTINGS)
    engine._tick()
    assert last_values(engine)[0] != 12


def test_unchanged_values_are_held_not_resent(engine):
    """The receiver repeats the last value, so an identical line is pointless."""
    engine.config.keepalive_s = 0.5
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    before = len(engine.link.lines)
    held_before = engine.frames_held
    run(engine, 0.1)
    assert len(engine.link.lines) == before
    assert engine.frames_held == held_before + 20


def test_keepalive_resends_the_held_frame(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    engine.config.keepalive_s = 0.25
    engine.clock.advance(0.005)
    engine._tick()                          # sends the keepalive now due
    before = len(engine.link.lines)
    run(engine, 0.25)
    assert len(engine.link.lines) == before + 1
    assert engine.last_frame.reason == "keepalive"
    assert engine.link.lines[-1] == b"1548 1548\n"


def test_a_change_sends_on_the_next_evaluation(engine):
    demand = [0.5]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    arm_fully(engine)
    before = len(engine.link.lines)
    demand[0] = 0.9
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1
    assert engine.last_frame.reason == "change"


def test_change_threshold_suppresses_small_moves(engine):
    demand = [0.50]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    arm_fully(engine)
    # Longer than the receiver failsafe: fine here, nothing measures gaps.
    engine.config.keepalive_s = 1.0
    engine.config.change_counts = 50
    before = len(engine.link.lines)
    demand[0] = 0.52               # ~20 counts in the 3-D range
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before
    demand[0] = 0.60               # ~100 counts, over the threshold
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1


def test_leaving_or_reaching_neutral_ignores_the_threshold(engine):
    engine.config.maps[0] = dshot.MotorMap(three_d=True, deadband=0.02)
    demand = [0.0]
    engine.set_source(lambda t: (demand[0], 0.0), "test")
    arm_fully(engine)
    # Longer than the receiver failsafe: fine here, nothing measures gaps.
    engine.config.keepalive_s = 1.0
    engine.config.change_counts = 200
    before = len(engine.link.lines)
    demand[0] = 0.03               # value 1079: tiny, but no longer neutral
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1
    demand[0] = 0.0
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 2
    assert last_values(engine)[0] == dshot.NEUTRAL_3D


def test_opening_the_port_forces_a_fresh_frame(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    # Longer than the receiver failsafe: fine here, nothing measures gaps.
    engine.config.keepalive_s = 1.0
    before = len(engine.link.lines)
    engine.link.opened.emit("/dev/fake")
    engine.clock.advance(0.005)
    engine._tick()
    assert len(engine.link.lines) == before + 1


def test_disarm_sends_immediately_and_repeats(engine):
    engine.set_source(lambda t: (0.5, 0.5), "test")
    arm_fully(engine)
    before = len(engine.link.lines)
    engine.disarm("test")
    assert len(engine.link.lines) - before == 8
    assert last_values(engine) == (0, 0)
    assert engine.last_frame.reason == "disarm"


def test_commands_bypass_change_detection(engine):
    """Every repeat of a command must go out, not be held as a duplicate."""
    engine.set_source(lambda t: (0.5, 0.5), "test")
    engine.queue_command(dshot.Command.BEEP1, repeats=10)
    before = len(engine.link.lines)
    run(engine, 0.05)
    assert len(engine.link.lines) == before + 10
    assert engine.last_frame.reason == "command"


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
