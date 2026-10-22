"""The packet panel must describe exactly what went on the wire."""
from controls import dshot, protocol
from controls.engine import TxFrame
from controls.packets import PacketView, describe


def make_frame(t, demand=(0.381, -0.319)):
    maps = [dshot.MotorMap(three_d=True), dshot.MotorMap(three_d=True)]
    values = tuple(m.value(d) for m, d in zip(maps, demand))
    words = tuple(dshot.frame(v) for v in values)
    line = protocol.encode(values[0], values[1])
    return TxFrame(t, demand, values, words, (False, False), True, line)


def test_describe_matches_the_frame():
    frame = make_frame(0.0)
    text = describe(frame.values[0], frame.words[0], frame.demand[0], 0)
    assert f"dshot {frame.values[0]}" in text
    assert f"0x{frame.words[0]:04X}" in text
    assert "crc" in text and "BAD" not in text


def test_describe_names_neutral():
    assert "3-D neutral" in describe(1048, dshot.frame(1048), 0.0, 0)


def test_describe_names_commands_and_disarm():
    assert "disarmed" in describe(0, dshot.frame(0), 0.0, 0)
    assert "command 1" in describe(1, dshot.frame(1), 0.0, 0)


def test_describe_flags_a_broken_checksum():
    bad = dshot.frame(1429) ^ 0x1
    assert "BAD" in describe(1429, bad, 0.0, 0)


def test_log_is_decimated_to_the_requested_rate(qt_app):
    view = PacketView()
    view.rate.setValue(10)
    before = view.log.blockCount()
    for step in range(200):          # one second at 200 Hz
        view.add(make_frame(step / 200.0))
    added = view.log.blockCount() - before
    assert 9 <= added <= 12


def test_burst_logs_every_frame(qt_app):
    view = PacketView()
    view.rate.setValue(1)
    view.burst_frames = 50
    view._start_burst()
    before = view.log.blockCount()
    for step in range(50):
        view.add(make_frame(step / 200.0))
    assert view.log.blockCount() - before >= 50


def test_freeze_stops_updating(qt_app):
    view = PacketView()
    view.pause.setChecked(True)
    before = view.log.blockCount()
    for step in range(200):
        view.add(make_frame(step / 200.0))
    assert view.log.blockCount() == before


def test_detail_shows_the_wire_bytes(qt_app):
    view = PacketView()
    frame = make_frame(0.0)
    view.add(frame)
    assert repr(frame.line) in view.detail.text()
