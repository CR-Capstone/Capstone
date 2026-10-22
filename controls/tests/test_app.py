"""The window builds, every page produces finite demand, and arming is gated."""
import math

from controls.app import MainWindow
from controls.pages import PAGES


def test_window_builds_with_every_page(qt_app):
    window = MainWindow()
    try:
        assert window.tabs.count() == len(PAGES)
        for index in range(window.tabs.count()):
            window.tabs.setCurrentIndex(index)
            page = window.pages[index]
            for step in range(8):
                demand = page.sample(step * 0.05)
                assert len(demand) == 2
                assert all(math.isfinite(v) and -1.0 <= v <= 1.0 for v in demand)
    finally:
        window.close()


def test_arming_is_disabled_without_a_port(qt_app):
    window = MainWindow()
    try:
        window.tabs.setCurrentIndex(2)
        assert window.arm_button.isEnabled() is False
        assert window.engine.arm() is False
    finally:
        window.close()


def test_link_page_provides_no_output(qt_app):
    window = MainWindow()
    try:
        window.tabs.setCurrentIndex(0)
        assert window.pages[0].provides_output is False
        assert window.engine._source is None
    finally:
        window.close()


def test_bench_page_offers_every_shipped_test(qt_app):
    from controls.sequences import TESTS

    window = MainWindow()
    try:
        bench = window.pages[1]
        names = [bench.tests.item(i).text() for i in range(bench.tests.count())]
        assert names[: len(TESTS)] == [s.name for s in TESTS]
        assert names[-1].startswith("Custom ramp")
        for row in range(bench.tests.count()):
            bench.tests.setCurrentRow(row)
            sequence = bench.selected_sequence()
            assert sequence is not None and sequence.duration() > 0.0
    finally:
        window.close()


def test_running_a_bench_test_drives_the_output(qt_app):
    window = MainWindow()
    try:
        bench = window.pages[1]
        window.tabs.setCurrentIndex(1)
        names = [bench.tests.item(i).text() for i in range(bench.tests.count())]
        bench.tests.setCurrentRow(names.index("Counter-rotate check"))
        bench.run_button.setChecked(True)
        assert bench.sample(0.0) == (0.0, 0.0)  # settles first
        assert bench.sample(3.0) == (0.12, -0.12)
        bench.run_button.setChecked(False)
        assert bench.sample(3.0) == (0.0, 0.0)  # sliders are back, at zero
    finally:
        window.close()
