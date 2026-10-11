import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication

from droopy.impulse_vectors import (
    ImpulseVectorPanel, component_vector_sum, impulse_components,
    initial_velocity_components, velocity_components, velocity_trace,
)
from droopy.motion_view import IntegratedMotionView


def snapshot(**changes):
    values = dict(
        reference_xy=(0.0, 1.0), net_impulse_ns=(-2.0, 3.0, 9.0),
        initial_velocity_mps=(4.0, 5.0, 0.0), mass_kg=2.0,
        samples=((0.1, 1.0, -2.0, 0.4, -0.1), (0.2, 3.0, 2.0, 0.8, 0.0)),
        along_displacement_m=0.8, lateral_displacement_m=0.0,
        time_s=0.2, ticks=2, residual_ns=(0.0, 0.0, 0.0),
    )
    values.update(changes)
    return SimpleNamespace(**values)


def test_rotated_fixed_reference_projection_and_vector_sum():
    data = snapshot()
    # Along is world +Y and positive side is world -X for this rotated basis.
    assert impulse_components(data) == (3.0, 2.0)
    assert initial_velocity_components(data) == (5.0, -4.0)
    assert component_vector_sum(*impulse_components(data)) == (3.0, 2.0)


def test_zero_impulse_retains_initial_velocity_and_trace():
    data = snapshot(net_impulse_ns=(0.0, 0.0, 0.0),
                    samples=((0.1, 0.0, 0.0, 0.5, 0.0),))
    assert velocity_components(data) == initial_velocity_components(data) == (5.0, -4.0)
    assert velocity_trace(data) == ((0.1, 5.0, -4.0),)


def test_panel_renders_and_freeze_reset_share_snapshot(tmp_path):
    app = QApplication.instance() or QApplication([])
    view = IntegratedMotionView()
    first, second = snapshot(ticks=1), snapshot(ticks=2)
    view.update_snapshot(first)
    assert view.vector_panel.data is view.canvas.data is first
    assert view.vector_panel.grab().save(str(tmp_path / "vectors.png"))
    view.capture_latest()
    view.update_snapshot(second)
    assert view.vector_panel.data is view.canvas.data is first
    view.freeze.setChecked(False)
    assert view.vector_panel.data is view.canvas.data is second
    view.reset()
    assert view.vector_panel.data is None and view.canvas.data is None
    view.close()
