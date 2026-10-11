import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PySide6.QtWidgets import QApplication
from droopy.ui import Window


def test_force_integration_updates_freezes_and_resets(tmp_path):
    app=QApplication.instance() or QApplication([])
    window=Window();window.timer.stop();window.show()
    window.enabled.setChecked(True);window.advance(100)
    first=window.motion_view.canvas.data
    assert first.ticks==100
    assert max(abs(x) for x in first.residual_ns)<1e-10
    window.motion_view.capture_latest()
    window.advance(100)
    assert window.motion_view.canvas.data is first
    assert window.motion_view.latest.ticks==200
    window.motion_view.freeze.setChecked(False)
    assert window.motion_view.canvas.data.ticks==200
    assert window.motion_view.canvas.grab().save(str(tmp_path/'motion.png'))
    window.reset()
    assert window.motion_view.canvas.data is None
    assert window.impulse.snapshot() is None
    window.close()
