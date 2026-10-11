import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from dataclasses import replace
import numpy as np
from PySide6.QtWidgets import QApplication
from droopy.physics import Simulator
from droopy.torque_vectors import torque_vectors


def test_exact_tick_torque_rotation_and_sum():
    sim=Simulator();sim.step((220.,-220.))
    d=replace(sim.last_diagnostic,rotation_before=(0.,-1.,0.,1.,0.,0.,0.,0.,1.),
              motor_reaction_nm=(1.,2.,3.),gyroscopic_nm=(-2.,1.,-3.),contact_torque_body_nm=(4.,-1.,2.))
    vectors=torque_vectors(d)
    assert len(vectors)==4
    assert vectors[0].world_nm==(-2.,1.,3.)
    np.testing.assert_allclose(vectors[-1].body_nm,(3.,2.,2.))
    np.testing.assert_allclose(vectors[-1].world_nm,np.sum([v.world_nm for v in vectors[:-1]],axis=0))
    assert torque_vectors(None)==()


def test_arena_torque_toggle_and_click_targets(tmp_path):
    from droopy.ui import Window
    app=QApplication.instance() or QApplication([])
    w=Window();w.timer.stop();w.enabled.setChecked(True);w.advance(20)
    w.show();app.processEvents()
    assert w.arena.grab().save(str(tmp_path/'torques.png'))
    assert any(key=='rotation' for key,a,b in w.arena._vector_hits)
    w.arena.top=True;w.arena.grab()
    assert any(key=='rotation' for key,a,b in w.arena._vector_hits)
    w.torques_check.setChecked(False);w.arena.grab()
    assert not any(key=='rotation' for key,a,b in w.arena._vector_hits)
    w.torques_check.setChecked(True);w.vectors_check.setChecked(False);w.arena.grab()
    assert not w.arena._vector_hits
    w.reset();w.vectors_check.setChecked(True);w.arena.grab()
    assert not any(key=='rotation' for key,a,b in w.arena._vector_hits)
    w.close()


def test_frozen_system_inspects_displayed_tick():
    from droopy.ui import Window
    app=QApplication.instance() or QApplication([])
    w=Window();w.timer.stop();w.advance(20)
    w.system_view.capture_latest();captured=w.system_view.diagnostic
    w.advance(20);w.system_view.inspect_requested.emit('rotation')
    assert w.inspector_capture[0]==captured
    assert w.inspector_capture[0]!=w.sim.last_diagnostic
    assert not w.live_math.isChecked()
    w.system_view.blocks['control'].inspect_button.click()
    assert w.inspector_capture[0]==captured
    assert 'speed clamp' in w.detail.text()
    assert 'Control demand unavailable' not in w.detail.text()
    assert 'PASS' in w.detail.text()
    from PySide6.QtTest import QTest
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QLabel
    hits=[];w.system_view.inspect_requested.connect(hits.append)
    heading=w.system_view.blocks['control'].findChild(QLabel,'systemBlockTitle')
    QTest.mouseClick(heading,Qt.MouseButton.LeftButton)
    assert hits==['control']
    w.close()
