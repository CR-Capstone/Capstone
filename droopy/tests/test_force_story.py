import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import numpy as np
import pytest
from PySide6.QtWidgets import QApplication
from droopy.force_story import ForceStory,TorqueMotionView
from droopy.lesson import GuidedLesson
from droopy.physics import Simulator,Config
from droopy.control import Command


def test_vectors_match_consumed_torque_force_and_velocity_change():
    sim=Simulator();sim.position[2]-=.001;sim.velocity[:]=[.2,-.1,-.03]
    sim.step((120,-100));d=sim.last_diagnostic
    s=ForceStory.from_tick(d,sim.config)
    np.testing.assert_allclose(s.total_torque,np.asarray(d.motor_reaction_nm)+d.gyroscopic_nm+d.contact_torque_body_nm)
    np.testing.assert_allclose(s.delta_v_xy,np.asarray(d.velocity_after_mps[:2])-d.velocity_before_mps[:2],atol=1e-15)
    assert s.time==d.time_before_s


def test_torque_does_not_imply_floor_push_without_grip():
    sim=Simulator(Config(friction=0));sim.position[2]-=.001
    sim.step((100,50));s=ForceStory.from_tick(sim.last_diagnostic,sim.config)
    assert np.linalg.norm(s.motor)>0
    assert s.force_xy==(0.,0.)
    assert s.delta_v_xy==(0.,0.)


def test_capture_synchronizes_waveform_and_torque_and_reset_clears(tmp_path):
    app=QApplication.instance() or QApplication([])
    sim=Simulator();lesson=GuidedLesson();cmd=Command(enabled=True)
    sim.position[2]-=.001
    sim.step(cmd.raw_targets(sim.time));lesson.update_state(sim,cmd,cmd)
    assert lesson.torque_view.story is None
    lesson.capture_current();saved=lesson.torque_view.story
    phase=lesson.waveform.time_s
    captured_contacts=lesson.contact_view.contacts
    assert sum(c.normal_n for c in captured_contacts)>0
    sim.position[2]=1.0
    changed=Command(enabled=True,spin=100,amplitude=50)
    sim.step(changed.raw_targets(sim.time));lesson.update_state(sim,changed,changed)
    assert lesson.torque_view.story is saved
    assert all(c.normal_n==0 for c in sim.last_diagnostic.contacts)
    assert lesson.contact_view.contacts==captured_contacts
    assert lesson.waveform.time_s==phase
    assert lesson.waveform.command==cmd
    lesson.live_check.setChecked(True)
    assert lesson.torque_view.story.time==sim.last_diagnostic.time_before_s
    assert lesson.contact_view.contacts==sim.last_diagnostic.contacts
    assert lesson.waveform.time_s==lesson.torque_view.story.time
    view=lesson.torque_view;view.resize(380,465);view.show();app.processEvents()
    assert view.grab().save(str(tmp_path/'torque.png'))
    lesson.reset_history();assert lesson.torque_view.story is None
    lesson.close()
