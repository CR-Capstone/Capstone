import os
os.environ.setdefault("QT_QPA_PLATFORM","offscreen")

from PySide6.QtWidgets import QApplication
from droopy.ui import Window


def test_ui_controls_reset_and_render(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.timer.stop()
    window.show()
    app.processEvents()
    from PySide6.QtWidgets import QFileDialog
    import json
    output = tmp_path / "trace.csv"
    monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *a: (str(output), ""))
    window.toggle_record()
    window.enabled.setChecked(True)
    window.advance(30)
    assert window.trace.samples[-1]["target_l"] == window.sim.last_diagnostic.requested_rad_s[0]
    window.set_plot_channel("torque_l", True)
    assert window.trace.visible["torque_l"]
    window.finish_record()
    rows = output.read_text().splitlines()
    assert len(rows) == 31  # header + one row per accepted fixed tick
    assert "target_left_rad_s" in rows[0] and "torque_right_nm" in rows[0]
    metadata = json.loads(output.with_suffix(".json").read_text())
    assert metadata["commands"][0]["command"]["enabled"] is True
    assert metadata["end"]["t"] > 0
    assert window.sim.time > 0
    assert any(abs(x) > 0 for x in window.sim.rotor_speeds)
    window.mode.setCurrentText("Manual rotors")
    assert window.inputs["left"].isEnabled()
    assert not window.inputs["spin"].isEnabled()
    window.settings["tilt_deg"].setValue(0)
    window.apply_config()
    assert window.sim.config.tilt_deg == 0
    assert window.sim.time == 0
    assert not window.enabled.isChecked()
    window.advance(1)
    assert window.sim.time == window.sim.config.dt
    consumed = window.sim.last_diagnostic.targets_rad_s
    window.inputs["spin"].setValue(300)
    window.refresh()
    assert window.sim.last_diagnostic.targets_rad_s == consumed
    assert "Pending next tick" in window.telemetry.text()
    window.open_explanation("control")
    assert "Consumed tick" in window.detail.text()
    assert "PASS" in window.detail.text()
    assert "wave=A" not in window.detail.text()  # disabled/manual mode has no sinusoid
    window.arena.vector_clicked.emit("translation")
    assert window.explain_select.currentText() == "Translation"
    assert "PASS" in window.detail.text()
    window.reset()
    assert "No completed tick" in window.detail.text()
    window.show_settings()
    assert window.settings_dialog.isVisible()
    window.settings_dialog.hide()
    assert window.grab().save(str(tmp_path / "ui.png"))
    window.toggle_run()
    assert window.running
    assert not window.step_button.isEnabled()
    window.reset()
    assert not window.running
    window.close()
    assert not window.timer.isActive()


def test_plot_sampling_pending_commands_and_presets():
    app = QApplication.instance() or QApplication([])
    window = Window(); window.timer.stop()
    window.enabled.setChecked(True)
    window.advance(1000)
    assert len(window.trace.samples) == 1000
    assert window.trace.samples[0]['t'] == 0
    assert abs(window.trace.samples[-1]['t'] - 999*window.sim.config.dt) < 1e-12
    saved = window.sim.last_diagnostic
    before = window.telemetry.text()
    window.inputs['spin'].setValue(300)
    assert window.sim.last_diagnostic is saved
    assert window.telemetry.text() != before  # refreshes while paused, no manual refresh
    window.inputs['bias'].setValue(300)
    window.advance(1)
    sample=window.trace.samples[-1]
    assert sample['target_l'] > window.sim.config.max_speed
    assert sample['requested_l'] == window.sim.config.max_speed
    assert 'clamped' in window.telemetry.text()
    window.plot_preset(('torque_l','torque_r'))
    assert {k for k,v in window.trace.visible.items() if v} == {'torque_l','torque_r'}
    window.set_plot_window(1)
    assert window.trace.window_s == 1
    window.step_button.click()
    assert window.last_consumed_command.spin == 300
    window.reset()
    assert not window.trace.samples
    window.close()


def test_frozen_math_and_playback_slider():
    app = QApplication.instance() or QApplication([])
    window=Window();window.timer.stop()
    window.speed.setValue(.01);assert abs(window.speed.value()-.01)<1e-12
    window.speed.setValue(1);assert window.speed.value()==1
    window.speed.setValue(4);assert window.speed.value()==4
    window.enabled.setChecked(True);window.advance(10)
    window.open_explanation('motor')
    text=window.detail.text();captured=window.inspector_capture[0]
    window.advance(100)
    assert window.detail.text()==text
    assert window.inspector_capture[0] is captured
    window.step_and_capture()
    assert window.inspector_capture[0] is window.sim.last_diagnostic
    assert window.inspector_capture[0] is not captured
    window.lesson_step(.25)
    assert window.lesson_ticks_remaining>0 and not window.run_button.isEnabled()
    window.reset()
    assert window.lesson_ticks_remaining==0 and window.run_button.isEnabled()
    window.settings['dt'].setValue(.00001);window.apply_config()
    window.lesson_step(2.5)
    assert 2.5 <= window.lesson_ticks_remaining*window.sim.config.dt < 2.500011
    window.reset()
    window.close()


def test_joystick_commands_motors_and_neutral_brakes():
    app=QApplication.instance() or QApplication([])
    window=Window();window.timer.stop()
    window.direction_demo()
    assert window.direction_enabled.isChecked()
    assert window.joystick.strength==1
    window.advance(40)
    assert window.last_consumed_command.bias != window.inputs['bias'].value()
    assert window.last_consumed_command.enabled
    before=window.sim.position.copy()
    window.joystick.centre()
    window.refresh()
    assert (window.sim.position==before).all()
    window.advance(1)
    assert not window.last_consumed_command.enabled
    assert window.sim.last_diagnostic.targets_rad_s==(0.,0.)
    window.close()


def test_robot_setup_defaults_to_nine_pounds_and_presets_reset():
    from droopy.presets import POUNDS_TO_KG
    app=QApplication.instance() or QApplication([])
    window=Window();window.timer.stop()
    assert abs(window.sim.config.mass-9*POUNDS_TO_KG)<1e-10
    assert '9.00 lb' in window.windowTitle()
    window.load_robot_preset('3lb')
    assert window.sim.config.mass==1.36
    window.load_robot_preset('9lb')
    assert abs(window.sim.config.mass-9*POUNDS_TO_KG)<1e-6
    assert not window.enabled.isChecked()
    assert not window.direction_enabled.isChecked()
    assert window.sim.time==0
    window.close()


def test_settings_reject_nonfinite_without_reset(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window(); window.timer.stop(); window.advance(1)
    old_sim, old_time = window.sim, window.sim.time
    class Nonfinite:
        def value(self): return float("nan")
    window.settings["mass"] = Nonfinite()
    from PySide6.QtWidgets import QMessageBox
    warnings=[]
    monkeypatch.setattr(QMessageBox,"warning",lambda *args: warnings.append(args))
    assert window.apply_config() is False
    assert warnings and window.sim is old_sim and window.sim.time == old_time
    window.close()
