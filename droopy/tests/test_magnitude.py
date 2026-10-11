import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import pytest
from PySide6.QtWidgets import QApplication
from droopy.ui import Trace,Window
from droopy.lesson import PhaseWaveform
from droopy.control import Command


def test_speed_magnitude_never_changes_signed_error_torque_or_history():
    app=QApplication.instance() or QApplication([])
    plot=Trace()
    sample={'requested_r':-300.,'speed_r':-250.,'error_r':-50.,'torque_r':-.8}
    assert plot.plotted_value(sample,'requested_r')==300
    assert plot.plotted_value(sample,'speed_r')==250
    assert plot.plotted_value(sample,'error_r')==-50
    assert plot.plotted_value(sample,'torque_r')==-.8
    assert sample['speed_r']==-250
    plot.speed_magnitude=False
    assert plot.plotted_value(sample,'speed_r')==-250
    assert plot.rotation_label(0)=='stopped'
    assert plot.rotation_label(-4)=='− axis'


def test_counterspin_magnitudes_alternate_and_clamp():
    app=QApplication.instance() or QApplication([])
    wave=PhaseWaveform()
    wave.set_state(Command(enabled=True,spin=200,amplitude=50,frequency=1),0,450)
    values=wave.cycle_values(5)
    assert values[0]==pytest.approx((200,200))
    assert values[1]==pytest.approx((250,150))
    assert values[3]==pytest.approx((150,250))
    wave.set_state(Command(enabled=True,mode='Manual rotors',left=-500,right=500),0,300)
    assert wave.cycle_values(5)==[(300,300)]*5
    wave.set_state(Command(enabled=False),0)
    assert wave.cycle_values(5)==[(0,0)]*5


def test_waveform_preserves_direction_and_cycle_reversals():
    app=QApplication.instance() or QApplication([])
    wave=PhaseWaveform()
    wave.set_state(Command(enabled=True,mode='Manual rotors',left=-100,right=100),0)
    first=wave.cycle_values(5)
    assert wave.direction_text=='At cursor: L − axis / R + axis'
    assert wave.cycle_directions(5)[0]==('− axis','+ axis')
    wave.set_state(Command(enabled=True,mode='Manual rotors',left=100,right=-100),0)
    assert wave.cycle_values(5)==first
    assert wave.direction_text=='At cursor: L + axis / R − axis'
    wave.set_state(Command(enabled=True,spin=50,amplitude=100,frequency=1),.25)
    signs=wave.cycle_directions(5)
    assert signs[1]==('+ axis','+ axis')
    assert signs[3]==('− axis','− axis')
    assert 'L + axis / R + axis' in wave.accessibleDescription()


def test_default_plot_magnitude_and_signed_toggle_render(tmp_path):
    app=QApplication.instance() or QApplication([])
    window=Window();window.timer.stop();window.show()
    assert window.trace.speed_magnitude and window.show_plot.isChecked()
    window.enabled.setChecked(True);window.advance(80)
    sample=window.trace.samples[-1].copy()
    window.magnitude_action.setChecked(False)
    assert not window.trace.speed_magnitude
    assert window.trace.samples[-1]==sample
    window.magnitude_action.setChecked(True)
    window.plot_preset(('requested_l','requested_r','speed_l','speed_r','error_l','torque_r'))
    app.processEvents()
    assert window.grab().save(str(tmp_path/'magnitude.png'))
    window.close()
