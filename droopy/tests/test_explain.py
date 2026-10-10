from dataclasses import replace

from droopy.control import Command
from droopy.explain import derive, explanation
from droopy.physics import Config, Simulator


def tick(targets=(220.0, -220.0)):
    sim=Simulator(Config(gravity=0.0))
    sim.position[2]=1.0
    sim.step(targets)
    return sim, sim.last_diagnostic


def test_control_checks_raw_and_clipped_targets_with_substitution():
    sim=Simulator()
    command=Command(enabled=True,spin=440,amplitude=40,frequency=1,bias=20)
    raw=command.raw_targets(sim.time)
    sim.step(raw)
    result=derive("control",sim.last_diagnostic,sim.config,command,0.0)
    assert result.ok
    rendered=result.render("")
    assert "440+(+0)+(+20)=+460" in rendered
    assert "gait raw targets" in rendered and "speed clamp" in rendered
    assert "✓ ALL CHECKS PASS" in rendered
    assert sim.last_diagnostic.targets_rad_s == (460.0,-420.0)
    assert sim.last_diagnostic.requested_rad_s == (450.0,-420.0)


def test_manual_and_disabled_do_not_claim_sinusoid():
    for command, targets, marker in [
        (Command(enabled=True,mode="Manual rotors",left=12,right=-34),(12,-34),"Manual"),
        (Command(enabled=False), (0,0), "Motors disabled"),
    ]:
        sim=Simulator(); sim.step(targets)
        text=explanation("control",sim.last_diagnostic,sim.config,command,0.25)
        assert marker in text
        assert "sin(" not in text and "Amplitude" not in text
        assert "PASS" in text


def test_tampered_motor_diagnostic_names_failed_check():
    sim,d=tick()
    assert derive("motor",d,sim.config).ok
    bad=replace(d,motor_torques_nm=(0.123,d.motor_torques_nm[1]))
    result=derive("motor",bad,sim.config)
    assert not result.ok
    rendered=result.render("")
    assert "✗ CHECKS FAILED" in rendered
    assert "✗ FAIL — motor torque clip + speed gate" in rendered


def test_motor_recompute_includes_positive_and_negative_speed_gate():
    sim,d=tick()
    # A deliberately constructed boundary record proves the explainer applies
    # physics' final defensive gate after torque clipping, rather than reporting
    # a false residual when recorded torque is zero.
    edge=replace(d,requested_rad_s=(451.0,-451.0),
                 rotor_speeds_before_rad_s=(450.0,-450.0),
                 motor_torques_nm=(0.0,0.0))
    result=derive("motor",edge,sim.config)
    assert result.ok
    text=result.render("")
    assert "speed-bound outward gate L/R=YES/YES" in text
    assert "✓ PASS — motor torque clip + speed gate" in text


def test_rotation_rebuilds_momentum_reaction_and_gyro_independently():
    sim,d=tick()
    for field,name in [('angular_momentum_body','angular momentum'),
                       ('motor_reaction_nm','motor reaction'),('gyroscopic_nm','gyroscopic torque')]:
        original=getattr(d,field)
        bad=replace(d,**{field:(original[0]+1,*original[1:])})
        result=derive('rotation',bad,sim.config)
        assert not result.ok
        assert f'FAIL — {name}' in result.render('')
    bad=replace(d,contact_force_n=(1.,0.,0.))
    assert not derive('translation',bad,sim.config).ok


def test_rotation_contacts_and_translation_have_named_pass_checks():
    sim=Simulator(); sim.position[2]-=.001; sim.velocity[:]=[1,-.5,-.1]; sim.step((50,-40))
    for key,name in [("rotation","Euler angular acceleration"),
                     ("contacts","foot 1 normal"),
                     ("translation","position integration")]:
        result=derive(key,sim.last_diagnostic,sim.config)
        assert result.ok
        assert name in result.render("")
