"""Illustrative geometric/dynamic scaling, not measured hardware selection."""
from dataclasses import replace
from .physics import Config
from .control import Command

POUNDS_TO_KG = 0.45359237
DEFAULT_PRESET = '9lb'


def robot_preset(name=DEFAULT_PRESET):
    """Return fresh config and gait. Gravity-preserving similarity scaling.

    L'=sL, m'=r m, time'=sqrt(s)*time, I'=r*s²*I,
    torque'=r*s*torque. Contacts k'=r/s*k, c'=r/sqrt(s)*c.
    The integration timestep stays fixed (more conservative for larger robots).
    """
    base=Config(); command=Command()
    if name=='3lb':
        return base,command
    if name!='9lb':
        raise ValueError(f'Unknown robot preset: {name}')
    mass=9*POUNDS_TO_KG
    ratio=mass/base.mass
    scale=ratio**(1/3)
    time_scale=scale**.5
    torque_scale=ratio*scale
    inertia_scale=ratio*scale**2
    changes={'mass':mass}
    for field in ('body_inertia_x','body_inertia_y','body_inertia_z','rotor_inertia'):
        changes[field]=getattr(base,field)*inertia_scale
    for field in ('rotor_offset_y','rotor_height','foot_half_length','foot_half_width','foot_drop'):
        changes[field]=getattr(base,field)*scale
    changes.update(max_torque=base.max_torque*torque_scale,
                   max_speed=base.max_speed/time_scale,
                   servo_gain=base.servo_gain*torque_scale*time_scale,
                   contact_stiffness=base.contact_stiffness*ratio/scale,
                   contact_damping=base.contact_damping*ratio/time_scale,
                   friction_velocity=base.friction_velocity*scale/time_scale)
    config=replace(base,**changes);config.validate()
    command=replace(command,spin=command.spin/time_scale,
                    amplitude=command.amplitude/time_scale,frequency=command.frequency/time_scale,
                    left=command.left/time_scale,right=command.right/time_scale)
    return config,command
