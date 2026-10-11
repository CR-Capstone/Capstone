import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from droopy.impulse import ImpulseAccumulator


def diagnostic(*, dt=1.0, position_before=(0, 0, 0),
               position_after=(0, 0, 0), velocity_before=(0, 0, 0),
               velocity_after=(0, 0, 0), contact_force=(0, 0, 0),
               rotation=(1, 0, 0, 0, 1, 0, 0, 0, 1)):
    return SimpleNamespace(
        dt_s=dt,
        position_before_m=position_before,
        position_after_m=position_after,
        velocity_before_mps=velocity_before,
        velocity_after_mps=velocity_after,
        contact_force_n=contact_force,
        rotation_before=rotation,
    )


def test_no_grip_has_zero_horizontal_impulse_and_gravity_integrates():
    accumulator = ImpulseAccumulator()
    accumulator.record(diagnostic(dt=.1, position_after=(0, 0, -.0981),
                                  velocity_after=(0, 0, -.981)), 2.0, 9.81)
    snap = accumulator.snapshot()
    assert snap.contact_impulse_ns[:2] == (0.0, 0.0)
    assert snap.gravity_impulse_ns == pytest.approx((0, 0, -1.962))
    assert snap.net_impulse_ns == snap.gravity_impulse_ns
    assert snap.momentum_change_kg_mps == pytest.approx(snap.net_impulse_ns)
    assert snap.residual_ns == pytest.approx((0, 0, 0))


def test_gravity_support_cancels_and_nonzero_initial_state_is_preserved():
    accumulator = ImpulseAccumulator()
    accumulator.record(diagnostic(
        dt=.25, position_before=(5, 7, 11), position_after=(5.5, 6.75, 11),
        velocity_before=(2, -1, 0), velocity_after=(2, -1, 0),
        contact_force=(0, 0, 19.62)), mass=2, gravity=9.81)
    snap = accumulator.snapshot()
    assert snap.initial_position_m == (5.0, 7.0, 11.0)
    assert snap.initial_velocity_mps == (2.0, -1.0, 0.0)
    assert snap.net_impulse_ns == pytest.approx((0, 0, 0))
    assert snap.displacement_m == pytest.approx((.5, -.25, 0))
    assert snap.predicted_position_m == pytest.approx((5.5, 6.75, 11))


def test_force_cancellation_can_leave_nonzero_travel_and_prediction_is_semi_implicit():
    accumulator = ImpulseAccumulator()
    accumulator.record(diagnostic(
        position_after=(1, 0, 0), velocity_after=(1, 0, 0),
        contact_force=(1, 0, 0)), 1, 0)
    accumulator.record(diagnostic(
        position_before=(1, 0, 0), position_after=(1, 0, 0),
        velocity_before=(1, 0, 0), velocity_after=(0, 0, 0),
        contact_force=(-1, 0, 0)), 1, 0)
    snap = accumulator.snapshot()
    assert snap.net_impulse_ns == pytest.approx((0, 0, 0))
    assert snap.displacement_m == pytest.approx((1, 0, 0))
    assert snap.predicted_position_m == pytest.approx((1, 0, 0))
    assert snap.predicted_velocity_mps == pytest.approx((0, 0, 0))
    assert snap.along_positive_ns == pytest.approx(1)  # -X force on second tick
    assert snap.along_negative_ns == pytest.approx(-1)
    assert snap.path_length_m == pytest.approx(1)
    assert snap.straightness_ratio == pytest.approx(1)


def test_rotated_initial_minus_x_is_fixed_reference_and_cross_track_is_absolute():
    # Initial body +X is world +Y, so reference body -X is world -Y.
    rotation_z_90 = (0, -1, 0, 1, 0, 0, 0, 0, 1)
    accumulator = ImpulseAccumulator()
    accumulator.record(diagnostic(
        position_after=(3, -4, 0), velocity_after=(3, -4, 0),
        contact_force=(3, -4, 0), rotation=rotation_z_90), 1, 0)
    # A later rotation must not alter the reference.
    accumulator.record(diagnostic(
        position_before=(3, -4, 0), position_after=(3, -4, 0),
        velocity_before=(3, -4, 0), velocity_after=(0, 0, 0),
        contact_force=(-3, 4, 0)), 1, 0)
    snap = accumulator.snapshot()
    assert snap.reference_xy == pytest.approx((0, -1))
    assert snap.along_displacement_m == pytest.approx(4)
    assert snap.lateral_displacement_m == pytest.approx(3)
    assert snap.max_abs_cross_track_m == pytest.approx(3)
    assert snap.along_positive_ns == pytest.approx(4)
    assert snap.along_negative_ns == pytest.approx(-4)
    assert snap.lateral_positive_ns == pytest.approx(3)
    assert snap.lateral_negative_ns == pytest.approx(-3)


def test_snapshot_is_immutable_json_safe_bounded_and_reset_clears_state():
    accumulator = ImpulseAccumulator(sample_interval_s=.02, max_samples=3)
    assert accumulator.snapshot() is None
    position = 0.0
    velocity = 0.0
    for _ in range(100):
        before_position, before_velocity = position, velocity
        velocity += .001
        position += .001 * velocity
        accumulator.record(diagnostic(
            dt=.001, position_before=(before_position, 0, 0),
            position_after=(position, 0, 0), velocity_before=(before_velocity, 0, 0),
            velocity_after=(velocity, 0, 0), contact_force=(1, 0, 0)), 1, 0)
    snap = accumulator.snapshot()
    assert len(snap.samples) == 3
    assert all(len(row) == 5 for row in snap.samples)
    json.dumps(asdict(snap), allow_nan=False)
    with pytest.raises(Exception):
        snap.ticks = 0
    accumulator.reset()
    assert accumulator.snapshot() is None


def test_mass_gravity_changes_require_reset_and_directional_identity_holds():
    accumulator = ImpulseAccumulator()
    accumulator.record(diagnostic(contact_force=(2, 3, 4), velocity_after=(2, 3, 4),
                                  position_after=(2, 3, 4)), 1, 0)
    snap = accumulator.snapshot()
    assert snap.along_positive_ns + snap.along_negative_ns == pytest.approx(
        -(snap.net_impulse_ns[0]))
    assert snap.lateral_positive_ns + snap.lateral_negative_ns == pytest.approx(
        -(snap.net_impulse_ns[1]))
    with pytest.raises(ValueError, match="cannot change"):
        accumulator.record(diagnostic(), 2, 0)
    with pytest.raises(ValueError, match="cannot change"):
        accumulator.record(diagnostic(), 1, 9.81)
    accumulator.reset()
    accumulator.record(diagnostic(), 2, 9.81)
