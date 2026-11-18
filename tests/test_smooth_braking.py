"""Dynamic checks for the active robot smoother baseline and braking envelope."""
import math
import numpy as np
import pytest

from hc_x1.command_smoothing import JointTrajectorySmoother, SmoothingLimits


@pytest.mark.parametrize('time_constant', [.03, .04])
@pytest.mark.parametrize('dt', [.0025, .005, .01])
@pytest.mark.parametrize('trajectory', ['step', 'ramp', 'sine', 'reversal'])
def test_real_limits_and_continuity(dt, trajectory, time_constant):
    limits = SmoothingLimits(time_constant=time_constant, max_velocity=2.5, max_acceleration=12., deadband=.0008)
    smoother = JointTrajectorySmoother(limits)
    smoother.reset(['j'], [0.])
    previous_position = previous_velocity = 0.
    errors = []
    for time in np.arange(0., 6., dt):
        if trajectory == 'step': target = .5 if time < 2. else 0.
        elif trajectory == 'ramp': target = min(.5, time*.25) if time < 3 else .5
        elif trajectory == 'sine': target = .25*math.sin(time*2.)
        else: target = .25 if time < 1. else -.25 if time < 3. else 0.
        positions, velocities = smoother.step(['j'], [target], dt)
        position, velocity = positions[0], velocities[0]
        assert math.isfinite(position) and math.isfinite(velocity)
        assert abs(velocity) <= limits.max_velocity + 1e-10
        assert abs(velocity-previous_velocity) <= limits.max_acceleration*dt + 1e-10
        assert abs(position-previous_position) <= limits.max_velocity*dt + 1e-10
        errors.append(abs(target-position))
        previous_position, previous_velocity = position, velocity
    if trajectory != 'sine':
        assert errors[-1] <= limits.deadband
        assert previous_velocity == 0.


def test_stationary_deadband_unchanged():
    smoother = JointTrajectorySmoother(SmoothingLimits(deadband=.0008))
    smoother.reset(['j'], [0.])
    for _ in range(100):
        assert smoother.step(['j'], [.0007], .005) == ([0.], [0.])


@pytest.mark.parametrize('damping', [0., 1.01, float('nan')])
def test_invalid_damping_is_rejected(damping):
    with pytest.raises(ValueError):
        JointTrajectorySmoother(SmoothingLimits(braking_damping=damping))


def test_feedback_lag_remains_observational_and_does_not_reset_trajectory():
    smoother = JointTrajectorySmoother(SmoothingLimits())
    smoother.reset(['j'], [0.])
    for _ in range(30): smoother.step(['j'], [1.], .005)
    before = dict(smoother.positions)
    assert smoother.divergent_joints(['j'], {'j': -.5}, .1) == ['j']
    assert smoother.positions == before
