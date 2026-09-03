import math
import unittest

from hc_x1.command_smoothing import JointTrajectorySmoother, SmoothingLimits


class JointTrajectorySmootherTests(unittest.TestCase):
    def setUp(self):
        self.limits = SmoothingLimits()
        self.smoother = JointTrajectorySmoother(self.limits)

    def test_low_latency_defaults_remain_acceleration_bounded(self):
        defaults = SmoothingLimits()
        self.assertEqual(defaults.time_constant, 0.04)
        self.assertEqual(defaults.max_acceleration, 12.0)

    def test_reset_starts_from_feedback_without_command_jump(self):
        self.assertEqual(
            self.smoother.reset(["joint"], [1.0], feedback={"joint": 0.25}),
            [0.25],
        )
        positions, velocities = self.smoother.step(["joint"], [1.0], 0.01)
        self.assertAlmostEqual(velocities[0], 0.12)
        self.assertAlmostEqual(positions[0], 0.2512)

    def test_velocity_and_acceleration_are_bounded(self):
        self.smoother.reset(["joint"], [2.0], feedback={"joint": 0.0})
        last_velocity = 0.0
        for _ in range(500):
            positions, velocities = self.smoother.step(["joint"], [2.0], 0.01)
            self.assertLessEqual(abs(velocities[0]), self.limits.max_velocity + 1e-12)
            self.assertLessEqual(
                abs(velocities[0] - last_velocity),
                self.limits.max_acceleration * 0.01 + 1e-12,
            )
            last_velocity = velocities[0]
        self.assertAlmostEqual(positions[0], 2.0, places=6)

    def test_deadband_holds_stationary_output(self):
        self.smoother.reset(["joint"], [0.0], feedback={"joint": 0.0})
        positions, velocities = self.smoother.step(["joint"], [0.0001], 0.01)
        self.assertEqual(positions, [0.0])
        self.assertEqual(velocities, [0.0])

    def test_reinitialize_from_feedback_discards_old_trajectory(self):
        # Move smoother towards an old target
        self.smoother.reset(['j1', 'j2'], [1.0, 1.0], feedback={'j1': 0.0, 'j2': 0.0})
        self.smoother.step(['j1', 'j2'], [1.0, 1.0], 0.02)
        self.assertGreater(self.smoother.positions['j1'], 0.0)

        # Reinitialize with fresh post-homing feedback
        new_feedback = {'j1': -0.5, 'j2': 0.3}
        output = self.smoother.reset(['j1', 'j2'], [-0.5, 0.3], feedback=new_feedback)
        self.assertEqual(output, [-0.5, 0.3])
        self.assertEqual(self.smoother.positions, {'j1': -0.5, 'j2': 0.3})
        self.assertEqual(self.smoother.velocities, {'j1': 0.0, 'j2': 0.0})

        # Next step towards a new target starts smoothly from new feedback without jump
        pos, vel = self.smoother.step(['j1', 'j2'], [-0.4, 0.4], 0.01)
        self.assertAlmostEqual(pos[0], -0.5 + vel[0] * 0.01)
        self.assertAlmostEqual(pos[1], 0.3 + vel[1] * 0.01)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "equal length"):
            self.smoother.step(["a"], [], 0.01)
        with self.assertRaisesRegex(ValueError, "finite"):
            self.smoother.step(["a"], [math.nan], 0.01)


if __name__ == "__main__":
    unittest.main()
