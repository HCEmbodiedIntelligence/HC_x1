import math
import unittest

from hc_x1.command_smoothing import JointTrajectorySmoother, SmoothingLimits


class JointTrajectorySmootherTests(unittest.TestCase):
    def setUp(self):
        self.limits = SmoothingLimits(
            time_constant=0.08,
            max_velocity=2.5,
            max_acceleration=8.0,
            deadband=0.00015,
        )
        self.smoother = JointTrajectorySmoother(self.limits)

    def test_reset_starts_from_feedback_without_command_jump(self):
        self.assertEqual(
            self.smoother.reset(["joint"], [1.0], feedback={"joint": 0.25}),
            [0.25],
        )
        positions, velocities = self.smoother.step(["joint"], [1.0], 0.01)
        self.assertAlmostEqual(velocities[0], 0.08)
        self.assertAlmostEqual(positions[0], 0.2508)

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

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "equal length"):
            self.smoother.step(["a"], [], 0.01)
        with self.assertRaisesRegex(ValueError, "finite"):
            self.smoother.step(["a"], [math.nan], 0.01)


if __name__ == "__main__":
    unittest.main()
