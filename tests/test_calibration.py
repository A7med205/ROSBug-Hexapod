import math
import unittest

from hardware.main import GoalReceiver


class CalibrationRegressionTest(unittest.TestCase):
    def setUp(self):
        # Conversion methods do not require board hardware.
        self.receiver = GoalReceiver.__new__(GoalReceiver)

    def test_centered_coxa_is_1500_microseconds(self):
        self.assertAlmostEqual(self.receiver.robot_angle_to_pulse("j11", 0.0), 1500.0)

    def test_negative_branch_calibration_is_unchanged(self):
        expected = 1500.0 + 10.8889 * -84.0
        self.assertAlmostEqual(
            self.receiver.robot_angle_to_pulse("j12", 0.0), expected, places=6
        )

    def test_pulses_remain_clamped(self):
        self.assertEqual(self.receiver.robot_angle_to_pulse("j21", math.pi), 2500.0)
        self.assertEqual(self.receiver.robot_angle_to_pulse("j13", 0.0), 500.0)


if __name__ == "__main__":
    unittest.main()
