import math
import unittest

from common.keyboard_input import KeyboardInput
from robot_core import Command, PostureAxis


class KeyboardInputTest(unittest.TestCase):
    def setUp(self):
        self.keyboard = KeyboardInput()

    def test_standup_skip_and_mode_commands(self):
        self.assertEqual(self.keyboard.feed_key("u").command, Command.stand_up())
        self.assertEqual(self.keyboard.feed_key("k").command, Command.skip_stand_up())
        self.assertEqual(self.keyboard.feed_key("j").command, Command.sit_down())
        self.assertEqual(self.keyboard.feed_key("t").command, Command.toggle_mode())

    def test_bare_zero_is_stop(self):
        self.assertEqual(self.keyboard.feed_key("0").command, Command.stop())

    def test_reset_tilt_clears_numeric_prefix(self):
        self.keyboard.feed_key("5")
        self.assertEqual(self.keyboard.feed_key("r").command, Command.reset_tilt())
        self.assertEqual(self.keyboard.numeric_prefix, "")

    def test_count_prefix_is_attached_to_movement(self):
        self.assertIsNone(self.keyboard.feed_key("1").command)
        self.assertIsNone(self.keyboard.feed_key("0").command)
        self.assertEqual(self.keyboard.feed_key("w").command, Command.walk(1, steps=10))
        self.assertEqual(self.keyboard.numeric_prefix, "")

    def test_bare_movement_remains_continuous(self):
        self.assertEqual(self.keyboard.feed_key("d").command, Command.walk(2))

    def test_backspace_and_escape_edit_the_count(self):
        self.keyboard.feed_key("1")
        self.keyboard.feed_key("2")
        self.keyboard.feed_key("\x7f")
        self.assertEqual(self.keyboard.feed_key("e").command, Command.walk(3, steps=1))

        self.keyboard.feed_key("9")
        self.keyboard.feed_key("\x1b")
        self.assertEqual(self.keyboard.feed_key("w").command, Command.walk(1))

    def test_direction_mapping_toggle_clears_count(self):
        self.keyboard.feed_key("5")
        self.keyboard.feed_key("m")
        self.assertEqual(self.keyboard.numeric_prefix, "")
        self.assertEqual(self.keyboard.feed_key("q").command, Command.walk(6))

    def test_posture_values_use_relative_millimeters_and_degrees(self):
        self.keyboard.feed_key("5")
        self.assertEqual(
            self.keyboard.feed_key("]").command,
            Command.elevation(0.005),
        )

        self.keyboard.feed_key("5")
        self.assertEqual(
            self.keyboard.feed_key("[").command,
            Command.elevation(-0.005),
        )

        self.keyboard.feed_key("5")
        command = self.keyboard.feed_key(",").command
        self.assertEqual(command.posture_axis, PostureAxis.PITCH)
        self.assertAlmostEqual(command.posture_value, math.radians(-5.0))

        self.keyboard.feed_key("3")
        command = self.keyboard.feed_key("'").command
        self.assertEqual(command.posture_axis, PostureAxis.ROLL)
        self.assertAlmostEqual(command.posture_value, math.radians(3.0))

    def test_posture_command_requires_numeric_prefix(self):
        for key in ("[", "]"):
            with self.subTest(key=key):
                result = self.keyboard.feed_key(key)
                self.assertIsNone(result.command)
                self.assertIn("requires a millimeter delta", result.notices[0])
        with self.assertRaises(ValueError):
            Command.posture(PostureAxis.ELEVATION, 0.001)


if __name__ == "__main__":
    unittest.main()
