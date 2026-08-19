import math
import unittest

from common.keyboard_input import KeyboardInput
from robot_core import Command, PostureAxis


class KeyboardInputTest(unittest.TestCase):
    def setUp(self):
        self.keyboard = KeyboardInput()

    def test_startup_skip_and_mode_commands(self):
        self.assertEqual(self.keyboard.feed_key("u").command, Command.startup())
        self.assertEqual(self.keyboard.feed_key("k").command, Command.skip_startup())
        self.assertEqual(self.keyboard.feed_key("t").command, Command.toggle_mode())

    def test_bare_zero_is_stop(self):
        self.assertEqual(self.keyboard.feed_key("0").command, Command.stop())

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

    def test_posture_values_use_millimeters_and_degrees(self):
        self.keyboard.feed_key("1")
        self.keyboard.feed_key("0")
        self.assertEqual(
            self.keyboard.feed_key("]").command,
            Command.posture(PostureAxis.ELEVATION, 0.010),
        )

        self.keyboard.feed_key("5")
        command = self.keyboard.feed_key(",").command
        self.assertEqual(command.posture_axis, PostureAxis.PITCH)
        self.assertAlmostEqual(command.posture_delta, math.radians(-5.0))

        self.keyboard.feed_key("3")
        command = self.keyboard.feed_key("'").command
        self.assertEqual(command.posture_axis, PostureAxis.ROLL)
        self.assertAlmostEqual(command.posture_delta, math.radians(3.0))

    def test_posture_command_requires_numeric_prefix(self):
        result = self.keyboard.feed_key("]")
        self.assertIsNone(result.command)
        self.assertIn("requires a numeric value", result.notices[0])


if __name__ == "__main__":
    unittest.main()
