import math
import unittest

from gait_core import (
    AutoJobStatus,
    Command,
    ControllerMode,
    ControllerState,
    LiteGaitCoordinator,
)


class GaitCoreTest(unittest.TestCase):
    def setUp(self):
        self.coordinator = LiteGaitCoordinator()

    def stand_up(self):
        self.assertTrue(self.coordinator.request(Command.startup()))
        pose = self.coordinator.next_batch()
        self.assertEqual(pose.phase_name, "startup pose")
        self.coordinator.complete_batch(pose.goal_id)
        descent = self.coordinator.next_batch()
        self.assertEqual(descent.phase_name, "startup descent")
        self.coordinator.complete_batch(descent.goal_id)
        return pose, descent

    def skip_startup(self):
        self.assertTrue(self.coordinator.request(Command.skip_startup()))
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)

    def enter_auto(self):
        self.skip_startup()
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)

    def test_startup_is_explicit_and_walk_is_rejected_before_it(self):
        self.assertEqual(self.coordinator.state, ControllerState.AWAITING_STARTUP)
        self.assertIsNone(self.coordinator.next_batch())
        self.assertFalse(self.coordinator.request(Command.walk(1)))

        pose, descent = self.stand_up()
        self.assertEqual(pose.point_count, 1)
        self.assertEqual(pose.hold_after, 2.0)
        self.assertEqual(descent.point_count, 60)
        self.assertEqual(
            list(descent.points[-1]), self.coordinator.model.neutral_joint_goal()
        )
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)

    def test_hardware_template_point_counts_are_canonical(self):
        expected = {
            1: ((11, 21), (11, 21)),
            2: ((11, 21), (11, 21)),
            3: ((9, 17), (9, 17)),
            4: ((9, 17), (9, 17)),
            5: ((9, 17), (8, 15)),
            6: ((8, 15), (9, 17)),
            7: ((13, 25), (13, 25)),
            8: ((11, 21), (11, 21)),
            9: ((11, 21), (11, 21)),
            10: ((9, 17), (9, 17)),
            11: ((9, 17), (9, 17)),
            12: ((9, 17), (8, 15)),
            13: ((8, 15), (9, 17)),
            14: ((13, 25), (13, 25)),
        }
        for trajectory_id, (expected_a, expected_b) in expected.items():
            durations = self.coordinator.model.duration_points[trajectory_id]
            self.assertEqual(
                (durations["A"]["half"], durations["A"]["full"]), expected_a
            )
            self.assertEqual(
                (durations["B"]["half"], durations["B"]["full"]), expected_b
            )

    def test_skip_startup_asserts_neutral_without_creating_a_batch(self):
        self.skip_startup()
        self.assertIsNone(self.coordinator.next_batch())
        self.assertEqual(
            self.coordinator.current_joint_goal,
            self.coordinator.model.neutral_joint_goal(),
        )
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)
        self.assertFalse(self.coordinator.request(Command.skip_startup()))
        self.assertFalse(self.coordinator.request(Command.startup()))

    def test_auto_mode_requires_stationary_counted_commands_of_at_least_two(self):
        self.enter_auto()
        self.assertFalse(self.coordinator.request(Command.walk(1)))
        self.assertFalse(self.coordinator.request(Command.walk(1, steps=1)))
        self.assertFalse(self.coordinator.request(Command.walk(1, steps=True)))
        self.assertTrue(self.coordinator.request(Command.walk(1, steps=2)))

        other = LiteGaitCoordinator()
        other.request(Command.skip_startup())
        self.assertFalse(other.request(Command.walk(1, steps=2)))

    def test_two_step_auto_job_is_start_full_final_and_returns_stationary(self):
        self.enter_auto()
        self.assertTrue(self.coordinator.request(Command.walk(1, steps=2)))

        names = []
        remaining = []
        while self.coordinator.auto_job is not None:
            batch = self.coordinator.next_batch()
            names.append(batch.phase_name)
            self.coordinator.complete_batch(batch.goal_id)
            if self.coordinator.auto_job is not None:
                remaining.append(self.coordinator.auto_job.remaining_half_steps)

        self.assertEqual(
            names,
            [
                "start half-step t1",
                "full-step-1 t1",
                "full-step-2 t1",
                "auto final half-step t1",
            ],
        )
        self.assertEqual(remaining, [3, 2, 1])
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertIsNone(self.coordinator.next_batch())

    def test_auto_job_discards_other_movement_commands(self):
        self.enter_auto()
        self.coordinator.request(Command.walk(1, steps=2))
        start = self.coordinator.next_batch()
        self.assertFalse(self.coordinator.request(Command.walk(4, steps=8)))
        self.assertEqual(self.coordinator.requested_trajectory_id, 1)
        self.coordinator.complete_batch(start.goal_id)

        while self.coordinator.auto_job is not None:
            batch = self.coordinator.next_batch()
            self.coordinator.complete_batch(batch.goal_id)

        self.assertIsNone(self.coordinator.next_batch())
        self.assertEqual(self.coordinator.requested_trajectory_id, 0)

    def test_stationary_aborts_auto_job_through_normal_safe_boundary(self):
        self.enter_auto()
        self.coordinator.request(Command.walk(1, steps=5))
        start = self.coordinator.next_batch()
        self.coordinator.complete_batch(start.goal_id)

        first_half = self.coordinator.next_batch()
        self.assertTrue(self.coordinator.request(Command.stop()))
        self.assertEqual(self.coordinator.auto_job.status, AutoJobStatus.ABORTING)
        self.coordinator.complete_batch(first_half.goal_id)

        second_half = self.coordinator.next_batch()
        self.assertIn("full-step-2", second_half.phase_name)
        self.coordinator.complete_batch(second_half.goal_id)
        final_half = self.coordinator.next_batch()
        self.assertIn("auto abort half-step", final_half.phase_name)
        self.coordinator.complete_batch(final_half.goal_id)

        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertIsNone(self.coordinator.auto_job)

    def test_mode_toggle_aborts_auto_job_and_activates_after_stationary(self):
        self.enter_auto()
        self.coordinator.request(Command.walk(1, steps=4))
        start = self.coordinator.next_batch()
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertEqual(self.coordinator.requested_mode, ControllerMode.NORMAL)
        self.coordinator.complete_batch(start.goal_id)

        final_half = self.coordinator.next_batch()
        self.assertIn("auto abort half-step", final_half.phase_name)
        self.coordinator.complete_batch(final_half.goal_id)
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)

    def test_mode_toggle_during_normal_walk_stops_before_entering_auto(self):
        self.skip_startup()
        self.coordinator.request(Command.walk(1))
        start = self.coordinator.next_batch()
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertFalse(self.coordinator.request(Command.walk(2)))
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)
        self.coordinator.complete_batch(start.goal_id)

        final_half = self.coordinator.next_batch()
        self.assertIn("final half-step", final_half.phase_name)
        self.coordinator.complete_batch(final_half.goal_id)
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)

    def test_stop_or_mode_toggle_cancels_an_auto_job_before_it_starts(self):
        self.enter_auto()
        self.coordinator.request(Command.walk(1, steps=3))
        self.assertTrue(self.coordinator.request(Command.stop()))
        self.assertIsNone(self.coordinator.auto_job)
        self.assertIsNone(self.coordinator.next_batch())

        self.coordinator.request(Command.walk(2, steps=3))
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)
        self.assertIsNone(self.coordinator.auto_job)
        self.assertIsNone(self.coordinator.next_batch())

    def test_stationary_mode_toggle_discards_requested_normal_walk(self):
        self.skip_startup()
        self.coordinator.request(Command.walk(1))
        self.coordinator.request(Command.toggle_mode())
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertEqual(self.coordinator.requested_trajectory_id, 0)
        self.assertIsNone(self.coordinator.next_batch())

    def test_commands_are_latched_and_switch_at_the_midpoint(self):
        self.stand_up()
        self.assertTrue(self.coordinator.request(Command.walk(1)))
        start = self.coordinator.next_batch()
        self.assertIn("t1", start.phase_name)

        # Multiple requests during the batch are not queued; only the latest
        # survives to the next legal transition point.
        self.assertTrue(self.coordinator.request(Command.walk(2)))
        self.assertTrue(self.coordinator.request(Command.walk(4)))
        self.coordinator.complete_batch(start.goal_id)

        first_half = self.coordinator.next_batch()
        self.assertEqual(first_half.trajectory_id, 1)
        self.assertIn("full-step-1", first_half.phase_name)
        self.coordinator.complete_batch(first_half.goal_id)

        second_half = self.coordinator.next_batch()
        self.assertEqual(second_half.trajectory_id, 4)
        self.assertIn("full-step-2", second_half.phase_name)

    def test_stop_received_during_first_half_finishes_step_then_stops(self):
        self.stand_up()
        self.coordinator.request(Command.walk(1))
        start = self.coordinator.next_batch()
        self.coordinator.complete_batch(start.goal_id)

        first_half = self.coordinator.next_batch()
        self.coordinator.request(Command.stop())
        self.coordinator.complete_batch(first_half.goal_id)

        second_half = self.coordinator.next_batch()
        self.assertIn("full-step-2", second_half.phase_name)
        self.coordinator.complete_batch(second_half.goal_id)

        final_half = self.coordinator.next_batch()
        self.assertIn("final half-step", final_half.phase_name)
        self.coordinator.complete_batch(final_half.goal_id)
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertIsNone(self.coordinator.next_batch())

    def test_all_generated_joint_values_are_finite_and_phase_sized(self):
        self.stand_up()
        for trajectory_id in range(1, 15):
            coordinator = LiteGaitCoordinator()
            coordinator.request(Command.startup())
            for _ in range(2):
                startup = coordinator.next_batch()
                coordinator.complete_batch(startup.goal_id)
            coordinator.request(Command.walk(trajectory_id))
            for _ in range(3):
                batch = coordinator.next_batch()
                self.assertLessEqual(batch.point_count, 64)
                for point in batch.points:
                    self.assertEqual(len(point), 18)
                    self.assertTrue(all(math.isfinite(value) for value in point))
                coordinator.complete_batch(batch.goal_id)


if __name__ == "__main__":
    unittest.main()
