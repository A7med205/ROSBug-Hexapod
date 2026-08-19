import math
import unittest

from gait_core import ControllerState
from posture_core import PostureConfig, PostureCoordinator, PostureState
from robot_core import BasePose3D, Command, ControllerMode, PostureAxis
from robot_core.coordinator import HexapodCoordinator


class PostureCoordinatorTest(unittest.TestCase):
    def setUp(self):
        coordinator = HexapodCoordinator()
        self.model = coordinator.model
        self.posture = PostureCoordinator(self.model)

    def drain(self):
        batches = []
        while True:
            batch = self.posture.next_batch()
            if batch is None:
                return batches
            batches.append(batch)
            self.posture.complete_batch(batch.goal_id)

    def test_fixed_foot_transform_preserves_world_tip_positions(self):
        pose = BasePose3D(
            z=0.008,
            roll=math.radians(3.0),
        )
        neutral_tips = self.model.neutral_tip_positions()
        transformed_tips = self.model.tips_for_base_pose(pose)

        def world_tip(base_pose, leg, local_tip):
            leg_rotation = self.model.rotation_leg_to_body(leg)
            local = (local_tip.x, local_tip.y, local_tip.z)
            rotated = self.model._mat_vec(leg_rotation, local)
            body = (
                leg.frame_pose.x + rotated[0],
                leg.frame_pose.y + rotated[1],
                rotated[2],
            )
            world_rotation = self.model.rotation_from_rpy(
                base_pose.roll,
                base_pose.pitch,
                base_pose.yaw,
            )
            world = self.model._mat_vec(world_rotation, body)
            return (
                base_pose.x + world[0],
                base_pose.y + world[1],
                base_pose.z + world[2],
            )

        for leg in self.model.legs:
            before = world_tip(BasePose3D(), leg, neutral_tips[leg.leg_id])
            after = world_tip(pose, leg, transformed_tips[leg.leg_id])
            for expected, actual in zip(before, after):
                self.assertAlmostEqual(expected, actual, places=10)

    def test_default_profile_is_smoothed_and_split_at_interrupt_boundaries(self):
        self.assertEqual(self.posture.config.elevation_velocity, 0.030)
        self.assertEqual(self.posture.config.elevation_acceleration, 0.050)
        self.assertAlmostEqual(
            math.degrees(self.posture.config.angular_velocity),
            10.0,
        )
        self.assertAlmostEqual(
            math.degrees(self.posture.config.angular_acceleration),
            40.0,
        )
        self.assertTrue(self.posture.request_delta(PostureAxis.ELEVATION, 0.010))
        positions = [0.0] + [
            sample.pose.z for sample in self.posture._job.samples
        ]
        velocities = [
            (positions[index] - positions[index - 1]) / self.posture.config.sample_period
            for index in range(1, len(positions))
        ]
        accelerations = [
            (velocities[index] - velocities[index - 1])
            / self.posture.config.sample_period
            for index in range(1, len(velocities))
        ]
        self.assertLessEqual(
            max(abs(value) for value in velocities),
            self.posture.config.elevation_velocity + 1.0e-9,
        )
        self.assertLessEqual(
            max(abs(value) for value in accelerations),
            self.posture.config.elevation_acceleration + 1.0e-9,
        )
        batches = self.drain()
        self.assertEqual([batch.point_count for batch in batches], [25, 25, 5])
        self.assertTrue(all(batch.sample_period == 0.02 for batch in batches))
        self.assertEqual(self.posture.state, PostureState.POSTURE_HOLD)
        self.assertAlmostEqual(self.posture.current_pose.z, 0.010)

    def test_relative_commands_accumulate_from_confirmed_pose(self):
        self.posture.request_delta(PostureAxis.ELEVATION, 0.004)
        self.drain()
        self.posture.request_delta(PostureAxis.ELEVATION, 0.003)
        self.drain()
        self.assertAlmostEqual(self.posture.current_pose.z, 0.007)

        self.posture.request_delta(PostureAxis.PITCH, math.radians(2.0))
        self.drain()
        self.posture.request_delta(PostureAxis.PITCH, math.radians(3.0))
        self.drain()
        self.assertAlmostEqual(math.degrees(self.posture.current_pose.pitch), 5.0)
        self.assertAlmostEqual(self.posture.current_pose.z, 0.007)

    def test_pitch_and_roll_are_mutually_exclusive_but_elevation_is_allowed(self):
        self.posture.request_delta(PostureAxis.PITCH, math.radians(4.0))
        self.drain()
        self.assertFalse(
            self.posture.request_delta(PostureAxis.ROLL, math.radians(1.0))
        )
        self.assertTrue(self.posture.request_delta(PostureAxis.ELEVATION, 0.002))
        self.drain()

        self.assertTrue(
            self.posture.request_delta(PostureAxis.PITCH, math.radians(-4.0))
        )
        self.drain()
        self.assertAlmostEqual(self.posture.current_pose.pitch, 0.0)
        self.assertTrue(
            self.posture.request_delta(PostureAxis.ROLL, math.radians(1.0))
        )

    def test_commands_are_not_queued(self):
        self.assertTrue(self.posture.request_delta(PostureAxis.ELEVATION, 0.010))
        self.assertFalse(self.posture.request_delta(PostureAxis.ELEVATION, 0.001))
        batch = self.posture.next_batch()
        self.assertFalse(self.posture.request_delta(PostureAxis.PITCH, 0.01))
        self.posture.complete_batch(batch.goal_id)
        self.assertFalse(self.posture.request_delta(PostureAxis.PITCH, 0.01))

    def test_tilt_reset_preserves_elevation_and_is_not_queued(self):
        self.posture.request_delta(PostureAxis.ELEVATION, 0.005)
        self.drain()
        self.posture.request_delta(PostureAxis.ROLL, math.radians(2.0))
        self.drain()

        self.assertTrue(self.posture.request_tilt_reset())
        self.assertFalse(self.posture.request_tilt_reset())
        batches = self.drain()

        self.assertTrue(batches)
        self.assertTrue(
            all(batch.phase_name == "posture reset roll" for batch in batches)
        )
        self.assertAlmostEqual(self.posture.current_pose.roll, 0.0)
        self.assertAlmostEqual(self.posture.current_pose.pitch, 0.0)
        self.assertAlmostEqual(self.posture.current_pose.z, 0.005)
        self.assertEqual(self.posture.state, PostureState.POSTURE_HOLD)

    def test_operating_envelope_clamps_the_complete_body_pose(self):
        self.assertTrue(self.posture.request_delta(PostureAxis.ELEVATION, 1.0))
        result = self.posture.last_plan_result
        self.assertTrue(result.was_clamped)
        self.assertAlmostEqual(result.applied_delta, 0.100, places=8)

        target = self.posture._job.samples[-1].pose
        tips = self.model.tips_for_base_pose(target)
        self.assertTrue(all(self.model.tip_is_reachable(tip) for tip in tips.values()))
        self.assertTrue(self.posture.pose_is_allowed(target))

        lower = PostureCoordinator(self.model)
        self.assertTrue(lower.request_delta(PostureAxis.ELEVATION, -1.0))
        self.assertAlmostEqual(
            lower.last_plan_result.applied_delta,
            -0.025,
            places=8,
        )

    def test_measured_angular_limits_are_scaled_and_interpolated(self):
        self.assertAlmostEqual(
            math.degrees(
                self.posture.angular_limit_at_elevation(PostureAxis.ROLL, 0.050)
            ),
            18.0,
        )
        self.assertAlmostEqual(
            math.degrees(
                self.posture.angular_limit_at_elevation(PostureAxis.PITCH, 0.050)
            ),
            22.5,
        )
        self.assertAlmostEqual(
            math.degrees(
                self.posture.angular_limit_at_elevation(PostureAxis.PITCH, 0.080)
            ),
            10.8,
        )

        self.posture.request_delta(PostureAxis.ELEVATION, 0.050)
        self.drain()
        self.posture.request_delta(PostureAxis.PITCH, math.radians(100.0))
        result = self.posture.last_plan_result
        self.assertTrue(result.was_clamped)
        self.assertAlmostEqual(math.degrees(result.applied_delta), 22.5, places=6)

    def test_elevation_change_cannot_cross_a_narrowing_tilt_envelope(self):
        self.posture.request_delta(PostureAxis.ELEVATION, 0.050)
        self.drain()
        self.posture.request_delta(PostureAxis.PITCH, math.radians(22.5))
        self.drain()

        self.assertTrue(self.posture.request_delta(PostureAxis.ELEVATION, 0.030))
        result = self.posture.last_plan_result
        self.assertTrue(result.was_clamped)
        self.assertAlmostEqual(result.applied_delta, 0.0, places=8)
        self.assertIsNone(self.posture.next_batch())

    def test_return_removes_tilt_before_elevation(self):
        self.posture.request_delta(PostureAxis.ELEVATION, 0.004)
        self.drain()
        self.posture.request_delta(PostureAxis.PITCH, math.radians(2.0))
        self.drain()

        self.posture.request_return_to_neutral()
        names = [batch.phase_name for batch in self.drain()]
        first_elevation = names.index("posture return elevation")
        self.assertTrue(all(name == "posture return pitch" for name in names[:first_elevation]))
        self.assertEqual(self.posture.state, PostureState.NEUTRAL)
        self.assertTrue(self.posture.is_neutral)


class ThreeModeCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.coordinator = HexapodCoordinator()
        self.assertTrue(self.coordinator.request(Command.skip_startup()))

    def drain(self):
        batches = []
        while True:
            batch = self.coordinator.next_batch()
            if batch is None:
                return batches
            batches.append(batch)
            self.coordinator.complete_batch(batch.goal_id)

    def enter_posture(self):
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertTrue(self.coordinator.request(Command.toggle_mode()))
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)

    def test_modes_cycle_normal_auto_posture_normal(self):
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)
        self.coordinator.request(Command.toggle_mode())
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.coordinator.request(Command.toggle_mode())
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        self.coordinator.request(Command.toggle_mode())
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)

    def test_posture_holds_and_exit_returns_to_stationary(self):
        self.enter_posture()
        self.assertTrue(
            self.coordinator.request(Command.posture(PostureAxis.ELEVATION, 0.005))
        )
        self.drain()
        self.assertEqual(self.coordinator.state, PostureState.POSTURE_HOLD)
        self.assertFalse(self.coordinator.is_stationary)

        self.assertTrue(
            self.coordinator.request(Command.set_mode(ControllerMode.AUTO))
        )
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        returning = self.drain()
        self.assertTrue(returning)
        self.assertEqual(self.coordinator.mode, ControllerMode.AUTO)
        self.assertEqual(self.coordinator.state, ControllerState.STATIONARY)
        self.assertTrue(self.coordinator.is_stationary)

    def test_mode_change_during_posture_command_finishes_then_returns(self):
        self.enter_posture()
        self.coordinator.request(Command.posture(PostureAxis.ELEVATION, 0.005))
        first = self.coordinator.next_batch()
        self.assertTrue(
            self.coordinator.request(Command.set_mode(ControllerMode.NORMAL))
        )
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        self.coordinator.complete_batch(first.goal_id)

        names = [batch.phase_name for batch in self.drain()]
        self.assertIn("posture elevation", names)
        self.assertIn("posture return elevation", names)
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)

    def test_entering_posture_while_walking_stops_first(self):
        self.coordinator.request(Command.walk(1))
        start = self.coordinator.next_batch()
        self.coordinator.request(Command.set_mode(ControllerMode.POSTURE))
        self.assertEqual(self.coordinator.mode, ControllerMode.NORMAL)
        self.coordinator.complete_batch(start.goal_id)

        final_half = self.coordinator.next_batch()
        self.assertIn("final half-step", final_half.phase_name)
        self.coordinator.complete_batch(final_half.goal_id)
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        self.assertTrue(self.coordinator.is_stationary)

    def test_stop_in_posture_returns_neutral_without_leaving_mode(self):
        self.enter_posture()
        self.coordinator.request(Command.posture(PostureAxis.ROLL, math.radians(2.0)))
        self.drain()
        self.assertTrue(self.coordinator.request(Command.stop()))
        self.drain()
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        self.assertTrue(self.coordinator.is_stationary)

    def test_reset_tilt_command_preserves_elevation(self):
        self.enter_posture()
        self.coordinator.request(Command.posture(PostureAxis.ELEVATION, 0.005))
        self.drain()
        self.coordinator.request(Command.posture(PostureAxis.PITCH, math.radians(2.0)))
        self.drain()

        self.assertTrue(self.coordinator.request(Command.reset_tilt()))
        self.drain()

        self.assertAlmostEqual(self.coordinator.posture.current_pose.pitch, 0.0)
        self.assertAlmostEqual(self.coordinator.posture.current_pose.z, 0.005)
        self.assertEqual(self.coordinator.state, PostureState.POSTURE_HOLD)

    def test_first_stop_interrupts_at_boundary_and_second_returns_neutral(self):
        self.enter_posture()
        self.coordinator.request(Command.posture(PostureAxis.ELEVATION, 0.020))
        first = self.coordinator.next_batch()
        self.assertEqual(first.point_count, 25)
        self.assertTrue(self.coordinator.request(Command.stop()))
        self.coordinator.complete_batch(first.goal_id)

        interrupted_z = self.coordinator.posture.current_pose.z
        self.assertGreater(interrupted_z, 0.0)
        self.assertLess(interrupted_z, 0.020)
        self.assertEqual(self.coordinator.state, PostureState.POSTURE_HOLD)
        self.assertIsNone(self.coordinator.next_batch())

        self.assertTrue(self.coordinator.request(Command.stop()))
        returning = self.drain()
        self.assertTrue(returning)
        self.assertEqual(self.coordinator.mode, ControllerMode.POSTURE)
        self.assertTrue(self.coordinator.is_stationary)

    def test_goal_ids_are_unique_across_gait_and_posture(self):
        other = HexapodCoordinator(
            posture_config=PostureConfig(
                elevation_velocity=0.05,
                elevation_acceleration=0.2,
            )
        )
        other.request(Command.startup())
        startup_ids = []
        for _ in range(2):
            batch = other.next_batch()
            startup_ids.append(batch.goal_id)
            other.complete_batch(batch.goal_id)
        other.request(Command.set_mode(ControllerMode.POSTURE))
        other.request(Command.posture(PostureAxis.ELEVATION, 0.001))
        posture = other.next_batch()
        self.assertGreater(posture.goal_id, max(startup_ids))


if __name__ == "__main__":
    unittest.main()
