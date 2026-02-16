#!/usr/bin/env python3

import math
from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class JointGoalSequenceSender(Node):
    JOINT_NAMES = [
        "jl11", "jl12", "jl13",
        "jl21", "jl22", "jl23",
        "jl31", "jl32", "jl33",
        "jl41", "jl42", "jl43",
        "jl51", "jl52", "jl53",
        "jl61", "jl62", "jl63",
    ]

    def __init__(self) -> None:
        super().__init__("joint_goal_sequence_sender")
        self._action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self._point_time_sec = float(self.declare_parameter("point_time_sec", 0.00).value)
        self._wait_timeout_sec = float(self.declare_parameter("wait_timeout_sec", 10.0).value)
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    def run(self) -> int:
        self.get_logger().info(f"Waiting for action server: {self._action_name}")
        if not self._client.wait_for_server(timeout_sec=self._wait_timeout_sec):
            self.get_logger().error(
                f"Action server not available after {self._wait_timeout_sec:.1f}s: {self._action_name}"
            )
            return 1

        jl13_values = self._build_jl12_sequence()
        total = len(jl13_values)
        self.get_logger().info(f"Sending {total} goals")

        for index, jl13_value in enumerate(jl13_values, start=1):
            positions = self._build_positions(jl13_value)
            if not self._send_goal_and_wait(positions, index, total, jl13_value):
                return 1

        self.get_logger().info("All goals completed. Shutting down.")
        return 0

    def _build_jl12_sequence(self) -> List[float]:
        step_rad = math.radians(10.0)
        values: List[float] = []
        current = 2.216

        while current > 0.646:
            values.append(current)
            current -= step_rad

        current = 0.646

        while current < 2.216:
            values.append(current)
            current += step_rad

        values.append(2.216)
        return values

    def _build_positions(self, jl13_value: float) -> List[float]:
        return [
            0.0, -0.955, jl13_value,
            0.0, -0.955, 2.216,
            0.0, -0.955, 2.216,
            0.0, -0.955, 2.216,
            0.0, -0.955, 2.216,
            0.0, -0.955, 2.216,
        ]

    def _send_goal_and_wait(
        self,
        positions: List[float],
        index: int,
        total: int,
        jl13_value: float,
    ) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=self._point_time_sec).to_msg()
        goal.trajectory.points = [point]

        self.get_logger().info(f"Goal {index}/{total}: jl12={jl13_value:.6f} rad")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"Goal {index}/{total} failed to send")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal {index}/{total} rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error(f"Goal {index}/{total} returned no result")
            return False

        result = result_wrapper.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Goal {index}/{total} failed with error_code={result.error_code}, "
                f"error_string='{result.error_string}'"
            )
            return False

        self.get_logger().info(f"Goal {index}/{total} completed successfully")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointGoalSequenceSender()
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
