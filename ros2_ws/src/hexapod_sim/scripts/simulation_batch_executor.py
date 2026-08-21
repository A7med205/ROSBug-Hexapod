"""ROS FollowJointTrajectory executor independent of keyboard input."""

from __future__ import annotations

import time
from typing import Callable

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_core import (
    BatchExecutionResult,
    BatchExecutionStatus,
    JointBatch,
)


JOINT_NAMES = [
    f"jl{leg}{joint}"
    for leg in range(1, 7)
    for joint in range(1, 4)
]


class SimulationBatchExecutor(Node):
    def __init__(self) -> None:
        super().__init__("hexapod_sim_batch_executor")
        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout = float(
            self.declare_parameter("wait_timeout", 10.0).value
        )
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
        )

    def wait_for_server(self) -> bool:
        return self.action_client.wait_for_server(timeout_sec=self.wait_timeout)

    def close(self) -> None:
        self.destroy_node()

    @staticmethod
    def _goal_from_batch(batch: JointBatch):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        for point_index, positions in enumerate(batch.points):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = Duration(
                seconds=(point_index + 1) * batch.sample_period
            ).to_msg()
            goal.trajectory.points.append(point)
        return goal

    def _wait_future(self, future, poll_commands: Callable[[], None]) -> bool:
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            poll_commands()
        return future.done()

    def execute(
        self,
        batch: JointBatch,
        poll_commands: Callable[[], None],
    ) -> BatchExecutionResult:
        try:
            send_future = self.action_client.send_goal_async(
                self._goal_from_batch(batch)
            )
            if not self._wait_future(send_future, poll_commands):
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.SHUTDOWN,
                    "ROS shutdown while sending goal",
                )
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.REJECTED,
                    "trajectory action server rejected goal",
                )

            result_future = goal_handle.get_result_async()
            if not self._wait_future(result_future, poll_commands):
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.SHUTDOWN,
                    "ROS shutdown while waiting for result",
                )
            wrapper = result_future.result()
            if wrapper is None:
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.TRANSPORT_ERROR,
                    "trajectory action returned no result",
                )
            result = wrapper.result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.REJECTED,
                    f"{result.error_code} {result.error_string}".strip(),
                )

            deadline = time.monotonic() + batch.hold_after
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                poll_commands()
            if not rclpy.ok():
                return BatchExecutionResult(
                    batch.goal_id,
                    BatchExecutionStatus.SHUTDOWN,
                    "ROS shutdown during post-batch hold",
                )
            return BatchExecutionResult(
                batch.goal_id,
                BatchExecutionStatus.COMPLETED,
                "simulation batch completed",
            )
        except Exception as error:
            return BatchExecutionResult(
                batch.goal_id,
                BatchExecutionStatus.TRANSPORT_ERROR,
                str(error),
            )
