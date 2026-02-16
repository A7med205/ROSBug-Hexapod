#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass(frozen=True)
class FramePose:
    x: float
    y: float
    theta_deg: float


@dataclass(frozen=True)
class LegInfo:
    leg_id: int
    joint_names: Tuple[str, str, str]
    frame_pose: FramePose
    tripod: str


@dataclass
class Point3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class BasePoseZPitch:
    z: float
    pitch: float


@dataclass(frozen=True)
class LocalDisplacementZ:
    dz_local: float


class ElevationPitchKeyboardController(Node):
    def __init__(self) -> None:
        super().__init__("elevation_pitch_keyboard_controller")

        # Hardcoded per-input steps.
        self.distance_per_input = 0.0006
        self.angle_per_input = 0.007
        self.goal_time_sec = 0.02

        # IK link lengths.
        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        # Neutral tip in leg-local frame.
        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        # Joint limits: j1/j2/j3 for all legs.
        self.joint_mins = [-2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi]
        self.joint_maxs = [2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi]

        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout_sec = float(self.declare_parameter("wait_timeout_sec", 10.0).value)

        self.legs: List[LegInfo] = [
            LegInfo(1, ("jl11", "jl12", "jl13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("jl21", "jl22", "jl23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("jl31", "jl32", "jl33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("jl41", "jl42", "jl43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("jl51", "jl52", "jl53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("jl61", "jl62", "jl63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]
        self.joint_names_flat = [joint for leg in self.legs for joint in leg.joint_names]

        self.current_tip_positions: Dict[int, Point3D] = {
            leg.leg_id: Point3D(self.home_x, self.home_y, self.home_z)
            for leg in self.legs
        }
        self.current_joint_values: Dict[int, Tuple[float, float, float]] = {}
        self.current_base_pose = BasePoseZPitch(z=0.0, pitch=0.0)
        self.mode = "elevation"  # default mode

        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def _deg_to_rad(deg: float) -> float:
        return deg * (math.pi / 180.0)

    def IK(self, tip: Point3D) -> Tuple[float, float, float]:
        y = tip.y
        x = tip.x
        z = tip.z
        j1 = -math.atan2(y, x)

        x_prime = math.sqrt(x * x + y * y) - self.L1
        d = math.sqrt(x_prime * x_prime + z * z)
        min_reach = abs(self.L2 - self.L3)
        max_reach = self.L2 + self.L3
        if d > max_reach or d < min_reach:
            self.get_logger().warn(
                f"IK warning: unreachable tip x'={x_prime:.3f}, z={z:.3f}, d={d:.3f}. Clamping."
            )
            d = self._clamp(d, min_reach, max_reach)

        alpha1 = math.atan2(-z, x_prime)
        cos_alpha2 = self._clamp(
            (self.L2 * self.L2 + d * d - self.L3 * self.L3) / (2.0 * self.L2 * d),
            -1.0,
            1.0,
        )
        alpha2 = math.acos(cos_alpha2)
        cos_knee = self._clamp(
            (self.L2 * self.L2 + self.L3 * self.L3 - d * d) / (2.0 * self.L2 * self.L3),
            -1.0,
            1.0,
        )
        j2 = alpha1 - alpha2
        j3 = math.pi - math.acos(cos_knee)
        return j1, j2, j3

    def base_delta_to_tip_delta_z(
        self,
        base1: BasePoseZPitch,
        base2: BasePoseZPitch,
        leg: LegInfo,
        tip_local_1: Point3D,
    ) -> LocalDisplacementZ:
        # Similar idea to base_delta_to_tip_delta but reduced to vertical motion.
        # Pitch is rotation about base x-axis; use tip forward offset in base frame.
        leg_yaw = self._deg_to_rad(leg.frame_pose.theta_deg)
        y_tip_in_base = (
            leg.frame_pose.y
            + math.sin(leg_yaw) * tip_local_1.x
            + math.cos(leg_yaw) * tip_local_1.y
        )

        tip_world_z = (
            base1.z
            + y_tip_in_base * math.sin(base1.pitch)
            + tip_local_1.z * math.cos(base1.pitch)
        )

        c2 = math.cos(base2.pitch)
        if abs(c2) < 1e-6:
            c2 = 1e-6 if c2 >= 0.0 else -1e-6
        tip_local_2_z = (tip_world_z - base2.z - y_tip_in_base * math.sin(base2.pitch)) / c2
        return LocalDisplacementZ(dz_local=(tip_local_2_z - tip_local_1.z))

    def _initialize_neutral_joint_values(self) -> None:
        for leg in self.legs:
            tip = self.current_tip_positions[leg.leg_id]
            j1, j2, j3 = self.IK(tip)
            self.current_joint_values[leg.leg_id] = (j1, j2, j3)
        self.get_logger().info("Initialized neutral tip and joint values")

    def _within_joint_limits(self, joints: Tuple[float, float, float]) -> bool:
        for idx, value in enumerate(joints):
            if value < self.joint_mins[idx] or value > self.joint_maxs[idx]:
                return False
        return True

    def _build_joint_vector(self, joints_by_leg: Dict[int, Tuple[float, float, float]]) -> List[float]:
        ordered: List[float] = []
        for leg in self.legs:
            j1, j2, j3 = joints_by_leg[leg.leg_id]
            ordered.extend((j1, j2, j3))
        return ordered

    def _send_joint_goal(self, joint_values_flat: List[float]) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names_flat

        point = JointTrajectoryPoint()
        point.positions = list(joint_values_flat)
        point.time_from_start = Duration(seconds=self.goal_time_sec).to_msg()
        goal.trajectory.points = [point]

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected or failed to send")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error("Goal returned no result")
            return False

        result = result_wrapper.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Goal failed error_code={result.error_code}, error='{result.error_string}'"
            )
            return False
        return True

    def _target_pose_one_step(self, direction: int) -> BasePoseZPitch:
        if self.mode == "elevation":
            dz = direction * self.distance_per_input
            return BasePoseZPitch(
                z=self.current_base_pose.z + dz,
                pitch=self.current_base_pose.pitch,
            )

        dp = direction * self.angle_per_input
        return BasePoseZPitch(
            z=self.current_base_pose.z,
            pitch=self.current_base_pose.pitch + dp,
        )

    def _target_pose_return_one_step(self) -> BasePoseZPitch:
        dz_step = self.distance_per_input
        dp_step = self.angle_per_input

        def step_to_zero(value: float, step: float) -> float:
            if abs(value) <= step:
                return 0.0
            return value - math.copysign(step, value)

        return BasePoseZPitch(
            z=step_to_zero(self.current_base_pose.z, dz_step),
            pitch=step_to_zero(self.current_base_pose.pitch, dp_step),
        )

    def _apply_one_iteration(self, target_pose: BasePoseZPitch) -> bool:
        candidate_tips: Dict[int, Point3D] = {}
        candidate_joints: Dict[int, Tuple[float, float, float]] = {}

        for leg in self.legs:
            tip_prev = self.current_tip_positions[leg.leg_id]
            dz = self.base_delta_to_tip_delta_z(self.current_base_pose, target_pose, leg, tip_prev)
            tip_next = Point3D(tip_prev.x, tip_prev.y, tip_prev.z + dz.dz_local)
            joints = self.IK(tip_next)
            if not self._within_joint_limits(joints):
                self.get_logger().warn(
                    f"Joint limits exceeded for leg {leg.leg_id}; step skipped"
                )
                return False
            candidate_tips[leg.leg_id] = tip_next
            candidate_joints[leg.leg_id] = joints

        goal_vector = self._build_joint_vector(candidate_joints)
        if not self._send_joint_goal(goal_vector):
            return False

        self.current_tip_positions = candidate_tips
        self.current_joint_values = candidate_joints
        self.current_base_pose = target_pose
        self.get_logger().info(
            f"Applied 1 step: mode={self.mode}, z={self.current_base_pose.z:.4f}, "
            f"pitch={self.current_base_pose.pitch:.4f} rad"
        )
        return True

    def _is_base_neutral(self) -> bool:
        return abs(self.current_base_pose.z) <= 1e-9 and abs(self.current_base_pose.pitch) <= 1e-9

    def _return_to_neutral_auto(self) -> None:
        if self._is_base_neutral():
            self.get_logger().info("Base already at neutral pose")
            return

        max_steps = 10000
        for _ in range(max_steps):
            if self._is_base_neutral():
                self.get_logger().info("Reached neutral base pose")
                return
            target = self._target_pose_return_one_step()
            if not self._apply_one_iteration(target):
                self.get_logger().warn("Auto-return stopped early due to failed step")
                return

        self.get_logger().warn("Auto-return reached max steps before neutral pose")

    def _toggle_mode(self) -> None:
        self.mode = "pitch" if self.mode == "elevation" else "elevation"
        self.get_logger().info(f"Mode changed to: {self.mode}")

    def run_keyboard_loop(self) -> int:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.get_logger().error(
                f"Action server unavailable after {self.wait_timeout_sec:.1f}s"
            )
            return 1

        self._initialize_neutral_joint_values()

        print("Elevation/Pitch keyboard controller")
        print("m: toggle mode (elevation/pitch)")
        print("i: + direction (up / pitch+)")
        print("k: - direction (down / pitch-)")
        print("r: auto-return to neutral base pose")
        print("q: quit")

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue

                key = sys.stdin.read(1)
                if key == "q":
                    return 0
                if key == "m":
                    self._toggle_mode()
                    continue
                if key == "i":
                    self._apply_one_iteration(self._target_pose_one_step(+1))
                    continue
                if key == "k":
                    self._apply_one_iteration(self._target_pose_one_step(-1))
                    continue
                if key == "r":
                    self._return_to_neutral_auto()
                    continue
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return 0


def main() -> None:
    rclpy.init()
    node = ElevationPitchKeyboardController()
    exit_code = 0
    try:
        exit_code = node.run_keyboard_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
