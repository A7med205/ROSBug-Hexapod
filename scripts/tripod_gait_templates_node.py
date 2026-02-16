#!/usr/bin/env python3

import math
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
class BasePose2D:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class LocalDisplacement2D:
    dx_local: float
    dy_local: float


class TripodGaitTemplatesNode(Node):
    PATH_TYPES = ("negativeF", "Positive", "Full")

    def __init__(self) -> None:
        super().__init__("tripod_gait_templates_node")

        # Required hard-coded gait constants.
        self.limit_radius = 0.05
        self.swing_height = 0.025
        self.sample_rate = 0.02
        self.min_angle = 1.0
        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        # Leg neutral tip target in local leg frame.
        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        self.trajectory_id = int(self.declare_parameter("trajectory_id", 0).value)
        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout_sec = float(self.declare_parameter("wait_timeout_sec", 10.0).value)
        self.master_linear_speed = 0.5

        self.legs: List[LegInfo] = [
            LegInfo(1, ("jl11", "jl12", "jl13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("jl21", "jl22", "jl23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("jl31", "jl32", "jl33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("jl41", "jl42", "jl43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("jl51", "jl52", "jl53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("jl61", "jl62", "jl63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]
        self.leg_by_id: Dict[int, LegInfo] = {leg.leg_id: leg for leg in self.legs}
        self.joint_names_flat: List[str] = [joint for leg in self.legs for joint in leg.joint_names]

        self.tip_paths_local: Dict[int, Dict[str, List[Point3D]]] = {
            leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES}
            for leg in self.legs
        }
        self.tip_swings_local: Dict[int, Dict[str, List[Point3D]]] = {
            leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES}
            for leg in self.legs
        }
        self.joint_paths: Dict[int, Dict[str, List[Tuple[float, float, float]]]] = {
            leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES}
            for leg in self.legs
        }
        self.joint_swings: Dict[int, Dict[str, List[Tuple[float, float, float]]]] = {
            leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES}
            for leg in self.legs
        }

        self.current_joint_goal = self._initial_joint_goal()
        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)

    def _initial_joint_goal(self) -> List[float]:
        neutral_tip = Point3D(self.home_x, self.home_y, self.home_z)
        values: List[float] = []
        for _leg in self.legs:
            j1, j2, j3 = self.IK(neutral_tip)
            values.extend((j1, j2, j3))
        return values

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def _deg_to_rad(deg: float) -> float:
        return deg * (math.pi / 180.0)

    def IK(self, tip: Point3D) -> Tuple[float, float, float]:
        # 3-DOF leg IK in local leg frame: x right, y forward, z up.
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

    def base_delta_to_tip_delta(
        self,
        base1: BasePose2D,
        base2: BasePose2D,
        leg: LegInfo,
        tip_local_1: Point3D,
    ) -> LocalDisplacement2D:
        leg_theta = self._deg_to_rad(leg.frame_pose.theta_deg)
        c1 = math.cos(base1.theta)
        s1 = math.sin(base1.theta)
        c2 = math.cos(base2.theta)
        s2 = math.sin(base2.theta)

        leg1_x = base1.x + c1 * leg.frame_pose.x - s1 * leg.frame_pose.y
        leg1_y = base1.y + s1 * leg.frame_pose.x + c1 * leg.frame_pose.y
        leg2_x = base2.x + c2 * leg.frame_pose.x - s2 * leg.frame_pose.y
        leg2_y = base2.y + s2 * leg.frame_pose.x + c2 * leg.frame_pose.y

        psi1 = base1.theta + leg_theta
        psi2 = base2.theta + leg_theta

        cp1 = math.cos(psi1)
        sp1 = math.sin(psi1)
        tip_world_x = leg1_x + cp1 * tip_local_1.x - sp1 * tip_local_1.y
        tip_world_y = leg1_y + sp1 * tip_local_1.x + cp1 * tip_local_1.y

        dx_world = tip_world_x - leg2_x
        dy_world = tip_world_y - leg2_y
        cp2 = math.cos(psi2)
        sp2 = math.sin(psi2)
        tip_local_2_x = cp2 * dx_world + sp2 * dy_world
        tip_local_2_y = -sp2 * dx_world + cp2 * dy_world

        return LocalDisplacement2D(
            tip_local_2_x - tip_local_1.x,
            tip_local_2_y - tip_local_1.y,
        )

    def master_path(self, t: float, trajectory_type_id: int) -> BasePose2D:
        # Type 0: straight line in +y with constant speed.
        if trajectory_type_id == 0:
            y = self.master_linear_speed * t
            return BasePose2D(0.0, y, 0.0)
        return BasePose2D(0.0, 0.0, 0.0)

    def pull_builder(self, tripod: str, trajectory_type_id: int, sign: int) -> Dict[int, List[Point3D]]:
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")

        selected_legs = [leg for leg in self.legs if leg.tripod == tripod]
        home_tip = Point3D(self.home_x, self.home_y, self.home_z)

        path_points: Dict[int, List[Point3D]] = {}
        current_tip: Dict[int, Point3D] = {}
        start_xy: Dict[int, Tuple[float, float]] = {}
        for leg in selected_legs:
            path_points[leg.leg_id] = [Point3D(home_tip.x, home_tip.y, home_tip.z)]
            current_tip[leg.leg_id] = Point3D(home_tip.x, home_tip.y, home_tip.z)
            start_xy[leg.leg_id] = (home_tip.x, home_tip.y)

        max_steps = 10000
        reached_limit = False
        t_prev = 0.0
        base_prev = self.master_path(t_prev, trajectory_type_id)

        for _ in range(max_steps):
            t_curr = t_prev + sign * self.sample_rate
            base_curr = self.master_path(t_curr, trajectory_type_id)

            any_tip_hit = False
            for leg in selected_legs:
                tip_prev = current_tip[leg.leg_id]
                delta = self.base_delta_to_tip_delta(base_prev, base_curr, leg, tip_prev)
                tip_next = Point3D(
                    tip_prev.x + delta.dx_local,
                    tip_prev.y + delta.dy_local,
                    home_tip.z,
                )
                current_tip[leg.leg_id] = tip_next
                path_points[leg.leg_id].append(tip_next)

                sx, sy = start_xy[leg.leg_id]
                if math.hypot(tip_next.x - sx, tip_next.y - sy) >= self.limit_radius:
                    any_tip_hit = True

            t_prev = t_curr
            base_prev = base_curr
            if any_tip_hit:
                reached_limit = True
                break

        if not reached_limit:
            self.get_logger().warn(
                f"pull_builder tripod={tripod} sign={sign:+d} hit max_steps before reaching limit_radius"
            )

        return path_points

    def swing_builder(self, path_end: Point3D, path_start: Point3D, point_count: int) -> List[Point3D]:
        if point_count < 2:
            point_count = 2

        points: List[Point3D] = []
        for idx in range(point_count):
            s = float(idx) / float(point_count - 1)
            x = path_end.x + s * (path_start.x - path_end.x)
            y = path_end.y + s * (path_start.y - path_end.y)
            z = (
                (1.0 - s) * path_end.z
                + s * path_start.z
                + self.swing_height * math.sin(math.pi * s)
            )
            points.append(Point3D(x, y, z))
        return points

    def _build_tripod_templates(self, tripod: str, trajectory_type_id: int) -> None:
        positive = self.pull_builder(tripod, trajectory_type_id, +1)
        negative = self.pull_builder(tripod, trajectory_type_id, -1)

        for leg in [leg for leg in self.legs if leg.tripod == tripod]:
            leg_id = leg.leg_id
            positive_path = positive.get(leg_id, [])
            negative_path = negative.get(leg_id, [])

            if not positive_path:
                positive_path = [Point3D(self.home_x, self.home_y, self.home_z)]
            if not negative_path:
                negative_path = [Point3D(self.home_x, self.home_y, self.home_z)]

            negative_f = list(reversed(negative_path))
            full_path = negative_f + positive_path[1:]

            self.tip_paths_local[leg_id]["negativeF"] = negative_f
            self.tip_paths_local[leg_id]["Positive"] = positive_path
            self.tip_paths_local[leg_id]["Full"] = full_path

            negf_count = len(negative_f)
            pos_count = len(positive_path)
            full_count = len(full_path)

            self.tip_swings_local[leg_id]["negativeF"] = self.swing_builder(
                path_end=negative_f[-1],
                path_start=negative_f[0],
                point_count=pos_count,
            )
            self.tip_swings_local[leg_id]["Positive"] = self.swing_builder(
                path_end=positive_path[-1],
                path_start=positive_path[0],
                point_count=negf_count,
            )
            self.tip_swings_local[leg_id]["Full"] = self.swing_builder(
                path_end=full_path[-1],
                path_start=full_path[0],
                point_count=full_count,
            )

            self.get_logger().info(
                f"Tripod {tripod} leg {leg_id}: "
                f"negativeF={negf_count}, Positive={pos_count}, Full={full_count}"
            )

    def p_to_joint_space(self) -> None:
        for leg in self.legs:
            leg_id = leg.leg_id
            for path_type in self.PATH_TYPES:
                self.joint_paths[leg_id][path_type] = [
                    self.IK(point) for point in self.tip_paths_local[leg_id][path_type]
                ]
                self.joint_swings[leg_id][path_type] = [
                    self.IK(point) for point in self.tip_swings_local[leg_id][path_type]
                ]

    def _send_joint_goal(self, joint_values: List[float]) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names_flat

        point = JointTrajectoryPoint()
        point.positions = list(joint_values)
        point.time_from_start = Duration(seconds=self.sample_rate).to_msg()
        goal.trajectory.points = [point]

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Action goal was rejected or not sent")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error("Action goal returned no result")
            return False

        result = result_wrapper.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Action failed error_code={result.error_code}, error_string='{result.error_string}'"
            )
            return False
        return True

    def execute_phase(
        self,
        phase_name: str,
        tripod_a_mode: Tuple[str, str],
        tripod_b_mode: Tuple[str, str],
    ) -> bool:
        tripod_mode = {"A": tripod_a_mode, "B": tripod_b_mode}
        leg_sequences: Dict[int, List[Tuple[float, float, float]]] = {}
        for leg in self.legs:
            mode_kind, path_type = tripod_mode[leg.tripod]
            if mode_kind == "path":
                seq = self.joint_paths[leg.leg_id][path_type]
            else:
                seq = self.joint_swings[leg.leg_id][path_type]

            if not seq:
                self.get_logger().error(
                    f"{phase_name}: empty sequence for leg={leg.leg_id}, mode={mode_kind}, type={path_type}"
                )
                return False
            leg_sequences[leg.leg_id] = seq

        phase_points = max(len(seq) for seq in leg_sequences.values())
        min_angle_rad = math.radians(self.min_angle)
        self.get_logger().info(
            f"Executing {phase_name}: points={phase_points}, A={tripod_a_mode}, B={tripod_b_mode}"
        )

        for idx in range(phase_points):
            desired_flat: List[float] = []
            for leg in self.legs:
                seq = leg_sequences[leg.leg_id]
                sample = seq[idx] if idx < len(seq) else seq[-1]
                desired_flat.extend(sample)

            for joint_idx, desired in enumerate(desired_flat):
                current = self.current_joint_goal[joint_idx]
                if (not math.isfinite(current)) or abs(desired - current) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired

            if not self._send_joint_goal(self.current_joint_goal):
                self.get_logger().error(f"{phase_name}: failed at point index {idx}")
                return False

        return True

    def coordinator(self) -> bool:
        self._build_tripod_templates("A", self.trajectory_id)
        self._build_tripod_templates("B", self.trajectory_id)
        self.p_to_joint_space()

        phases = [
            ("first half-step", ("path", "Positive"), ("swing", "negativeF")),
            ("first full-step", ("swing", "Full"), ("path", "Full")),
            ("final half-step", ("path", "negativeF"), ("swing", "Positive")),
        ]
        for name, mode_a, mode_b in phases:
            if not self.execute_phase(name, mode_a, mode_b):
                return False
        return True

    def run(self) -> int:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.get_logger().error(
                f"Action server unavailable after {self.wait_timeout_sec:.1f}s: {self.action_name}"
            )
            return 1

        if not self.coordinator():
            return 1

        self.get_logger().info("Finished half-step + full-step + half-step sequence. Shutting down.")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TripodGaitTemplatesNode()
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
