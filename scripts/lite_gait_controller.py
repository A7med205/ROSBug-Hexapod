#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32
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


@dataclass(frozen=True)
class BasePose2D:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class LocalDisplacement2D:
    dx_local: float
    dy_local: float


@dataclass
class Point3D:
    x: float
    y: float
    z: float


class LiteGaitController(Node):
    PATH_TYPES = ("negativeF", "Positive", "Full")
    MOVING_TRAJECTORY_IDS = (1, 2, 3, 4, 5)
    STATIONARY_ID = 0

    def __init__(self) -> None:
        super().__init__("lite_gait_controller")

        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout_sec = float(self.declare_parameter("wait_timeout_sec", 10.0).value)
        self.trajectory_topic = str(self.declare_parameter("trajectory_topic", "/trajectory_type").value)

        self._init_state()

        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.trajectory_sub = self.create_subscription(Int32, self.trajectory_topic, self._trajectory_callback, 10)

    def _init_state(self) -> None:
        self.limit_radius = 0.05
        self.swing_height = 0.025
        self.sample_rate = 0.02
        self.min_angle = 1.0
        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        self.linear_speed_y = 0.6
        self.angular_speed = 1.57
        self.external_radius = 0.30
        self.external_center_x = self.external_radius
        self.external_center_y = 0.0

        self.legs: List[LegInfo] = [
            LegInfo(1, ("jl11", "jl12", "jl13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("jl21", "jl22", "jl23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("jl31", "jl32", "jl33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("jl41", "jl42", "jl43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("jl51", "jl52", "jl53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("jl61", "jl62", "jl63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]
        self.joint_names_flat = [joint for leg in self.legs for joint in leg.joint_names]

        self._reset_template_stores()
        self.current_joint_goal = self._initial_joint_goal()

        self.requested_trajectory_id = self.STATIONARY_ID
        self.active_trajectory_id = self.STATIONARY_ID
        self.next_full_pull_tripod = "B"

    def _new_leg_store(self):
        return {leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES} for leg in self.legs}

    def _new_transition_store(self):
        return {
            from_id: {
                to_id: {leg.leg_id: [] for leg in self.legs}
                for to_id in self.MOVING_TRAJECTORY_IDS
                if to_id != from_id
            }
            for from_id in self.MOVING_TRAJECTORY_IDS
        }

    def _reset_template_stores(self) -> None:
        self.tip_paths = {traj_id: self._new_leg_store() for traj_id in self.MOVING_TRAJECTORY_IDS}
        self.tip_swings = {traj_id: self._new_leg_store() for traj_id in self.MOVING_TRAJECTORY_IDS}
        self.tip_transition_paths = self._new_transition_store()
        self.tip_transition_swings = self._new_transition_store()

        self.joint_paths = {traj_id: self._new_leg_store() for traj_id in self.MOVING_TRAJECTORY_IDS}
        self.joint_swings = {traj_id: self._new_leg_store() for traj_id in self.MOVING_TRAJECTORY_IDS}
        self.joint_transition_paths = self._new_transition_store()
        self.joint_transition_swings = self._new_transition_store()

    def _initial_joint_goal(self) -> List[float]:
        neutral_tip = Point3D(self.home_x, self.home_y, self.home_z)
        values: List[float] = []
        for _ in self.legs:
            values.extend(self.IK(neutral_tip))
        return values

    def _trajectory_callback(self, msg: Int32) -> None:
        value = int(msg.data)
        if value == self.STATIONARY_ID or value in self.MOVING_TRAJECTORY_IDS:
            self.requested_trajectory_id = value

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def _deg_to_rad(deg: float) -> float:
        return deg * (math.pi / 180.0)

    @staticmethod
    def _opposite_tripod(tripod: str) -> str:
        return "B" if tripod == "A" else "A"

    def _tripod_legs(self, tripod: str) -> List[LegInfo]:
        return [leg for leg in self.legs if leg.tripod == tripod]

    def IK(self, tip: Point3D) -> Tuple[float, float, float]:
        y = tip.y
        x = tip.x
        z = tip.z
        j1 = -math.atan2(y, x)

        x_prime = math.sqrt(x * x + y * y) - self.L1
        d = math.sqrt(x_prime * x_prime + z * z)

        min_reach = abs(self.L2 - self.L3)
        max_reach = self.L2 + self.L3
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

        return LocalDisplacement2D(tip_local_2_x - tip_local_1.x, tip_local_2_y - tip_local_1.y)

    def master_path(self, t: float, trajectory_type_id: int) -> BasePose2D:
        if trajectory_type_id == 1:
            return BasePose2D(0.0, self.linear_speed_y * t, 0.0)
        if trajectory_type_id == 2:
            return BasePose2D(0.0, 0.0, self.angular_speed * t)
        if trajectory_type_id == 3:
            return BasePose2D(0.0, 0.0, -self.angular_speed * t)
        if trajectory_type_id in (4, 5):
            direction = 1.0 if trajectory_type_id == 4 else -1.0
            phi = math.pi + direction * self.angular_speed * t
            x = self.external_center_x + self.external_radius * math.cos(phi)
            y = self.external_center_y + self.external_radius * math.sin(phi)
            return BasePose2D(x, y, direction * self.angular_speed * t)
        return BasePose2D(0.0, 0.0, 0.0)

    def pull_builder(self, tripod: str, trajectory_type_id: int, sign: int) -> Dict[int, List[Point3D]]:
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")

        selected_legs = self._tripod_legs(tripod)
        home_tip = Point3D(self.home_x, self.home_y, self.home_z)

        path_points: Dict[int, List[Point3D]] = {
            leg.leg_id: [Point3D(home_tip.x, home_tip.y, home_tip.z)]
            for leg in selected_legs
        }
        current_tip: Dict[int, Point3D] = {
            leg.leg_id: Point3D(home_tip.x, home_tip.y, home_tip.z)
            for leg in selected_legs
        }
        start_xy: Dict[int, Tuple[float, float]] = {
            leg.leg_id: (home_tip.x, home_tip.y)
            for leg in selected_legs
        }

        t_prev = 0.0
        base_prev = self.master_path(t_prev, trajectory_type_id)

        for _ in range(10000):
            t_curr = t_prev + sign * self.sample_rate
            base_curr = self.master_path(t_curr, trajectory_type_id)

            hit_limit = False
            for leg in selected_legs:
                tip_prev = current_tip[leg.leg_id]
                delta = self.base_delta_to_tip_delta(base_prev, base_curr, leg, tip_prev)
                tip_next = Point3D(tip_prev.x + delta.dx_local, tip_prev.y + delta.dy_local, home_tip.z)
                current_tip[leg.leg_id] = tip_next
                path_points[leg.leg_id].append(tip_next)

                sx, sy = start_xy[leg.leg_id]
                if math.hypot(tip_next.x - sx, tip_next.y - sy) >= self.limit_radius:
                    hit_limit = True

            t_prev = t_curr
            base_prev = base_curr
            if hit_limit:
                break

        return path_points

    def swing_builder(self, path_end: Point3D, path_start: Point3D, point_count: int) -> List[Point3D]:
        point_count = max(point_count, 2)
        out: List[Point3D] = []
        for idx in range(point_count):
            s = float(idx) / float(point_count - 1)
            out.append(
                Point3D(
                    x=path_end.x + s * (path_start.x - path_end.x),
                    y=path_end.y + s * (path_start.y - path_end.y),
                    z=(1.0 - s) * path_end.z + s * path_start.z + self.swing_height * math.sin(math.pi * s),
                )
            )
        return out

    def _build_tripod_templates(self, trajectory_type_id: int, tripod: str) -> None:
        positive = self.pull_builder(tripod, trajectory_type_id, +1)
        negative = self.pull_builder(tripod, trajectory_type_id, -1)

        for leg in self._tripod_legs(tripod):
            leg_id = leg.leg_id
            positive_path = positive.get(leg_id, [Point3D(self.home_x, self.home_y, self.home_z)])
            negative_path = negative.get(leg_id, [Point3D(self.home_x, self.home_y, self.home_z)])

            negative_f = list(reversed(negative_path))
            full_path = negative_f + positive_path[1:]

            self.tip_paths[trajectory_type_id][leg_id]["negativeF"] = negative_f
            self.tip_paths[trajectory_type_id][leg_id]["Positive"] = positive_path
            self.tip_paths[trajectory_type_id][leg_id]["Full"] = full_path

            self.tip_swings[trajectory_type_id][leg_id]["negativeF"] = self.swing_builder(
                path_end=negative_f[-1],
                path_start=negative_f[0],
                point_count=len(positive_path),
            )
            self.tip_swings[trajectory_type_id][leg_id]["Positive"] = self.swing_builder(
                path_end=positive_path[-1],
                path_start=positive_path[0],
                point_count=len(negative_f),
            )
            self.tip_swings[trajectory_type_id][leg_id]["Full"] = self.swing_builder(
                path_end=full_path[-1],
                path_start=full_path[0],
                point_count=len(full_path),
            )

    def _build_transition_templates(self) -> None:
        home = Point3D(self.home_x, self.home_y, self.home_z)
        for from_id in self.MOVING_TRAJECTORY_IDS:
            for to_id in self.MOVING_TRAJECTORY_IDS:
                if from_id == to_id:
                    continue
                for leg in self.legs:
                    leg_id = leg.leg_id
                    from_negative_f = self.tip_paths[from_id][leg_id]["negativeF"] or [home]
                    to_positive = self.tip_paths[to_id][leg_id]["Positive"] or [home]
                    from_full = self.tip_paths[from_id][leg_id]["Full"] or [home]
                    to_full = self.tip_paths[to_id][leg_id]["Full"] or [home]

                    transition_path = [Point3D(p.x, p.y, p.z) for p in from_negative_f]
                    transition_path.extend(Point3D(p.x, p.y, p.z) for p in to_positive)

                    self.tip_transition_paths[from_id][to_id][leg_id] = transition_path
                    self.tip_transition_swings[from_id][to_id][leg_id] = self.swing_builder(
                        path_end=from_full[-1],
                        path_start=to_full[0],
                        point_count=len(transition_path),
                    )

    def _build_all_templates(self) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            self._build_tripod_templates(trajectory_type_id, "A")
            self._build_tripod_templates(trajectory_type_id, "B")
        self._build_transition_templates()

    def _convert_tip_store_to_joint_store(self, src, dst) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for path_type in self.PATH_TYPES:
                    dst[trajectory_type_id][leg_id][path_type] = [
                        self.IK(point) for point in src[trajectory_type_id][leg_id][path_type]
                    ]

    def p_to_joint_space(self) -> None:
        self._convert_tip_store_to_joint_store(self.tip_paths, self.joint_paths)
        self._convert_tip_store_to_joint_store(self.tip_swings, self.joint_swings)

        for from_id in self.MOVING_TRAJECTORY_IDS:
            for to_id in self.MOVING_TRAJECTORY_IDS:
                if from_id == to_id:
                    continue
                for leg in self.legs:
                    leg_id = leg.leg_id
                    self.joint_transition_paths[from_id][to_id][leg_id] = [
                        self.IK(point) for point in self.tip_transition_paths[from_id][to_id][leg_id]
                    ]
                    self.joint_transition_swings[from_id][to_id][leg_id] = [
                        self.IK(point) for point in self.tip_transition_swings[from_id][to_id][leg_id]
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
            self.get_logger().error("Action goal rejected")
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

    def _execute_joint_sequences(
        self,
        phase_name: str,
        leg_sequences: Dict[int, List[Tuple[float, float, float]]],
    ) -> bool:
        if not leg_sequences:
            self.get_logger().error(f"{phase_name}: no sequences")
            return False
        for leg_id, seq in leg_sequences.items():
            if not seq:
                self.get_logger().error(f"{phase_name}: empty sequence for leg {leg_id}")
                return False

        phase_points = max(len(seq) for seq in leg_sequences.values())
        min_angle_rad = math.radians(self.min_angle)

        for point_idx in range(phase_points):
            desired_flat: List[float] = []
            for leg in self.legs:
                seq = leg_sequences[leg.leg_id]
                sample = seq[point_idx] if point_idx < len(seq) else seq[-1]
                desired_flat.extend(sample)

            for joint_idx, desired in enumerate(desired_flat):
                current = self.current_joint_goal[joint_idx]
                if (not math.isfinite(current)) or abs(desired - current) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired

            if not self._send_joint_goal(self.current_joint_goal):
                self.get_logger().error(f"{phase_name}: failed at point {point_idx}")
                return False

        return True

    def _collect_phase_sequences(
        self,
        source_paths,
        source_swings,
        tripod_a_mode: Tuple[str, str],
        tripod_b_mode: Tuple[str, str],
    ) -> Dict[int, List[Tuple[float, float, float]]]:
        tripod_mode = {"A": tripod_a_mode, "B": tripod_b_mode}
        out: Dict[int, List[Tuple[float, float, float]]] = {}
        for leg in self.legs:
            mode_kind, path_type = tripod_mode[leg.tripod]
            store = source_paths if mode_kind == "path" else source_swings
            out[leg.leg_id] = store[leg.leg_id][path_type]
        return out

    def _execute_standard_phase(
        self,
        phase_name: str,
        trajectory_type_id: int,
        tripod_a_mode: Tuple[str, str],
        tripod_b_mode: Tuple[str, str],
    ) -> bool:
        leg_sequences = self._collect_phase_sequences(
            source_paths=self.joint_paths[trajectory_type_id],
            source_swings=self.joint_swings[trajectory_type_id],
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )
        return self._execute_joint_sequences(phase_name, leg_sequences)

    def _execute_transition_phase(self, from_id: int, to_id: int, pull_tripod: str) -> bool:
        phase_name = f"transition {from_id}->{to_id}"
        leg_sequences: Dict[int, List[Tuple[float, float, float]]] = {}
        for leg in self.legs:
            if leg.tripod == pull_tripod:
                leg_sequences[leg.leg_id] = self.joint_transition_paths[from_id][to_id][leg.leg_id]
            else:
                leg_sequences[leg.leg_id] = self.joint_transition_swings[from_id][to_id][leg.leg_id]
        return self._execute_joint_sequences(phase_name, leg_sequences)

    def _execute_half_step_start(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "Positive"),
            other: ("swing", "negativeF"),
        }
        return self._execute_standard_phase(
            phase_name=f"start half-step t{trajectory_type_id}",
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=modes["A"],
            tripod_b_mode=modes["B"],
        )

    def _execute_half_step_final(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "negativeF"),
            other: ("swing", "Positive"),
        }
        return self._execute_standard_phase(
            phase_name=f"final half-step t{trajectory_type_id}",
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=modes["A"],
            tripod_b_mode=modes["B"],
        )

    def _execute_full_step(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "Full"),
            other: ("swing", "Full"),
        }
        return self._execute_standard_phase(
            phase_name=f"full-step t{trajectory_type_id}",
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=modes["A"],
            tripod_b_mode=modes["B"],
        )

    def coordinator(self) -> bool:
        self.get_logger().info("Building gait templates")
        self._build_all_templates()
        self.get_logger().info("Converting templates to joint space")
        self.p_to_joint_space()
        self.get_logger().info("Controller ready (stationary)")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.active_trajectory_id == self.STATIONARY_ID:
                if self.requested_trajectory_id not in self.MOVING_TRAJECTORY_IDS:
                    continue
                self.active_trajectory_id = self.requested_trajectory_id
                self.get_logger().info(f"Starting trajectory type {self.active_trajectory_id}")
                if not self._execute_half_step_start(self.active_trajectory_id, pull_tripod="A"):
                    return False
                self.next_full_pull_tripod = "B"
                continue

            requested = self.requested_trajectory_id
            if requested == self.STATIONARY_ID:
                if not self._execute_half_step_final(self.active_trajectory_id, self.next_full_pull_tripod):
                    return False
                self.active_trajectory_id = self.STATIONARY_ID
                self.requested_trajectory_id = self.STATIONARY_ID
                self.get_logger().info("Entered stationary mode")
                continue

            if requested in self.MOVING_TRAJECTORY_IDS and requested != self.active_trajectory_id:
                from_id = self.active_trajectory_id
                to_id = requested
                self.get_logger().info(f"Transition {from_id} -> {to_id}")
                if not self._execute_transition_phase(from_id, to_id, self.next_full_pull_tripod):
                    return False
                self.active_trajectory_id = to_id
                self.next_full_pull_tripod = self._opposite_tripod(self.next_full_pull_tripod)
                continue

            pull_tripod = self.next_full_pull_tripod
            if not self._execute_full_step(self.active_trajectory_id, pull_tripod):
                return False
            self.next_full_pull_tripod = self._opposite_tripod(pull_tripod)

        return True

    def run(self) -> int:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if not self.action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            self.get_logger().error(
                f"Action server unavailable after {self.wait_timeout_sec:.1f}s: {self.action_name}"
            )
            return 1
        return 0 if self.coordinator() else 1


def main() -> None:
    rclpy.init(args=None)
    node = LiteGaitController()
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
