#!/usr/bin/env python3

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
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


class _OfflineLogger:
    def info(self, msg: str) -> None:
        print(f"[INFO] {msg}")

    def warn(self, msg: str) -> None:
        print(f"[WARN] {msg}")

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}")


class LiteGaitController(Node):
    PATH_TYPES = ("negativeF", "Positive", "Full")
    MOVING_TRAJECTORY_IDS = (1, 2, 3, 4, 5)
    STATIONARY_ID = 0

    def _init_state(self) -> None:
        # Requested hard-coded variables.
        self.limit_radius = 0.05
        self.swing_height = 0.025
        self.sample_rate = 0.02
        self.min_angle = 1.0
        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        # Local neutral tip target.
        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        # Master path constants.
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
        self.joint_names_flat: List[str] = [joint for leg in self.legs for joint in leg.joint_names]

        self.tip_paths: Dict[int, Dict[int, Dict[str, List[Point3D]]]] = {
            traj_id: self._empty_tip_store() for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.tip_swings: Dict[int, Dict[int, Dict[str, List[Point3D]]]] = {
            traj_id: self._empty_tip_store() for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.tip_transition_paths: Dict[int, Dict[int, Dict[int, List[Point3D]]]] = {
            from_id: {
                to_id: {leg.leg_id: [] for leg in self.legs}
                for to_id in self.MOVING_TRAJECTORY_IDS
                if to_id != from_id
            }
            for from_id in self.MOVING_TRAJECTORY_IDS
        }
        self.tip_transition_swings: Dict[int, Dict[int, Dict[int, List[Point3D]]]] = {
            from_id: {
                to_id: {leg.leg_id: [] for leg in self.legs}
                for to_id in self.MOVING_TRAJECTORY_IDS
                if to_id != from_id
            }
            for from_id in self.MOVING_TRAJECTORY_IDS
        }

        self.joint_paths: Dict[int, Dict[int, Dict[str, List[Tuple[float, float, float]]]]] = {
            traj_id: self._empty_joint_store() for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.joint_swings: Dict[int, Dict[int, Dict[str, List[Tuple[float, float, float]]]]] = {
            traj_id: self._empty_joint_store() for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.joint_transition_paths: Dict[int, Dict[int, Dict[int, List[Tuple[float, float, float]]]]] = {
            from_id: {
                to_id: {leg.leg_id: [] for leg in self.legs}
                for to_id in self.MOVING_TRAJECTORY_IDS
                if to_id != from_id
            }
            for from_id in self.MOVING_TRAJECTORY_IDS
        }
        self.joint_transition_swings: Dict[int, Dict[int, Dict[int, List[Tuple[float, float, float]]]]] = {
            from_id: {
                to_id: {leg.leg_id: [] for leg in self.legs}
                for to_id in self.MOVING_TRAJECTORY_IDS
                if to_id != from_id
            }
            for from_id in self.MOVING_TRAJECTORY_IDS
        }

        self.current_joint_goal = self._initial_joint_goal()
        self.requested_trajectory_id = self.STATIONARY_ID
        self.active_trajectory_id = self.STATIONARY_ID
        self.next_full_pull_tripod = "B"

    def __init__(self) -> None:
        self._offline_logger = None
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
        self.trajectory_sub = self.create_subscription(
            Int32,
            self.trajectory_topic,
            self._trajectory_callback,
            10,
        )

    @classmethod
    def create_compute_only(cls) -> "LiteGaitController":
        controller = cls.__new__(cls)
        controller._offline_logger = _OfflineLogger()
        controller.action_name = "/joint_trajectory_controller/follow_joint_trajectory"
        controller.wait_timeout_sec = 10.0
        controller.trajectory_topic = "/trajectory_type"
        controller._init_state()
        return controller

    def get_logger(self):  # type: ignore[override]
        if self._offline_logger is not None:
            return self._offline_logger
        return super().get_logger()

    def _empty_tip_store(self) -> Dict[int, Dict[str, List[Point3D]]]:
        return {leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES} for leg in self.legs}

    def _empty_joint_store(self) -> Dict[int, Dict[str, List[Tuple[float, float, float]]]]:
        return {leg.leg_id: {path_type: [] for path_type in self.PATH_TYPES} for leg in self.legs}

    def _initial_joint_goal(self) -> List[float]:
        neutral_tip = Point3D(self.home_x, self.home_y, self.home_z)
        values: List[float] = []
        for _ in self.legs:
            j1, j2, j3 = self.IK(neutral_tip)
            values.extend((j1, j2, j3))
        return values

    def _trajectory_callback(self, msg: Int32) -> None:
        value = int(msg.data)
        if value == self.STATIONARY_ID or value in self.MOVING_TRAJECTORY_IDS:
            self.requested_trajectory_id = value
            self.get_logger().info(f"Requested trajectory id: {value}")
        else:
            self.get_logger().warn(f"Ignoring unsupported trajectory id: {value}")

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
            theta = direction * self.angular_speed * t
            return BasePose2D(x, y, theta)
        return BasePose2D(0.0, 0.0, 0.0)

    def pull_builder(self, tripod: str, trajectory_type_id: int, sign: int) -> Dict[int, List[Point3D]]:
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")

        selected_legs = self._tripod_legs(tripod)
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
                tip_next = Point3D(tip_prev.x + delta.dx_local, tip_prev.y + delta.dy_local, home_tip.z)
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
                f"pull_builder tripod={tripod} traj={trajectory_type_id} sign={sign:+d} did not hit limit_radius"
            )

        return path_points

    def swing_builder(self, path_end: Point3D, path_start: Point3D, point_count: int) -> List[Point3D]:
        point_count = max(point_count, 2)
        points: List[Point3D] = []
        for idx in range(point_count):
            s = float(idx) / float(point_count - 1)
            x = path_end.x + s * (path_start.x - path_end.x)
            y = path_end.y + s * (path_start.y - path_end.y)
            z = (1.0 - s) * path_end.z + s * path_start.z + self.swing_height * math.sin(math.pi * s)
            points.append(Point3D(x, y, z))
        return points

    def _linear_path(self, start: Point3D, end: Point3D, point_count: int) -> List[Point3D]:
        point_count = max(point_count, 2)
        path: List[Point3D] = []
        for idx in range(point_count):
            s = float(idx) / float(point_count - 1)
            path.append(
                Point3D(
                    start.x + s * (end.x - start.x),
                    start.y + s * (end.y - start.y),
                    start.z + s * (end.z - start.z),
                )
            )
        return path

    def _build_tripod_templates(self, trajectory_type_id: int, tripod: str) -> None:
        positive = self.pull_builder(tripod, trajectory_type_id, +1)
        negative = self.pull_builder(tripod, trajectory_type_id, -1)

        for leg in self._tripod_legs(tripod):
            leg_id = leg.leg_id
            positive_path = positive.get(leg_id, [])
            negative_path = negative.get(leg_id, [])
            if not positive_path:
                positive_path = [Point3D(self.home_x, self.home_y, self.home_z)]
            if not negative_path:
                negative_path = [Point3D(self.home_x, self.home_y, self.home_z)]

            negative_f = list(reversed(negative_path))
            full_path = negative_f + positive_path[1:]
            self.tip_paths[trajectory_type_id][leg_id]["negativeF"] = negative_f
            self.tip_paths[trajectory_type_id][leg_id]["Positive"] = positive_path
            self.tip_paths[trajectory_type_id][leg_id]["Full"] = full_path

            negf_count = len(negative_f)
            pos_count = len(positive_path)
            full_count = len(full_path)

            self.tip_swings[trajectory_type_id][leg_id]["negativeF"] = self.swing_builder(
                path_end=negative_f[-1],
                path_start=negative_f[0],
                point_count=pos_count,
            )
            self.tip_swings[trajectory_type_id][leg_id]["Positive"] = self.swing_builder(
                path_end=positive_path[-1],
                path_start=positive_path[0],
                point_count=negf_count,
            )
            self.tip_swings[trajectory_type_id][leg_id]["Full"] = self.swing_builder(
                path_end=full_path[-1],
                path_start=full_path[0],
                point_count=full_count,
            )

    def _build_transition_templates(self) -> None:
        for from_id in self.MOVING_TRAJECTORY_IDS:
            for to_id in self.MOVING_TRAJECTORY_IDS:
                if from_id == to_id:
                    continue
                for leg in self.legs:
                    leg_id = leg.leg_id
                    from_negative_f = self.tip_paths[from_id][leg_id]["negativeF"]
                    to_positive = self.tip_paths[to_id][leg_id]["Positive"]
                    from_full = self.tip_paths[from_id][leg_id]["Full"]
                    to_full = self.tip_paths[to_id][leg_id]["Full"]
                    if not from_negative_f:
                        from_negative_f = [Point3D(self.home_x, self.home_y, self.home_z)]
                    if not to_positive:
                        to_positive = [Point3D(self.home_x, self.home_y, self.home_z)]
                    if not from_full:
                        from_full = [Point3D(self.home_x, self.home_y, self.home_z)]
                    if not to_full:
                        to_full = [Point3D(self.home_x, self.home_y, self.home_z)]

                    # Transition path is direct concatenation: negativeF(from) + Positive(to).
                    transition_path = [Point3D(p.x, p.y, p.z) for p in from_negative_f]
                    transition_path.extend(Point3D(p.x, p.y, p.z) for p in to_positive)
                    transition_points = len(transition_path)
                    transition_swing = self.swing_builder(
                        # Transition swing follows end(full(from)) -> start(full(to)).
                        path_end=from_full[-1],
                        path_start=to_full[0],
                        point_count=transition_points,
                    )

                    self.tip_transition_paths[from_id][to_id][leg_id] = transition_path
                    self.tip_transition_swings[from_id][to_id][leg_id] = transition_swing

    def _build_all_templates(self) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            self._build_tripod_templates(trajectory_type_id, "A")
            self._build_tripod_templates(trajectory_type_id, "B")
        self._build_transition_templates()

    @staticmethod
    def _decimate_points(points: List[Point3D], stride: int) -> List[Point3D]:
        if stride <= 1 or len(points) <= 2:
            return points
        sampled = points[::stride]
        if sampled[-1] is not points[-1]:
            sampled.append(points[-1])
        return sampled

    @staticmethod
    def _points_to_json(points: List[Point3D]) -> List[Dict[str, float]]:
        return [{"x": p.x, "y": p.y, "z": p.z} for p in points]

    @staticmethod
    def _short_points(points: List[Point3D]) -> str:
        return ", ".join(f"({p.x:.3f},{p.y:.3f})" for p in points)

    def _log_decimated_paths(self, decimation: int) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for path_type in self.PATH_TYPES:
                    points = self.tip_paths[trajectory_type_id][leg_id][path_type]
                    sampled = self._decimate_points(points, decimation)
                    self.get_logger().info(
                        f"path t{trajectory_type_id} leg{leg_id} {path_type}: "
                        f"points={len(points)} sampled={len(sampled)} {self._short_points(sampled)}"
                    )

        for from_id in self.MOVING_TRAJECTORY_IDS:
            for to_id in self.MOVING_TRAJECTORY_IDS:
                if from_id == to_id:
                    continue
                for leg in self.legs:
                    leg_id = leg.leg_id
                    points = self.tip_transition_paths[from_id][to_id][leg_id]
                    sampled = self._decimate_points(points, decimation)
                    self.get_logger().info(
                        f"transition {from_id}->{to_id} leg{leg_id}: "
                        f"points={len(points)} sampled={len(sampled)} {self._short_points(sampled)}"
                    )

    def _dump_templates_json(self, output_path: str) -> None:
        payload: Dict[str, object] = {
            "meta": {
                "sample_rate": self.sample_rate,
                "limit_radius": self.limit_radius,
                "swing_height": self.swing_height,
                "types": list(self.MOVING_TRAJECTORY_IDS),
            },
            "legs": {
                str(leg.leg_id): {
                    "tripod": leg.tripod,
                    "joint_names": list(leg.joint_names),
                    "frame_pose": {
                        "x": leg.frame_pose.x,
                        "y": leg.frame_pose.y,
                        "theta_deg": leg.frame_pose.theta_deg,
                    },
                }
                for leg in self.legs
            },
            "types": {},
            "transitions": {},
        }

        types_data: Dict[str, object] = {}
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            paths_by_leg: Dict[str, object] = {}
            swings_by_leg: Dict[str, object] = {}
            for leg in self.legs:
                leg_id = str(leg.leg_id)
                paths_by_leg[leg_id] = {
                    path_type: self._points_to_json(self.tip_paths[trajectory_type_id][leg.leg_id][path_type])
                    for path_type in self.PATH_TYPES
                }
                swings_by_leg[leg_id] = {
                    path_type: self._points_to_json(self.tip_swings[trajectory_type_id][leg.leg_id][path_type])
                    for path_type in self.PATH_TYPES
                }

            types_data[str(trajectory_type_id)] = {
                "paths": paths_by_leg,
                "swings": swings_by_leg,
            }
        payload["types"] = types_data

        transitions_data: Dict[str, object] = {}
        for from_id in self.MOVING_TRAJECTORY_IDS:
            for to_id in self.MOVING_TRAJECTORY_IDS:
                if from_id == to_id:
                    continue
                key = f"{from_id}->{to_id}"
                transitions_data[key] = {
                    "paths": {
                        str(leg.leg_id): self._points_to_json(
                            self.tip_transition_paths[from_id][to_id][leg.leg_id]
                        )
                        for leg in self.legs
                    },
                    "swings": {
                        str(leg.leg_id): self._points_to_json(
                            self.tip_transition_swings[from_id][to_id][leg.leg_id]
                        )
                        for leg in self.legs
                    },
                }
        payload["transitions"] = transitions_data

        out_path = Path(output_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.get_logger().info(f"Wrote template dump: {out_path.resolve()}")

    def compute_templates_only(self, output_path: str, decimation: int) -> int:
        if decimation < 1:
            self.get_logger().error("log decimation must be >= 1")
            return 1
        self.get_logger().info("Compute-only mode: building path templates")
        self._build_all_templates()
        self._log_decimated_paths(decimation)
        self._dump_templates_json(output_path)
        self.get_logger().info("Compute-only mode complete")
        return 0

    def p_to_joint_space(self) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for path_type in self.PATH_TYPES:
                    self.joint_paths[trajectory_type_id][leg_id][path_type] = [
                        self.IK(point) for point in self.tip_paths[trajectory_type_id][leg_id][path_type]
                    ]
                    self.joint_swings[trajectory_type_id][leg_id][path_type] = [
                        self.IK(point) for point in self.tip_swings[trajectory_type_id][leg_id][path_type]
                    ]

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

    def _execute_joint_sequences(
        self,
        phase_name: str,
        leg_sequences: Dict[int, List[Tuple[float, float, float]]],
    ) -> bool:
        if not leg_sequences:
            self.get_logger().error(f"{phase_name}: no sequences to execute")
            return False

        phase_points = max(len(seq) for seq in leg_sequences.values())
        min_angle_rad = math.radians(self.min_angle)
        self.get_logger().info(f"Executing {phase_name} with {phase_points} points")

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
                self.get_logger().error(f"{phase_name}: failed at point index {point_idx}")
                return False

        return True

    def _execute_standard_phase(
        self,
        phase_name: str,
        trajectory_type_id: int,
        tripod_a_mode: Tuple[str, str],
        tripod_b_mode: Tuple[str, str],
    ) -> bool:
        tripod_mode = {"A": tripod_a_mode, "B": tripod_b_mode}
        leg_sequences: Dict[int, List[Tuple[float, float, float]]] = {}
        for leg in self.legs:
            mode_kind, path_type = tripod_mode[leg.tripod]
            if mode_kind == "path":
                seq = self.joint_paths[trajectory_type_id][leg.leg_id][path_type]
            else:
                seq = self.joint_swings[trajectory_type_id][leg.leg_id][path_type]
            if not seq:
                self.get_logger().error(
                    f"{phase_name}: empty sequence for leg={leg.leg_id}, mode={mode_kind}, type={path_type}"
                )
                return False
            leg_sequences[leg.leg_id] = seq
        return executor(self, phase_name, leg_sequences)

    def _execute_transition_phase(self, from_id: int, to_id: int, pull_tripod: str) -> bool:
        phase_name = f"transition {from_id}->{to_id} (pull tripod {pull_tripod})"
        leg_sequences: Dict[int, List[Tuple[float, float, float]]] = {}
        for leg in self.legs:
            if leg.tripod == pull_tripod:
                seq = self.joint_transition_paths[from_id][to_id][leg.leg_id]
            else:
                seq = self.joint_transition_swings[from_id][to_id][leg.leg_id]
            if not seq:
                self.get_logger().error(f"{phase_name}: empty sequence for leg={leg.leg_id}")
                return False
            leg_sequences[leg.leg_id] = seq
        return executor(self, phase_name, leg_sequences)

    def _execute_half_step_start(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "Positive"),
            other: ("swing", "negativeF"),
        }
        return self._execute_standard_phase(
            f"start half-step t{trajectory_type_id} pull {pull_tripod}",
            trajectory_type_id,
            modes["A"],
            modes["B"],
        )

    def _execute_half_step_final(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "negativeF"),
            other: ("swing", "Positive"),
        }
        return self._execute_standard_phase(
            f"final half-step t{trajectory_type_id} pull {pull_tripod}",
            trajectory_type_id,
            modes["A"],
            modes["B"],
        )

    def _execute_full_step(self, trajectory_type_id: int, pull_tripod: str) -> bool:
        other = self._opposite_tripod(pull_tripod)
        modes = {
            pull_tripod: ("path", "Full"),
            other: ("swing", "Full"),
        }
        return self._execute_standard_phase(
            f"full-step t{trajectory_type_id} pull {pull_tripod}",
            trajectory_type_id,
            modes["A"],
            modes["B"],
        )

    def coordinator(self) -> bool:
        self.get_logger().info("Building gait templates")
        self._build_all_templates()
        self.get_logger().info("Converting templates to joint space")
        self.p_to_joint_space()
        self.get_logger().info("Controller ready. Stationary until trajectory command arrives.")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.active_trajectory_id == self.STATIONARY_ID:
                if self.requested_trajectory_id not in self.MOVING_TRAJECTORY_IDS:
                    continue

                self.active_trajectory_id = self.requested_trajectory_id
                if not self._execute_half_step_start(self.active_trajectory_id, pull_tripod="A"):
                    return False
                self.next_full_pull_tripod = "B"
                continue

            # Check for changes only before full steps.
            requested = self.requested_trajectory_id
            if requested == self.STATIONARY_ID:
                final_pull_tripod = self.next_full_pull_tripod
                if not self._execute_half_step_final(self.active_trajectory_id, final_pull_tripod):
                    return False
                self.active_trajectory_id = self.STATIONARY_ID
                self.requested_trajectory_id = self.STATIONARY_ID
                self.get_logger().info("Entered stationary mode")
                continue

            if requested in self.MOVING_TRAJECTORY_IDS and requested != self.active_trajectory_id:
                from_id = self.active_trajectory_id
                to_id = requested
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
        if not self.coordinator():
            return 1
        return 0


def executor(
    controller: LiteGaitController,
    phase_name: str,
    leg_sequences: Dict[int, List[Tuple[float, float, float]]],
) -> bool:
    return controller._execute_joint_sequences(phase_name, leg_sequences)


def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lite gait controller")
    parser.add_argument(
        "--compute-only",
        action="store_true",
        help="Build templates, log decimated paths, dump JSON, and exit.",
    )
    parser.add_argument(
        "--dump-json",
        default="visualizer/generated/lite_gait_templates.json",
        help="Output JSON path for computed templates in compute-only mode.",
    )
    parser.add_argument(
        "--log-decimation",
        type=int,
        default=10,
        help="Sampling stride used for decimated path logging.",
    )
    return parser.parse_args()


def main() -> None:
    cli = parse_cli_args()
    if cli.compute_only:
        node = LiteGaitController.create_compute_only()
        exit_code = 0
        try:
            exit_code = node.compute_templates_only(
                output_path=cli.dump_json,
                decimation=cli.log_decimation,
            )
        except KeyboardInterrupt:
            node.get_logger().info("Interrupted by user")
            exit_code = 130
        raise SystemExit(exit_code)

    rclpy.init(args=None)
    node = LiteGaitController()
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
