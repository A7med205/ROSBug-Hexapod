#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

sample_rate = 0.04


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
class BasePose3D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryTerm:
    dimension: str
    rate: float
    lower: float
    upper: float
    start: float = 0.0


@dataclass(frozen=True)
class TrajectorySpec:
    description: str
    terms: Tuple[TrajectoryTerm, ...] = ()
    pose_at: Callable[[float], BasePose3D] | None = None
    is_complete: Callable[[float], bool] | None = None


class ElevationPitchKeyboardController(Node):
    def __init__(self) -> None:
        super().__init__("elevation_pitch_pose_controller")

        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        self.joint_mins = [-2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi]
        self.joint_maxs = [2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi]

        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout_sec = float(self.declare_parameter("wait_timeout_sec", 10.0).value)
        pose_text = str(self.declare_parameter("pose", "0,0,0,0,0,0").value)
        self.trajectory_id = str(self.declare_parameter("trajectory_id", "").value).strip()
        self.target_base_pose = self._parse_pose_parameter(pose_text)

        self.legs: List[LegInfo] = [
            LegInfo(1, ("jl11", "jl12", "jl13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("jl21", "jl22", "jl23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("jl31", "jl32", "jl33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("jl41", "jl42", "jl43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("jl51", "jl52", "jl53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("jl61", "jl62", "jl63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]
        self.joint_names_flat = [joint for leg in self.legs for joint in leg.joint_names]
        self.neutral_tip_positions: Dict[int, Point3D] = {
            leg.leg_id: Point3D(self.home_x, self.home_y, self.home_z)
            for leg in self.legs
        }
        self.neutral_base_pose = BasePose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        circle_radius = 0.045
        circle_radial_speed = 0.02
        circle_angular_velocity = math.radians(200.0)
        twist_angular_velocity = math.radians(30.0)
        swing_radius = 0.15
        swing_arc_limit = math.radians(20.0)
        swing_angular_velocity = math.radians(30.0)
        rock_max_lean = math.radians(15.0)
        rock_lean_rate = math.radians(5.0)
        rock_axis_yaw_rate = math.radians(200.0)
        self.trajectories: Dict[str, TrajectorySpec] = {
            "slide_y": TrajectorySpec(
                description="y(t) = 0.01 * t, y in [0.0, 0.05]",
                terms=(TrajectoryTerm("y", 0.025, 0.0, 0.05),),
            ),
            "rise_z": TrajectorySpec(
                description="z(t) = -0.005 * t, z in [-0.03, 0.0]",
                terms=(TrajectoryTerm("z", -0.005, -0.03, 0.0),),
            ),
            "yaw_sweep": TrajectorySpec(
                description="yaw(t) = 8.0 * t deg, yaw in [0.0, 12.0] deg",
                terms=(TrajectoryTerm("yaw", math.radians(8.0), 0.0, math.radians(12.0)),),
            ),
            "lean_and_shift": TrajectorySpec(
                description="x(t) = 0.008 * t, pitch(t) = 5.0 * t deg",
                terms=(
                    TrajectoryTerm("x", 0.008, 0.0, 0.03),
                    TrajectoryTerm("pitch", math.radians(5.0), 0.0, math.radians(10.0)),
                ),
            ),
            "xyz_step": self._make_piecewise_translation_trajectory(
                "Translate +5 cm in y, then +5 cm in x, then +5 cm in z, and return in reverse at 5 cm/s",
                waypoints=(
                    (0.0, 0.04, 0.0),
                    (0.04, 0.04, 0.0),
                    (0.04, 0.04, 0.04),
                    (0.04, 0.04, 0.0),
                    (0.0, 0.04, 0.0),
                    (0.0, 0.0, 0.0),
                ),
                speed=0.05,
            ),
            "spiral_xy": self._make_spiral_trajectory(
                "Spiral in x/y with R=0.025 m",
                plane="xy",
                radius=circle_radius,
                radial_speed=circle_radial_speed,
                angular_velocity=circle_angular_velocity,
            ),
            "spiral_yz": self._make_spiral_trajectory(
                "Spiral in y/z with R=0.025 m",
                plane="yz",
                radius=circle_radius,
                radial_speed=circle_radial_speed,
                angular_velocity=circle_angular_velocity,
            ),
            "spiral_xz": self._make_spiral_trajectory(
                "Spiral in x/z with R=0.025 m",
                plane="xz",
                radius=circle_radius,
                radial_speed=circle_radial_speed,
                angular_velocity=circle_angular_velocity,
            ),
            "double_twist": self._make_double_twist_trajectory(
                "Yaw 0 -> +30 deg -> -30 deg -> 0 deg",
                max_yaw=math.radians(30.0),
                angular_velocity=twist_angular_velocity,
            ),
            "swing_yz_arc": self._make_swing_arc_trajectory(
                "Swing along a y/z arc with tangent-aligned orientation",
                radius=swing_radius,
                arc_limit=swing_arc_limit,
                angular_velocity=swing_angular_velocity,
            ),
            "rocking_precess": self._make_rocking_trajectory(
                "Lean to 20 deg while the lean axis rotates around z, then return upright",
                max_lean=rock_max_lean,
                lean_rate=rock_lean_rate,
                axis_yaw_rate=rock_axis_yaw_rate,
            ),
        }

        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))

    @staticmethod
    def _deg_to_rad(deg: float) -> float:
        return deg * (math.pi / 180.0)

    @staticmethod
    def _vec_add(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    @staticmethod
    def _vec_sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    @staticmethod
    def _mat_vec(
        mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
        vec: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        return (
            mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
            mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
            mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
        )

    @staticmethod
    def _mat_mul(
        a: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
        b: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        return (
            (
                a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
                a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
            ),
            (
                a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
                a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
            ),
            (
                a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
                a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
                a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
            ),
        )

    @staticmethod
    def _mat_transpose(
        mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        return (
            (mat[0][0], mat[1][0], mat[2][0]),
            (mat[0][1], mat[1][1], mat[2][1]),
            (mat[0][2], mat[1][2], mat[2][2]),
        )

    @staticmethod
    def _mat_sub_identity(
        mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        return (
            (mat[0][0] - 1.0, mat[0][1], mat[0][2]),
            (mat[1][0], mat[1][1] - 1.0, mat[1][2]),
            (mat[2][0], mat[2][1], mat[2][2] - 1.0),
        )

    @staticmethod
    def _rotation_from_rpy(
        roll: float, pitch: float, yaw: float
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    def _rotation_leg_to_body(
        self, leg: LegInfo
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        yaw = self._deg_to_rad(leg.frame_pose.theta_deg)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return (
            (cy, -sy, 0.0),
            (sy, cy, 0.0),
            (0.0, 0.0, 1.0),
        )

    def _parse_pose_parameter(self, pose_text: str) -> BasePose3D:
        values = [item.strip() for item in pose_text.split(",")]
        if len(values) != 6:
            raise ValueError(
                "pose must contain 6 comma-separated values: x,y,z,roll_deg,pitch_deg,yaw_deg"
            )

        try:
            x, y, z, roll_deg, pitch_deg, yaw_deg = (float(value) for value in values)
        except ValueError as exc:
            raise ValueError(
                "pose values must be numeric: x,y,z,roll_deg,pitch_deg,yaw_deg"
            ) from exc

        return BasePose3D(
            x=x,
            y=y,
            z=z,
            roll=self._deg_to_rad(roll_deg),
            pitch=self._deg_to_rad(pitch_deg),
            yaw=self._deg_to_rad(yaw_deg),
        )

    def _build_pose_from_terms(self, terms: Tuple[TrajectoryTerm, ...], t_sec: float) -> BasePose3D:
        values = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        for term in terms:
            raw_value = term.start + term.rate * t_sec
            values[term.dimension] = self._clamp(raw_value, term.lower, term.upper)
        return BasePose3D(**values)

    @staticmethod
    def _term_reached_boundary(term: TrajectoryTerm, t_sec: float) -> bool:
        raw_value = term.start + term.rate * t_sec
        if term.rate > 0.0:
            return raw_value >= term.upper
        if term.rate < 0.0:
            return raw_value <= term.lower
        return True

    def _trajectory_complete(self, terms: Tuple[TrajectoryTerm, ...], t_sec: float) -> bool:
        return all(self._term_reached_boundary(term, t_sec) for term in terms)

    @staticmethod
    def _rpy_from_rotation_matrix(
        mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
    ) -> Tuple[float, float, float]:
        pitch = math.asin(-max(-1.0, min(1.0, mat[2][0])))
        cp = math.cos(pitch)
        if abs(cp) > 1.0e-8:
            roll = math.atan2(mat[2][1], mat[2][2])
            yaw = math.atan2(mat[1][0], mat[0][0])
        else:
            roll = math.atan2(-mat[1][2], mat[1][1])
            yaw = 0.0
        return roll, pitch, yaw

    def _pose_from_axis_angle(self, axis_yaw: float, angle: float) -> BasePose3D:
        ux = math.cos(axis_yaw)
        uy = math.sin(axis_yaw)
        uz = 0.0
        c = math.cos(angle)
        s = math.sin(angle)
        one_c = 1.0 - c
        rot = (
            (c + ux * ux * one_c, ux * uy * one_c - uz * s, ux * uz * one_c + uy * s),
            (uy * ux * one_c + uz * s, c + uy * uy * one_c, uy * uz * one_c - ux * s),
            (uz * ux * one_c - uy * s, uz * uy * one_c + ux * s, c + uz * uz * one_c),
        )
        roll, pitch, yaw = self._rpy_from_rotation_matrix(rot)
        return BasePose3D(0.0, 0.0, 0.0, roll, pitch, yaw)

    def _make_spiral_trajectory(
        self,
        description: str,
        plane: str,
        radius: float,
        radial_speed: float,
        angular_velocity: float,
    ) -> TrajectorySpec:
        out_time = radius / radial_speed
        total_time = 2.0 * out_time

        def pose_at(t_sec: float) -> BasePose3D:
            phase = min(max(t_sec, 0.0), total_time)
            if phase <= out_time:
                radial = radial_speed * phase
            else:
                radial = radius - radial_speed * (phase - out_time)
            theta = angular_velocity * phase
            c = radial * math.cos(theta)
            s = radial * math.sin(theta)
            if plane == "xy":
                return BasePose3D(c, s, 0.0, 0.0, 0.0, 0.0)
            if plane == "yz":
                return BasePose3D(0.0, c, s, 0.0, 0.0, 0.0)
            return BasePose3D(c, 0.0, s, 0.0, 0.0, 0.0)

        return TrajectorySpec(
            description=description,
            pose_at=pose_at,
            is_complete=lambda t_sec: t_sec >= total_time,
        )

    @staticmethod
    def _piecewise_ramp(
        t_sec: float, start: float, end: float, rate: float
    ) -> Tuple[float, bool]:
        distance = abs(end - start)
        duration = distance / rate if rate > 0.0 else 0.0
        if duration == 0.0:
            return end, True
        phase = min(t_sec, duration)
        direction = 1.0 if end >= start else -1.0
        value = start + direction * rate * phase
        return value, t_sec >= duration

    @staticmethod
    def _smooth_segment(
        t_sec: float, start: float, end: float, duration: float
    ) -> Tuple[float, bool]:
        if duration <= 0.0:
            return end, True
        phase = max(0.0, min(t_sec, duration))
        u = phase / duration
        blend = 0.5 - 0.5 * math.cos(math.pi * u)
        value = start + (end - start) * blend
        return value, t_sec >= duration

    def _make_double_twist_trajectory(
        self, description: str, max_yaw: float, angular_velocity: float
    ) -> TrajectorySpec:
        segment_1 = abs(max_yaw) / angular_velocity
        segment_2 = (2.0 * abs(max_yaw)) / angular_velocity
        segment_3 = abs(max_yaw) / angular_velocity
        total_time = segment_1 + segment_2 + segment_3

        def pose_at(t_sec: float) -> BasePose3D:
            phase = min(max(t_sec, 0.0), total_time)
            if phase <= segment_1:
                yaw, _ = self._smooth_segment(phase, 0.0, max_yaw, segment_1)
            elif phase <= segment_1 + segment_2:
                yaw, _ = self._smooth_segment(phase - segment_1, max_yaw, -max_yaw, segment_2)
            else:
                yaw, _ = self._smooth_segment(phase - segment_1 - segment_2, -max_yaw, 0.0, segment_3)
            return BasePose3D(0.0, 0.0, 0.0, 0.0, 0.0, yaw)

        return TrajectorySpec(
            description=description,
            pose_at=pose_at,
            is_complete=lambda t_sec: t_sec >= total_time,
        )

    def _make_swing_arc_trajectory(
        self, description: str, radius: float, arc_limit: float, angular_velocity: float
    ) -> TrajectorySpec:
        segment_1 = arc_limit / angular_velocity
        segment_2 = (2.0 * arc_limit) / angular_velocity
        segment_3 = arc_limit / angular_velocity
        total_time = segment_1 + segment_2 + segment_3

        def pose_at(t_sec: float) -> BasePose3D:
            phase = min(max(t_sec, 0.0), total_time)
            if phase <= segment_1:
                arc_angle, _ = self._smooth_segment(phase, 0.0, -arc_limit, segment_1)
            elif phase <= segment_1 + segment_2:
                arc_angle, _ = self._smooth_segment(phase - segment_1, -arc_limit, arc_limit, segment_2)
            else:
                arc_angle, _ = self._smooth_segment(phase - segment_1 - segment_2, arc_limit, 0.0, segment_3)
            y = radius * math.sin(arc_angle)
            z = radius * (1.0 - math.cos(arc_angle))
            return BasePose3D(0.0, y, z, arc_angle, 0.0, 0.0)

        return TrajectorySpec(
            description=description,
            pose_at=pose_at,
            is_complete=lambda t_sec: t_sec >= total_time,
        )

    def _make_rocking_trajectory(
        self, description: str, max_lean: float, lean_rate: float, axis_yaw_rate: float
    ) -> TrajectorySpec:
        up_time = max_lean / lean_rate
        total_time = 2.0 * up_time

        def pose_at(t_sec: float) -> BasePose3D:
            phase = min(max(t_sec, 0.0), total_time)
            if phase <= up_time:
                lean = lean_rate * phase
            else:
                lean = max_lean - lean_rate * (phase - up_time)
            axis_yaw = axis_yaw_rate * phase
            return self._pose_from_axis_angle(axis_yaw, lean)

        return TrajectorySpec(
            description=description,
            pose_at=pose_at,
            is_complete=lambda t_sec: t_sec >= total_time,
        )

    def _make_piecewise_translation_trajectory(
        self,
        description: str,
        waypoints: Tuple[Tuple[float, float, float], ...],
        speed: float,
    ) -> TrajectorySpec:
        if speed <= 0.0:
            raise ValueError("piecewise translation speed must be positive")

        segment_starts = ((0.0, 0.0, 0.0),) + waypoints[:-1]
        segment_durations = []
        total_time = 0.0
        for start, end in zip(segment_starts, waypoints):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            duration = distance / speed
            segment_durations.append(duration)
            total_time += duration

        def pose_at(t_sec: float) -> BasePose3D:
            phase = min(max(t_sec, 0.0), total_time)
            elapsed = 0.0
            for start, end, duration in zip(segment_starts, waypoints, segment_durations):
                if phase <= elapsed + duration:
                    if duration == 0.0:
                        return BasePose3D(end[0], end[1], end[2], 0.0, 0.0, 0.0)
                    blend = (phase - elapsed) / duration
                    x = start[0] + (end[0] - start[0]) * blend
                    y = start[1] + (end[1] - start[1]) * blend
                    z = start[2] + (end[2] - start[2]) * blend
                    return BasePose3D(x, y, z, 0.0, 0.0, 0.0)
                elapsed += duration
            last = waypoints[-1]
            return BasePose3D(last[0], last[1], last[2], 0.0, 0.0, 0.0)

        return TrajectorySpec(
            description=description,
            pose_at=pose_at,
            is_complete=lambda t_sec: t_sec >= total_time,
        )

    def _trajectory_pose_at(self, spec: TrajectorySpec, t_sec: float) -> BasePose3D:
        if spec.pose_at is not None:
            return spec.pose_at(t_sec)
        return self._build_pose_from_terms(spec.terms, t_sec)

    def _trajectory_is_complete(self, spec: TrajectorySpec, t_sec: float) -> bool:
        if spec.is_complete is not None:
            return spec.is_complete(t_sec)
        return self._trajectory_complete(spec.terms, t_sec)

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

    def base_delta_to_tip_delta_3d(
        self,
        base1: BasePose3D,
        base2: BasePose3D,
        leg: LegInfo,
        tip_local_1: Point3D,
    ) -> Point3D:
        r_world_body1 = self._rotation_from_rpy(base1.roll, base1.pitch, base1.yaw)
        r_world_body2 = self._rotation_from_rpy(base2.roll, base2.pitch, base2.yaw)
        r_body1_world = self._mat_transpose(r_world_body1)

        delta_world = (base2.x - base1.x, base2.y - base1.y, base2.z - base1.z)
        delta_body = self._mat_vec(r_body1_world, delta_world)
        r_body = self._mat_mul(r_body1_world, r_world_body2)

        r_bl = self._rotation_leg_to_body(leg)
        r_lb = self._mat_transpose(r_bl)
        mount_body = (leg.frame_pose.x, leg.frame_pose.y, 0.0)

        delta_leg = self._mat_vec(
            r_lb,
            self._vec_add(delta_body, self._mat_vec(self._mat_sub_identity(r_body), mount_body)),
        )
        r_leg = self._mat_mul(self._mat_mul(r_lb, r_body), r_bl)

        tip1 = (tip_local_1.x, tip_local_1.y, tip_local_1.z)
        tip2 = self._mat_vec(self._mat_transpose(r_leg), self._vec_sub(tip1, delta_leg))
        return Point3D(tip2[0] - tip1[0], tip2[1] - tip1[1], tip2[2] - tip1[2])

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

    def _plan_pose(
        self, base_pose: BasePose3D
    ) -> Tuple[List[float], Dict[int, Point3D]] | None:
        candidate_joints: Dict[int, Tuple[float, float, float]] = {}
        tip_positions: Dict[int, Point3D] = {}
        for leg in self.legs:
            tip_prev = self.neutral_tip_positions[leg.leg_id]
            delta = self.base_delta_to_tip_delta_3d(
                self.neutral_base_pose,
                base_pose,
                leg,
                tip_prev,
            )
            tip_next = Point3D(
                tip_prev.x + delta.x,
                tip_prev.y + delta.y,
                tip_prev.z + delta.z,
            )
            joints = self.IK(tip_next)
            if not self._within_joint_limits(joints):
                self.get_logger().error(
                    f"Joint limits exceeded for leg {leg.leg_id} at requested pose"
                )
                return None
            candidate_joints[leg.leg_id] = joints
            tip_positions[leg.leg_id] = tip_next
        return self._build_joint_vector(candidate_joints), tip_positions

    def _format_pose_and_tips(
        self, t_sec: float, base_pose: BasePose3D, tip_positions: Dict[int, Point3D]
    ) -> str:
        leg_parts: List[str] = []
        for leg in self.legs:
            tip = tip_positions[leg.leg_id]
            leg_parts.append(f"L{leg.leg_id}: ({tip.x:.4f}, {tip.y:.4f}, {tip.z:.4f})")

        return (
            f"t={t_sec:.2f}s | "
            f"base=({base_pose.x:.4f}, {base_pose.y:.4f}, {base_pose.z:.4f}, "
            f"{math.degrees(base_pose.roll):.3f}deg, {math.degrees(base_pose.pitch):.3f}deg, "
            f"{math.degrees(base_pose.yaw):.3f}deg)\n"
            f"  {leg_parts[0]}, {leg_parts[1]}\n"
            f"  {leg_parts[2]}, {leg_parts[3]}\n"
            f"  {leg_parts[4]}, {leg_parts[5]}"
        )

    def _send_joint_trajectory(self, joint_trajectory: List[List[float]]) -> bool:
        if not joint_trajectory:
            self.get_logger().error("No trajectory points to send")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names_flat

        goal_points: List[JointTrajectoryPoint] = []
        for idx, joint_values_flat in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = list(joint_values_flat)
            point.time_from_start = Duration(seconds=(idx + 1) * sample_rate).to_msg()
            goal_points.append(point)
        goal.trajectory.points = goal_points

        action_start = time.perf_counter()
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected or failed to send")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_end = time.perf_counter()
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
        self.get_logger().info(
            f"Goal execution time: {(action_end - action_start):.3f} s"
        )
        return True

    def _wait_for_action_server(self) -> bool:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if self.action_client.wait_for_server(timeout_sec=self.wait_timeout_sec):
            return True
        self.get_logger().error(
            f"Action server unavailable after {self.wait_timeout_sec:.1f}s"
        )
        return False

    def _execute_pose(self, base_pose: BasePose3D) -> int:
        plan = self._plan_pose(base_pose)
        if plan is None:
            return 2
        joint_goal, tip_positions = plan

        self.get_logger().info(
            "Sending snap goal for pose "
            f"x={base_pose.x:.3f}, y={base_pose.y:.3f}, z={base_pose.z:.3f}, "
            f"roll={math.degrees(base_pose.roll):.3f} deg, "
            f"pitch={math.degrees(base_pose.pitch):.3f} deg, "
            f"yaw={math.degrees(base_pose.yaw):.3f} deg"
        )
        self.get_logger().info(self._format_pose_and_tips(0.0, base_pose, tip_positions))
        return 0 if self._send_joint_trajectory([joint_goal]) else 3

    def execute_trajectory(self, trajectory_id: str) -> int:
        if trajectory_id not in self.trajectories:
            available = ", ".join(sorted(self.trajectories))
            self.get_logger().error(
                f"Unknown trajectory_id '{trajectory_id}'. Available trajectories: {available}"
            )
            return 2

        spec = self.trajectories[trajectory_id]
        self.get_logger().info(
            f"Executing trajectory '{trajectory_id}' at sample_rate={sample_rate:.3f}s: {spec.description}"
        )

        joint_trajectory: List[List[float]] = []
        t_sec = 0.0
        while True:
            base_pose = self._trajectory_pose_at(spec, t_sec)
            plan = self._plan_pose(base_pose)
            if plan is None:
                return 2
            joint_goal, tip_positions = plan
            self.get_logger().info(self._format_pose_and_tips(t_sec, base_pose, tip_positions))
            if t_sec > 0.0:
                joint_trajectory.append(joint_goal)
            if self._trajectory_is_complete(spec, t_sec):
                break
            t_sec += sample_rate

        self.get_logger().info(
            f"Sending trajectory with {len(joint_trajectory)} sampled points"
        )
        return 0 if self._send_joint_trajectory(joint_trajectory) else 3

    def run(self) -> int:
        if not self._wait_for_action_server():
            return 1
        if self.trajectory_id:
            return self.execute_trajectory(self.trajectory_id)
        return self._execute_pose(self.target_base_pose)


def main() -> None:
    # Usage: ros2 run hexapod_sim pose_manipulator --ros-args -p pose:=0,0,0,0,10,0
    # pose values: x, y, z, roll_deg, pitch_deg, yaw_deg
    # trajectory example: ros2 run hexapod_sim pose_manipulator --ros-args -p trajectory_id:=slide_y
    rclpy.init()
    exit_code = 0
    node = None
    try:
        node = ElevationPitchKeyboardController()
        exit_code = node.run()
    except ValueError as exc:
        print(f"Invalid pose parameter: {exc}")
        exit_code = 2
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
