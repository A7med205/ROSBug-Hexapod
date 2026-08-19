"""Shared hexapod geometry, fixed-foot transforms, and inverse kinematics."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class RobotConfig:
    l1: float = 0.0385
    l2: float = 0.0700
    l3: float = 0.1020

    home_x: float = 0.110
    home_y: float = 0.000
    home_z: float = -0.050


@dataclass(frozen=True)
class FramePose:
    x: float
    y: float
    theta_deg: float


@dataclass(frozen=True)
class LegInfo:
    leg_id: int
    frame_pose: FramePose
    tripod: str


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class BasePose3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


JointAngles = Tuple[float, float, float]
Matrix3 = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]
Vector3 = Tuple[float, float, float]


class UnreachableTipError(ValueError):
    pass


class HexapodModel:
    """Canonical robot model shared by gait and posture generation."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.legs: Tuple[LegInfo, ...] = (
            LegInfo(1, FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, FramePose(0.0535, -0.0900, -45.0), "B"),
        )

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def opposite_tripod(tripod: str) -> str:
        return "B" if tripod == "A" else "A"

    def tripod_legs(self, tripod: str) -> List[LegInfo]:
        return [leg for leg in self.legs if leg.tripod == tripod]

    def tip_reach_distance(self, tip: Point3D) -> float:
        radial = math.hypot(tip.x, tip.y) - self.config.l1
        return math.hypot(radial, tip.z)

    def tip_is_reachable(self, tip: Point3D, tolerance: float = 1.0e-12) -> bool:
        distance = self.tip_reach_distance(tip)
        minimum = abs(self.config.l2 - self.config.l3)
        maximum = self.config.l2 + self.config.l3
        return minimum - tolerance <= distance <= maximum + tolerance

    def inverse_kinematics(
        self,
        tip: Point3D,
        *,
        clamp_reach: bool = True,
    ) -> JointAngles:
        cfg = self.config
        j1 = -math.atan2(tip.y, tip.x)
        x_prime = math.hypot(tip.x, tip.y) - cfg.l1
        distance = math.hypot(x_prime, tip.z)
        minimum = abs(cfg.l2 - cfg.l3)
        maximum = cfg.l2 + cfg.l3
        if not self.tip_is_reachable(tip):
            if not clamp_reach:
                raise UnreachableTipError(
                    f"tip reach {distance:.9f} is outside [{minimum:.9f}, {maximum:.9f}]"
                )
            distance = self._clamp(distance, minimum, maximum)

        alpha1 = math.atan2(-tip.z, x_prime)
        cos_alpha2 = self._clamp(
            (cfg.l2 * cfg.l2 + distance * distance - cfg.l3 * cfg.l3)
            / (2.0 * cfg.l2 * distance),
            -1.0,
            1.0,
        )
        alpha2 = math.acos(cos_alpha2)
        cos_knee = self._clamp(
            (cfg.l2 * cfg.l2 + cfg.l3 * cfg.l3 - distance * distance)
            / (2.0 * cfg.l2 * cfg.l3),
            -1.0,
            1.0,
        )
        return j1, alpha1 - alpha2, math.pi - math.acos(cos_knee)

    def neutral_tip_positions(self) -> Dict[int, Point3D]:
        cfg = self.config
        return {
            leg.leg_id: Point3D(cfg.home_x, cfg.home_y, cfg.home_z)
            for leg in self.legs
        }

    def neutral_joint_goal(self) -> List[float]:
        cfg = self.config
        angles = self.inverse_kinematics(Point3D(cfg.home_x, cfg.home_y, cfg.home_z))
        return list(angles) * len(self.legs)

    @staticmethod
    def _vec_add(a: Vector3, b: Vector3) -> Vector3:
        return a[0] + b[0], a[1] + b[1], a[2] + b[2]

    @staticmethod
    def _vec_sub(a: Vector3, b: Vector3) -> Vector3:
        return a[0] - b[0], a[1] - b[1], a[2] - b[2]

    @staticmethod
    def _mat_vec(matrix: Matrix3, vector: Vector3) -> Vector3:
        return tuple(
            sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _mat_mul(a: Matrix3, b: Matrix3) -> Matrix3:
        return tuple(
            tuple(sum(a[row][k] * b[k][column] for k in range(3)) for column in range(3))
            for row in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _mat_transpose(matrix: Matrix3) -> Matrix3:
        return tuple(
            tuple(matrix[column][row] for column in range(3))
            for row in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _mat_sub_identity(matrix: Matrix3) -> Matrix3:
        return tuple(
            tuple(matrix[row][column] - (1.0 if row == column else 0.0) for column in range(3))
            for row in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> Matrix3:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def rotation_leg_to_body(leg: LegInfo) -> Matrix3:
        yaw = math.radians(leg.frame_pose.theta_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))

    def base_delta_to_tip_delta_3d(
        self,
        base1: BasePose3D,
        base2: BasePose3D,
        leg: LegInfo,
        tip_local_1: Point3D,
    ) -> Point3D:
        """Return the local leg-frame delta that keeps a stance tip fixed."""

        r_world_body1 = self.rotation_from_rpy(base1.roll, base1.pitch, base1.yaw)
        r_world_body2 = self.rotation_from_rpy(base2.roll, base2.pitch, base2.yaw)
        r_body1_world = self._mat_transpose(r_world_body1)

        delta_world = (base2.x - base1.x, base2.y - base1.y, base2.z - base1.z)
        delta_body = self._mat_vec(r_body1_world, delta_world)
        r_body = self._mat_mul(r_body1_world, r_world_body2)

        r_bl = self.rotation_leg_to_body(leg)
        r_lb = self._mat_transpose(r_bl)
        mount_body = (leg.frame_pose.x, leg.frame_pose.y, 0.0)
        delta_leg = self._mat_vec(
            r_lb,
            self._vec_add(
                delta_body,
                self._mat_vec(self._mat_sub_identity(r_body), mount_body),
            ),
        )
        r_leg = self._mat_mul(self._mat_mul(r_lb, r_body), r_bl)
        tip1 = (tip_local_1.x, tip_local_1.y, tip_local_1.z)
        tip2 = self._mat_vec(
            self._mat_transpose(r_leg),
            self._vec_sub(tip1, delta_leg),
        )
        return Point3D(tip2[0] - tip1[0], tip2[1] - tip1[1], tip2[2] - tip1[2])

    def tips_for_base_pose(self, base_pose: BasePose3D) -> Dict[int, Point3D]:
        neutral_base = BasePose3D()
        neutral_tips = self.neutral_tip_positions()
        output: Dict[int, Point3D] = {}
        for leg in self.legs:
            neutral = neutral_tips[leg.leg_id]
            delta = self.base_delta_to_tip_delta_3d(
                neutral_base,
                base_pose,
                leg,
                neutral,
            )
            output[leg.leg_id] = Point3D(
                neutral.x + delta.x,
                neutral.y + delta.y,
                neutral.z + delta.z,
            )
        return output

    def joint_goal_for_base_pose(
        self,
        base_pose: BasePose3D,
        *,
        clamp_reach: bool = False,
    ) -> List[float]:
        tips = self.tips_for_base_pose(base_pose)
        output: List[float] = []
        for leg in self.legs:
            output.extend(
                self.inverse_kinematics(
                    tips[leg.leg_id],
                    clamp_reach=clamp_reach,
                )
            )
        return output

    def base_pose_is_reachable(self, base_pose: BasePose3D) -> bool:
        return all(
            self.tip_is_reachable(tip)
            for tip in self.tips_for_base_pose(base_pose).values()
        )
