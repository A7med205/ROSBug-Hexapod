"""Transport-independent elevation, pitch, and roll posture planning."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

from robot_core import (
    BasePose3D,
    GoalIdAllocator,
    HexapodModel,
    JointBatch,
    PostureAxis,
)


class PostureState:
    NEUTRAL = "stationary"
    MOVING = "posture_moving"
    POSTURE_HOLD = "posture_hold"
    RETURNING = "posture_returning"


@dataclass(frozen=True)
class PostureConfig:
    sample_period: float = 0.02
    elevation_velocity: float = 0.005
    angular_velocity: float = math.radians(5.0)
    elevation_acceleration: float = 0.020
    angular_acceleration: float = math.radians(20.0)
    zero_tolerance: float = 1.0e-9
    ik_boundary_iterations: int = 52
    max_batch_points: int = 64


@dataclass(frozen=True)
class PosturePlanResult:
    axis: str
    requested_delta: float
    applied_delta: float

    @property
    def was_clamped(self) -> bool:
        return not math.isclose(
            self.requested_delta,
            self.applied_delta,
            rel_tol=1.0e-8,
            abs_tol=1.0e-10,
        )


@dataclass(frozen=True)
class _PostureSample:
    pose: BasePose3D
    joints: Tuple[float, ...]


@dataclass
class _PostureJob:
    phase_name: str
    samples: List[_PostureSample]
    is_return: bool = False
    cursor: int = 0
    stop_after_pending: bool = False


@dataclass
class _PendingPostureBatch:
    batch: JointBatch
    end_index: int
    end_pose: BasePose3D
    end_joint_goal: List[float]


class PostureCoordinator:
    """Plan atomic relative posture commands and retain their confirmed pose."""

    def __init__(
        self,
        model: HexapodModel,
        config: Optional[PostureConfig] = None,
        goal_id_allocator: Optional[GoalIdAllocator] = None,
    ) -> None:
        self.model = model
        self.config = config or PostureConfig()
        self._validate_config()
        self._goal_ids = goal_id_allocator or GoalIdAllocator()
        self.state = PostureState.NEUTRAL
        self.current_pose = BasePose3D()
        self.current_joint_goal = self.model.neutral_joint_goal()
        self.last_plan_result: Optional[PosturePlanResult] = None
        self._job: Optional[_PostureJob] = None
        self._pending: Optional[_PendingPostureBatch] = None
        self._return_requested = False

    def _validate_config(self) -> None:
        cfg = self.config
        positive = (
            cfg.sample_period,
            cfg.elevation_velocity,
            cfg.angular_velocity,
            cfg.elevation_acceleration,
            cfg.angular_acceleration,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("posture timing, velocity, and acceleration must be positive")
        if cfg.max_batch_points < 1:
            raise ValueError("max_batch_points must be positive")
        if cfg.ik_boundary_iterations < 1:
            raise ValueError("ik_boundary_iterations must be positive")

    @property
    def pending_batch(self) -> Optional[JointBatch]:
        return self._pending.batch if self._pending else None

    @property
    def is_busy(self) -> bool:
        return self._job is not None or self._pending is not None

    @property
    def is_neutral(self) -> bool:
        cfg = self.config
        pose = self.current_pose
        return (
            abs(pose.z) <= cfg.zero_tolerance
            and abs(pose.roll) <= cfg.zero_tolerance
            and abs(pose.pitch) <= cfg.zero_tolerance
        )

    def reset_neutral(self, joint_goal: Optional[List[float]] = None) -> None:
        if self.is_busy:
            raise RuntimeError("cannot reset posture while a batch is active")
        self.current_pose = BasePose3D()
        self.current_joint_goal = list(
            joint_goal if joint_goal is not None else self.model.neutral_joint_goal()
        )
        self.last_plan_result = None
        self._return_requested = False
        self.state = PostureState.NEUTRAL

    def _axis_value(self, pose: BasePose3D, axis: str) -> float:
        if axis == PostureAxis.ELEVATION:
            return pose.z
        if axis == PostureAxis.PITCH:
            return pose.pitch
        if axis == PostureAxis.ROLL:
            return pose.roll
        raise ValueError(f"unknown posture axis: {axis}")

    @staticmethod
    def _with_axis(pose: BasePose3D, axis: str, value: float) -> BasePose3D:
        if axis == PostureAxis.ELEVATION:
            return replace(pose, z=value)
        if axis == PostureAxis.PITCH:
            return replace(pose, pitch=value)
        if axis == PostureAxis.ROLL:
            return replace(pose, roll=value)
        raise ValueError(f"unknown posture axis: {axis}")

    @staticmethod
    def _interpolate_pose(start: BasePose3D, target: BasePose3D, fraction: float) -> BasePose3D:
        return BasePose3D(
            x=start.x + fraction * (target.x - start.x),
            y=start.y + fraction * (target.y - start.y),
            z=start.z + fraction * (target.z - start.z),
            roll=start.roll + fraction * (target.roll - start.roll),
            pitch=start.pitch + fraction * (target.pitch - start.pitch),
            yaw=start.yaw + fraction * (target.yaw - start.yaw),
        )

    def _clamp_target_to_ik(self, target: BasePose3D) -> BasePose3D:
        if self.model.base_pose_is_reachable(target):
            return target
        if not self.model.base_pose_is_reachable(self.current_pose):
            raise RuntimeError("confirmed posture is outside the IK workspace")

        low = 0.0
        high = 1.0
        for _ in range(self.config.ik_boundary_iterations):
            middle = (low + high) * 0.5
            candidate = self._interpolate_pose(self.current_pose, target, middle)
            if self.model.base_pose_is_reachable(candidate):
                low = middle
            else:
                high = middle
        return self._interpolate_pose(self.current_pose, target, low)

    def _profile_fractions(
        self,
        distance: float,
        maximum_velocity: float,
        maximum_acceleration: float,
    ) -> List[float]:
        if distance <= self.config.zero_tolerance:
            return []

        # Cubic smoothstep has max normalized velocity 1.5 and acceleration 6.
        duration = max(
            1.5 * distance / maximum_velocity,
            math.sqrt(6.0 * distance / maximum_acceleration),
            self.config.sample_period,
        )
        point_count = max(1, math.ceil(duration / self.config.sample_period))
        fractions: List[float] = []
        for index in range(1, point_count + 1):
            u = index / point_count
            fractions.append(u * u * (3.0 - 2.0 * u))
        fractions[-1] = 1.0
        return fractions

    def _build_samples(
        self,
        start: BasePose3D,
        target: BasePose3D,
        axis: str,
    ) -> List[_PostureSample]:
        distance = abs(self._axis_value(target, axis) - self._axis_value(start, axis))
        if axis == PostureAxis.ELEVATION:
            velocity = self.config.elevation_velocity
            acceleration = self.config.elevation_acceleration
        else:
            velocity = self.config.angular_velocity
            acceleration = self.config.angular_acceleration

        samples: List[_PostureSample] = []
        for fraction in self._profile_fractions(distance, velocity, acceleration):
            pose = self._interpolate_pose(start, target, fraction)
            joints = tuple(self.model.joint_goal_for_base_pose(pose, clamp_reach=False))
            samples.append(_PostureSample(pose, joints))
        return samples

    def _make_job(
        self,
        start: BasePose3D,
        target: BasePose3D,
        axis: str,
        phase_name: str,
        *,
        is_return: bool = False,
    ) -> Optional[_PostureJob]:
        samples = self._build_samples(start, target, axis)
        if not samples:
            return None
        return _PostureJob(phase_name=phase_name, samples=samples, is_return=is_return)

    def request_delta(self, axis: str, delta: float) -> bool:
        if axis not in PostureAxis.ALL:
            return False
        if isinstance(delta, bool) or not isinstance(delta, (int, float)):
            return False
        delta = float(delta)
        if not math.isfinite(delta) or abs(delta) <= self.config.zero_tolerance:
            return False
        if self.is_busy or self._return_requested:
            return False
        if axis == PostureAxis.PITCH and abs(self.current_pose.roll) > self.config.zero_tolerance:
            return False
        if axis == PostureAxis.ROLL and abs(self.current_pose.pitch) > self.config.zero_tolerance:
            return False

        start_value = self._axis_value(self.current_pose, axis)
        target_value = start_value + delta
        if abs(target_value) <= self.config.zero_tolerance:
            target_value = 0.0
        requested_target = self._with_axis(self.current_pose, axis, target_value)
        if not all(
            math.isfinite(value)
            for value in (
                requested_target.z,
                requested_target.roll,
                requested_target.pitch,
            )
        ):
            return False
        target = self._clamp_target_to_ik(requested_target)
        applied_delta = self._axis_value(target, axis) - start_value
        self.last_plan_result = PosturePlanResult(axis, delta, applied_delta)
        self._job = self._make_job(
            self.current_pose,
            target,
            axis,
            f"posture {axis}",
        )
        if self._job is not None:
            self.state = PostureState.MOVING
        return True

    def request_return_to_neutral(self) -> bool:
        self._return_requested = True
        if not self.is_busy and self.is_neutral:
            self._return_requested = False
            self.state = PostureState.NEUTRAL
        elif self._job is None or self._job.is_return:
            self.state = PostureState.RETURNING
        return True

    def cancel_return(self) -> None:
        self._return_requested = False
        if self._job is not None and self._job.is_return:
            if self._pending is None:
                self._job = None
            else:
                self._job.stop_after_pending = True

    def _prepare_return_stage(self) -> None:
        if not self._return_requested or self._job is not None:
            return
        tolerance = self.config.zero_tolerance
        if abs(self.current_pose.pitch) > tolerance:
            target = replace(self.current_pose, pitch=0.0)
            axis = PostureAxis.PITCH
        elif abs(self.current_pose.roll) > tolerance:
            target = replace(self.current_pose, roll=0.0)
            axis = PostureAxis.ROLL
        elif abs(self.current_pose.z) > tolerance:
            target = replace(self.current_pose, z=0.0)
            axis = PostureAxis.ELEVATION
        else:
            self.current_pose = BasePose3D()
            self.current_joint_goal = self.model.neutral_joint_goal()
            self._return_requested = False
            self.state = PostureState.NEUTRAL
            return

        self._job = self._make_job(
            self.current_pose,
            target,
            axis,
            f"posture return {axis}",
            is_return=True,
        )
        self.state = PostureState.RETURNING

    def next_batch(self) -> Optional[JointBatch]:
        if self._pending is not None:
            return None
        self._prepare_return_stage()
        if self._job is None:
            return None

        end_index = min(
            self._job.cursor + self.config.max_batch_points,
            len(self._job.samples),
        )
        chunk = self._job.samples[self._job.cursor:end_index]
        if not chunk:
            raise RuntimeError("posture job contains no remaining points")
        batch = JointBatch(
            goal_id=self._goal_ids.allocate(),
            phase_name=self._job.phase_name,
            points=tuple(sample.joints for sample in chunk),
            sample_period=self.config.sample_period,
        )
        final_sample = chunk[-1]
        self._pending = _PendingPostureBatch(
            batch=batch,
            end_index=end_index,
            end_pose=final_sample.pose,
            end_joint_goal=list(final_sample.joints),
        )
        return batch

    def complete_batch(self, goal_id: int, succeeded: bool = True) -> None:
        if self._pending is None or self._pending.batch.goal_id != goal_id:
            raise ValueError(f"goal {goal_id} is not the pending posture batch")
        pending = self._pending
        self._pending = None
        if not succeeded:
            self._job = None
            return
        if self._job is None:
            raise RuntimeError("pending posture batch has no owning job")

        self.current_pose = pending.end_pose
        self.current_joint_goal = pending.end_joint_goal
        self._job.cursor = pending.end_index
        if self._job.stop_after_pending:
            self._job = None
            self.state = PostureState.POSTURE_HOLD if not self.is_neutral else PostureState.NEUTRAL
            return
        if self._job.cursor < len(self._job.samples):
            return

        was_return = self._job.is_return
        self._job = None
        if was_return and self._return_requested:
            self._prepare_return_stage()
            return
        self.state = PostureState.NEUTRAL if self.is_neutral else PostureState.POSTURE_HOLD
