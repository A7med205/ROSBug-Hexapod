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
    elevation_velocity: float = 0.030
    angular_velocity: float = math.radians(10.0)
    elevation_acceleration: float = 0.050
    angular_acceleration: float = math.radians(40.0)
    # Raw objective body elevations above the stance-tip plane. The operating
    # scale contracts these knots around the stationary objective (-home_z).
    elevation_knots: Tuple[float, ...] = (0.025, 0.050, 0.080, 0.098, 0.110)
    roll_limit_knots: Tuple[float, ...] = tuple(
        math.radians(value) for value in (0.0, 12.0, 20.0, 10.0, 0.0)
    )
    pitch_limit_knots: Tuple[float, ...] = tuple(
        math.radians(value) for value in (0.0, 12.0, 25.0, 12.0, 0.0)
    )
    operating_limit_scale: float = 0.9
    zero_tolerance: float = 1.0e-9
    ik_boundary_iterations: int = 52
    max_batch_points: int = 25


@dataclass(frozen=True)
class PosturePlanResult:
    axis: str
    requested_value: float
    applied_value: float

    @property
    def was_clamped(self) -> bool:
        return not math.isclose(
            self.requested_value,
            self.applied_value,
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
    """Plan bounded elevation and relative tilt from confirmed state."""

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
        if not math.isfinite(cfg.operating_limit_scale) or not (
            0.0 < cfg.operating_limit_scale <= 1.0
        ):
            raise ValueError("operating_limit_scale must be in (0, 1]")
        knot_count = len(cfg.elevation_knots)
        if knot_count < 2 or any(
            len(limits) != knot_count
            for limits in (cfg.roll_limit_knots, cfg.pitch_limit_knots)
        ):
            raise ValueError("posture limit curves must have matching knot counts")
        if any(
            not math.isfinite(value)
            for curve in (
                cfg.elevation_knots,
                cfg.roll_limit_knots,
                cfg.pitch_limit_knots,
            )
            for value in curve
        ):
            raise ValueError("posture limit curves must be finite")
        if any(
            following <= previous
            for previous, following in zip(
                cfg.elevation_knots,
                cfg.elevation_knots[1:],
            )
        ):
            raise ValueError("elevation knots must be strictly increasing")
        if any(
            value < 0.0
            for limits in (cfg.roll_limit_knots, cfg.pitch_limit_knots)
            for value in limits
        ):
            raise ValueError("posture angular limits must be non-negative")

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

    def angular_limit_at_elevation(self, axis: str, elevation: float) -> float:
        """Return the scaled angular limit at an objective elevation."""

        cfg = self.config
        elevation_knots = self.operating_elevation_knots
        if axis == PostureAxis.ROLL:
            limits = cfg.roll_limit_knots
        elif axis == PostureAxis.PITCH:
            limits = cfg.pitch_limit_knots
        else:
            raise ValueError("angular limits only apply to roll and pitch")

        if elevation <= elevation_knots[0]:
            raw_limit = limits[0]
        elif elevation >= elevation_knots[-1]:
            raw_limit = limits[-1]
        else:
            raw_limit = limits[-1]
            for index in range(len(elevation_knots) - 1):
                lower_z = elevation_knots[index]
                upper_z = elevation_knots[index + 1]
                if lower_z <= elevation <= upper_z:
                    fraction = (elevation - lower_z) / (upper_z - lower_z)
                    raw_limit = limits[index] + fraction * (
                        limits[index + 1] - limits[index]
                    )
                    break
        return raw_limit * cfg.operating_limit_scale

    @property
    def stationary_elevation(self) -> float:
        """Return the objective elevation represented by the gait home pose."""

        return -self.model.config.home_z

    @property
    def operating_elevation_knots(self) -> Tuple[float, ...]:
        """Return elevation knots scaled around the stationary objective."""

        stationary = self.stationary_elevation
        scale = self.config.operating_limit_scale
        return tuple(
            stationary + scale * (elevation - stationary)
            for elevation in self.config.elevation_knots
        )

    @property
    def current_elevation(self) -> float:
        return self.elevation_for_pose(self.current_pose)

    def elevation_for_pose(self, pose: BasePose3D) -> float:
        """Convert internal body displacement into objective elevation."""

        return self.stationary_elevation + pose.z

    def pose_is_allowed(self, pose: BasePose3D) -> bool:
        cfg = self.config
        tolerance = cfg.zero_tolerance
        objective_elevation = self.elevation_for_pose(pose)
        elevation_knots = self.operating_elevation_knots
        if not (
            elevation_knots[0] - tolerance
            <= objective_elevation
            <= elevation_knots[-1] + tolerance
        ):
            return False
        if abs(pose.roll) > tolerance and abs(pose.pitch) > tolerance:
            return False
        if abs(pose.roll) > self.angular_limit_at_elevation(
            PostureAxis.ROLL,
            objective_elevation,
        ) + tolerance:
            return False
        pitch_limit = self.angular_limit_at_elevation(
            PostureAxis.PITCH,
            objective_elevation,
        )
        if abs(pose.pitch) > pitch_limit + tolerance:
            return False
        return self.model.base_pose_is_reachable(pose)

    def _clamp_target(self, target: BasePose3D) -> BasePose3D:
        if self.pose_is_allowed(target):
            return target
        if not self.pose_is_allowed(self.current_pose):
            raise RuntimeError("confirmed posture is outside its operating limits")

        low = 0.0
        high = 1.0
        for _ in range(self.config.ik_boundary_iterations):
            middle = (low + high) * 0.5
            candidate = self._interpolate_pose(self.current_pose, target, middle)
            if self.pose_is_allowed(candidate):
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
            if not self.pose_is_allowed(pose):
                raise RuntimeError("posture profile left its operating limits")
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
        if axis not in (PostureAxis.PITCH, PostureAxis.ROLL):
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
        target = self._clamp_target(requested_target)
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

    def request_elevation(self, objective_elevation: float) -> bool:
        """Move to an absolute body elevation above the stance-tip plane."""

        if isinstance(objective_elevation, bool) or not isinstance(
            objective_elevation,
            (int, float),
        ):
            return False
        objective_elevation = float(objective_elevation)
        if not math.isfinite(objective_elevation):
            return False
        if self.is_busy or self._return_requested:
            return False

        requested_target = replace(
            self.current_pose,
            z=objective_elevation - self.stationary_elevation,
        )
        target = self._clamp_target(requested_target)
        applied_elevation = self.elevation_for_pose(target)
        self.last_plan_result = PosturePlanResult(
            PostureAxis.ELEVATION,
            objective_elevation,
            applied_elevation,
        )
        self._job = self._make_job(
            self.current_pose,
            target,
            PostureAxis.ELEVATION,
            "posture elevation",
        )
        if self._job is not None:
            self.state = PostureState.MOVING
        return True

    def request_elevation_delta(self, delta: float) -> bool:
        """Move by a relative elevation delta while retaining absolute limits."""

        if isinstance(delta, bool) or not isinstance(delta, (int, float)):
            return False
        delta = float(delta)
        if not math.isfinite(delta) or self.is_busy or self._return_requested:
            return False

        start_elevation = self.current_elevation
        accepted = self.request_elevation(start_elevation + delta)
        if accepted and self.last_plan_result is not None:
            applied_delta = self.last_plan_result.applied_value - start_elevation
            self.last_plan_result = PosturePlanResult(
                PostureAxis.ELEVATION,
                delta,
                applied_delta,
            )
        return accepted

    def request_tilt_reset(self) -> bool:
        """Smoothly zero the active tilt axis while preserving elevation."""

        if self.is_busy or self._return_requested:
            return False

        tolerance = self.config.zero_tolerance
        if abs(self.current_pose.pitch) > tolerance:
            axis = PostureAxis.PITCH
        elif abs(self.current_pose.roll) > tolerance:
            axis = PostureAxis.ROLL
        else:
            return True

        start_value = self._axis_value(self.current_pose, axis)
        target = self._with_axis(self.current_pose, axis, 0.0)
        self.last_plan_result = PosturePlanResult(
            axis,
            -start_value,
            -start_value,
        )
        self._job = self._make_job(
            self.current_pose,
            target,
            axis,
            f"posture reset {axis}",
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

    def request_interrupt(self) -> bool:
        """Discard the active profile after its currently executing batch."""

        if self._job is None:
            return False
        self._return_requested = False
        if self._pending is None:
            self._job = None
            self.state = (
                PostureState.NEUTRAL
                if self.is_neutral
                else PostureState.POSTURE_HOLD
            )
        else:
            self._job.stop_after_pending = True
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
