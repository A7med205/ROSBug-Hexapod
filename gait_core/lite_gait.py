"""Shared lite tripod gait implementation.

This module owns gait geometry, template generation, joint gating, and the
latest-command state machine.  It intentionally contains no terminal, serial,
ROS, or wall-clock code.  Both output adapters consume the same ``JointBatch``
objects produced here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from robot_core import (
    Command,
    CommandKind,
    ControllerMode,
    GoalIdAllocator,
    HexapodModel,
    JointAngles,
    JointBatch,
    Point3D,
    RobotConfig,
)


class ControllerState:
    AWAITING_STARTUP = "awaiting_startup"
    STARTING = "starting"
    STATIONARY = "stationary"
    WALKING = "walking"


class AutoJobStatus:
    RUNNING = "running"
    ABORTING = "aborting"


@dataclass
class AutoJob:
    trajectory_id: int
    requested_steps: int
    remaining_half_steps: int
    status: str = AutoJobStatus.RUNNING

    @property
    def completed_half_steps(self) -> int:
        return self.requested_steps * 2 - self.remaining_half_steps


@dataclass(frozen=True)
class GaitConfig(RobotConfig):
    # Canonical values are the physical-hardware values.
    limit_radius: float = 0.04
    swing_height: float = 0.025
    sample_period: float = 0.02
    min_angle_deg: float = 1.0
    startup_z: float = 0.01
    startup_velocity: float = 0.05
    startup_hold: float = 2.0

    linear_speed_y: float = 0.20
    linear_speed_x: float = 0.20
    diagonal_speed: float = 0.20
    self_angular_speed: float = 0.80
    orbit_angular_speed: float = 0.60
    external_radius: float = 0.30


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
class _PendingBatch:
    batch: JointBatch
    end_joint_goal: List[float]
    transition: str
    trajectory_id: int = 0
    pull_tripod: str = ""


class LiteGaitModel(HexapodModel):
    PATH_TYPES = ("half1", "half2", "full")
    SWING_TYPES = ("half1", "half2", "full1", "full2")
    MOVING_TRAJECTORY_IDS = tuple(range(1, 15))
    TRIPODS = ("A", "B")

    def __init__(self, config: GaitConfig) -> None:
        super().__init__(config)
        self._reset_template_stores()
        self._build_all_templates()
        self._convert_templates_to_joint_space()

    def _new_leg_store(self, type_names: Tuple[str, ...]):
        return {
            leg.leg_id: {type_name: [] for type_name in type_names}
            for leg in self.legs
        }

    def _reset_template_stores(self) -> None:
        ids = self.MOVING_TRAJECTORY_IDS
        self.tip_paths = {traj: self._new_leg_store(self.PATH_TYPES) for traj in ids}
        self.tip_swings = {traj: self._new_leg_store(self.SWING_TYPES) for traj in ids}
        self.joint_paths = {traj: self._new_leg_store(self.PATH_TYPES) for traj in ids}
        self.joint_swings = {traj: self._new_leg_store(self.SWING_TYPES) for traj in ids}
        self.duration_points = {
            traj: {tripod: {"half": 0, "full": 0} for tripod in self.TRIPODS}
            for traj in ids
        }

    def base_delta_to_tip_delta(
        self,
        base1: BasePose2D,
        base2: BasePose2D,
        leg: LegInfo,
        tip_local_1: Point3D,
    ) -> LocalDisplacement2D:
        """Transform a base delta into a stance-locked local tip delta."""

        leg_theta = math.radians(leg.frame_pose.theta_deg)
        body_rotation = base2.theta - base1.theta

        c_base1 = math.cos(base1.theta)
        s_base1 = math.sin(base1.theta)
        delta_world_x = base2.x - base1.x
        delta_world_y = base2.y - base1.y
        delta_body_x = c_base1 * delta_world_x + s_base1 * delta_world_y
        delta_body_y = -s_base1 * delta_world_x + c_base1 * delta_world_y

        c_body = math.cos(body_rotation)
        s_body = math.sin(body_rotation)
        mount_rot_x = c_body * leg.frame_pose.x - s_body * leg.frame_pose.y
        mount_rot_y = s_body * leg.frame_pose.x + c_body * leg.frame_pose.y

        c_leg = math.cos(leg_theta)
        s_leg = math.sin(leg_theta)
        delta_leg_x = c_leg * (
            delta_body_x + mount_rot_x - leg.frame_pose.x
        ) + s_leg * (delta_body_y + mount_rot_y - leg.frame_pose.y)
        delta_leg_y = -s_leg * (
            delta_body_x + mount_rot_x - leg.frame_pose.x
        ) + c_leg * (delta_body_y + mount_rot_y - leg.frame_pose.y)

        tip_shift_x = tip_local_1.x - delta_leg_x
        tip_shift_y = tip_local_1.y - delta_leg_y
        tip_local_2_x = c_body * tip_shift_x + s_body * tip_shift_y
        tip_local_2_y = -s_body * tip_shift_x + c_body * tip_shift_y
        return LocalDisplacement2D(
            tip_local_2_x - tip_local_1.x,
            tip_local_2_y - tip_local_1.y,
        )

    def master_path(self, t: float, trajectory_id: int) -> BasePose2D:
        cfg = self.config
        if 8 <= trajectory_id <= 14:
            return self.master_path(-t, trajectory_id - 7)
        if trajectory_id == 1:
            return BasePose2D(0.0, cfg.linear_speed_y * t, 0.0)
        if trajectory_id == 2:
            return BasePose2D(cfg.linear_speed_x * t, 0.0, 0.0)
        if trajectory_id == 3:
            return BasePose2D(cfg.diagonal_speed * t, cfg.diagonal_speed * t, 0.0)
        if trajectory_id == 4:
            return BasePose2D(-cfg.diagonal_speed * t, cfg.diagonal_speed * t, 0.0)
        if trajectory_id in (5, 6):
            center_x = cfg.external_radius if trajectory_id == 5 else -cfg.external_radius
            phi0 = math.pi if trajectory_id == 5 else 0.0
            phi = phi0 + cfg.orbit_angular_speed * t
            return BasePose2D(
                center_x + cfg.external_radius * math.cos(phi),
                cfg.external_radius * math.sin(phi),
                cfg.orbit_angular_speed * t,
            )
        if trajectory_id == 7:
            return BasePose2D(0.0, 0.0, -cfg.self_angular_speed * t)
        return BasePose2D(0.0, 0.0, 0.0)

    def _pull_builder(
        self, tripod: str, trajectory_id: int, sign: int
    ) -> Dict[int, List[Point3D]]:
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")
        cfg = self.config
        selected_legs = self.tripod_legs(tripod)
        home = Point3D(cfg.home_x, cfg.home_y, cfg.home_z)
        paths = {leg.leg_id: [home] for leg in selected_legs}
        current = {leg.leg_id: home for leg in selected_legs}
        starts = {leg.leg_id: (home.x, home.y) for leg in selected_legs}

        t_previous = 0.0
        base_previous = self.master_path(t_previous, trajectory_id)
        for _ in range(10000):
            t_current = t_previous + sign * cfg.sample_period
            base_current = self.master_path(t_current, trajectory_id)
            hit_limit = False
            for leg in selected_legs:
                previous = current[leg.leg_id]
                delta = self.base_delta_to_tip_delta(
                    base_previous, base_current, leg, previous
                )
                following = Point3D(
                    previous.x + delta.dx_local,
                    previous.y + delta.dy_local,
                    home.z,
                )
                current[leg.leg_id] = following
                paths[leg.leg_id].append(following)
                start_x, start_y = starts[leg.leg_id]
                if math.hypot(following.x - start_x, following.y - start_y) >= cfg.limit_radius:
                    hit_limit = True
            t_previous = t_current
            base_previous = base_current
            if hit_limit:
                break
        return paths

    @staticmethod
    def _copy_point(point: Point3D) -> Point3D:
        return Point3D(point.x, point.y, point.z)

    def _resample_xy_path(self, path: List[Point3D], point_count: int) -> List[Point3D]:
        point_count = max(point_count, 2)
        if not path:
            cfg = self.config
            home = Point3D(cfg.home_x, cfg.home_y, cfg.home_z)
            return [home, home]
        if len(path) == 1:
            return [self._copy_point(path[0]) for _ in range(point_count)]

        cumulative = [0.0]
        for index in range(1, len(path)):
            cumulative.append(
                cumulative[-1]
                + math.hypot(
                    path[index].x - path[index - 1].x,
                    path[index].y - path[index - 1].y,
                )
            )
        total = cumulative[-1]
        if total <= 1e-12:
            start = path[0]
            end = path[-1]
            return [
                Point3D(
                    start.x + index / (point_count - 1) * (end.x - start.x),
                    start.y + index / (point_count - 1) * (end.y - start.y),
                    start.z + index / (point_count - 1) * (end.z - start.z),
                )
                for index in range(point_count)
            ]

        output: List[Point3D] = []
        segment = 0
        for index in range(point_count):
            target = index / (point_count - 1) * total
            while segment < len(cumulative) - 2 and cumulative[segment + 1] < target:
                segment += 1
            segment_start = cumulative[segment]
            segment_end = cumulative[segment + 1]
            local = (
                0.0
                if segment_end <= segment_start
                else (target - segment_start) / (segment_end - segment_start)
            )
            point0 = path[segment]
            point1 = path[segment + 1]
            output.append(
                Point3D(
                    point0.x + local * (point1.x - point0.x),
                    point0.y + local * (point1.y - point0.y),
                    point0.z + local * (point1.z - point0.z),
                )
            )
        return output

    def _swing_builder(self, path: List[Point3D], point_count: int) -> List[Point3D]:
        cfg = self.config
        source = list(reversed(path))
        if not source:
            source = [Point3D(cfg.home_x, cfg.home_y, cfg.home_z)]
        shadow = self._resample_xy_path(source, point_count)
        output: List[Point3D] = []
        for index, sample in enumerate(shadow):
            progress = index / (len(shadow) - 1) if len(shadow) > 1 else 0.0
            output.append(
                Point3D(
                    sample.x,
                    sample.y,
                    (1.0 - progress) * source[0].z
                    + progress * source[-1].z
                    + cfg.swing_height * math.sin(math.pi * progress),
                )
            )
        return output

    def _build_tripod_templates(self, trajectory_id: int, tripod: str) -> None:
        positive = self._pull_builder(tripod, trajectory_id, 1)
        negative = self._pull_builder(tripod, trajectory_id, -1)
        cfg = self.config
        for leg in self.tripod_legs(tripod):
            fallback = [Point3D(cfg.home_x, cfg.home_y, cfg.home_z)]
            positive_path = positive.get(leg.leg_id, fallback)
            negative_path = negative.get(leg.leg_id, fallback)
            half1 = list(reversed(negative_path))
            half2 = positive_path
            self.tip_paths[trajectory_id][leg.leg_id]["half1"] = half1
            self.tip_paths[trajectory_id][leg.leg_id]["half2"] = half2
            self.tip_paths[trajectory_id][leg.leg_id]["full"] = half1 + half2[1:]

    def _set_duration_points(self, trajectory_id: int, tripod: str) -> None:
        legs = self.tripod_legs(tripod)
        first_leg_id = legs[0].leg_id
        self.duration_points[trajectory_id][tripod]["half"] = max(
            len(self.tip_paths[trajectory_id][first_leg_id]["half1"]), 2
        )
        self.duration_points[trajectory_id][tripod]["full"] = max(
            len(self.tip_paths[trajectory_id][first_leg_id]["full"]), 2
        )

    @staticmethod
    def _split_swing(
        swing: List[Point3D], split_index: int
    ) -> Tuple[List[Point3D], List[Point3D]]:
        split_index = max(1, min(split_index, len(swing) - 1))
        return swing[:split_index], swing[split_index:]

    def _build_tripod_swings(self, trajectory_id: int, tripod: str) -> None:
        other = self.opposite_tripod(tripod)
        half_points = self.duration_points[trajectory_id][other]["half"]
        full_points = self.duration_points[trajectory_id][other]["full"]
        for leg in self.tripod_legs(tripod):
            leg_id = leg.leg_id
            paths = self.tip_paths[trajectory_id][leg_id]
            swings = self.tip_swings[trajectory_id][leg_id]
            swings["half1"] = self._swing_builder(paths["half1"], half_points)
            swings["half2"] = self._swing_builder(paths["half2"], half_points)
            full_swing = self._swing_builder(paths["full"], full_points)
            swings["full2"], swings["full1"] = self._split_swing(
                full_swing, half_points
            )

    def _build_all_templates(self) -> None:
        for trajectory_id in self.MOVING_TRAJECTORY_IDS:
            for tripod in self.TRIPODS:
                self._build_tripod_templates(trajectory_id, tripod)
            for tripod in self.TRIPODS:
                self._set_duration_points(trajectory_id, tripod)
            for tripod in self.TRIPODS:
                self._build_tripod_swings(trajectory_id, tripod)

    def _convert_templates_to_joint_space(self) -> None:
        for trajectory_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for path_type in self.PATH_TYPES:
                    self.joint_paths[trajectory_id][leg_id][path_type] = [
                        self.inverse_kinematics(point)
                        for point in self.tip_paths[trajectory_id][leg_id][path_type]
                    ]
                for swing_type in self.SWING_TYPES:
                    self.joint_swings[trajectory_id][leg_id][swing_type] = [
                        self.inverse_kinematics(point)
                        for point in self.tip_swings[trajectory_id][leg_id][swing_type]
                    ]

    def collect_phase_sequences(
        self,
        trajectory_id: int,
        pull_tripod: str,
        pull_path_type: str,
        swing_type: str,
    ) -> Dict[int, List[JointAngles]]:
        output: Dict[int, List[JointAngles]] = {}
        for leg in self.legs:
            if leg.tripod == pull_tripod:
                output[leg.leg_id] = self.joint_paths[trajectory_id][leg.leg_id][
                    pull_path_type
                ]
            else:
                output[leg.leg_id] = self.joint_swings[trajectory_id][leg.leg_id][
                    swing_type
                ]
        return output


class LiteGaitCoordinator:
    """Shared mode, readiness, and half-step gait coordinator."""

    def __init__(
        self,
        config: Optional[GaitConfig] = None,
        goal_id_allocator: Optional[GoalIdAllocator] = None,
    ) -> None:
        self.config = config or GaitConfig()
        self.model = LiteGaitModel(self.config)
        self.state = ControllerState.AWAITING_STARTUP
        self.mode = ControllerMode.NORMAL
        self.requested_mode = ControllerMode.NORMAL
        self.requested_trajectory_id = 0
        self.active_trajectory_id = 0
        self.auto_job: Optional[AutoJob] = None
        self.next_full_pull_tripod = "B"
        self._walk_half = "first"
        self._startup_stage = ""
        self._goal_ids = goal_id_allocator or GoalIdAllocator()
        self._pending: Optional[_PendingBatch] = None
        self.current_joint_goal = self.model.neutral_joint_goal()

    @property
    def pending_batch(self) -> Optional[JointBatch]:
        return self._pending.batch if self._pending else None

    @property
    def is_stationary(self) -> bool:
        return self.state == ControllerState.STATIONARY and self._pending is None

    def _reset_motion_to_stationary(self) -> None:
        self.state = ControllerState.STATIONARY
        self.active_trajectory_id = 0
        self.requested_trajectory_id = 0
        self.next_full_pull_tripod = "B"
        self._walk_half = "first"

    def _finish_motion(self) -> None:
        self._reset_motion_to_stationary()
        self.auto_job = None
        self.mode = self.requested_mode

    def _request_mode(self, target_mode: str) -> bool:
        if target_mode not in (ControllerMode.NORMAL, ControllerMode.AUTO):
            return False
        if self.state in (ControllerState.AWAITING_STARTUP, ControllerState.STARTING):
            return False

        motion_in_progress = self.state == ControllerState.WALKING or self._pending is not None
        self.requested_mode = target_mode
        if self.state == ControllerState.STATIONARY and self._pending is None:
            # A mode change at rest cancels any requested-but-not-started
            # movement or auto job; nothing is deferred into the new mode.
            self.requested_trajectory_id = 0
            self.auto_job = None
            self.mode = self.requested_mode
            return True

        # Mode changes are only activated after a guaranteed stationary
        # transition. Toggling an auto job is therefore also an explicit abort.
        self.requested_trajectory_id = 0
        if self.auto_job is not None:
            self.auto_job.status = AutoJobStatus.ABORTING
        return True

    def _request_mode_toggle(self) -> bool:
        base_mode = (
            self.requested_mode
            if self.state == ControllerState.WALKING or self._pending is not None
            else self.mode
        )
        target_mode = (
            ControllerMode.AUTO
            if base_mode == ControllerMode.NORMAL
            else ControllerMode.NORMAL
        )
        return self._request_mode(target_mode)

    def request(self, command: Command) -> bool:
        if command.kind == CommandKind.STARTUP:
            if self.state != ControllerState.AWAITING_STARTUP or self._pending:
                return False
            self.state = ControllerState.STARTING
            self._startup_stage = "pose"
            return True

        if command.kind == CommandKind.SKIP_STARTUP:
            if self.state != ControllerState.AWAITING_STARTUP or self._pending:
                return False
            # This is an assertion, not a movement: the caller guarantees that
            # the physical robot already matches the canonical standing pose.
            self.current_joint_goal = self.model.neutral_joint_goal()
            self.mode = ControllerMode.NORMAL
            self.requested_mode = ControllerMode.NORMAL
            self.auto_job = None
            self._startup_stage = ""
            self._reset_motion_to_stationary()
            return True

        if command.kind == CommandKind.TOGGLE_MODE:
            return self._request_mode_toggle()

        if command.kind == CommandKind.SET_MODE:
            return command.mode is not None and self._request_mode(command.mode)

        if command.kind == CommandKind.STOP:
            self.requested_trajectory_id = 0
            if self.auto_job is not None:
                if self.state == ControllerState.STATIONARY and self._pending is None:
                    self.auto_job = None
                else:
                    self.auto_job.status = AutoJobStatus.ABORTING
            return True

        if command.kind != CommandKind.WALK:
            return False
        if command.trajectory_id not in self.model.MOVING_TRAJECTORY_IDS:
            return False
        if self.state in (ControllerState.AWAITING_STARTUP, ControllerState.STARTING):
            return False

        # Auto jobs are atomic. Movement commands received while one is active
        # are discarded rather than deferred for later execution.
        if self.auto_job is not None:
            return False
        if self.requested_mode != self.mode:
            return False

        if self.mode == ControllerMode.NORMAL:
            if command.steps is not None:
                return False
            self.requested_trajectory_id = command.trajectory_id
            return True

        if self.state != ControllerState.STATIONARY:
            return False
        if (
            isinstance(command.steps, bool)
            or not isinstance(command.steps, int)
            or command.steps < 2
        ):
            return False
        self.auto_job = AutoJob(
            trajectory_id=command.trajectory_id,
            requested_steps=command.steps,
            remaining_half_steps=command.steps * 2,
        )
        self.requested_trajectory_id = command.trajectory_id
        return True

    def _allocate_goal_id(self) -> int:
        return self._goal_ids.allocate()

    def _make_batch(
        self,
        phase_name: str,
        points: List[List[float]],
        trajectory_id: int = 0,
        hold_after: float = 0.0,
    ) -> JointBatch:
        if not points:
            raise ValueError("a joint batch must contain at least one point")
        frozen_points = tuple(tuple(point) for point in points)
        if any(len(point) != 18 for point in frozen_points):
            raise ValueError("every joint point must contain exactly 18 values")
        return JointBatch(
            goal_id=self._allocate_goal_id(),
            phase_name=phase_name,
            points=frozen_points,
            sample_period=self.config.sample_period,
            hold_after=hold_after,
            trajectory_id=trajectory_id,
        )

    def _gate_sequences(
        self, sequences: Dict[int, List[JointAngles]]
    ) -> Tuple[List[List[float]], List[float]]:
        if not sequences or any(not sequence for sequence in sequences.values()):
            raise ValueError("all six legs require a non-empty sequence")
        point_count = max(len(sequence) for sequence in sequences.values())
        threshold = math.radians(self.config.min_angle_deg)
        next_goal = list(self.current_joint_goal)
        output: List[List[float]] = []
        for point_index in range(point_count):
            desired: List[float] = []
            for leg in self.model.legs:
                sequence = sequences[leg.leg_id]
                desired.extend(
                    sequence[point_index]
                    if point_index < len(sequence)
                    else sequence[-1]
                )
            for joint_index, angle in enumerate(desired):
                current = next_goal[joint_index]
                if not math.isfinite(current) or abs(angle - current) >= threshold:
                    next_goal[joint_index] = angle
            output.append(list(next_goal))
        return output, next_goal

    def _prepare_gait_batch(
        self,
        phase_name: str,
        trajectory_id: int,
        pull_tripod: str,
        pull_path_type: str,
        swing_type: str,
        transition: str,
    ) -> JointBatch:
        sequences = self.model.collect_phase_sequences(
            trajectory_id,
            pull_tripod,
            pull_path_type,
            swing_type,
        )
        points, end_goal = self._gate_sequences(sequences)
        batch = self._make_batch(phase_name, points, trajectory_id)
        self._pending = _PendingBatch(
            batch=batch,
            end_joint_goal=end_goal,
            transition=transition,
            trajectory_id=trajectory_id,
            pull_tripod=pull_tripod,
        )
        return batch

    def _prepare_startup_pose(self) -> JointBatch:
        cfg = self.config
        startup_angles = self.model.inverse_kinematics(
            Point3D(cfg.home_x, cfg.home_y, cfg.startup_z)
        )
        end_goal = list(startup_angles) * len(self.model.legs)
        batch = self._make_batch(
            "startup pose",
            [end_goal],
            hold_after=cfg.startup_hold,
        )
        self._pending = _PendingBatch(batch, end_goal, "startup_pose")
        return batch

    def _prepare_startup_descent(self) -> JointBatch:
        cfg = self.config
        if cfg.startup_velocity <= 0.0:
            raise ValueError("startup_velocity must be positive")
        delta = cfg.home_z - cfg.startup_z
        step_distance = cfg.startup_velocity * cfg.sample_period
        step_count = max(1, math.ceil(abs(delta) / step_distance))
        threshold = math.radians(cfg.min_angle_deg)
        next_goal = list(self.current_joint_goal)
        points: List[List[float]] = []
        for step_index in range(1, step_count + 1):
            progress = step_index / step_count
            tip = Point3D(
                cfg.home_x,
                cfg.home_y,
                cfg.startup_z + progress * delta,
            )
            desired = list(self.model.inverse_kinematics(tip)) * len(self.model.legs)
            for joint_index, angle in enumerate(desired):
                current = next_goal[joint_index]
                if not math.isfinite(current) or abs(angle - current) >= threshold:
                    next_goal[joint_index] = angle
            points.append(list(next_goal))

        # Threshold gating must not leave the physical or simulated robot short
        # of the canonical standing pose.
        home_goal = self.model.neutral_joint_goal()
        points[-1] = list(home_goal)
        batch = self._make_batch("startup descent", points)
        self._pending = _PendingBatch(batch, home_goal, "startup_descent")
        return batch

    def next_batch(self) -> Optional[JointBatch]:
        if self._pending is not None:
            return None

        if self.state == ControllerState.STARTING:
            if self._startup_stage == "pose":
                return self._prepare_startup_pose()
            if self._startup_stage == "descent":
                return self._prepare_startup_descent()
            raise RuntimeError("invalid startup stage")

        if self.state == ControllerState.AWAITING_STARTUP:
            return None

        if self.state == ControllerState.STATIONARY:
            if self.requested_trajectory_id not in self.model.MOVING_TRAJECTORY_IDS:
                return None
            trajectory_id = self.requested_trajectory_id
            return self._prepare_gait_batch(
                f"start half-step t{trajectory_id}",
                trajectory_id,
                "A",
                "half2",
                "half1",
                "gait_start",
            )

        if self.state != ControllerState.WALKING:
            raise RuntimeError(f"invalid controller state: {self.state}")

        if self._walk_half == "first":
            if self.auto_job is not None:
                pull = self.next_full_pull_tripod
                if self.auto_job.status == AutoJobStatus.ABORTING:
                    return self._prepare_gait_batch(
                        f"auto abort half-step t{self.active_trajectory_id}",
                        self.active_trajectory_id,
                        pull,
                        "half1",
                        "half2",
                        "auto_abort",
                    )
                if self.auto_job.remaining_half_steps == 1:
                    return self._prepare_gait_batch(
                        f"auto final half-step t{self.active_trajectory_id}",
                        self.active_trajectory_id,
                        pull,
                        "half1",
                        "half2",
                        "auto_complete",
                    )
            if self.requested_trajectory_id == 0:
                pull = self.next_full_pull_tripod
                return self._prepare_gait_batch(
                    f"final half-step t{self.active_trajectory_id}",
                    self.active_trajectory_id,
                    pull,
                    "half1",
                    "half2",
                    "gait_stop",
                )
            pull = self.next_full_pull_tripod
            return self._prepare_gait_batch(
                f"full-step-1 t{self.active_trajectory_id}",
                self.active_trajectory_id,
                pull,
                "half1",
                "full2",
                "gait_first",
            )

        if self._walk_half == "second":
            trajectory_id = self.active_trajectory_id
            if (
                self.auto_job is None
                and self.requested_trajectory_id in self.model.MOVING_TRAJECTORY_IDS
            ):
                trajectory_id = self.requested_trajectory_id
            pull = self.next_full_pull_tripod
            return self._prepare_gait_batch(
                f"full-step-2 t{trajectory_id}",
                trajectory_id,
                pull,
                "half2",
                "full1",
                "gait_second",
            )

        raise RuntimeError(f"invalid walk half: {self._walk_half}")

    def complete_batch(self, goal_id: int, succeeded: bool = True) -> None:
        if self._pending is None or self._pending.batch.goal_id != goal_id:
            raise ValueError(f"goal {goal_id} is not the pending batch")
        pending = self._pending
        self._pending = None
        if not succeeded:
            return

        self.current_joint_goal = list(pending.end_joint_goal)
        if (
            self.auto_job is not None
            and self.auto_job.status == AutoJobStatus.RUNNING
            and pending.transition
            in ("gait_start", "gait_first", "gait_second", "auto_complete")
        ):
            if self.auto_job.remaining_half_steps <= 0:
                raise RuntimeError("auto job half-step count underflow")
            self.auto_job.remaining_half_steps -= 1

        if pending.transition == "startup_pose":
            self._startup_stage = "descent"
        elif pending.transition == "startup_descent":
            self._startup_stage = ""
            self._finish_motion()
        elif pending.transition == "gait_start":
            self.state = ControllerState.WALKING
            self.active_trajectory_id = pending.trajectory_id
            self.next_full_pull_tripod = "B"
            self._walk_half = "first"
        elif pending.transition == "gait_first":
            self._walk_half = "second"
        elif pending.transition == "gait_second":
            self.active_trajectory_id = pending.trajectory_id
            self.next_full_pull_tripod = self.model.opposite_tripod(
                pending.pull_tripod
            )
            self._walk_half = "first"
        elif pending.transition == "gait_stop":
            self._finish_motion()
        elif pending.transition == "auto_complete":
            if (
                self.auto_job is not None
                and self.auto_job.status == AutoJobStatus.RUNNING
                and self.auto_job.remaining_half_steps != 0
            ):
                raise RuntimeError("auto job completed with remaining half-steps")
            self._finish_motion()
        elif pending.transition == "auto_abort":
            self._finish_motion()
        else:
            raise RuntimeError(f"unknown batch transition: {pending.transition}")
