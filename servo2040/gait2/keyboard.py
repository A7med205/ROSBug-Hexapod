#!/usr/bin/env python3

import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Dict, List, Tuple

import serial


LINE_KEYS = {
    "w": 1,
    "d": 2,
    "s": 8,
    "a": 9,
}
DIAGONAL_KEYS = {
    "q": 4,
    "e": 3,
    "z": 10,
    "c": 11,
}
ORBIT_KEYS = {
    "q": 6,
    "e": 12,
    "z": 13,
    "c": 5,
}
ROTATION_KEYS = {
    "o": 14,
    "p": 7,
}


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


class BoardClient:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200) -> None:
        self.serial = serial.Serial(port, baudrate, timeout=0.1)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def _read_response(self, expected_prefixes: Tuple[str, ...], timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("ERR") or line.startswith("Traceback") or line.startswith("OK ") is False:
                print(f"<< {line}")
            if line in ("PING",) or line.startswith("GOAL "):
                continue
            if any(line.startswith(prefix) for prefix in expected_prefixes):
                return line
        return ""

    def ping(self, timeout_sec: float = 1.0) -> bool:
        print(">> PING")
        self.serial.reset_input_buffer()
        self.serial.write(b"PING\n")
        self.serial.flush()
        return bool(self._read_response(("PONG", "ERR"), timeout_sec))

    def wait_for_ping(self, total_timeout_sec: float = 8.0, retry_interval_sec: float = 0.25) -> bool:
        deadline = time.monotonic() + total_timeout_sec
        while time.monotonic() < deadline:
            if self.ping(timeout_sec=min(1.0, max(0.1, deadline - time.monotonic()))):
                return True
            time.sleep(retry_interval_sec)
        print("no ping response from board")
        return False

    def send_goal(self, joint_values: List[float]) -> bool:
        payload = " ".join(f"{value:.6f}" for value in joint_values)
        self.serial.write(f"GOAL {payload}\n".encode("utf-8"))
        self.serial.flush()
        return bool(self._read_response(("OK GOAL", "ERR"), 2.0))


class LiteController:
    PATH_TYPES = ("half1", "half2", "full")
    SWING_TYPES = ("half1", "half2", "full1", "full2")
    MOVING_TRAJECTORY_IDS = tuple(range(1, 15))
    TRIPODS = ("A", "B")
    STATIONARY_ID = 0

    def __init__(self) -> None:
        self.mode = "diagonal"
        self.board = BoardClient()
        self._init_state()
        self.running = True

    def _init_state(self) -> None:
        self.limit_radius = 0.04
        self.swing_height = 0.025
        self.sample_rate = 0.02
        self.min_angle = 1.0
        self.startup_z = 0.05
        self.startup_vel = 0.05
        self.L1 = 0.0385
        self.L2 = 0.0700
        self.L3 = 0.1020

        self.home_x = 0.110
        self.home_y = 0.000
        self.home_z = -0.050

        self.linear_speed_y = 0.12
        self.linear_speed_x = 0.12
        self.diagonal_speed = 0.12
        self.self_angular_speed = 0.75
        self.orbit_angular_speed = 0.75
        self.external_radius = 0.30

        self.legs: List[LegInfo] = [
            LegInfo(1, ("j11", "j12", "j13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("j21", "j22", "j23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("j31", "j32", "j33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("j41", "j42", "j43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("j51", "j52", "j53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("j61", "j62", "j63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]

        self._reset_template_stores()

        self.current_joint_goal = self._initial_joint_goal()
        self.requested_trajectory_id = self.STATIONARY_ID
        self.active_trajectory_id = self.STATIONARY_ID
        self.next_full_pull_tripod = "B"

    def _new_leg_store(self, type_names: Tuple[str, ...]):
        return {leg.leg_id: {type_name: [] for type_name in type_names} for leg in self.legs}

    def _reset_template_stores(self) -> None:
        self.tip_paths = {
            traj_id: self._new_leg_store(self.PATH_TYPES) for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.tip_swings = {
            traj_id: self._new_leg_store(self.SWING_TYPES) for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.joint_paths = {
            traj_id: self._new_leg_store(self.PATH_TYPES) for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.joint_swings = {
            traj_id: self._new_leg_store(self.SWING_TYPES) for traj_id in self.MOVING_TRAJECTORY_IDS
        }
        self.duration_points = {
            traj_id: {tripod: {"half": 0, "full": 0} for tripod in self.TRIPODS}
            for traj_id in self.MOVING_TRAJECTORY_IDS
        }

    def _poll_keyboard(self) -> None:
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return
        key = sys.stdin.read(1)
        if not key:
            return
        k = key.lower()
        if k == "x":
            self.running = False
            return
        if k == "m":
            self.mode = "orbit" if self.mode == "diagonal" else "diagonal"
            print(f"q/e/z/c mode: {self.mode}")
            return
        value = self.map_key(k)
        if value is not None:
            self.requested_trajectory_id = value
            print(f"trajectory -> {value}")

    def map_key(self, key: str) -> int | None:
        if key == "0":
            return 0
        if key in ROTATION_KEYS:
            return ROTATION_KEYS[key]
        if key in LINE_KEYS:
            return LINE_KEYS[key]
        table = DIAGONAL_KEYS if self.mode == "diagonal" else ORBIT_KEYS
        return table.get(key)

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

    def _initial_joint_goal(self) -> List[float]:
        neutral_tip = Point3D(self.home_x, self.home_y, self.home_z)
        values: List[float] = []
        for _ in self.legs:
            values.extend(self.IK(neutral_tip))
        return values

    def IK(self, tip: Point3D) -> Tuple[float, float, float]:
        y = tip.y
        x = tip.x
        z = tip.z
        j1 = -math.atan2(y, x)
        x_prime = math.sqrt(x * x + y * y) - self.L1
        d = math.sqrt(x_prime * x_prime + z * z)
        d = self._clamp(d, abs(self.L2 - self.L3), self.L2 + self.L3)
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
        return j1, alpha1 - alpha2, math.pi - math.acos(cos_knee)

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
        if 8 <= trajectory_type_id <= 14:
            return self.master_path(-t, trajectory_type_id - 7)
        if trajectory_type_id == 1:
            return BasePose2D(0.0, self.linear_speed_y * t, 0.0)
        if trajectory_type_id == 2:
            return BasePose2D(self.linear_speed_x * t, 0.0, 0.0)
        if trajectory_type_id == 3:
            return BasePose2D(self.diagonal_speed * t, self.diagonal_speed * t, 0.0)
        if trajectory_type_id == 4:
            return BasePose2D(-self.diagonal_speed * t, self.diagonal_speed * t, 0.0)
        if trajectory_type_id in (5, 6):
            center_x = self.external_radius if trajectory_type_id == 5 else -self.external_radius
            phi0 = math.pi if trajectory_type_id == 5 else 0.0
            phi = phi0 + self.orbit_angular_speed * t
            return BasePose2D(
                center_x + self.external_radius * math.cos(phi),
                self.external_radius * math.sin(phi),
                self.orbit_angular_speed * t,
            )
        if trajectory_type_id == 7:
            return BasePose2D(0.0, 0.0, -self.self_angular_speed * t)
        return BasePose2D(0.0, 0.0, 0.0)

    def pull_builder(self, tripod: str, trajectory_type_id: int, sign: int) -> Dict[int, List[Point3D]]:
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")
        selected_legs = self._tripod_legs(tripod)
        home_tip = Point3D(self.home_x, self.home_y, self.home_z)
        path_points = {leg.leg_id: [Point3D(home_tip.x, home_tip.y, home_tip.z)] for leg in selected_legs}
        current_tip = {leg.leg_id: Point3D(home_tip.x, home_tip.y, home_tip.z) for leg in selected_legs}
        start_xy = {leg.leg_id: (home_tip.x, home_tip.y) for leg in selected_legs}
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
                if math.sqrt((tip_next.x - sx) ** 2 + (tip_next.y - sy) ** 2) >= self.limit_radius:
                    hit_limit = True
            t_prev = t_curr
            base_prev = base_curr
            if hit_limit:
                break
        return path_points

    @staticmethod
    def _copy_point(point: Point3D) -> Point3D:
        return Point3D(point.x, point.y, point.z)

    def _resample_xy_path(self, path: List[Point3D], point_count: int) -> List[Point3D]:
        point_count = max(point_count, 2)
        if not path:
            home = Point3D(self.home_x, self.home_y, self.home_z)
            return [home, home]
        if len(path) == 1:
            return [self._copy_point(path[0]) for _ in range(point_count)]
        cumulative = [0.0]
        for idx in range(1, len(path)):
            cumulative.append(
                cumulative[-1] + math.sqrt((path[idx].x - path[idx - 1].x) ** 2 + (path[idx].y - path[idx - 1].y) ** 2)
            )
        total = cumulative[-1]
        if total <= 1e-12:
            start = path[0]
            end = path[-1]
            out: List[Point3D] = []
            for idx in range(point_count):
                s = float(idx) / float(point_count - 1)
                out.append(
                    Point3D(
                        start.x + s * (end.x - start.x),
                        start.y + s * (end.y - start.y),
                        start.z + s * (end.z - start.z),
                    )
                )
            return out
        out: List[Point3D] = []
        seg_idx = 0
        for idx in range(point_count):
            target = (float(idx) / float(point_count - 1)) * total
            while seg_idx < len(cumulative) - 2 and cumulative[seg_idx + 1] < target:
                seg_idx += 1
            seg_start = cumulative[seg_idx]
            seg_end = cumulative[seg_idx + 1]
            local_s = 0.0 if seg_end <= seg_start else (target - seg_start) / (seg_end - seg_start)
            p0 = path[seg_idx]
            p1 = path[seg_idx + 1]
            out.append(Point3D(
                p0.x + local_s * (p1.x - p0.x),
                p0.y + local_s * (p1.y - p0.y),
                p0.z + local_s * (p1.z - p0.z),
            ))
        return out

    def swing_builder(self, associated_path: List[Point3D], point_count: int) -> List[Point3D]:
        source = list(reversed(associated_path))
        if not source:
            source = [Point3D(self.home_x, self.home_y, self.home_z)]
        shadow = self._resample_xy_path(source, point_count)
        z_start = source[0].z
        z_end = source[-1].z
        out: List[Point3D] = []
        n = len(shadow)
        for idx, sample in enumerate(shadow):
            s = float(idx) / float(n - 1) if n > 1 else 0.0
            out.append(Point3D(sample.x, sample.y, (1.0 - s) * z_start + s * z_end + self.swing_height * math.sin(math.pi * s)))
        return out

    def _build_tripod_templates(self, trajectory_type_id: int, tripod: str) -> None:
        positive = self.pull_builder(tripod, trajectory_type_id, +1)
        negative = self.pull_builder(tripod, trajectory_type_id, -1)
        for leg in self._tripod_legs(tripod):
            leg_id = leg.leg_id
            positive_path = positive.get(leg_id, [Point3D(self.home_x, self.home_y, self.home_z)])
            negative_path = negative.get(leg_id, [Point3D(self.home_x, self.home_y, self.home_z)])
            half1 = list(reversed(negative_path))
            half2 = positive_path
            full = half1 + half2[1:]
            self.tip_paths[trajectory_type_id][leg_id]["half1"] = half1
            self.tip_paths[trajectory_type_id][leg_id]["half2"] = half2
            self.tip_paths[trajectory_type_id][leg_id]["full"] = full

    def _set_tripod_duration_points(self, trajectory_type_id: int, tripod: str) -> None:
        tripod_legs = self._tripod_legs(tripod)
        if not tripod_legs:
            return
        first_leg_id = tripod_legs[0].leg_id
        half_points = len(self.tip_paths[trajectory_type_id][first_leg_id]["half1"])
        full_points = len(self.tip_paths[trajectory_type_id][first_leg_id]["full"])
        self.duration_points[trajectory_type_id][tripod]["half"] = max(half_points, 2)
        self.duration_points[trajectory_type_id][tripod]["full"] = max(full_points, 2)

    @staticmethod
    def _split_swing_points(swing: List[Point3D], split_idx: int) -> Tuple[List[Point3D], List[Point3D]]:
        split_idx = max(1, min(split_idx, len(swing) - 1))
        return swing[:split_idx], swing[split_idx:]

    def _build_tripod_swings(self, trajectory_type_id: int, tripod: str) -> None:
        other = self._opposite_tripod(tripod)
        half_points = self.duration_points[trajectory_type_id][other]["half"]
        full_points = self.duration_points[trajectory_type_id][other]["full"]
        for leg in self._tripod_legs(tripod):
            leg_id = leg.leg_id
            half1_path = self.tip_paths[trajectory_type_id][leg_id]["half1"]
            half2_path = self.tip_paths[trajectory_type_id][leg_id]["half2"]
            full_path = self.tip_paths[trajectory_type_id][leg_id]["full"]
            self.tip_swings[trajectory_type_id][leg_id]["half1"] = self.swing_builder(half1_path, half_points)
            self.tip_swings[trajectory_type_id][leg_id]["half2"] = self.swing_builder(half2_path, half_points)
            full_swing = self.swing_builder(full_path, full_points)
            full2, full1 = self._split_swing_points(full_swing, half_points)
            self.tip_swings[trajectory_type_id][leg_id]["full2"] = full2
            self.tip_swings[trajectory_type_id][leg_id]["full1"] = full1

    def _build_all_templates(self) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for tripod in self.TRIPODS:
                self._build_tripod_templates(trajectory_type_id, tripod)
            for tripod in self.TRIPODS:
                self._set_tripod_duration_points(trajectory_type_id, tripod)
            for tripod in self.TRIPODS:
                self._build_tripod_swings(trajectory_type_id, tripod)

    def _convert_tip_store_to_joint_store(self, src, dst, type_names: Tuple[str, ...]) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for type_name in type_names:
                    dst[trajectory_type_id][leg_id][type_name] = [
                        self.IK(point) for point in src[trajectory_type_id][leg_id][type_name]
                    ]

    def p_to_joint_space(self) -> None:
        self._convert_tip_store_to_joint_store(self.tip_paths, self.joint_paths, self.PATH_TYPES)
        self._convert_tip_store_to_joint_store(self.tip_swings, self.joint_swings, self.SWING_TYPES)

    def _send_joint_goal(self, joint_values: List[float]) -> bool:
        if not self.board.send_goal(joint_values):
            print("goal send failed")
            return False
        time.sleep(self.sample_rate)
        self._poll_keyboard()
        return self.running

    def startup_(self) -> bool:
        startup_tip = Point3D(self.home_x, self.home_y, self.startup_z)
        startup_joint_values: List[float] = []
        for _ in self.legs:
            startup_joint_values.extend(self.IK(startup_tip))
        if not self._send_joint_goal(startup_joint_values):
            return False
        self.current_joint_goal = list(startup_joint_values)
        time.sleep(2.0)
        z_delta = self.home_z - self.startup_z
        if abs(z_delta) <= 1e-9:
            return True
        z_step = abs(self.startup_vel) * self.sample_rate
        step_count = max(1, math.ceil(abs(z_delta) / z_step))
        min_angle_rad = math.radians(self.min_angle)
        for step_idx in range(1, step_count + 1):
            alpha = step_idx / step_count
            tip = Point3D(self.home_x, self.home_y, self.startup_z + alpha * z_delta)
            desired_flat: List[float] = []
            for _ in self.legs:
                desired_flat.extend(self.IK(tip))
            changed = False
            for joint_idx, desired in enumerate(desired_flat):
                if abs(desired - self.current_joint_goal[joint_idx]) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired
                    changed = True
            if changed and not self._send_joint_goal(self.current_joint_goal):
                return False
        return True

    def _execute_joint_sequences(
        self,
        phase_name: str,
        leg_sequences: Dict[int, List[Tuple[float, float, float]]],
    ) -> bool:
        if not leg_sequences:
            print(f"{phase_name}: no sequences")
            return False
        for leg_id, seq in leg_sequences.items():
            if not seq:
                print(f"{phase_name}: empty sequence for leg {leg_id}")
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
                if abs(desired - self.current_joint_goal[joint_idx]) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired
            if not self._send_joint_goal(self.current_joint_goal):
                print(f"{phase_name}: failed at point {point_idx}")
                return False
        return True

    def _collect_phase_sequences(
        self,
        source_paths,
        source_swings,
        tripod_a_mode,
        tripod_b_mode,
    ):
        tripod_mode = {"A": tripod_a_mode, "B": tripod_b_mode}
        out = {}
        for leg in self.legs:
            mode_kind, path_type = tripod_mode[leg.tripod]
            store = source_paths if mode_kind == "path" else source_swings
            out[leg.leg_id] = store[leg.leg_id][path_type]
        return out

    def _execute_standard_phase(
        self,
        phase_name: str,
        trajectory_type_id: int,
        tripod_a_mode,
        tripod_b_mode,
    ) -> bool:
        leg_sequences = self._collect_phase_sequences(
            source_paths=self.joint_paths[trajectory_type_id],
            source_swings=self.joint_swings[trajectory_type_id],
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )
        return self._execute_joint_sequences(phase_name, leg_sequences)

    def _execute_phase_by_tripod(
        self,
        phase_name: str,
        trajectory_type_id: int,
        pull_tripod: str,
        pull_path_type: str,
        swing_type: str,
    ) -> bool:
        tripod_a_mode = ("path", pull_path_type) if pull_tripod == "A" else ("swing", swing_type)
        tripod_b_mode = ("path", pull_path_type) if pull_tripod == "B" else ("swing", swing_type)
        return self._execute_standard_phase(
            phase_name=phase_name,
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )

    def coordinator(self) -> bool:
        print("Building gait templates")
        self._build_all_templates()
        print("Converting templates to joint space")
        self.p_to_joint_space()
        print("Controller ready (stationary)")
        while self.running:
            self._poll_keyboard()
            if self.active_trajectory_id == self.STATIONARY_ID:
                if self.requested_trajectory_id not in self.MOVING_TRAJECTORY_IDS:
                    time.sleep(0.02)
                    continue
                self.active_trajectory_id = self.requested_trajectory_id
                print(f"starting trajectory {self.active_trajectory_id}")
                if not self._execute_phase_by_tripod(
                    phase_name=f"start half-step t{self.active_trajectory_id}",
                    trajectory_type_id=self.active_trajectory_id,
                    pull_tripod="A",
                    pull_path_type="half2",
                    swing_type="half1",
                ):
                    return False
                self.next_full_pull_tripod = "B"
                continue

            if self.requested_trajectory_id == self.STATIONARY_ID:
                print(f"stopping trajectory {self.active_trajectory_id}")
                if not self._execute_phase_by_tripod(
                    phase_name=f"final half-step t{self.active_trajectory_id}",
                    trajectory_type_id=self.active_trajectory_id,
                    pull_tripod=self.next_full_pull_tripod,
                    pull_path_type="half1",
                    swing_type="half2",
                ):
                    return False
                self.active_trajectory_id = self.STATIONARY_ID
                self.requested_trajectory_id = self.STATIONARY_ID
                continue

            pull_tripod = self.next_full_pull_tripod
            if not self._execute_phase_by_tripod(
                phase_name=f"full-step-1 t{self.active_trajectory_id}",
                trajectory_type_id=self.active_trajectory_id,
                pull_tripod=pull_tripod,
                pull_path_type="half1",
                swing_type="full2",
            ):
                return False
            requested_mid = self.requested_trajectory_id
            second_half_trajectory = self.active_trajectory_id
            if requested_mid in self.MOVING_TRAJECTORY_IDS and requested_mid != self.active_trajectory_id:
                second_half_trajectory = requested_mid
            if not self._execute_phase_by_tripod(
                phase_name=f"full-step-2 t{second_half_trajectory}",
                trajectory_type_id=second_half_trajectory,
                pull_tripod=pull_tripod,
                pull_path_type="half2",
                swing_type="full1",
            ):
                return False
            self.active_trajectory_id = second_half_trajectory
            self.next_full_pull_tripod = self._opposite_tripod(pull_tripod)
        return True

    def run(self) -> int:
        if not self.board.wait_for_ping():
            return 1
        if not self.startup_():
            return 1
        return 0 if self.coordinator() else 1


def print_help() -> None:
    print("Lite controller keyboard + gait controller")
    print("Stop: 0")
    print("Lines: w(+Y), d(+X), s(-Y), a(-X)")
    print("Self rotation: o(CCW), p(CW)")
    print("q/e/z/c mode toggle: press m (diagonal <-> orbit)")
    print("Diagonal mode q/e/z/c: (+Y,-X), (+Y,+X), (-Y,-X), (-Y,+X)")
    print("Orbit mode q/e/z/c: (-X center), reverse(+X), reverse(-X), (+X center)")
    print("Quit: x")


def main() -> None:
    controller = LiteController()
    old_settings = termios.tcgetattr(sys.stdin)
    print_help()
    exit_code = 0
    try:
        tty.setraw(sys.stdin.fileno())
        exit_code = controller.run()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        controller.board.close()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
