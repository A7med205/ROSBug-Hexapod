#!/usr/bin/env python3

import gc
import math
import sys
import time
import uselect

from servo import ServoCluster, servo2040


class FramePose:
    def __init__(self, x, y, theta_deg):
        self.x = x
        self.y = y
        self.theta_deg = theta_deg


class LegInfo:
    def __init__(self, leg_id, joint_names, frame_pose, tripod):
        self.leg_id = leg_id
        self.joint_names = joint_names
        self.frame_pose = frame_pose
        self.tripod = tripod


class BasePose2D:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class LocalDisplacement2D:
    def __init__(self, dx_local, dy_local):
        self.dx_local = dx_local
        self.dy_local = dy_local


class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class LiteGaitController:
    PATH_TYPES = ("half1", "half2", "full")
    SWING_TYPES = ("half1", "half2", "full1", "full2")
    MOVING_TRAJECTORY_IDS = tuple(range(1, 15))
    TRIPODS = ("A", "B")
    STATIONARY_ID = 0
    SERVO_COUNT = 18
    CENTER_PULSE = 1500.0
    MIN_PULSE = 500.0
    MAX_PULSE = 2500.0
    SERIAL_POLL_MS = 0

    def __init__(self) -> None:
        self.wait_timeout_sec = 10.0
        self.sample_rate = 0.02
        self._load_calibration()
        self._init_state()
        self._init_servo_cluster()
        self.serial_poll = uselect.poll()
        self.serial_poll.register(sys.stdin, uselect.POLLIN)

    def _log(self, message: str) -> None:
        print(message)

    def _load_calibration(self) -> None:
        namespace = {}
        candidate_paths = (
            "calibration.txt",
            "servo2040/gait2/calibration.txt",
        )
        for path in candidate_paths:
            try:
                with open(path, "r") as handle:
                    exec(handle.read(), {}, namespace)
                self.calibration = namespace["CALIBRATION"]
                self.robot_to_servo = namespace["ROBOT_TO_SERVO"]
                return
            except OSError:
                continue
        raise OSError("unable to load calibration.txt")

    def _init_servo_cluster(self) -> None:
        gc.collect()
        start_pin = servo2040.SERVO_1
        end_pin = getattr(servo2040, "SERVO_%d" % self.SERVO_COUNT)
        self.servos = ServoCluster(pio=0, sm=0, pins=list(range(start_pin, end_pin + 1)))
        self.servos.enable_all()

    def _init_state(self) -> None:
        self.limit_radius = 0.04
        self.swing_height = 0.025
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

        self.legs = [
            LegInfo(1, ("j11", "j12", "j13"), FramePose(-0.0535, 0.0900, 135.0), "A"),
            LegInfo(2, ("j21", "j22", "j23"), FramePose(-0.0700, 0.0000, 180.0), "B"),
            LegInfo(3, ("j31", "j32", "j33"), FramePose(-0.0535, -0.0900, -135.0), "A"),
            LegInfo(4, ("j41", "j42", "j43"), FramePose(0.0535, 0.0900, 45.0), "B"),
            LegInfo(5, ("j51", "j52", "j53"), FramePose(0.0700, 0.0000, 0.0), "A"),
            LegInfo(6, ("j61", "j62", "j63"), FramePose(0.0535, -0.0900, -45.0), "B"),
        ]
        self.joint_names_flat = [joint for leg in self.legs for joint in leg.joint_names]
        self.joint_to_channel = {joint: idx for idx, joint in enumerate(self.joint_names_flat)}

        self._reset_template_stores()
        self.current_joint_goal = self._initial_joint_goal()

        self.requested_trajectory_id = self.STATIONARY_ID
        self.active_trajectory_id = self.STATIONARY_ID
        self.next_full_pull_tripod = "B"

    def _new_leg_store(self, type_names):
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
            traj_id: {tripod: {"half": 0, "full": 0} for tripod in ("A", "B")}
            for traj_id in self.MOVING_TRAJECTORY_IDS
        }

    def _initial_joint_goal(self):
        neutral_tip = Point3D(self.home_x, self.home_y, self.home_z)
        values = []
        for _ in self.legs:
            values.extend(self.IK(neutral_tip))
        return values

    def _set_requested_trajectory(self, value) -> None:
        if value == self.STATIONARY_ID or value in self.MOVING_TRAJECTORY_IDS:
            self.requested_trajectory_id = value

    def _handle_serial_line(self, line: str) -> None:
        parts = line.strip().split()
        if not parts:
            return
        command = parts[0].upper()

        try:
            if command == "PING":
                print("PONG")
            elif command == "TRAJ":
                if len(parts) < 2:
                    print("ERR MISSING_TRAJ_ID")
                    return
                value = int(parts[1])
                if value != self.STATIONARY_ID and value not in self.MOVING_TRAJECTORY_IDS:
                    print("ERR BAD_TRAJ", value)
                    return
                self._set_requested_trajectory(value)
                print("OK TRAJ", value)
            else:
                print("ERR UNKNOWN_CMD")
        except Exception as exc:
            print("ERR", repr(exc))

    def _poll_serial_commands(self) -> None:
        while self.serial_poll.poll(self.SERIAL_POLL_MS):
            line = sys.stdin.readline()
            if not line:
                break
            self._handle_serial_line(line)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    @staticmethod
    def _deg_to_rad(deg):
        return deg * (math.pi / 180.0)

    @staticmethod
    def _rad_to_deg(rad):
        return rad * (180.0 / math.pi)

    @staticmethod
    def _opposite_tripod(tripod):
        return "B" if tripod == "A" else "A"

    def _tripod_legs(self, tripod):
        return [leg for leg in self.legs if leg.tripod == tripod]

    def robot_to_servo_deg(self, joint_name, robot_angle_rad):
        slope, intercept = self.robot_to_servo[joint_name]
        return slope * self._rad_to_deg(robot_angle_rad) + intercept

    def servo_deg_to_pulse(self, joint_name, servo_deg):
        slope_positive, slope_negative = self.calibration[joint_name]
        if servo_deg >= 0.0:
            return self.CENTER_PULSE + slope_positive * servo_deg
        return self.CENTER_PULSE + slope_negative * servo_deg

    def robot_angle_to_pulse(self, joint_name, robot_angle_rad):
        servo_deg = self.robot_to_servo_deg(joint_name, robot_angle_rad)
        pulse = self.servo_deg_to_pulse(joint_name, servo_deg)
        return self._clamp(pulse, self.MIN_PULSE, self.MAX_PULSE)

    # Convert one leg tip in the local leg frame to joint angles (radians).
    def IK(self, tip):
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

    # Convert base motion delta into local tip delta for a stance-locked foot.
    def base_delta_to_tip_delta(self, base1, base2, leg, tip_local_1):
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

    # Sample the commanded base pose polynomial for each trajectory type.
    def master_path(self, t, trajectory_type_id):
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
            x = center_x + self.external_radius * math.cos(phi)
            y = self.external_radius * math.sin(phi)
            return BasePose2D(x, y, self.orbit_angular_speed * t)
        if trajectory_type_id == 7:
            return BasePose2D(0.0, 0.0, -self.self_angular_speed * t)
        return BasePose2D(0.0, 0.0, 0.0)

    # Build local pull paths by integrating base deltas until any leg hits radius limit.
    def pull_builder(self, tripod, trajectory_type_id, sign):
        if sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")

        selected_legs = self._tripod_legs(tripod)
        home_tip = Point3D(self.home_x, self.home_y, self.home_z)

        path_points = {
            leg.leg_id: [Point3D(home_tip.x, home_tip.y, home_tip.z)]
            for leg in selected_legs
        }
        current_tip = {
            leg.leg_id: Point3D(home_tip.x, home_tip.y, home_tip.z)
            for leg in selected_legs
        }
        start_xy = {
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

    @staticmethod
    def _copy_point(point):
        return Point3D(point.x, point.y, point.z)

    def _resample_xy_path(self, path, point_count):
        point_count = max(point_count, 2)
        if not path:
            home = Point3D(self.home_x, self.home_y, self.home_z)
            return [home, home]
        if len(path) == 1:
            return [self._copy_point(path[0]) for _ in range(point_count)]

        cumulative = [0.0]
        for idx in range(1, len(path)):
            step = math.hypot(path[idx].x - path[idx - 1].x, path[idx].y - path[idx - 1].y)
            cumulative.append(cumulative[-1] + step)
        total = cumulative[-1]

        if total <= 1e-12:
            start = path[0]
            end = path[-1]
            out = []
            for idx in range(point_count):
                s = float(idx) / float(point_count - 1)
                out.append(
                    Point3D(
                        x=start.x + s * (end.x - start.x),
                        y=start.y + s * (end.y - start.y),
                        z=start.z + s * (end.z - start.z),
                    )
                )
            return out

        out = []
        seg_idx = 0
        for idx in range(point_count):
            target = (float(idx) / float(point_count - 1)) * total
            while seg_idx < len(cumulative) - 2 and cumulative[seg_idx + 1] < target:
                seg_idx += 1
            seg_start = cumulative[seg_idx]
            seg_end = cumulative[seg_idx + 1]
            if seg_end <= seg_start:
                local_s = 0.0
            else:
                local_s = (target - seg_start) / (seg_end - seg_start)
            p0 = path[seg_idx]
            p1 = path[seg_idx + 1]
            out.append(
                Point3D(
                    x=p0.x + local_s * (p1.x - p0.x),
                    y=p0.y + local_s * (p1.y - p0.y),
                    z=p0.z + local_s * (p1.z - p0.z),
                )
            )
        return out

    # Build swing points over reversed path x/y shadow with sinusoidal z lift.
    def swing_builder(self, associated_path, point_count):
        source = list(reversed(associated_path))
        if not source:
            source = [Point3D(self.home_x, self.home_y, self.home_z)]
        shadow = self._resample_xy_path(source, point_count)
        z_start = source[0].z
        z_end = source[-1].z

        out = []
        n = len(shadow)
        for idx, sample in enumerate(shadow):
            s = float(idx) / float(n - 1) if n > 1 else 0.0
            out.append(
                Point3D(
                    x=sample.x,
                    y=sample.y,
                    z=(1.0 - s) * z_start + s * z_end + self.swing_height * math.sin(math.pi * s),
                )
            )
        return out

    def _build_tripod_templates(self, trajectory_type_id, tripod) -> None:
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

    def _set_tripod_duration_points(self, trajectory_type_id, tripod) -> None:
        tripod_legs = self._tripod_legs(tripod)
        if not tripod_legs:
            return
        first_leg_id = tripod_legs[0].leg_id
        half_points = len(self.tip_paths[trajectory_type_id][first_leg_id]["half1"])
        full_points = len(self.tip_paths[trajectory_type_id][first_leg_id]["full"])
        half_points = max(half_points, 2)
        full_points = max(full_points, 2)
        self.duration_points[trajectory_type_id][tripod]["half"] = half_points
        self.duration_points[trajectory_type_id][tripod]["full"] = full_points

    @staticmethod
    def _split_swing_points(swing, split_idx):
        split_idx = max(1, min(split_idx, len(swing) - 1))
        return swing[:split_idx], swing[split_idx:]

    def _build_tripod_swings(self, trajectory_type_id, tripod) -> None:
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

    # Precompute all path/swing templates for all supported trajectory types.
    def _build_all_templates(self) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for tripod in self.TRIPODS:
                self._build_tripod_templates(trajectory_type_id, tripod)
            for tripod in self.TRIPODS:
                self._set_tripod_duration_points(trajectory_type_id, tripod)
            for tripod in self.TRIPODS:
                self._build_tripod_swings(trajectory_type_id, tripod)

    def _convert_tip_store_to_joint_store(self, src, dst, type_names) -> None:
        for trajectory_type_id in self.MOVING_TRAJECTORY_IDS:
            for leg in self.legs:
                leg_id = leg.leg_id
                for type_name in type_names:
                    dst[trajectory_type_id][leg_id][type_name] = [
                        self.IK(point) for point in src[trajectory_type_id][leg_id][type_name]
                    ]

    # Convert precomputed cartesian templates into joint-space templates.
    def p_to_joint_space(self) -> None:
        self._convert_tip_store_to_joint_store(self.tip_paths, self.joint_paths, self.PATH_TYPES)
        self._convert_tip_store_to_joint_store(self.tip_swings, self.joint_swings, self.SWING_TYPES)

    def _joint_values_to_pulses(self, joint_values):
        pulses = [0.0] * self.SERVO_COUNT
        for idx, joint_name in enumerate(self.joint_names_flat):
            channel = self.joint_to_channel[joint_name]
            pulses[channel] = self.robot_angle_to_pulse(joint_name, joint_values[idx])
        return pulses

    def _apply_pulses(self, pulses) -> None:
        for channel, pulse in enumerate(pulses):
            if pulse <= 0.0:
                self.servos.disable(channel)
            else:
                self.servos.pulse(channel, pulse)

    # Send one single-point servo goal, then wait instead of using time_from_start.
    def _send_joint_goal(self, joint_values) -> bool:
        try:
            self._apply_pulses(self._joint_values_to_pulses(joint_values))
            self._poll_serial_commands()
            time.sleep(self.sample_rate)
            return True
        except Exception as exc:
            self._log("servo goal failed: %r" % (exc,))
            return False

    def startup_(self) -> bool:
        startup_tip = Point3D(self.home_x, self.home_y, self.startup_z)
        startup_joint_values = []
        for _ in self.legs:
            startup_joint_values.extend(self.IK(startup_tip))

        if not self._send_joint_goal(startup_joint_values):
            self._log("startup_: failed to send startup pose")
            return False
        self.current_joint_goal = list(startup_joint_values)
        time.sleep(2.0)

        z_delta = self.home_z - self.startup_z
        if math.isclose(z_delta, 0.0, abs_tol=1e-9):
            return True

        startup_speed = abs(self.startup_vel)
        if startup_speed <= 0.0:
            self._log("startup_: startup_vel must be positive")
            return False

        z_step = startup_speed * self.sample_rate
        step_count = max(1, math.ceil(abs(z_delta) / z_step))
        min_angle_rad = math.radians(self.min_angle)

        for step_idx in range(1, step_count + 1):
            alpha = float(step_idx) / float(step_count)
            z = self.startup_z + alpha * z_delta
            tip = Point3D(self.home_x, self.home_y, z)

            desired_flat = []
            for _ in self.legs:
                desired_flat.extend(self.IK(tip))

            changed = False
            for joint_idx, desired in enumerate(desired_flat):
                current = self.current_joint_goal[joint_idx]
                if (not math.isfinite(current)) or abs(desired - current) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired
                    changed = True

            if changed and not self._send_joint_goal(self.current_joint_goal):
                self._log("startup_: failed during interpolation step %d" % step_idx)
                return False

        home_tip = Point3D(self.home_x, self.home_y, self.home_z)
        home_joint_values = []
        for _ in self.legs:
            home_joint_values.extend(self.IK(home_tip))

        if any(
            (not math.isfinite(current)) or abs(desired - current) > 1e-9
            for current, desired in zip(self.current_joint_goal, home_joint_values)
        ):
            if not self._send_joint_goal(home_joint_values):
                self._log("startup_: failed to send final home pose")
                return False
            self.current_joint_goal = list(home_joint_values)

        return True

    # Execute one phase point-by-point with 1-degree joint update gating.
    def _execute_joint_sequences(self, phase_name, leg_sequences) -> bool:
        if not leg_sequences:
            self._log("%s: no sequences" % phase_name)
            return False
        for leg_id, seq in leg_sequences.items():
            if not seq:
                self._log("%s: empty sequence for leg %d" % (phase_name, leg_id))
                return False

        phase_points = max(len(seq) for seq in leg_sequences.values())
        min_angle_rad = math.radians(self.min_angle)

        for point_idx in range(phase_points):
            self._poll_serial_commands()
            desired_flat = []
            for leg in self.legs:
                seq = leg_sequences[leg.leg_id]
                sample = seq[point_idx] if point_idx < len(seq) else seq[-1]
                desired_flat.extend(sample)

            for joint_idx, desired in enumerate(desired_flat):
                current = self.current_joint_goal[joint_idx]
                if (not math.isfinite(current)) or abs(desired - current) >= min_angle_rad:
                    self.current_joint_goal[joint_idx] = desired

            if not self._send_joint_goal(self.current_joint_goal):
                self._log("%s: failed at point %d" % (phase_name, point_idx))
                return False

        return True

    def _collect_phase_sequences(self, source_paths, source_swings, tripod_a_mode, tripod_b_mode):
        tripod_mode = {"A": tripod_a_mode, "B": tripod_b_mode}
        out = {}
        for leg in self.legs:
            mode_kind, path_type = tripod_mode[leg.tripod]
            store = source_paths if mode_kind == "path" else source_swings
            out[leg.leg_id] = store[leg.leg_id][path_type]
        return out

    def _execute_standard_phase(self, phase_name, trajectory_type_id, tripod_a_mode, tripod_b_mode) -> bool:
        leg_sequences = self._collect_phase_sequences(
            source_paths=self.joint_paths[trajectory_type_id],
            source_swings=self.joint_swings[trajectory_type_id],
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )
        return self._execute_joint_sequences(phase_name, leg_sequences)

    # Execute one pull/swing phase by selecting modes for tripod A and B.
    def _execute_phase_by_tripod(self, phase_name, trajectory_type_id, pull_tripod, pull_path_type, swing_type):
        tripod_a_mode = ("path", pull_path_type) if pull_tripod == "A" else ("swing", swing_type)
        tripod_b_mode = ("path", pull_path_type) if pull_tripod == "B" else ("swing", swing_type)
        return self._execute_standard_phase(
            phase_name=phase_name,
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )

    # Main gait state machine: stationary/start/full(mid-switch)/final-stop.
    def coordinator(self) -> bool:
        self._log("Building gait templates")
        self._build_all_templates()
        self._log("Converting templates to joint space")
        self.p_to_joint_space()
        self._log("Controller ready (stationary)")
        print("READY")

        while True:
            self._poll_serial_commands()

            if self.active_trajectory_id == self.STATIONARY_ID:
                if self.requested_trajectory_id not in self.MOVING_TRAJECTORY_IDS:
                    time.sleep(0.02)
                    continue
                self.active_trajectory_id = self.requested_trajectory_id
                self._log("Starting trajectory type %d" % self.active_trajectory_id)
                if not self._execute_phase_by_tripod(
                    phase_name="start half-step t%d" % self.active_trajectory_id,
                    trajectory_type_id=self.active_trajectory_id,
                    pull_tripod="A",
                    pull_path_type="half2",
                    swing_type="half1",
                ):
                    return False
                self.next_full_pull_tripod = "B"
                continue

            if self.requested_trajectory_id == self.STATIONARY_ID:
                final_pull_tripod = self.next_full_pull_tripod
                if not self._execute_phase_by_tripod(
                    phase_name="final half-step t%d" % self.active_trajectory_id,
                    trajectory_type_id=self.active_trajectory_id,
                    pull_tripod=final_pull_tripod,
                    pull_path_type="half1",
                    swing_type="half2",
                ):
                    return False
                self.active_trajectory_id = self.STATIONARY_ID
                self.requested_trajectory_id = self.STATIONARY_ID
                self._log("Entered stationary mode")
                continue

            pull_tripod = self.next_full_pull_tripod
            if not self._execute_phase_by_tripod(
                phase_name="full-step-1 t%d" % self.active_trajectory_id,
                trajectory_type_id=self.active_trajectory_id,
                pull_tripod=pull_tripod,
                pull_path_type="half1",
                swing_type="full2",
            ):
                return False

            requested_mid = self.requested_trajectory_id
            second_half_trajectory = self.active_trajectory_id
            if (
                requested_mid in self.MOVING_TRAJECTORY_IDS
                and requested_mid != self.active_trajectory_id
            ):
                self._log("Transition %d -> %d" % (self.active_trajectory_id, requested_mid))
                second_half_trajectory = requested_mid

            if not self._execute_phase_by_tripod(
                phase_name="full-step-2 t%d" % second_half_trajectory,
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
        if not self.startup_():
            return 1
        return 0 if self.coordinator() else 1


def main() -> None:
    controller = LiteGaitController()
    exit_code = 0
    try:
        exit_code = controller.run()
    except KeyboardInterrupt:
        exit_code = 130
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
