import time
import sys
import uselect
import gc
import math
from servo import ServoCluster, servo2040

# =========================
# Constants / Configuration
# =========================

# Servo board
SERVO_COUNT = 18  # Servo 2040 has 18 channels

# Kinematics and gait timing (from gait_controller.cpp)
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
DISCRETIZATION_TIME_STEP = 0.01  # 10ms
MIN_ANGLE_CHANGE_RAD = 1.0 * DEG_TO_RAD  # threshold to skip tiny updates

# Robot link lengths (meters)
LEG_L1 = 0.0385  # Coxa
LEG_L2 = 0.0700  # Femur
LEG_L3 = 0.1020  # Tibia

# Home posture
Z_HOME = -0.050  # meters (down is negative)
Y_HOME = 0.110   # meters (radius from body center)

# Direction handling: offsets applied to each leg's rotation (degrees)
DIRECTION_OFFSETS_DEG = {
    "w": 0.0,
    "d": 90.0,
    "s": 180.0,
    "a": -90.0,
}

# Joint order on channels: j11..j13, j21..j23, ..., j61..j63
JOINT_ORDER = [
    "j11", "j21", "j31",
    "j41", "j51", "j61",
    "j12", "j22", "j32",
    "j42", "j52", "j62",
    "j13", "j23", "j33",
    "j43", "j53", "j63",
]
 
# Map each joint key to its hardware channel index according to JOINT_ORDER
JOINT_TO_CHANNEL = {key: idx for idx, key in enumerate(JOINT_ORDER)}
assert len(JOINT_ORDER) == SERVO_COUNT, "JOINT_ORDER length must match SERVO_COUNT"

# Leg configuration (mirrors gait_controller.cpp; angles in radians, tripod membership)
LEG_CONFIGS = [
    {"leg_id": 1, "joint_keys": ["j11", "j12", "j13"], "rotation_angle": 135.0 * DEG_TO_RAD,  "is_tripod_a": True},
    {"leg_id": 2, "joint_keys": ["j21", "j22", "j23"], "rotation_angle": 180.0 * DEG_TO_RAD,  "is_tripod_a": False},
    {"leg_id": 3, "joint_keys": ["j31", "j32", "j33"], "rotation_angle": -135.0 * DEG_TO_RAD, "is_tripod_a": True},
    {"leg_id": 4, "joint_keys": ["j41", "j42", "j43"], "rotation_angle": 45.0 * DEG_TO_RAD,   "is_tripod_a": False},
    {"leg_id": 5, "joint_keys": ["j51", "j52", "j53"], "rotation_angle": 0.0 * DEG_TO_RAD,    "is_tripod_a": True},
    {"leg_id": 6, "joint_keys": ["j61", "j62", "j63"], "rotation_angle": -45.0 * DEG_TO_RAD,  "is_tripod_a": False},
]

# Angle-to-pulse calibration (µs/deg), asymmetric slopes per joint; Center = 1500us
CENTER_PULSE = 1500.0
CALIBRATION = {
    "j11": (10.6667, 10.8889),
    "j12": (10.4444, 11.7778),
    "j13": (10.4444, 11.3333),
    "j21": (10.3333, 11.3333),
    "j22": (10.5556, 12.0000),
    "j23": (11.1667, 11.1667),
    "j31": (11.1111, 10.6667),
    "j32": (11.1111, 10.8889),
    "j33": (11.0000, 10.6667),
    "j41": (10.5556, 10.8889),
    "j42": (11.1111, 11.0000),
    "j43": (10.7778, 11.1111),
    "j51": (12.0000, 11.1111),
    "j52": (10.6667, 11.1111),
    "j53": (10.6667, 10.8889),
    "j61": (10.8889, 11.1111),
    "j62": (11.5556, 10.4444),
    "j63": (11.0000, 10.5556),
}

# Robot-to-servo angle mapping (degrees) per robot_to_servo.txt
# model: servo_deg = s * robot_deg + b
ROBOT_TO_SERVO = {
    "j11": ( 1.0,    0.0),
    "j12": (-1.0,  -78.0),
    "j13": ( 1.0, -121.0),

    "j21": ( 1.0,    0.0),
    "j22": (-1.0,  -69.0),
    "j23": ( 1.0, -114.0),

    "j31": ( 1.0,    0.0),
    "j32": (-1.0,  -79.0),
    "j33": ( 1.0, -121.0),

    "j41": ( 1.0,    0.0),
    "j42": (-1.0,  -74.0),
    "j43": ( 1.0, -118.0),

    "j51": ( 1.0,    0.0),
    "j52": (-1.0,  -67.0),
    "j53": ( 1.0, -112.0),

    "j61": ( 1.0,    0.0),
    "j62": (-1.0,  -78.0),
    "j63": ( 1.0, -121.0),
}

# =========================
# Utility / Math
# =========================

def clamp(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

def angle_to_pulse_us(joint_key: str, theta_deg: float) -> float:
    """Convert servo angle (deg in servo frame) to pulse (µs)."""
    m_plus, m_minus = CALIBRATION[joint_key]
    if theta_deg >= 0.0:
        return CENTER_PULSE + m_plus * theta_deg
    else:
        return CENTER_PULSE + m_minus * theta_deg  # theta negative

def robot_to_servo_deg(joint_key: str, theta_robot_deg: float) -> float:
    """Map robot frame angle (deg) to servo frame (deg) using per-joint affine map."""
    s, b = ROBOT_TO_SERVO[joint_key]
    return s * theta_robot_deg + b

def rotate_coordinates(x_c, y_c, x_in, y_in, angle_rad):
    """Rotate point (x_in, y_in) around center (x_c, y_c) by angle (rad)."""
    x_t = x_in - x_c
    y_t = y_in - y_c
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    x_r = x_t * cos_a - y_t * sin_a
    y_r = x_t * sin_a + y_t * cos_a
    return x_r + x_c, y_r + y_c

def compute_trapezoidal_pos(t, T, L_total):
    """L(t) for a 1/3-1/3-1/3 trapezoidal velocity profile."""
    if T <= 0.0:
        return L_total if t > 0.0 else 0.0
    ta = T / 3.0
    vmax = 1.5 * L_total / T
    a = vmax / ta
    if t <= 0.0:
        return 0.0
    if t < ta:
        return 0.5 * a * t * t
    elif t < 2.0 * ta:
        return 0.5 * a * ta * ta + vmax * (t - ta)
    elif t <= T:
        return L_total - 0.5 * a * (T - t) * (T - t)
    else:
        return L_total

def compute_leg_position(x_start, x_end, t, duration, is_swing, step_height):
    """Compute (x_global, z_global) for swing/pull trajectories."""
    if is_swing:
        # Swing along arc with height
        C = abs(x_end - x_start)
        h = step_height
        if C < 1e-6 or h < 1e-6:
            L_total = abs(x_end - x_start)
            L = compute_trapezoidal_pos(t, duration, L_total)
            x_g = x_start + (x_end - x_start) * (L / (L_total if L_total > 1e-9 else 1.0))
            return x_g, Z_HOME
        R = (h * h + (C / 2.0) * (C / 2.0)) / (2.0 * h)
        theta = 2.0 * math.asin(C / (2.0 * R))
        Lt = R * theta
        x_center = (x_start + x_end) / 2.0
        L = compute_trapezoidal_pos(t, duration, Lt)
        phi = -theta / 2.0 + L / R
        x_local = R * math.sin(phi)
        z_local = math.sqrt(max(R * R - x_local * x_local, 0.0)) - math.sqrt(max(R * R - (C/2.0)*(C/2.0), 0.0))
        return x_center + x_local, Z_HOME + z_local
    else:
        # Pull: straight line at ground height
        L_total = abs(x_end - x_start)
        L = compute_trapezoidal_pos(t, duration, L_total)
        x_g = x_start + (x_end - x_start) * (L / (L_total if L_total > 1e-9 else 1.0))
        return x_g, Z_HOME

def IK(L1, L2, L3, X, Y, Z):
    """Inverse kinematics returning (J1,J2,J3) in radians."""
    J1 = math.atan2(Y, X) - math.pi / 2.0
    x_prime = math.sqrt(X * X + Y * Y) - L1
    D = math.sqrt(x_prime * x_prime + Z * Z)
    # Clamp for reachable workspace
    D = min(D, L2 + L3)
    D = max(D, abs(L2 - L3))
    alpha1 = math.atan2(-Z, x_prime)
    # Guard domain for acos
    c2 = (L2 * L2 + D * D - L3 * L3) / (2.0 * L2 * D)
    c2 = clamp(c2, -1.0, 1.0)
    alpha2 = math.acos(c2)
    J2 = alpha1 - alpha2
    c3 = (L2 * L2 + L3 * L3 - D * D) / (2.0 * L2 * L3)
    c3 = clamp(c3, -1.0, 1.0)
    J3 = math.pi - math.acos(c3)
    return J1, J2, J3

def joints_to_pulses(joint_angles_rad_per_leg, direction_offset_rad):
    """Convert per-leg joint angles (rad) to a flat list of pulses for 18 channels."""
    pulses = [0.0] * SERVO_COUNT
    # Flatten in JOINT_ORDER order
    for leg_idx, leg in enumerate(LEG_CONFIGS):
        # Joint order for this leg
        jkeys = leg["joint_keys"]
        # Grab computed angles
        j1, j2, j3 = joint_angles_rad_per_leg[leg_idx]
        # Convert to degrees in robot frame
        j_deg_robot = [j1 * RAD_TO_DEG, j2 * RAD_TO_DEG, j3 * RAD_TO_DEG]
        # Map each to servo frame and to pulses
        for k in range(3):
            jkey = jkeys[k]
            servo_deg = robot_to_servo_deg(jkey, j_deg_robot[k])
            pulse = angle_to_pulse_us(jkey, servo_deg)
            # Conservative clamp
            pulse = clamp(pulse, 500.0, 2500.0)
            ch = JOINT_TO_CHANNEL[jkey]
            pulses[ch] = pulse
    return pulses

def set_all_pulses(pulses):
    """Apply a full set of 18 pulses to the servo cluster."""
    for i, p in enumerate(pulses):
        if p == 0:
            servos.disable(i)
        else:
            servos.pulse(i, p)

def compute_leg_angles_for_xyz(x_global, y_global, z_global, direction_offset_rad):
    """Compute angles (J1,J2,J3) for all legs at a single tip pose (x_global, y_global, z_global)."""
    out = []
    for leg in LEG_CONFIGS:
        eff_rot = leg["rotation_angle"] + direction_offset_rad
        # Rotate tip coordinates for this leg about (0, Y_HOME)
        x_rot, y_rot = rotate_coordinates(0.0, Y_HOME, x_global, Y_HOME, eff_rot)
        # IK for this leg
        j1, j2, j3 = IK(LEG_L1, LEG_L2, LEG_L3, x_rot, y_rot, z_global)
        out.append((j1, j2, j3))
    return out

def set_all_to_xyz(x_global, y_global, z_global, direction_offset_rad):
    """Compute and set pulses for all legs at the same tip coordinates."""
    angles = compute_leg_angles_for_xyz(x_global, y_global, z_global, direction_offset_rad)
    pulses = joints_to_pulses(angles, direction_offset_rad)
    set_all_pulses(pulses)

def execute_coordinated_phase(x_a_start, x_a_end, x_b_start, x_b_end, duration_s, tripod_a_swing, tripod_b_swing, step_height, direction_offset_rad):
    """Run one coordinated phase (A pulls/B swings or vice-versa) in real time."""
    last_angles = [(float("inf"), float("inf"), float("inf"))] * len(LEG_CONFIGS)
    t = 0.0
    start_time = time.ticks_ms()
    # Loop with ~10ms discretization
    while t <= duration_s + 1e-9:
        any_change = False
        leg_angles = []
        for leg in LEG_CONFIGS:
            if leg["is_tripod_a"]:
                x_start = x_a_start
                x_end = x_a_end
                is_swing = tripod_a_swing
            else:
                x_start = x_b_start
                x_end = x_b_end
                is_swing = tripod_b_swing
            x_g, z_g = compute_leg_position(x_start, x_end, t, duration_s, is_swing, step_height)
            # Rotate about (0, Y_HOME) using effective leg rotation (+ direction offset applied later in FK)
            eff_rot = leg["rotation_angle"] + direction_offset_rad
            x_rot, y_rot = rotate_coordinates(0.0, Y_HOME, x_g, Y_HOME, eff_rot)
            j1, j2, j3 = IK(LEG_L1, LEG_L2, LEG_L3, x_rot, y_rot, z_g)
            idx = leg["leg_id"] - 1
            la = last_angles[idx]
            # Detect meaningful changes
            if (abs(j1 - la[0]) > MIN_ANGLE_CHANGE_RAD or
                abs(j2 - la[1]) > MIN_ANGLE_CHANGE_RAD or
                abs(j3 - la[2]) > MIN_ANGLE_CHANGE_RAD):
                any_change = True
                last_angles[idx] = (j1, j2, j3)
            leg_angles.append((j1, j2, j3))

        if any_change:
            pulses = joints_to_pulses(leg_angles, direction_offset_rad)
            set_all_pulses(pulses)

        # Maintain rate
        t += DISCRETIZATION_TIME_STEP
        # Sleep based on wall time to approximate 10ms cadence
        # Avoid cumulative drift: compute target elapsed and sleep remaining
        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_time)
        target_ms = int(t * 1000.0)
        sleep_ms = target_ms - elapsed_ms
        if sleep_ms > 0:
            time.sleep(sleep_ms / 1000.0)

def execute_full_step(step_length, step_height, step_duration, direction_char):
    """Execute one full coordinated step (two phases) given L/H/T and direction (w/a/s/d)."""
    dir_char = direction_char.lower()
    if dir_char not in DIRECTION_OFFSETS_DEG:
        dir_char = "w"
    direction_offset_rad = DIRECTION_OFFSETS_DEG[dir_char] * DEG_TO_RAD

    # Define start/end x for each tripod
    x_home = 0.0
    x_fwd = x_home + step_length / 2.0
    x_back = x_home - step_length / 2.0

    # Phase 1: Tripod A pulls backward, Tripod B swings forward
    execute_coordinated_phase(x_fwd, x_back, x_back, x_fwd, step_duration, False, True, step_height, direction_offset_rad)
    # Phase 2: Tripod A swings forward, Tripod B pulls backward
    execute_coordinated_phase(x_back, x_fwd, x_fwd, x_back, step_duration, True, False, step_height, direction_offset_rad)

def execute_half_step_to_home(step_length, step_height, step_duration, direction_char):
    """
    Execute a 'final half-step to home' as in gait_controller.cpp:
    - Tripod A: pull from x_forward -> x_home (not swing)
    - Tripod B: swing from x_backward -> x_home (swing)
    Uses 0.75 * step_duration for timing, and applies direction offset.
    """
    dir_char = direction_char.lower()
    if dir_char not in DIRECTION_OFFSETS_DEG:
        dir_char = "w"
    direction_offset_rad = DIRECTION_OFFSETS_DEG[dir_char] * DEG_TO_RAD

    x_home = 0.0
    x_fwd = x_home + step_length / 2.0
    x_back = x_home - step_length / 2.0
    half_T = 0.75 * step_duration

    # Final half-step to home: A pulls, B swings
    execute_coordinated_phase(x_fwd, x_home, x_back, x_home, half_T, False, True, step_height, direction_offset_rad)

# =========================
# Hardware init
# =========================

# Free up hardware resources before creating the ServoCluster
gc.collect()

# Create a servo cluster covering pins SERVO_1 .. SERVO_18
START_PIN = servo2040.SERVO_1
END_PIN = getattr(servo2040, f"SERVO_{SERVO_COUNT}")
servos = ServoCluster(pio=0, sm=0, pins=list(range(START_PIN, END_PIN + 1)))

# Enable all servos
servos.enable_all()

# =========================
# Startup sequence
# =========================

def startup_sequence():
    # Direction offset none at startup
    direction_offset_rad = 0.0
    # 1) Touch ground: X=0, Y=Y_HOME, Z=0
    set_all_to_xyz(0.0, Y_HOME, 0.0, direction_offset_rad)
    time.sleep(0.5)
    # 2) Stand-up: X=0, Y=Y_HOME, Z=Z_HOME
    set_all_to_xyz(0.0, Y_HOME, Z_HOME, direction_offset_rad)
    time.sleep(0.5)

print("READY")  # Host can wait for this
# Run startup positioning
startup_sequence()

# =========================
# Command loop
# =========================

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

while True:
    if poll.poll(0):
        line = sys.stdin.readline()
        if not line:
            continue
        line = line.strip()
        if not line:
            continue

        parts = line.split()
        cmd = parts[0].upper()

        try:
            if cmd == 'PING':
                print("PONG")

            elif cmd == 'STEP':
                # STEP length height duration direction
                if len(parts) < 5:
                    print("ERR MISSING_ARGS")
                else:
                    try:
                        step_len = float(parts[1])
                        step_hgt = float(parts[2])
                        step_dur = float(parts[3])
                        step_dir = parts[4].lower()
                        if step_dir not in DIRECTION_OFFSETS_DEG:
                            print("ERR BAD_DIR", step_dir)
                        else:
                            print("OK STEP_START", step_len, step_hgt, step_dur, step_dir)
                            execute_full_step(step_len, step_hgt, step_dur, step_dir)
                            print("OK STEP_DONE")
                    except Exception as e:
                        print("ERR BAD_STEP_ARGS", repr(e))

            elif cmd == 'HALF':
                # HALF length height duration direction  -> half-step to home (center) using given dir
                if len(parts) < 5:
                    print("ERR MISSING_ARGS")
                else:
                    try:
                        step_len = float(parts[1])
                        step_hgt = float(parts[2])
                        step_dur = float(parts[3])
                        step_dir = parts[4].lower()
                        if step_dir not in DIRECTION_OFFSETS_DEG:
                            print("ERR BAD_DIR", step_dir)
                        else:
                            print("OK HALF_START", step_len, step_hgt, step_dur, step_dir)
                            execute_half_step_to_home(step_len, step_hgt, step_dur, step_dir)
                            print("OK HALF_DONE")
                    except Exception as e:
                        print("ERR BAD_HALF_ARGS", repr(e))

            elif cmd in ('PULSE', 'P'):
                # PULSE ch us
                if len(parts) < 3:
                    print("ERR MISSING_ARGS")
                else:
                    ch = int(parts[1])
                    if not (1 <= ch <= SERVO_COUNT):
                        print("ERR BAD_CH", ch)
                    else:
                        p = float(parts[2])
                        if p == 0:
                            servos.disable(ch - 1)  # disable one channel
                        else:
                            servos.pulse(ch - 1, p)
                        print("OK PULSE", ch, p)

            elif cmd in ('PULSES', 'PS'):
                # PULSES u1 u2 ... u18
                values = [float(x) for x in parts[1:]]
                if len(values) != SERVO_COUNT:
                    print("ERR WRONG_COUNT", len(values), SERVO_COUNT)
                else:
                    for i, p in enumerate(values):
                        if p == 0:
                            servos.disable(i)
                        else:
                            servos.pulse(i, p)
                    print("OK PULSES")

            elif cmd in ('SET', 'S'):
                # SET ch1 us1 [ch2 us2 ...]
                args = parts[1:]
                if len(args) == 0 or len(args) % 2 != 0:
                    print("ERR WRONG_COUNT_ARGS")
                else:
                    ok = True
                    pairs = []
                    try:
                        for i in range(0, len(args), 2):
                            ch = int(args[i])
                            us = float(args[i + 1])
                            if not (1 <= ch <= SERVO_COUNT):
                                print("ERR BAD_CH", ch)
                                ok = False
                                break
                            pairs.append((ch, us))
                    except Exception:
                        ok = False
                    if ok:
                        for ch, us in pairs:
                            if us == 0:
                                servos.disable(ch - 1)
                            else:
                                servos.pulse(ch - 1, us)
                        print("OK SET", len(pairs))

            elif cmd == 'OFF':
                # OFF           -> all off
                # OFF ch        -> one channel off
                if len(parts) == 1:
                    servos.disable_all()
                    print("OK OFF ALL")
                else:
                    ch = int(parts[1])
                    if not (1 <= ch <= SERVO_COUNT):
                        print("ERR BAD_CH", ch)
                    else:
                        servos.disable(ch - 1)
                        print("OK OFF", ch)

            else:
                print("ERR UNKNOWN_CMD")

        except Exception as e:
            # Compact error report
            print("ERR", repr(e))

    time.sleep(0.005)
