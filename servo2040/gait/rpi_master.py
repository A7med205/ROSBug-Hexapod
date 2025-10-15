# pip install pyserial
import time
import serial
import sys
import tty
import termios
import select

# -------------------------
# Gait parameters (hardcoded)
# -------------------------
# These match the defaults in gait_controller.cpp and are sent on each STEP/HALF command.
STEP_LENGTH = 0.05     # meters
STEP_HEIGHT = 0.03     # meters
STEP_DURATION = 3.0    # seconds

# A new STEP should be sent if a direction key was pressed within the last 100 ms
ACTIVE_WINDOW_S = 0.100

# Mapping of direction keys to command argument
DIRECTION_KEYS = {"w", "a", "s", "d"}  # w: forward, d:+90, s:+180, a:-90

# Key to force half-step to center (if not centered)
HALF_KEY = "o"  # uppercase 'O' or lowercase 'o'

# -------------------------
# Servo board configuration
# -------------------------
PORT = "/dev/ttyACM0"  # adjust with: ls /dev/ttyACM*
BAUD = 115200

# -------------------------
# Serial communication setup
# -------------------------
ser = serial.Serial(PORT, BAUD, timeout=0.5)

def send(cmd: str) -> str:
    """Send a single-line command and return first response line (if any)."""
    ser.write((cmd + "\n").encode("utf-8"))
    line = ser.readline().decode(errors="ignore").strip()
    return line

def send_step(direction: str) -> bool:
    """
    Send a STEP command with hardcoded length/height/duration and the given direction.
    Blocks until 'OK STEP_DONE' or an ERR is seen. Returns True on success.
    """
    cmd = f"STEP {STEP_LENGTH} {STEP_HEIGHT} {STEP_DURATION} {direction}"
    print(">>", cmd)
    ser.write((cmd + "\n").encode("utf-8"))
    success = False
    # Expect first 'OK STEP_START ...', then after motion finishes 'OK STEP_DONE'
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            # Keep waiting; step can be long-running
            continue
        print("<<", line)
        if line.startswith("OK STEP_DONE"):
            success = True
            break
        if line.startswith("ERR"):
            break
    return success

def send_half(direction: str) -> bool:
    """
    Send a HALF command (last-direction kind) to return to center/home.
    Board uses 0.75 * duration internally; pass the same L/H/T and direction.
    """
    cmd = f"HALF {STEP_LENGTH} {STEP_HEIGHT} {STEP_DURATION} {direction}"
    print(">>", cmd)
    ser.write((cmd + "\n").encode("utf-8"))
    success = False
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        print("<<", line)
        if line.startswith("OK HALF_DONE"):
            success = True
            break
        if line.startswith("ERR"):
            break
    return success

def turn_off():
    print(">> OFF")
    line = send("OFF")
    if line:
        print("<<", line)

# -------------------------
# Terminal input (raw, non-blocking)
# -------------------------
class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old_settings:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

def read_key_nonblocking() -> str:
    """
    Return a single key if available, else ''.
    Uses select to avoid blocking.
    """
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        try:
            ch = sys.stdin.read(1)
            return ch
        except Exception:
            return ""
    return ""

# -------------------------
# Main logic
# -------------------------
print("Waiting for READY...")
start = time.time()
ready = False
while time.time() - start < 5.0:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print("<<", line)
    if line == "READY":
        ready = True
        break
if not ready:
    print("Didn't see READY, continuing anyway...")

# Keep PING
print(">> PING")
print("<<", send("PING"))

print("Control: 'w','a','s','d' for direction, 'O' to half-step to center, 'q' to quit.")
print("- On direction change, a half-step of the last direction is executed first to return to center.")

last_dir_key = ""         # last direction key seen (w/a/s/d) from keyboard
last_press_time = 0.0     # last time we saw a direction key
prev_dir_sent = ""        # last direction that was actually sent to the board
centered = True           # startup sequence centers the robot
stepping = False          # whether we're currently waiting for a STEP/HALF to finish

try:
    with RawTerminal():
        while True:
            # Poll for a key without blocking
            key = read_key_nonblocking()
            now = time.monotonic()

            if key:
                if key == "q":
                    turn_off()
                    break
                k = key.lower()

                if k in DIRECTION_KEYS:
                    last_dir_key = k
                    last_press_time = now
                    print(f"[KEY] dir={last_dir_key}")

                elif k == HALF_KEY:
                    # Force half-step to center if not centered and not currently stepping
                    if not stepping and not centered:
                        print("[CMD] HALF (manual) to center")
                        stepping = True
                        half_dir = prev_dir_sent if prev_dir_sent else "w"
                        ok = send_half(half_dir)
                        centered = True if ok else centered
                        stepping = False
                    else:
                        print("[INFO] Already centered or busy; HALF ignored")

                else:
                    # Ignore other keys
                    pass

            # If not currently stepping, and a recent direction keypress occurred, send a new step
            if not stepping and last_dir_key and (now - last_press_time) <= ACTIVE_WINDOW_S:
                desired_dir = last_dir_key

                # If direction changed and we're not centered, half-step first using last direction
                if prev_dir_sent and desired_dir != prev_dir_sent and not centered:
                    print(f"[SEQ] Direction change {prev_dir_sent} -> {desired_dir}: HALF to center first")
                    stepping = True
                    ok_half = send_half(prev_dir_sent)
                    centered = True if ok_half else centered
                    stepping = False

                # Now send the full step in desired direction
                print(f"[SEQ] STEP dir={desired_dir}")
                stepping = True
                ok_step = send_step(desired_dir)
                centered = False if ok_step else centered
                prev_dir_sent = desired_dir if ok_step else prev_dir_sent
                stepping = False

            # Small idle to avoid busy-wait
            time.sleep(0.01)

finally:
    try:
        ser.close()
    except Exception:
        pass
    print("Serial closed, exiting.")
