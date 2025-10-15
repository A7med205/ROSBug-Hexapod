# pip install pyserial
import time
import serial

# Adjust this if needed; use: ls /dev/ttyACM* to find the device
PORT = "/dev/ttyACM0"
BAUD = 115200  # For USB CDC it's largely ignored but must be set
SERVO_COUNT = 18  # 18 channels on Servo 2040

ser = serial.Serial(PORT, BAUD, timeout=1)

def send(cmd):
    ser.write((cmd + "\n").encode("utf-8"))
    # Read one reply line (protocol is one line per command)
    line = ser.readline().decode(errors="ignore").strip()
    return line

# Wait for board to announce it's ready
print("Waiting for READY...")
start = time.time()
ready = False
while time.time() - start < 5:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print("<<", line)
    if line == "READY":
        ready = True
        break
if not ready:
    print("Didn't see READY, continuing anyway...")

# Keep PING
print(">>", "PING")
print("<<", send("PING"))

def do_pulse(ch, pulse_us, delay_s=0.1):
    # ch is 1-based index: 1..18
    cmd = f"PULSE {ch} {int(pulse_us)}"
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

def do_pulses(pulses, delay_s=0.1):
    # pulses: list/tuple of length SERVO_COUNT
    if len(pulses) != SERVO_COUNT:
        raise ValueError(f"Expected {SERVO_COUNT} pulse values, got {len(pulses)}")
    cmd = "PULSES " + " ".join(str(int(p)) for p in pulses)
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

def do_set(pairs, delay_s=0.1):
    # pairs: iterable of (ch, pulse_us) with ch in 1..SERVO_COUNT
    for ch, p in pairs:
        if not (1 <= ch <= SERVO_COUNT):
            raise ValueError(f"Channel out of range: {ch}")
    cmd = "SET " + " ".join(f"{int(ch)} {int(p)}" for ch, p in pairs)
    print(">>", cmd)
    print("<<", send(cmd))
    time.sleep(delay_s)

# Demonstrations

# 1) Mid position (approx 1500us) for all
do_pulses([1500] * SERVO_COUNT, delay_s=1.0)

# 2) Move each channel a bit to one side then other then back to mid
for ch in range(1, SERVO_COUNT + 1):
    do_pulse(ch, 1700, delay_s=0.15)
    do_pulse(ch, 1300, delay_s=0.15)
    do_pulse(ch, 1500, delay_s=0.05)

# 3) Simple "sweep-like" demonstration by alternating pulses in bulk
for _ in range(4):
    do_pulses([1200 if i % 2 == 0 else 1800 for i in range(SERVO_COUNT)], delay_s=0.5)
    do_pulses([1800 if i % 2 == 0 else 1200 for i in range(SERVO_COUNT)], delay_s=0.5)

# 3b) Different value per servo (e.g., gradient across channels)
vals = [1100 + int(i * (800 / (SERVO_COUNT - 1))) for i in range(SERVO_COUNT)]
do_pulses(vals, delay_s=1.0)

# 3c) Update a subset of channels only using SET
do_set([(1, 1200), (5, 1500), (9, 1800)], delay_s=0.5)

# 4) Disable all by sending OFF (board will .pulse(0) each channel)
print(">>", "OFF")
print("<<", send("OFF"))

ser.close()
