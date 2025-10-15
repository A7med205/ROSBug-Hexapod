Great goal. The simplest way to do this is:
- Run a small MicroPython “command listener” on the Servo 2040 that reads commands over USB serial and moves the servo.
- Send those commands from your Raspberry Pi using pyserial.

Below are drop‑in scripts for both sides plus setup steps and troubleshooting.

Plan A: USB serial (recommended)
- One USB cable between Raspberry Pi and Servo 2040.
- No extra wiring; baud rate on the host doesn’t really matter for USB CDC.

MicroPython on Servo 2040 (save as main.py on the board)
This listens for simple text commands like ENABLE, DISABLE, MID, MIN, MAX, SET <deg>, SWEEP, STOP, PING.

```
import time
import math
import sys
import uselect
from servo import Servo, servo2040

# Create and enable a servo on SERVO_1
s = Servo(servo2040.SERVO_1)
s.enable()

print("READY")  # Host can wait for this

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

# Optional sweep state
sweep = None  # dict or None

def clamp_angle(a, min_a=-90.0, max_a=90.0):
    return max(min(a, max_a), min_a)

last_cycle_key = 'last_cycle'

while True:
    # Non-blocking read of a command line from USB serial
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
            if cmd in ('EN', 'ENABLE'):
                s.enable()
                print("OK ENABLED")

            elif cmd in ('DIS', 'DISABLE'):
                s.disable()
                sweep = None
                print("OK DISABLED")

            elif cmd in ('MID', 'CENTER', 'CENTRE'):
                s.to_mid()
                sweep = None
                print("OK MID")

            elif cmd == 'MIN':
                s.to_min()
                sweep = None
                print("OK MIN")

            elif cmd == 'MAX':
                s.to_max()
                sweep = None
                print("OK MAX")

            elif cmd in ('SET', 'ANGLE'):
                if len(parts) < 2:
                    print("ERR MISSING_ANGLE")
                else:
                    a = float(parts[1])
                    a = clamp_angle(a)
                    s.value(a)
                    sweep = None
                    print("OK SET", a)

            elif cmd == 'SWEEP':
                # SWEEP [amplitude_deg=90] [period_ms=2000] [reps=0 for infinite]
                amp = float(parts[1]) if len(parts) > 1 else 90.0
                period = float(parts[2]) if len(parts) > 2 else 2000.0
                reps = int(parts[3]) if len(parts) > 3 else 0
                period = max(period, 100.0)
                sweep = {
                    'amp': abs(amp),
                    'period': period,
                    'reps': reps,
                    't0': time.ticks_ms(),
                    'count': 0,
                    last_cycle_key: -1,
                }
                print("OK SWEEP", sweep['amp'], sweep['period'], sweep['reps'])

            elif cmd == 'STOP':
                sweep = None
                print("OK STOP")

            elif cmd == 'PING':
                print("PONG")

            else:
                print("ERR UNKNOWN_CMD")

        except Exception as e:
            # Avoid huge tracebacks over serial; send compact error
            print("ERR", repr(e))

    # Run sweep (non-blocking)
    if sweep:
        now = time.ticks_ms()
        t = time.ticks_diff(now, sweep['t0'])
        phase = (t % sweep['period']) / sweep['period'] * 2.0 * math.pi
        angle = math.sin(phase) * sweep['amp']
        s.value(angle)

        # Count completed cycles if reps was specified
        cycle = int(t // sweep['period'])
        if cycle != sweep[last_cycle_key]:
            sweep[last_cycle_key] = cycle
            if sweep['reps'] > 0:
                sweep['count'] += 1
                if sweep['count'] >= sweep['reps']:
                    sweep = None

        time.sleep(0.01)
    else:
        time.sleep(0.01)
```

Raspberry Pi side (Python 3)
Install pyserial:
- pip install pyserial

Then run something like:
```
import time
import serial

# Adjust this if needed; use: ls /dev/ttyACM* to find the device
PORT = "/dev/ttyACM0"
BAUD = 115200  # For USB CDC it's largely ignored but must be set

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

print(">>", "PING")
print("<<", send("PING"))

print(">>", "ENABLE")
print("<<", send("ENABLE"))

print(">>", "MID")
print("<<", send("MID"))
time.sleep(1.0)

print(">>", "SET 45")
print("<<", send("SET 45"))
time.sleep(1.5)

print(">>", "SET -45")
print("<<", send("SET -45"))
time.sleep(1.5)

print(">>", "SWEEP 60 1500 4")
print("<<", send("SWEEP 60 1500 4"))

# Optionally read async status lines until sweep finishes
t0 = time.time()
while time.time() - t0 < 10:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print("<<", line)

print(">>", "DISABLE")
print("<<", send("DISABLE"))

ser.close()
```

Command summary
- ENABLE: power the servo output
- DISABLE: power off the servo output
- MID: center position
- MIN, MAX: travel endpoints
- SET <deg>: set angle in degrees relative to mid (clamped to about -90..+90 by default)
- SWEEP [amp_deg] [period_ms] [reps]: sine sweep with amplitude, period, and number of cycles (0 = forever)
- STOP: stop an active sweep
- PING: test connectivity

Notes and setup tips
- Power: Don’t try to power a real servo from USB alone. Ensure the Servo 2040’s servo power rail is correctly powered as per Pimoroni’s docs.
- USB device name: On the Pi, the board usually appears as /dev/ttyACM0. If it changes, run ls /dev/ttyACM* after plugging in.
- Permissions: If you get a permission error, either run with sudo or add your user to the dialout group and re-login.
- REPL vs your program: Put the MicroPython code on the Servo 2040 as main.py so it starts at boot. When your script is running, the incoming USB serial is read by your script. Ctrl-C on the serial will interrupt it (KeyboardInterrupt).
- Thonny/IDE: Close Thonny’s serial connection before your Pi script opens the port; only one connection at a time.
- Timing: The USB baud rate on the host is ignored by the device’s USB CDC, but still set it to something standard (e.g., 115200).
- Ranges: If your servo supports a wider/narrower range, adjust the clamp in the MicroPython script or use the Pimoroni Servo configuration for min/max pulse widths.

Alternative: use the hardware UART pins
If you prefer wiring TX/RX between the Pi and the Servo 2040 instead of USB:
- Wire Pi GPIO14 (TXD) to Servo 2040 RX, Pi GPIO15 (RXD) to Servo 2040 TX, and GND to GND. Levels are 3.3V on both sides.
- On the Pi, use /dev/serial0 (disable login console on UART first).
- On the Servo 2040, replace the sys.stdin/uselect loop with a machine.UART instance, then read lines from uart.readline() and process with the same command handler.

If you share the code you tried and the exact error or behavior you see, I can adapt the above to your setup and fix whatever’s blocking you.