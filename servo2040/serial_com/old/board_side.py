import time
import sys
import uselect
from servo import Servo, servo2040

# Create and enable a servo on SERVO_1
s = Servo(servo2040.SERVO_1)
s.enable()

print("READY")  # Host can wait for this

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

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
            if cmd == 'PING':
                print("PONG")

            elif cmd in ('PULSE', 'P'):
                if len(parts) < 2:
                    print("ERR MISSING_PULSE")
                else:
                    p = float(parts[1])
                    # .pulse(p) enables if disabled (except 0 disables)
                    s.pulse(p)
                    print("OK PULSE", p)

            else:
                print("ERR UNKNOWN_CMD")

        except Exception as e:
            # Avoid huge tracebacks over serial; send compact error
            print("ERR", repr(e))

    time.sleep(0.01)
