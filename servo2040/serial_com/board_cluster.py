import time
import sys
import uselect
import gc
from servo import ServoCluster, servo2040

SERVO_COUNT = 18  # Servo 2040 has 18 channels

# Free up hardware resources before creating the ServoCluster
gc.collect()

# Create a servo cluster covering pins SERVO_1 .. SERVO_18
START_PIN = servo2040.SERVO_1
END_PIN = getattr(servo2040, f"SERVO_{SERVO_COUNT}")
servos = ServoCluster(pio=0, sm=0, pins=list(range(START_PIN, END_PIN + 1)))

# Enable all servos
servos.enable_all()

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

    time.sleep(0.01)

