import time
import sys
import uselect
from servo import Servo, servo2040

SERVO_COUNT = 18  # Servo 2040 has 18 channels

# Create and enable 18 servos (1-based SERVO_1..SERVO_18)
servo_pins = [getattr(servo2040, f"SERVO_{i}") for i in range(1, SERVO_COUNT + 1)]
servos = [Servo(pin) for pin in servo_pins]
for s in servos:
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
                # PULSE ch us  (1-based ch in [1..18])
                if len(parts) < 3:
                    print("ERR MISSING_ARGS")
                else:
                    ch = int(parts[1])
                    if not (1 <= ch <= len(servos)):
                        print("ERR BAD_CH", ch)
                    else:
                        p = float(parts[2])
                        # .pulse(p) enables if disabled; p==0 disables
                        servos[ch - 1].pulse(p)
                        print("OK PULSE", ch, p)

            elif cmd in ('PULSES', 'PS'):
                # PULSES u1 u2 ... u18 (exactly 18 values)
                values = [float(x) for x in parts[1:]]
                if len(values) != len(servos):
                    print("ERR WRONG_COUNT", len(values), len(servos))
                else:
                    for i, p in enumerate(values):
                        servos[i].pulse(p)
                    print("OK PULSES")

            elif cmd in ('SET', 'S'):
                # SET ch1 us1 [ch2 us2 ...] update a subset with different values
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
                            if not (1 <= ch <= len(servos)):
                                print("ERR BAD_CH", ch)
                                ok = False
                                break
                            pairs.append((ch, us))
                    except Exception:
                        ok = False
                    if ok:
                        for ch, us in pairs:
                            servos[ch - 1].pulse(us)
                        print("OK SET", len(pairs))

            elif cmd == 'OFF':
                # OFF           -> all off
                # OFF ch        -> channel ch off
                if len(parts) == 1:
                    for s in servos:
                        s.pulse(0)
                    print("OK OFF ALL")
                else:
                    ch = int(parts[1])
                    if not (1 <= ch <= len(servos)):
                        print("ERR BAD_CH", ch)
                    else:
                        servos[ch - 1].pulse(0)
                        print("OK OFF", ch)

            else:
                print("ERR UNKNOWN_CMD")

        except Exception as e:
            # Avoid huge tracebacks over serial; send compact error
            print("ERR", repr(e))

    time.sleep(0.01)
