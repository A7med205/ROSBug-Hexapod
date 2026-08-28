[Project home](../README.md) · [Documentation index](README.md)

# Hardware setup

The hardware adapter runs on the robot's Raspberry Pi. From a machine with
this repository open, create the destination and copy the runtime files:

```bash
ssh pi@<ip-address> 'mkdir -p /home/pi/ROSBug-Hexapod'
scp -r \
  common \
  robot_core \
  gait_core \
  posture_core \
  hardware \
  requirements.txt \
  pi@<ip-address>:/home/pi/ROSBug-Hexapod/
```

SSH into the Pi before running the remaining commands:

```bash
ssh pi@<ip-address>
cd /home/pi/ROSBug-Hexapod
```

These instructions assume the Servo 2040 is attached to the Pi over USB.

## Install host dependencies

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install mpremote
```

## Install or update the Servo 2040 software

The repository includes the Pimoroni MicroPython `v1.27.0` RP2040 runtime used
by the Servo 2040:

| Runtime property | Value |
| --- | --- |
| Bundled UF2 | `hardware/firmware/pico-v1.27.0-pimoroni-micropython.uf2` |
| Target | Raspberry Pi RP2040 (`0xe48bff56`) |
| Size | 1,300,992 bytes / 2,541 UF2 blocks |
| SHA-256 | `5160050bef88dacd43496cd9fcfe14d3a8844d9d640a9a3131f65c5fd833a14c` |
| Upstream release | [Pimoroni Pico MicroPython v1.27.0](https://github.com/pimoroni/pimoroni-pico/releases/tag/v1.27.0) |

Routine updates only require copying `main.py` and `protocol.py`; do not
reflash the UF2 for normal application changes. If the MicroPython runtime is
missing or corrupt, follow the
[Servo 2040 hard-reset procedure](../hardware/firmware/servo2040_hard_reset.txt).
It explains the long BOOT hold, USB disconnect/reconnect, `RPI-RP2` identity
check, mount-device changes such as `sda1` to `sdb1`, flashing, and restoring
the application files.

Pimoroni's
[installation guide](https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-micropython.md)
identifies the generic `pico-...-pimoroni-micropython.uf2` build for Servo 2040
and other non-wireless RP2040 boards.

Copy both MicroPython files to the board; `main.py` imports `protocol.py` from
its filesystem:

```bash
mpremote connect /dev/ttyACM0 cp hardware/protocol.py :protocol.py
mpremote connect /dev/ttyACM0 cp hardware/main.py :main.py
```

## Run the physical robot

```bash
python3 -m hardware.board_interface --port /dev/ttyACM0
```

| Argument | Default | Meaning |
| --- | ---: | --- |
| `--port` | `/dev/ttyACM0` | Servo 2040 serial device |
| `--baudrate` | `115200` | PySerial baud setting |
| `--response-timeout` | `8.0` s | ACK/DONE timeout, including the five-second firmware boot delay |

Wait for `READY 1` before diagnosing an initial timeout. The firmware accepts
one batch at a time. Playback carries the next point deadline across batches;
a late frame applies its first point immediately and starts a fresh board-clock
cadence.
