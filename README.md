# Hexapod shared motion controller

This repository combines the physical Servo 2040 controller and the ROS 2
simulation around the same transport-independent robot model, gait generator,
posture generator, and mode coordinator.

## Layout

- `robot_core/model.py`: canonical dimensions, leg frames, fixed-foot body
  transforms, and inverse kinematics.
- `robot_core/motion_batch.py`: transport-neutral joint batches and shared goal
  IDs.
- `robot_core/coordinator.py`: startup and normal/auto/posture mode arbitration.
- `gait_core/lite_gait.py`: body-frame stance displacement, hardware-tuned gait
  templates, joint gating, and gait state-machine primitives.
- `posture_core/posture.py`: relative elevation, pitch, and roll planning,
  smoothed profiles, global IK-boundary clamping, and posture state.
- `common/keyboard_input.py`: common direct keyboard mapping.
- `hardware/board_interface.py`: host keyboard and binary serial adapter.
- `hardware/main.py`: Servo 2040 validation, calibration, and timed discrete
  playback.
- `hardware/protocol.py`: the protocol shared by the host and MicroPython.
- `ros2_ws/src/hexapod_sim/scripts/sim_interface.py`: direct keyboard to ROS
  `FollowJointTrajectory` adapter.

Both adapters receive the same `JointBatch` values from the core. Gait batches
end at half-step boundaries. Posture profiles are split into at most 64 points
per batch for the existing firmware protocol without changing the logical
posture command.

## Keyboard controls

| Key | Command |
| --- | --- |
| `u` | Explicit startup/stand sequence |
| `k` | Skip startup and assert the robot is already standing |
| `0` | Graceful gait stop, or return posture to neutral |
| `t` | Cycle normal → auto → posture → normal |
| `5w` | Example auto command: five steps in `+Y` |
| `w`, `d`, `s`, `a` | Linear motion |
| `q`, `e`, `z`, `c` | Diagonal or orbit motion |
| `m` | Toggle diagonal/orbit mapping |
| `o`, `p` | Self rotation |
| `10]`, `10[` | Raise/lower the body by 10 mm in posture mode |
| `5.`, `5,` | Add/subtract 5° pitch in posture mode |
| `5'`, `5;` | Add/subtract 5° roll in posture mode |
| `x` | Quit after the currently executing batch |

Walking commands are ignored until startup completes or `k` explicitly skips
it. Skip-startup sends no movement: it asserts that the physical robot already
matches the canonical neutral standing pose. Startup first commands the
configured initial tip pose, holds it for two seconds, and then plays a
60-point descent to that standing pose.

Normal mode accepts bare movement keys as continuous commands. Auto mode only
accepts counted movement commands entered as a numeric prefix followed by a
movement key. Counts must be integers of at least two. A count of `N` executes:

```text
starting half-step + (N - 1) full steps + final half-step
```

The starting and final halves contribute one combined step. Auto jobs always
start and end stationary. Movement commands entered while an auto job is
active are ignored and never deferred. `0` aborts through the normal graceful
stop path. Toggling modes while moving does the same, and activates the new
mode only after stationary is reached.

Posture mode can only be entered from the canonical stationary pose. Posture
numbers are relative deltas: repeated elevation, pitch, or roll commands
accumulate from the last confirmed pose. Elevation can coexist with pitch or
roll. Pitch and roll cannot coexist; the active tilt must first be commanded
back to zero before the other tilt axis is accepted. Posture commands are not
queued, and a completed non-neutral command enters `POSTURE_HOLD`, where the
next posture command can be accepted.

Posture trajectories use a 20 ms cubic smoothstep profile. Defaults are 5 mm/s
and 20 mm/s² for elevation, and 5°/s and 20°/s² for pitch and roll. A requested
pose outside the six-leg IK workspace is reduced along the requested rigid-body
motion to the last globally reachable pose; individual legs are never clamped
independently. The adapter reports the requested and applied delta when this
happens. These are geometric limits only and are intended to support simulation
commissioning of the final operational limits.

Leaving posture mode removes pitch or roll first, then returns elevation to
zero, and activates normal or auto mode only after the canonical stationary
pose is confirmed. `0` performs the same neutral return while remaining in
posture mode.

There is currently no emergency-stop protocol. A serial or ROS batch already
in progress runs to completion. Use `0` for the coordinated final half-step.

## Servo 2040 firmware

Copy both firmware files to the board; `main.py` imports `protocol.py` locally:

```bash
mpremote connect /dev/ttyACM0 cp hardware/protocol.py :protocol.py
mpremote connect /dev/ttyACM0 cp hardware/main.py :main.py
```

The firmware accepts one batch at a time. It validates protocol version, goal
identity, shape, sample period, finite joint values, payload size, and CRC
before acknowledging it. It applies each point at an absolute board-clock
deadline and does not interpolate between points.

Run the host adapter from the repository root:

```bash
python3 -m pip install -r requirements.txt
python3 -m hardware.board_interface --port /dev/ttyACM0
```

The existing calibration tables, robot-to-servo conversion, channel mapping,
and pulse clamping are retained in `hardware/main.py`.

## ROS 2 simulation

Build and launch the existing simulation, then run the direct-keyboard adapter:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run hexapod_sim sim_interface.py
```

The adapter has no trajectory command topic. It polls the terminal while ROS
action goals execute, so the latest command can be latched during a half-step.
The ROS trajectory controller may interpolate between points; the Servo 2040
firmware deliberately uses discrete playback.

## Tests

From the repository root:

```bash
python3 -m unittest discover -s tests -v
```

Tests cover hardware template sizes, startup and three-mode transition
boundaries, relative posture composition, pitch/roll exclusion, smoothed batch
splitting, IK-boundary clamping, neutral return ordering, latest-command
behavior, protocol validation, finite batch values, and calibrated pulse
regression checks.

## Imported history

The original repositories remain untouched. Their heads are preserved as
branches in this repository:

- `hardware-history`: original `robot_brain`/RPi history.
- `simulation-history`: original `hexapod_sim` history.
- `main`: integration history containing both as ancestors.
