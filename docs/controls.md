[Project home](../README.md) · [Documentation index](README.md)

# Controls and operating modes

The keyboard controls and coordinator behavior are identical for hardware and
simulation.

## Modes and state behavior

- At startup, the coordinator rejects counted motion until the standup `u`
  command or the skip assertion completes.
- `normal` accepts bare movement keys as continuous commands. A new movement
  direction is latched and applied at the next full-step midpoint.
- `auto` only accepts a movement command with an integer step count of at least
  two. Auto jobs start and end stationary, and are aborted by `0`.
- `posture` starts from canonical stationary and accepts relative elevation,
  pitch, and roll changes. Elevation can coexist with either tilt. Pitch and
  roll cannot coexist; reset the active tilt before selecting the other axis.

## Keyboard controls

| Key | Command |
| --- | --- |
| `u` | Explicit standup sequence |
| `k` | Skip standup and assert that the robot is already standing |
| `j` | Sit down from stationary and restore the standup lock |
| `0` | Graceful gait stop; in posture, interrupt and then return neutral |
| `t` | Cycle auto → posture → normal → auto (auto is the default) |
| `w`, `d`, `s`, `a` | Move `+Y`, `+X`, `-Y`, `-X` |
| `q`, `e`, `z`, `c` | Diagonal motion, or orbit motion after pressing `m` |
| `m` | Toggle the `q/e/z/c` mapping between diagonal and orbit |
| `o`, `p` | Rotate counterclockwise / clockwise |
| `5w` | Example auto command: move `+Y` for five counted steps |
| `5[` / `5]` | Lower / raise the body by 5 mm in posture mode |
| `5.`, `5,` | Add / subtract 5° pitch in posture mode |
| `5'`, `5;` | Add / subtract 5° roll in posture mode |
| `r` | Reset pitch or roll to zero while preserving elevation |
| Backspace | Delete the last numeric-prefix digit |
| Escape | Clear the numeric prefix |
| `x` | Quit after the current batch |

| Simulation keyboard controls | Simulation posture controls |
| --- | --- |
| ![Simulation keyboard controls](controls.gif) | ![Simulation posture controls](posture.gif) |

Posture commands require a numeric prefix. `[` and `]` lower and raise
elevation by the prefixed number of millimetres; pitch and roll numbers are
relative degree changes from the confirmed pose. Every elevation change is
clamped to the 27.5–104 mm scaled objective-height envelope.

During an active posture command, the first `0` discards remaining points
after the current 25-point boundary and enters `POSTURE_HOLD`; a second `0`
from that hold returns to the stationary objective elevation and zero tilt.
`r` zeros the active tilt without changing elevation.
