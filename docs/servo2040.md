[Project home](../README.md) · [Documentation index](README.md)

# Servo 2040 firmware and protocol

The physical adapter sends timed batches of 18-joint setpoints to the Servo
2040 over USB serial. The board runs Pimoroni MicroPython and acts as a small,
generation-agnostic batch player: it validates the frame, converts robot
angles through per-servo calibration, and applies the points at their stated
period. It has no knowledge of whether a batch came from gait generation,
posture generation, or another compatible producer.

## Protocol and playback

| Parameter | Value |
| --- | ---: |
| Protocol magic / version / type | `HXGB` / 1 / batch type 1 |
| Joint values per point | 18 float32 radians / 72 bytes |
| Fixed frame overhead | 28-byte header + 4-byte CRC |
| Frame size | `32 + 72 × point_count` bytes |
| Maximum points per frame | 64 |
| Accepted sample-period range | 1–1000 ms |
| Default serial configuration | `/dev/ttyACM0`, 115200 baud |
| Firmware boot wait | 5 s |

The header contains magic, protocol version, message type, header size,
session ID, unique goal ID, joint count, point count, sample period, and
payload length. The board rejects malformed, stale, or CRC-invalid goals.
Joint payload values are trusted after the CRC succeeds and are unpacked only
during playback. Retries reuse the same session/goal ID and are idempotent.

## Measured batch handoff timing

A diagnostic build was tested on trajectory 7 using continuous 24-point
batches at the 20 ms sample period. Goals 35–39 produced these steady-state
ranges; startup, user idle, and the intentional stand-up hold were excluded:

| Stage | Diagnostic range |
| --- | ---: |
| Pi completion processing before the next executor call | 1.2–3.0 ms |
| Host batch encoding and CRC | 0.29–0.34 ms |
| Host USB write and flush | 21.57–21.65 ms |
| Board header receipt and validation | 1.45–1.46 ms |
| Board payload receipt | 18.49–18.64 ms |
| Board native CRC calculation | 0.70–0.73 ms |
| Board garbage collection | 3.94–5.48 ms |
| Board ACK formatting and flush | 0.78–0.80 ms |
| First-point deadline wait after a late-frame reset | 0.42–0.51 ms |
| First joint conversion and servo update | 10.86–11.07 ms |
| **Raw carried-deadline miss per batch** | **40.53–42.51 ms** |

Host and board rows are views of some of the same USB transfer and therefore
must not be added together as independent sequential stages.

## Channel order and calibration

The incoming vector is leg-major, but physical channels are joint-major:

| Board channels | Joints in channel order |
| --- | --- |
| 1–6 | `j11, j21, j31, j41, j51, j61` |
| 7–12 | `j12, j22, j32, j42, j52, j62` |
| 13–18 | `j13, j23, j33, j43, j53, j63` |

For each joint:

```text
servo_deg = s × robot_deg + b
pulse_us   = 1500 + k × servo_deg
```

`k+` is used for non-negative servo degrees and `k-` for negative servo
degrees. Pulses are clamped to 500–2500 µs. These are the runtime values in
`hardware/main.py`:

| Joint | `s` | `b` (deg) | `k+` (µs/deg) | `k-` (µs/deg) |
| --- | ---: | ---: | ---: | ---: |
| `j11` | +1 | 0 | 10.6667 | 10.8889 |
| `j12` | -1 | -84 | 11.1556 | 10.8889 |
| `j13` | +1 | -121 | 10.4444 | 11.3333 |
| `j21` | +1 | 0 | 10.3333 | 11.3333 |
| `j22` | -1 | -69 | 10.5556 | 12.0000 |
| `j23` | +1 | -114 | 11.1667 | 11.1667 |
| `j31` | +1 | 0 | 11.1111 | 10.6667 |
| `j32` | -1 | -79 | 11.1111 | 10.8889 |
| `j33` | +1 | -121 | 11.0000 | 10.6667 |
| `j41` | +1 | 0 | 10.5556 | 10.8889 |
| `j42` | -1 | -74 | 11.1111 | 11.0000 |
| `j43` | +1 | -118 | 10.7778 | 11.1111 |
| `j51` | +1 | 0 | 12.0000 | 11.1111 |
| `j52` | -1 | -67 | 10.6667 | 11.1111 |
| `j53` | +1 | -112 | 10.6667 | 10.8889 |
| `j61` | +1 | 0 | 10.8889 | 11.1111 |
| `j62` | -1 | -78 | 11.5556 | 10.4444 |
| `j63` | +1 | -121 | 11.0000 | 10.5556 |
