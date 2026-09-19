# MCU serial protocol

UART configuration: `921600 8N1`, little-endian. Real-time sensor and motor
frames are normally sent every 10 ms. Multi-byte fields use little-endian byte
order. Low-rate messages such as EKF position use their own schedule.

## Common frame

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 2 | sync | `A5 5A` |
| 2 | 1 | version | `1` |
| 3 | 1 | type | Message type |
| 4 | 2 | payload_len | Payload byte count |
| 6 | 2 | sequence | Wraps naturally at 65535 |
| 8 | N | payload | Type-specific data |
| 8+N | 2 | crc16 | CRC-16/CCITT-FALSE over header and payload |

CRC parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no
final XOR. A receiver must search for the sync bytes again after a CRC or length
error.

## PC to MCU: sensor state (`type = 0x01`, 26-byte payload)

Sent every 10 ms. Latitude and longitude hold the most recent GPS fix; bit 0 of
`flags` is set for exactly one frame when the 1 Hz GPS model produces a new fix.

| Payload offset | Type | Field | Unit |
|---:|---|---|---|
| 0 | uint32 | simulation_time | ms |
| 4 | uint16 | flags | bit 0 = GPS_NEW |
| 6 | float32 | yaw | rad, north=0, clockwise positive, range `[-pi, pi)` |
| 10 | float32 | yaw rate | rad/s, clockwise positive |
| 14 | int32 | latitude | degrees x 1e-7 |
| 18 | int32 | longitude | degrees x 1e-7 |
| 22 | float32 | GPS accuracy radius | m, approximate 95% radius |

The simulator's **Invert gyro yaw rate** option changes the sign of the
`yaw rate` field only. It does not change the true trajectory or the `yaw`
heading field.

## PC to MCU: target (`type = 0x02`, 16-byte payload)

Sent after connection and whenever the operator presses **Send target**.

| Payload offset | Type | Field | Unit |
|---:|---|---|---|
| 0 | uint32 | simulation_time | ms |
| 4 | int32 | target latitude | degrees x 1e-7 |
| 8 | int32 | target longitude | degrees x 1e-7 |
| 12 | float32 | arrival radius | m |

## MCU to PC: motor command (`type = 0x81`, 8-byte payload)

Sent every 10 ms. Both motor values are clamped by the PC to
`[-10000, 10000]`; zero is stopped and negative values are reverse thrust.

| Payload offset | Type | Field | Unit |
|---:|---|---|---|
| 0 | uint32 | MCU time | ms |
| 4 | int16 | left motor | -10000..10000 |
| 6 | int16 | right motor | -10000..10000 |

If no valid motor frame is received for 200 ms, the simulator sets both motor
commands to zero. The MCU should apply the same watchdog rule to stale sensor
frames.

## MCU to PC: EKF position (`type = 0x82`, 12-byte payload)

Send this frame every 500 ms (2 Hz). A 1-second period (1 Hz) is also supported.
It is deliberately separate from the real-time motor frame.

| Payload offset | Type | Field | Unit |
|---:|---|---|---|
| 0 | uint32 | MCU time | ms |
| 4 | int32 | EKF latitude | degrees x 1e-7 |
| 8 | int32 | EKF longitude | degrees x 1e-7 |

The EKF coordinate is plotted in green. Repeating an unchanged position is
allowed; the PC suppresses duplicate display points.
