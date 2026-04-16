# CAN Protocol Basics

## Frame Structure

Standard frame, max data field = 8 bytes.

**Downlink (host → driver):**

| CAN ID | DLC | Byte1 (code) | Byte2..Byte(n-1) (data) | Byte n (CRC) |
|--------|-----|-------------|------------------------|--------------|

**Uplink (driver → host):** Same structure.

## CAN ID

- Range: 0x00 ~ 0x7FF (0~2047)
- 0x00 = broadcast (slaves do not respond)
- Default: 0x01
- 0x01~0x10 settable via OLED; >0x10 via 0x8B command

## CRC Calculation

CRC = (CAN_ID + all data bytes except CRC) & 0xFF

Example: ID=0x01, data=`83 0C 80`
CRC = (0x01 + 0x83 + 0x0C + 0x80) & 0xFF = 0x10

Data is **big-endian**. CAN_ID 即為馬達地址（0x01~0x06）。

## Unit Conversions

| Quantity | Value |
|----------|-------|
| 1 revolution | 16384 encoder counts (0x4000) |
| 16-subdivision pulse/rev | 3200 pulses (0x0C80) |
| Angle error unit | 0~51200 = 0°~360° (142.22/degree) |
| Coordinate values | Cumulative multi-turn encoder (16384/turn) |

## Absolute vs Relative Position

- **Absolute**: Target from fixed origin (zero point). Same command to same position = no movement.
- **Relative**: Target from current position. Same command = same displacement each time.

## Response Convention

All write commands respond DLC=3: `[code, status, CRC]`.
- status=0: fail
- status=1: success

Motion commands may also return:
- status=2: completed
- status=3: endstop stopped
- status=5: sync mode received
