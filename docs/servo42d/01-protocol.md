# CAN Protocol Basics

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/01-protocol|CAN Protocol Basics]]

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

> 🔀 **vs 42ES：** encoder resolution is identical (both 16384 counts/rev = 0x4000). The [[servo42es/01-protocol|42ES]] manual additionally types its read frames explicitly (`0x30` = int32 carry + uint16 value, `0x31` = int48_t cumulative); the byte-level encoding matches.

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

> 🔀 **vs 42ES：** the status table (0 fail / 1 success / 2 completed / 3 endstop / 5 sync) is identical, but persistence differs. On SERVO42D, config/write commands persist per-command. On [[servo42es/01-protocol|42ES]] write-only parameters live only in RAM until you batch-commit them with a single **`0x60` (SAVE all)** — otherwise every config write is lost on power-down. Keep this in mind when porting config flows between the two manuals.
