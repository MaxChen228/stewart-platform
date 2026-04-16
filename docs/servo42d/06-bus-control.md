# Bus Control General Commands

Only valid in SR_OPEN/SR_CLOSE/SR_vFOC modes.

## 0xF1 — Read Motor Operating Status

Send DLC=2: `[F1, CRC]`
Response DLC=3: `[F1, status, CRC]`

| status | Meaning |
|--------|---------|
| 0 | Query failed |
| 1 | Stopped |
| 2 | Speeding up |
| 3 | Slowing down |
| 4 | Full speed |
| 5 | Homing |
| 6 | Calibrating |

Notes:
- In CR modes, can only query calibration (status=6).
- If motor has no magnet or magnet is far, may return incorrect values. Use 0x95 to disable encoder judgment first, then use open-loop mode to check.

## 0xF3 — Set Motor Enable

Send DLC=3: `[F3, enable, CRC]`

- enable=0x00: disabled (loose shaft, free rotation)
- enable=0x01: enabled (shaft lock, holding torque)

Response DLC=3: `[F3, status, CRC]`

In bus control mode, enable is controlled by this command, not the EN pin.
Note: 0x85 Set EN pin level 僅作用於 pulse mode，見 [04-config-commands](04-config-commands.md)。

## 0xF7 — Emergency Stop

Send DLC=2: `[F7, CRC]`
Response DLC=3: `[F7, status, CRC]`

- 0=failed, 1=success

**Warning: Not recommended when motor speed exceeds 1000 RPM!**

Can also use 0xF6/0xFD/0xFE stop commands (with acc≠0) for controlled deceleration.
