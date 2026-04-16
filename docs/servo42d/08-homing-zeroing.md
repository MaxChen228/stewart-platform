# Homing & Zero-Point Setting

## Four Homing Methods

1. **Direct zero point (0x92)**: Sets current position as zero. Low precision.
2. **Origin switch homing**: Motor searches for limit switch, marks position as zero. High precision.
3. **Mechanical limit homing**: Motor runs at fixed torque until stalled, reverses offset distance, marks as zero. No physical switch needed.
4. **Single-turn power-on zero-return**: Auto returns to zero (within one turn) on power-on.

Two zero-return execution modes:
- **Origin return** (0x91, mode=0x00): Searches for zero point via switch/mechanical limit/single-turn.
- **Coordinate zeroing** (0x91, mode=0x01): Directly runs to known "coordinate zero point" position. Must execute origin return first to establish zero point.

## 0x92 — Set Current Position as Zero

DLC=2: `[92, CRC]`
Response DLC=3: `[92, status, CRC]` (0=fail, 1=success)

Sets current position as coordinate zero point immediately.

## 0x91 — Execute Zero-Return

DLC=3: `[91, goZeroMode, CRC]`
- goZeroMode=0x00: Execute "Origin Return" (search for zero)
- goZeroMode=0x01: Execute "Coordinate Zeroing" (run to known zero)

If DLC=2 `[91, CRC]`, defaults to origin return (goZeroMode=0x00).

Response: 0=fail, 1=starting, 2=complete

Note: Motor must be enabled before executing coordinate zeroing.

## 0x9E — Configure Port Mapping (Bus Mode Only)

DLC=3: `[9E, enable, CRC]`
- 0x00=disable (default), 0x01=enable

When enabled: En pin → left-limit, Dir pin → right-limit. COM must be high.
28/35/42D only has IN_1 (left limit). Enable remapping to add right-limit via Dir pin.

## 0x90 — Set Homing Parameters

DLC=8: `[90, homeTrig, homeDir, homeSpeed_hi, homeSpeed_lo, EndLimit, hm_mode, CRC]`

| Field | Values |
|-------|--------|
| homeTrig | 0=low level when closed (default), 1=high level |
| homeDir | 0=CW (default), 1=CCW |
| homeSpeed | 0~3000 RPM (uint16) |
| EndLimit | 0=disable limit, 1=enable limit |
| hm_mode | 0=origin switch, 1=mechanical limit, 2=single-turn |

**Important**: After using limit function for the first time or changing parameters, must perform "return to zero" once (Menu→GoHome or 0x91).

## 0x94 — Set Homing Torque & Offset

DLC=8: `[94, Hm_offset(int32), Hm_ma(uint16), CRC]`

- Hm_offset: Origin offset (int32). 0x4000=full turn, **0x2000=half turn (default)**. Invalid for switch/single-turn modes.
- Hm_ma: Mechanical limit return current (uint16 mA). Set low to avoid damage. Invalid for switch/single-turn modes.
  - 42D default: 400mA, 57D default: 800mA, 28D/35D default: 200mA

## 0x9A — Set Single-Cycle Zero-Return Parameters

DLC=6: `[9A, mode, enable, speed, dir, CRC]`

| Field | Values |
|-------|--------|
| mode | 0=Disable, 1=DirMode (fixed direction), 2=NearMode (shortest path) |
| enable | 0=Clear zero point, 1=Set zero point, 2=Keep current zero point |
| speed | 0~4 (0=slowest, 4=fastest) |
| dir | 0=CW (default), 1=CCW |

## Limit Switch Operation

啟用限位開關後（0x90 EndLimit=1）：
- 左限位觸發 → 馬達停止向左運動
- 右限位觸發 → 馬達停止向右運動
- **首次啟用或參數變更後必須執行一次歸零**（Menu→GoHome 或 0x91）
- 脈衝控制模式中限位功能**無效**（僅 bus mode 有效）

28/35/42D 只有 IN_1（左限位）。需要右限位時用 0x9E 啟用端口映射（En→左限位, Dir→右限位）。

## Homing Procedures

### Origin Switch Homing
1. Set working mode: `[82, 05, CRC]` (SR_vFOC)
2. Set parameters: `[90, homeTrig, homeDir, speed_hi, speed_lo, EndLimit, 0x00, CRC]`
3. Execute: `[91, CRC]`

### Mechanical Limit Homing
1. Set working mode: `[82, 05, CRC]`
2. Set parameters: `[90, 00, homeDir, speed_hi, speed_lo, EndLimit, 0x01, CRC]`
3. Set torque & offset: `[94, offset(int32), Hm_ma(uint16), CRC]`
4. Execute: `[91, CRC]`

### Single-Turn Power-On Zero-Return
1. Set working mode: `[82, 05, CRC]`
2. Set homing mode: `[90, 00, 00, speed_hi, speed_lo, 00, 0x02, CRC]`
3. Set parameters: `[9A, mode, enable, speed, dir, CRC]`
4. Zero-return happens automatically on next power-on.
