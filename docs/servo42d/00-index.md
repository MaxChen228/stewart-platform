# MKS SERVO42D/57D CAN Manual — Agent Index

Source: MKS SERVO42&57D_CAN User Manual V1.0.9

## Command Code → File Lookup

| Code | Name | File |
|------|------|------|
| 0x00 | Read config parameter | [03-read-commands](03-read-commands.md) |
| 0x01 | Auto-return read-only param | [03-read-commands](03-read-commands.md) |
| 0x0F | No-verification mode | [04-config-commands](04-config-commands.md) |
| 0x30 | Read multi-turn encoder | [03-read-commands](03-read-commands.md) |
| 0x31 | Read cumulative encoder | [03-read-commands](03-read-commands.md) |
| 0x32 | Read real-time speed | [03-read-commands](03-read-commands.md) |
| 0x33 | Read pulse count | [03-read-commands](03-read-commands.md) |
| 0x34 | Read IO port status | [10-io-ports](10-io-ports.md) |
| 0x35 | Read raw accumulated encoder | [03-read-commands](03-read-commands.md) |
| 0x36 | Write IO port data | [10-io-ports](10-io-ports.md) |
| 0x39 | Read position angle error | [03-read-commands](03-read-commands.md) |
| 0x3A | Read motor enable status | [03-read-commands](03-read-commands.md) |
| 0x3B | Read return-to-zero status | [03-read-commands](03-read-commands.md) |
| 0x3D | Release stall | [09-protection](09-protection.md) |
| 0x3E | Read stall status | [09-protection](09-protection.md) |
| 0x3F | Restore factory settings | [12-errors-recovery](12-errors-recovery.md) |
| 0x40 | Read version info | [03-read-commands](03-read-commands.md) |
| 0x41 | Reset and restart | [12-errors-recovery](12-errors-recovery.md) |
| 0x42 | Read/Write user ID | [03-read-commands](03-read-commands.md) |
| 0x4A | Enable/disable sync flag | [11-multi-motor-sync](11-multi-motor-sync.md) |
| 0x4B | Sync trigger | [11-multi-motor-sync](11-multi-motor-sync.md) |
| 0x80 | Calibrate encoder | [04-config-commands](04-config-commands.md) |
| 0x82 | Set working mode | [04-config-commands](04-config-commands.md) |
| 0x83 | Set working current | [04-config-commands](04-config-commands.md) |
| 0x84 | Set subdivisions | [04-config-commands](04-config-commands.md) |
| 0x85 | Set EN pin level | [04-config-commands](04-config-commands.md) |
| 0x86 | Set direction | [04-config-commands](04-config-commands.md) |
| 0x87 | Set auto screen-off | [04-config-commands](04-config-commands.md) |
| 0x88 | Set overcurrent protection | [09-protection](09-protection.md) |
| 0x89 | Set subdivision interpolation | [04-config-commands](04-config-commands.md) |
| 0x8A | Set CAN bit rate | [04-config-commands](04-config-commands.md) |
| 0x8B | Set slave CAN ID | [04-config-commands](04-config-commands.md) |
| 0x8C | Configure slave response | [04-config-commands](04-config-commands.md) |
| 0x8D | Set group CAN ID | [11-multi-motor-sync](11-multi-motor-sync.md) |
| 0x8F | Set key lock | [04-config-commands](04-config-commands.md) |
| 0x90 | Set homing parameters | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x91 | Execute zero-return | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x92 | Set current position as zero | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x94 | Set homing torque & offset | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x95 | Set position reach threshold | [04-config-commands](04-config-commands.md) |
| 0x96 | Set vFOC PID | [07-pid-tuning](07-pid-tuning.md) |
| 0x97 | Set CLOSE PID | [07-pid-tuning](07-pid-tuning.md) |
| 0x98 | Set heartbeat protection | [09-protection](09-protection.md) |
| 0x99 | Pulse frequency division | [10-io-ports](10-io-ports.md) |
| 0x9A | Set single-cycle zero-return | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x9B | Set holding current % | [04-config-commands](04-config-commands.md) |
| 0x9D | Set position out-of-tolerance | [09-protection](09-protection.md) |
| 0x9E | Configure port mapping | [08-homing-zeroing](08-homing-zeroing.md) |
| 0x9F | Configure IN_1 mode | [10-io-ports](10-io-ports.md) |
| 0xF1 | Read motor operating status | [06-bus-control](06-bus-control.md) |
| 0xF3 | Set motor enable | [06-bus-control](06-bus-control.md) |
| 0xF4 | Relative coordinate motion | [05-motion-commands](05-motion-commands.md) |
| 0xF5 | Absolute coordinate motion | [05-motion-commands](05-motion-commands.md) |
| 0xF6 | Speed control mode | [05-motion-commands](05-motion-commands.md) |
| 0xF7 | Emergency stop | [06-bus-control](06-bus-control.md) |
| 0xFD | Relative pulse motion | [05-motion-commands](05-motion-commands.md) |
| 0xFE | Absolute pulse motion | [05-motion-commands](05-motion-commands.md) |
| 0xFF | Auto-start on power-on | [05-motion-commands](05-motion-commands.md) |

## Disambiguation — Common Lookups

**「Zero」有三種不同含義：**
- **0x80 Calibrate encoder** → 硬體校正，卸載狀態下執行一次 → [04-config-commands](04-config-commands.md)
- **0x92 Set zero point** → 軟體零點，設定當前位置為座標原點 → [08-homing-zeroing](08-homing-zeroing.md)
- **0x91 Execute homing** → 搜尋物理零點（限位開關/機械堵轉/單圈）→ [08-homing-zeroing](08-homing-zeroing.md)

**「Enable」有不同層級：**
- **0xF3 Motor enable** → Bus mode 下啟用/禁用馬達（最常用）→ [06-bus-control](06-bus-control.md)
- **0x85 EN pin level** → Pulse mode 下的硬體腳位設定 → [04-config-commands](04-config-commands.md)

**「Stop」有多種方式：**
- **0xF7 Emergency stop** → 立即停止（>1000RPM 慎用）→ [06-bus-control](06-bus-control.md)
- **0xF6/FD/FE/F4/F5 stop** → 各運動模式各自的減速停止 → [05-motion-commands](05-motion-commands.md)

**「Direction」取決於模式：**
- **Bus mode** → 由 0xF6/FD 的 dir bit (Byte2 bit7) 控制 → [05-motion-commands](05-motion-commands.md)
- **Pulse mode** → 由 0x86 Set direction 控制 → [04-config-commands](04-config-commands.md)

**「Encoder」讀取：**
- **0x30** → 單圈位置 + 圈數（carry + value）→ [03-read-commands](03-read-commands.md)
- **0x31** → 累積座標值（受 0x92 零點影響，用於 F4/F5 座標控制）→ [03-read-commands](03-read-commands.md)
- **0x35** → 原始累積值（不受 0x92 影響）→ [03-read-commands](03-read-commands.md)

**「Speed」實際 RPM 取決於 subdivision：**
- `actual_RPM = speed × 16 / subdivision`（16/32/64 subdivision 下 speed 值 = RPM）
- 詳見 [02-specs-and-modes](02-specs-and-modes.md)

## Common Task → Command Flow

**首次啟動（Bus vFOC 模式）：**
1. `0x82` Set mode → SR_vFOC (0x05)
2. `0x92` Set zero point
3. `0xF3` Enable motor
4. `0xF5` Send position command

**讀取當前狀態：**
- 位置：`0x31`（座標）或 `0x30`（單圈+圈數）
- 速度：`0x32`
- 是否啟用：`0x3A`
- 運行狀態：`0xF1`
- 堵轉：`0x3E`

**異常處理：**
- OLED 顯示 Wrong → `0x3D` 釋放堵轉 → 見 [09-protection](09-protection.md)
- 通訊異常 → `0x98` 心跳保護 → 見 [09-protection](09-protection.md)
- 恢復出廠 → `0x3F` → 見 [12-errors-recovery](12-errors-recovery.md)

## File Overview

| File | Scope |
|------|-------|
| [01-protocol](01-protocol.md) | CAN frame format, CRC, units, coordinate system, absolute vs relative |
| [02-specs-and-modes](02-specs-and-modes.md) | Hardware params, 6 working modes, speed/acc parameters |
| [03-read-commands](03-read-commands.md) | All read-only queries: encoder, speed, pulses, error, status, version |
| [04-config-commands](04-config-commands.md) | Write-only setup: mode, current, subdivision, CAN, response, etc. |
| [05-motion-commands](05-motion-commands.md) | Speed (F6), position pulse (FD/FE), position coordinate (F4/F5), stop, auto-start |
| [06-bus-control](06-bus-control.md) | Enable/disable (F3), operating status (F1), emergency stop (F7) |
| [07-pid-tuning](07-pid-tuning.md) | vFOC (0x96) and CLOSE (0x97) PID parameters |
| [08-homing-zeroing](08-homing-zeroing.md) | All zero-return methods, parameters, port mapping |
| [09-protection](09-protection.md) | Overcurrent, position out-of-tolerance, heartbeat, stall release |
| [10-io-ports](10-io-ports.md) | Read/write IO, IN_1 mode config, pulse frequency division |
| [11-multi-motor-sync](11-multi-motor-sync.md) | Broadcast, group CAN ID, synchronization flag |
| [12-errors-recovery](12-errors-recovery.md) | OLED error messages, factory reset, restart |
