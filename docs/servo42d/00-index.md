# MKS SERVO42D/57D CAN Manual — Agent Index

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/00-index|SERVO42ES/57ES CAN Manual — Agent Index]]

Source: MKS SERVO42&57D_CAN User Manual V1.0.9

## Command Code → File Lookup

| Code | Name | File |
|------|------|------|
| 0x00 | Read config parameter | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x01 | Auto-return read-only param | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x0F | No-verification mode | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x30 | Read multi-turn encoder | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x31 | Read cumulative encoder | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x32 | Read real-time speed | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x33 | Read pulse count | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x34 | Read IO port status | [[servo42d/10-io-ports\|10-io-ports]] |
| 0x35 | Read raw accumulated encoder | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x36 | Write IO port data | [[servo42d/10-io-ports\|10-io-ports]] |
| 0x39 | Read position angle error | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x3A | Read motor enable status | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x3B | Read return-to-zero status | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x3D | Release stall | [[servo42d/09-protection\|09-protection]] |
| 0x3E | Read stall status | [[servo42d/09-protection\|09-protection]] |
| 0x3F | Restore factory settings | [[servo42d/12-errors-recovery\|12-errors-recovery]] |
| 0x40 | Read version info | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x41 | Reset and restart | [[servo42d/12-errors-recovery\|12-errors-recovery]] |
| 0x42 | Read/Write user ID | [[servo42d/03-read-commands\|03-read-commands]] |
| 0x4A | Enable/disable sync flag | [[servo42d/11-multi-motor-sync\|11-multi-motor-sync]] |
| 0x4B | Sync trigger | [[servo42d/11-multi-motor-sync\|11-multi-motor-sync]] |
| 0x80 | Calibrate encoder | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x82 | Set working mode | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x83 | Set working current | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x84 | Set subdivisions | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x85 | Set EN pin level | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x86 | Set direction | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x87 | Set auto screen-off | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x88 | Set overcurrent protection | [[servo42d/09-protection\|09-protection]] |
| 0x89 | Set subdivision interpolation | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x8A | Set CAN bit rate | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x8B | Set slave CAN ID | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x8C | Configure slave response | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x8D | Set group CAN ID | [[servo42d/11-multi-motor-sync\|11-multi-motor-sync]] |
| 0x8F | Set key lock | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x90 | Set homing parameters | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x91 | Execute zero-return | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x92 | Set current position as zero | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x94 | Set homing torque & offset | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x95 | Set position reach threshold | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x96 | Set vFOC PID | [[servo42d/07-pid-tuning\|07-pid-tuning]] |
| 0x97 | Set CLOSE PID | [[servo42d/07-pid-tuning\|07-pid-tuning]] |
| 0x98 | Set heartbeat protection | [[servo42d/09-protection\|09-protection]] |
| 0x99 | Pulse frequency division | [[servo42d/10-io-ports\|10-io-ports]] |
| 0x9A | Set single-cycle zero-return | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x9B | Set holding current % | [[servo42d/04-config-commands\|04-config-commands]] |
| 0x9D | Set position out-of-tolerance | [[servo42d/09-protection\|09-protection]] |
| 0x9E | Configure port mapping | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] |
| 0x9F | Configure IN_1 mode | [[servo42d/10-io-ports\|10-io-ports]] |
| 0xF1 | Read motor operating status | [[servo42d/06-bus-control\|06-bus-control]] |
| 0xF3 | Set motor enable | [[servo42d/06-bus-control\|06-bus-control]] |
| 0xF4 | Relative coordinate motion | [[servo42d/05-motion-commands\|05-motion-commands]] |
| 0xF5 | Absolute coordinate motion | [[servo42d/05-motion-commands\|05-motion-commands]] |
| 0xF6 | Speed control mode | [[servo42d/05-motion-commands\|05-motion-commands]] |
| 0xF7 | Emergency stop | [[servo42d/06-bus-control\|06-bus-control]] |
| 0xFD | Relative pulse motion | [[servo42d/05-motion-commands\|05-motion-commands]] |
| 0xFE | Absolute pulse motion | [[servo42d/05-motion-commands\|05-motion-commands]] |
| 0xFF | Auto-start on power-on | [[servo42d/05-motion-commands\|05-motion-commands]] |

> 🔀 **vs 42ES：** PID 設定指令 `0x96`/`0x97` 在 42ES **完全移除**（CAN 無法寫 PID，僅能用 CANGAROO USB-CAN 上位機）；working-mode `0x82` 的 mode 數字映射也不同（勿沿用 42D 的數字）。詳見 [[servo42es/00-index|42ES]]。

## Disambiguation — Common Lookups

**「Zero」有三種不同含義：**
- **0x80 Calibrate encoder** → 硬體校正，卸載狀態下執行一次 → [[servo42d/04-config-commands|04-config-commands]]
- **0x92 Set zero point** → 軟體零點，設定當前位置為座標原點 → [[servo42d/08-homing-zeroing|08-homing-zeroing]]
- **0x91 Execute homing** → 搜尋物理零點（限位開關/機械堵轉/單圈）→ [[servo42d/08-homing-zeroing|08-homing-zeroing]]

**「Enable」有不同層級：**
- **0xF3 Motor enable** → Bus mode 下啟用/禁用馬達（最常用）→ [[servo42d/06-bus-control|06-bus-control]]
- **0x85 EN pin level** → Pulse mode 下的硬體腳位設定 → [[servo42d/04-config-commands|04-config-commands]]

**「Stop」有多種方式：**
- **0xF7 Emergency stop** → 立即停止（>1000RPM 慎用）→ [[servo42d/06-bus-control|06-bus-control]]
- **0xF6/FD/FE/F4/F5 stop** → 各運動模式各自的減速停止 → [[servo42d/05-motion-commands|05-motion-commands]]

**「Direction」取決於模式：**
- **Bus mode** → 由 0xF6/FD 的 dir bit (Byte2 bit7) 控制 → [[servo42d/05-motion-commands|05-motion-commands]]
- **Pulse mode** → 由 0x86 Set direction 控制 → [[servo42d/04-config-commands|04-config-commands]]

**「Encoder」讀取：**
- **0x30** → 單圈位置 + 圈數（carry + value）→ [[servo42d/03-read-commands|03-read-commands]]
- **0x31** → 累積座標值（受 0x92 零點影響，用於 F4/F5 座標控制）→ [[servo42d/03-read-commands|03-read-commands]]
- **0x35** → 原始累積值（不受 0x92 影響）→ [[servo42d/03-read-commands|03-read-commands]]

**「Speed」實際 RPM 取決於 subdivision：**
- `actual_RPM = speed × 16 / subdivision`（16/32/64 subdivision 下 speed 值 = RPM）
- 詳見 [[servo42d/02-specs-and-modes|02-specs-and-modes]]

## Common Task → Command Flow

**首次啟動（Bus vFOC 模式）：**
1. `0x82` Set mode → SR_vFOC (0x05)
2. `0x92` Set zero point
3. `0xF3` Enable motor
4. `0xF5` Send position command

> 🔀 **vs 42ES：** 此流程的 `0x82` mode 值在 42ES 不同——42ES bus closed-loop FOC = `0x05H`（描述式分組命名，非 42D 的 SR_vFOC 編號巧合相同；其餘 mode 數字皆異），務必對照 [[servo42es/02-specs-and-modes|42ES work modes]] 再下值。

**讀取當前狀態：**
- 位置：`0x31`（座標）或 `0x30`（單圈+圈數）
- 速度：`0x32`
- 是否啟用：`0x3A`
- 運行狀態：`0xF1`
- 堵轉：`0x3E`

**異常處理：**
- OLED 顯示 Wrong → `0x3D` 釋放堵轉 → 見 [[servo42d/09-protection|09-protection]]
- 通訊異常 → `0x98` 心跳保護 → 見 [[servo42d/09-protection|09-protection]]
- 恢復出廠 → `0x3F` → 見 [[servo42d/12-errors-recovery|12-errors-recovery]]

## File Overview

| File | Scope |
|------|-------|
| [[servo42d/01-protocol\|01-protocol]] | CAN frame format, CRC, units, coordinate system, absolute vs relative |
| [[servo42d/02-specs-and-modes\|02-specs-and-modes]] | Hardware params, 6 working modes, speed/acc parameters |
| [[servo42d/03-read-commands\|03-read-commands]] | All read-only queries: encoder, speed, pulses, error, status, version |
| [[servo42d/04-config-commands\|04-config-commands]] | Write-only setup: mode, current, subdivision, CAN, response, etc. |
| [[servo42d/05-motion-commands\|05-motion-commands]] | Speed (F6), position pulse (FD/FE), position coordinate (F4/F5), stop, auto-start |
| [[servo42d/06-bus-control\|06-bus-control]] | Enable/disable (F3), operating status (F1), emergency stop (F7) |
| [[servo42d/07-pid-tuning\|07-pid-tuning]] | vFOC (0x96) and CLOSE (0x97) PID parameters |
| [[servo42d/08-homing-zeroing\|08-homing-zeroing]] | All zero-return methods, parameters, port mapping |
| [[servo42d/09-protection\|09-protection]] | Overcurrent, position out-of-tolerance, heartbeat, stall release |
| [[servo42d/10-io-ports\|10-io-ports]] | Read/write IO, IN_1 mode config, pulse frequency division |
| [[servo42d/11-multi-motor-sync\|11-multi-motor-sync]] | Broadcast, group CAN ID, synchronization flag |
| [[servo42d/12-errors-recovery\|12-errors-recovery]] | OLED error messages, factory reset, restart |

## 跨產品對照（42D ↔ 42ES）

> 🔀 **vs 42ES — 三大關鍵分歧**（僅列已確認差異，逐章細節見 42ES 對應頁）：
> - **PID 調參**：42D 走 CAN（`0x96` vFOC / `0x97` CLOSE，任何 bus master 可寫）；42ES **CAN 無 PID 指令**，只能用 **CANGAROO 上位機 + USB-CAN dongle**。對應 [[servo42es/07-pid-tuning|42ES 07-pid-tuning]]。
> - **參數存盤**：42D 文檔集無顯式 SAVE，write 類即生效；42ES write-only 參數須額外執行 **`0x60` SAVE** 才落 NVS，否則下電丟失。對應 [[servo42es/04-config-commands|42ES 04-config-commands]]。
> - **硬體平台**：42D = N32L40x MCU / MT6816 14-bit 編碼器 / 12-24V；42ES = **STM32F302 / MT6826 / 20-48V**（功率級上拉、max pulse 300kHz、新增 RS485+MODBUS）。對應 [[servo42es/02-specs-and-modes|42ES 02-specs-and-modes]]。
>
> 應用層 CAN 位元組多數相容（`0xF5`/`0x92`/`0xF7`/`0x30`/`0x31` 等運動與讀取指令、CRC 算法一致）；不相容處集中在上述三項。
