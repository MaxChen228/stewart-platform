# MKS SERVO42ES/57ES CAN Manual — Agent Index

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 雙生索引 [[servo42d/00-index|SERVO42D 索引]]

Source: MKS SERVO42ES/57ES CAN User Manual V1.0.1 (firmware V1.0.1)

涵蓋 **SERVO42ES V1.0 與 SERVO57ES V1.0**（同韌體/協議；差別僅電流額定與 I/O 數：42ES 單 `IN`、57ES `IN_1`+`IN_2`+`IN_COM`）。Frame = 標準 CAN frame，data field ≤ 8 bytes。Downlink/Uplink 同形：`[CAN_ID][DLC][Byte1=code][data...][CRC]`，data **big-endian**。CRC = CHECKSUM 8-bit = `(CAN_ID + 所有 data bytes except CRC) & 0xFF`，**與 SERVO42D byte-identical**。CAN_ID = 馬達位址，`0x00`~`0x7FF`，預設 `0x01`；`0x00` = broadcast（slave **不回應**）；`0x01`~`0x10` 可於 OLED 設定，`>0x10` 須走 `0x8B`。寫入指令回 `DLC=3` `[code,status,CRC]`：status `0`=fail / `1`=success；運動類另有 `2`=completed / `3`=endstop-stopped / `5`=sync-received。

> ⚠️ **42ES-CRITICAL（持久化）：** write-only / config 參數寫入後**僅存在 RAM、不落 NVS**，必須執行 **SAVE 指令 `0x60`** 才一次性存檔（Part 4 note 3 + §4.5.1）。SERVO42D 文檔組**無**此顯式 save 指令。**任何要在斷電後保留的設定都要在序列末尾補一條 `0x60`。** 見 [[servo42es/12-errors-recovery|0x60 SAVE]]。

> ⚠️ **42ES-CRITICAL（PID）：** 42ES **沒有 `0x96`/`0x97` 之類的 CAN PID 設定指令** —— PID/三環 FOC 調參**只能**透過 **CANGAROO** 上位機軟體 + USB-CAN（MKS CANable / candle0）進行（Part 12）。任何 host 端包裝 `0x96` 的工具（本專案 `K`/`KS`/`KRESET`）對 42ES 無效。見 [[servo42es/07-pid-tuning|07-pid-tuning]]。

## Command Code → File Lookup

| Code | Name | File |
|------|------|------|
| 0x00 | Read single config parameter | [[servo42es/03-read-commands|03-read-commands]] |
| 0x01 | Auto-return read-only param at intervals | [[servo42es/03-read-commands|03-read-commands]] |
| 0x0F | No-verification mode (test-only, never saved) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x30 | Read multi-turn encoder (carry + value) | [[servo42es/03-read-commands|03-read-commands]] |
| 0x31 | Read cumulative encoder (int48, 座標源) | [[servo42es/03-read-commands|03-read-commands]] |
| 0x32 | Read real-time speed | [[servo42es/03-read-commands|03-read-commands]] |
| 0x33 | Read received pulse count | [[servo42es/03-read-commands|03-read-commands]] |
| 0x34 | Read IO port status | [[servo42es/10-io-ports|10-io-ports]] |
| 0x36 | Write IO port data (ALM/PEND) | [[servo42es/10-io-ports|10-io-ports]] |
| 0x37 | Read motor alarm status (8-state) | [[servo42es/03-read-commands|03-read-commands]] |
| 0x39 | Read position angle error | [[servo42es/03-read-commands|03-read-commands]] |
| 0x3A | Read motor enable status | [[servo42es/03-read-commands|03-read-commands]] |
| 0x3B | Read return-to-zero status | [[servo42es/03-read-commands|03-read-commands]] |
| 0x3D | Release stall | [[servo42es/09-protection|09-protection]] |
| 0x3E | Read stall status | [[servo42es/09-protection|09-protection]] |
| 0x3F | Restore factory settings (auto-restart) | [[servo42es/12-errors-recovery|12-errors-recovery]] |
| 0x40 | Read version info (D/E series, cal, hardVer) | [[servo42es/03-read-commands|03-read-commands]] |
| 0x41 | Reset and restart (config 不變) | [[servo42es/12-errors-recovery|12-errors-recovery]] |
| 0x42 | Read/Write user-defined ID | [[servo42es/03-read-commands|03-read-commands]] |
| 0x4A | Enable/disable sync flag | [[servo42es/11-multi-motor-sync|11-multi-motor-sync]] |
| 0x4B | Sync trigger (broadcast only) | [[servo42es/11-multi-motor-sync|11-multi-motor-sync]] |
| 0x60 | **SAVE parameters (42ES-only)** | [[servo42es/12-errors-recovery|12-errors-recovery]] |
| 0x80 | Calibrate encoder (卸載執行一次) | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x82 | Set working mode | [[servo42es/04-config-commands|04-config-commands]] |
| 0x83 | Set working current | [[servo42es/04-config-commands|04-config-commands]] |
| 0x84 | Set subdivision (`0`=256) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x85 | Set EN pin active level | [[servo42es/04-config-commands|04-config-commands]] |
| 0x86 | Set direction (亦改 bus 方向) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x87 | Set pulse delay (≠42D screen-off) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x88 | Set stall protection + over-tolerance | [[servo42es/09-protection|09-protection]] |
| 0x89 | Set heartbeat protection time | [[servo42es/09-protection|09-protection]] |
| 0x8A | Set CAN bit rate | [[servo42es/04-config-commands|04-config-commands]] |
| 0x8B | Set slave CAN ID (>0x10) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x8C | Configure slave response method | [[servo42es/04-config-commands|04-config-commands]] |
| 0x8D | Set group CAN ID | [[servo42es/11-multi-motor-sync|11-multi-motor-sync]] |
| 0x8F | Lock axis on bus-mode start (≠42D key-lock) | [[servo42es/04-config-commands|04-config-commands]] |
| 0x91 | Execute zero-return (origin / coord) | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x92 | Set current position as zero | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x95 | Set zero-return mode/dir/Hi/Lo speed | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x96 | Set zero-return torque & origin offset | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x97 | Configure homing trigger method & timeout | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| 0x98 | Set position-reach threshold | [[servo42es/09-protection|09-protection]] |
| 0x9F | IO pulse frequency division (PEND) | [[servo42es/10-io-ports|10-io-ports]] |
| 0x9E | Set limit-switch params (EndLimit/Level/reMap) | [[servo42es/08-homing-zeroing|08-homing-zeroing]] |
| **PID** | **無 CAN 指令** — CANGAROO-only tuning | [[servo42es/07-pid-tuning|07-pid-tuning]] |
| 0xF1 | Read motor operating status | [[servo42es/06-bus-control|06-bus-control]] |
| 0xF3 | Set motor enable | [[servo42es/06-bus-control|06-bus-control]] |
| 0xF4 | Relative coordinate motion | [[servo42es/05-motion-commands|05-motion-commands]] |
| 0xF5 | Absolute coordinate motion (即時更新, 本平台用) | [[servo42es/05-motion-commands|05-motion-commands]] |
| 0xF6 | Speed control mode | [[servo42es/05-motion-commands|05-motion-commands]] |
| 0xF7 | Emergency stop (>1000 RPM 慎用) | [[servo42es/06-bus-control|06-bus-control]] |
| 0xFD | Relative pulse motion | [[servo42es/05-motion-commands|05-motion-commands]] |
| 0xFE | Absolute pulse motion | [[servo42es/05-motion-commands|05-motion-commands]] |
| 0xFF | Auto-start on power-on (speed=0 停用) | [[servo42es/05-motion-commands|05-motion-commands]] |

> 🔀 **42ES vs 42D 碼差異：** 42ES **缺**：`0x35`（不受 0x92 影響的原始累積讀值）、**CAN PID 設定**（42D 的 `0x96` vFOC / `0x97` CLOSE 在 42ES 不存在）、`0x9A`（單圈歸零併入 `0x95`）、`0x90`/`0x94`（42D 歸零參數；42ES 改用 `0x95`/`0x96`/`0x97`）、`0x9D`（超差併入 `0x88`）、42D `0x9F` 的「IN_1 配置」功能（輸入/方向改走 `0x85`/`0x86`）。42ES **新增/重映射**：`0x60` SAVE、`0x37` 8 態警報、歸零三碼 `0x95`/`0x96`/`0x97`、位置到位門檻移至 `0x98`（42D 為 `0x95`）、心跳移至 `0x89`（42D 為 `0x98`）、脈衝分頻移至 `0x9F`（42D 為 `0x99`）、`0x9E` 完整限位設定。詳見下方「跨產品碼重映射」表。

## Disambiguation — Common Lookups

**「PID 調參」—— 42ES 與 42D 最大分歧：**
- **42D**：`0x96`(vFOC) / `0x97`(CLOSE) 直接走 CAN 設定 → 任何 bus master（含 ESP32）可改。
- **42ES**：**無任何 CAN PID 指令** → 只能用 **CANGAROO 上位機 + USB-CAN（MKS CANable / candle0）** 改，改完仍須 `0x60` 落盤 → [[servo42es/07-pid-tuning|07-pid-tuning]]。本專案 `K`/`KS`/`KRESET`、`spid`/`pidn` 對 42ES **失效**。

**「跨產品碼重映射」—— 同一 code 在 42D / 42ES 指不同功能（移植 42D 韌體必查）：**

| Code | 42D 功能 | 42ES 功能 |
|------|---------|-----------|
| `0x89` | （42D 未用於心跳） | 心跳保護（[[servo42es/09-protection|09]]，PDF §4.3.12 印 `89H`） |
| `0x95` | 位置到位門檻 | 歸零模式/方向/速度（[[servo42es/08-homing-zeroing|08]]） |
| `0x96` | vFOC PID | 歸零力矩 + origin offset（[[servo42es/08-homing-zeroing|08]]，**非 PID**） |
| `0x97` | CLOSE PID | 歸零觸發法 + 逾時（[[servo42es/08-homing-zeroing|08]]，**非 PID**） |
| `0x98` | 心跳保護 | 位置到位門檻（[[servo42es/09-protection|09]]，PDF §4.3.13 印 `98H`） |
| `0x99` | 脈衝分頻（57D Only） | （42ES 不用 `0x99`） |
| `0x9F` | Configure IN_1 mode | IO 脈衝分頻輸出（[[servo42es/10-io-ports|10]]，PDF §5.3 印 `9FH`） |
| `0x87` | 自動熄屏 | 脈衝延遲 |
| `0x8F` | 按鍵鎖 key-lock | bus 啟動鎖軸 |
| `0x9E` | DLC=3 port-map 開關 | DLC=5 完整限位設定 |

> ⚠️ 上表 `0x95`/`0x96`/`0x97`/`0x98` 四碼在兩產品線間整體洗牌、`0x9F` 與 `0x99` 對調語意。移植時每碼都要改。心跳=`0x89`、位置到位門檻=`0x98`、脈衝分頻=`0x9F` 均已對 PDF V1.0.1 逐表查證（非歧義）。

**「Zero」有四種不同含義：**
- **0x80 Calibrate encoder** → 硬體校正，卸載狀態下執行一次（回 no data，馬達重啟入校正）→ [[servo42es/08-homing-zeroing|08-homing-zeroing]]
- **0x92 Set zero point** → 軟體零點，設當前位置為座標原點（42ES DLC=3、Byte2=`0x00`）→ [[servo42es/08-homing-zeroing|08-homing-zeroing]]
- **0x91 Execute zero-return** → 搜尋物理零點（goZeroMode=0 origin / =1 coordinate）→ [[servo42es/08-homing-zeroing|08-homing-zeroing]]
- **0x95/0x96/0x97 歸零參數** → 模式/方向/速度、力矩/offset、觸發法/逾時（42ES 拆三碼，42D 擠進 `0x90`/`0x94`）→ [[servo42es/08-homing-zeroing|08-homing-zeroing]]

**「Enable」有不同層級：**
- **0xF3 Motor enable** → Bus mode 下啟用/禁用馬達（最常用）→ [[servo42es/06-bus-control|06-bus-control]]
- **0x85 EN pin level** → Pulse mode 下硬體腳位（新增 `02`=Hold 常使能）→ [[servo42es/04-config-commands|04-config-commands]]

**「Stop」有多種方式：**
- **0xF7 Emergency stop** → 立即硬急停（>1000 RPM 慎用）→ [[servo42es/06-bus-control|06-bus-control]]
- **0xF6/FD/FE/F4/F5 stop** → 各運動模式 speed=0 + acc 減速停（acc=0 立即停）→ [[servo42es/05-motion-commands|05-motion-commands]]

**「Direction」取決於模式：**
- **Bus mode** → F6/FD 的 dir bit (Byte2 bit7)；**或** `0x86`（42ES 的 0x86 亦改 bus 方向）→ [[servo42es/05-motion-commands|05-motion-commands]] / [[servo42es/04-config-commands|04-config-commands]]
- **Pulse mode** → `0x86` Set direction → [[servo42es/04-config-commands|04-config-commands]]

**「Encoder」讀取：**
- **0x30** → carry(int32) + value(uint16 單圈 0~0x3FFF)→ [[servo42es/03-read-commands|03-read-commands]]
- **0x31** → 累積多圈 int48（座標源，F4/F5/FE 用此值當座標）→ [[servo42es/03-read-commands|03-read-commands]]
- **無 0x35**（42ES 無此獨立原始累積讀指令；42D 才有）

**「Speed」實際 RPM 取決於 subdivision：**
- `actual_RPM = speed × 16 / subdivision`（16/32/64 細分下 speed 值 = RPM）
- speed 範圍 0–3000、acc 0–255，量化 `(256−acc)×50µs/step`，詳見 [[servo42es/02-specs-and-modes|02-specs-and-modes]] / [[servo42es/05-motion-commands|05-motion-commands]]

## Common Task → Command Flow

**首次啟動（Bus FOC 模式）：**
1. `0x82` Set mode → `0x05` (CAN Bus Closed-Loop FOC)
2. **`0x60` SAVE**（模式為 write 參數，不存則重啟回預設）
3. `0x92` Set zero point
4. `0xF3` Enable motor
5. `0xF5` Send position command

> ⚠️ **任何 config write 之後務必補 `0x60`。** 標準節奏：寫一批參數 → 一條 `0x60` → power-cycle 驗證。官方歸零/開機自啟範例（§6.x / §9.5）皆在參數後緊跟 `01 60 01 62`。

**讀取當前狀態：**
- 位置：`0x31`（累積座標）或 `0x30`（carry+單圈）
- 速度：`0x32`
- 是否啟用：`0x3A`
- 運行狀態：`0xF1`
- 堵轉：`0x3E`；警報（8 態）：`0x37`
- 寫入後驗證值：`0x00` 讀回任一 write-only 參數

**PID 調參（42ES 唯一路徑）：**
1. OLED 設 `Mode→05` / `CanRate→500K` / `CanID→01`
2. CANGAROO + MKS CANable（candle0, 500000, 87.5%）連線，`0x31` 讀編碼器確認鏈路
3. 在 CANGAROO 改增益 → **`0x60` SAVE** 落 NVS（CANGAROO 存 workspace ≠ 存馬達參數）
- 見 [[servo42es/07-pid-tuning|07-pid-tuning]]

**多機同步：**
- Broadcast（`0x00`）或 group（`0x8D`）發同一 frame → 全動、無 ack
- 不同指令同步：`0x4A` 設 sync flag（+`0x60`）→ 各馬達預載 F4/F5/F6/FD/FE → `0x4B` broadcast 觸發
- 見 [[servo42es/11-multi-motor-sync|11-multi-motor-sync]]

**歸零（origin switch 命令觸發）：**
1. `0x82`→05 · 2. `0x9E` 限位/觸發電平 · 3. `0x95` 模式/方向/Hi/Lo · 4. `0x96` 力矩/offset · 5. `0x97` 觸發法/逾時 · 6. **`0x60` SAVE** · 7. `0x91 00` 執行
- 見 [[servo42es/08-homing-zeroing|08-homing-zeroing]]

**異常處理：**
- 堵轉（LED 1 Red / 馬達解鎖）→ `0x3E` 確認 → `0x3D` 釋放 → [[servo42es/09-protection|09-protection]]
- 通訊中斷失控 → `0x89` 心跳保護（+`0x60`）→ [[servo42es/09-protection|09-protection]]
- LED 故障碼（1 Green + N Red）→ [[servo42es/12-errors-recovery|12-errors-recovery]]
- 清暫態故障保留設定 → `0x41` reset/restart → [[servo42es/12-errors-recovery|12-errors-recovery]]
- 恢復出廠（連 `0x60` 存檔一併清）→ `0x3F` → [[servo42es/12-errors-recovery|12-errors-recovery]]

## File Overview

| File | Scope |
|------|-------|
| [[servo42es/01-protocol|01-protocol]] | CAN frame, CRC (byte-identical 42D), units, encoder encoding, 0x60 持久化通則 |
| [[servo42es/02-specs-and-modes|02-specs-and-modes]] | 硬體規格、工作模式、speed/acc 參數、42ES/57ES 差異 |
| [[servo42es/03-read-commands|03-read-commands]] | 唯讀查詢：encoder/speed/pulse/error/alarm/enable/zero-status/version/user-ID（無 0x35） |
| [[servo42es/04-config-commands|04-config-commands]] | write-only 設定：mode/current/subdiv/EN-level/dir/pulse-delay/CAN/response/lock-axis/no-verify（皆須 0x60） |
| [[servo42es/05-motion-commands|05-motion-commands]] | speed(F6)、pulse pos(FD/FE)、coord pos(F4/F5)、stop、auto-start(FF) |
| [[servo42es/06-bus-control|06-bus-control]] | enable(F3)、operating status(F1)、emergency stop(F7) |
| [[servo42es/07-pid-tuning|07-pid-tuning]] | **無 CAN PID 指令** — CANGAROO + USB-CAN only；本專案韌體影響 |
| [[servo42es/08-homing-zeroing|08-homing-zeroing]] | 歸零五碼(0x95/96/97/92/91)、calibrate(0x80)、Part 7 左右限位開關(0x9E) |
| [[servo42es/09-protection|09-protection]] | 堵轉+超差(0x88)、stall read/release(0x3E/0x3D)、heartbeat(0x89)、pos-reach(0x98) |
| [[servo42es/10-io-ports|10-io-ports]] | read/write IO(0x34/0x36)、pulse 分頻(0x9F)、EN-level/dir context、42ES 單 IN vs 57ES 雙 IN |
| [[servo42es/11-multi-motor-sync|11-multi-motor-sync]] | broadcast、group ID(0x8D)、sync flag(0x4A)、sync trigger(0x4B) |
| [[servo42es/12-errors-recovery|12-errors-recovery]] | LED 故障碼表、**0x60 SAVE**、factory reset(0x3F)、reset/restart(0x41)、Part 13 FAQ |

> 雙生對照矩陣與兩線差異總覽見 [[servo-can-hub|CAN 文檔總覽]]；逐章 42D 對應見各章開頭 `🔀 42D 對應` 連結與 [[servo42d/00-index|SERVO42D 索引]]。
