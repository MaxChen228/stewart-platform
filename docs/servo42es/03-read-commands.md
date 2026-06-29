# Read Commands (4.2 Read-only Parameters)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/03-read-commands|42D Read Commands]]

All read commands: send DLC=2 `[code, CRC]`, receive variable-length response. Frame shape per [[servo42es/01-protocol|protocol]]: Downlink `[CAN_ID][DLC][Byte1=code][data...][CRC]`, Uplink same shape. Data 為 big-endian。CRC = `(CAN_ID + 所有 data bytes) & 0xFF`（見 [[servo42es/01-protocol|CRC]]，與 42D byte-identical）。

**Encoder 注意**：磁編碼器硬體為單圈絕對（0~0x3FFF = 0~16383）。多圈累積由馬達 MCU 韌體軟體追蹤 wrap-around 實現。`Note: The encoder value is updated regardless of whether the motor is enabled or not`（4.2.1）—— 讀數不需 enable。

> 🔀 **vs 42D：** 42ES uplink 表頭明寫 `code` 佔 Byte1（與 42D 一致），但 0x40/0x31 等多處數值語意不同，逐條標註於下。

---

## 0x30 — Read Multi-Turn Encoder (4.2.1)

Send: `[30, CRC]` (DLC=2)

Response DLC=8:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `30H` |
| 2..5 | carry (int32) | encoder carry |
| 6..7 | value (uint16) | current encoder value, 0~0x3FFF |
| 8 | CRC | checksum |

- value: 0~0x3FFF（單圈原始值）
- carry: 圈數計數器。value > 0x3FFF 時 carry +=1；value < 0 時 carry -=1
- 組合座標 = carry × 0x4000 + value
- Example（PDF 原例）：current `carry|value = 0x3FF0`
  - after one turn **CCW** → `0x13FF0`（`+0x4000`）
  - after one turn **CW** → `0xFFFFFFFF3FF0`（`-0x4000`，carry int32 借位為 -1）

CRC 範例（CAN_ID=0x01）：`[01][02][30][CRC]` → CRC = `(0x01 + 0x30) & 0xFF` = **0x31**。完整下行 `01 02 30 31`。

> 🔀 **vs 42D：** value/carry 拆分與組合公式相同。42ES PDF 明確示範 CW 借位 uplink 為 `0xFFFFFFFF3FF0`（carry 視為 signed int32 = -1）；42D twin 同例文字描述為 `0x13FF0`（CCW）但未列 CW 借位 hex。

---

## 0x31 — Read Cumulative Multi-Turn Encoder (4.2.2)

Send: `[31, CRC]` (DLC=2)

Response DLC=8:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `31H` |
| 2..7 | value (int48) | cumulative multi-turn encoder value |
| 8 | CRC | checksum |

- 資料型別 `int48_t`（6 bytes，big-endian）
- CW: value **+= 0x4000** / turn；CCW: value **-= 0x4000** / turn
- Example（PDF 原例）：current `0x3FF0`
  - after one turn **CCW** → `0x7FF0`（`+0x4000`）
  - after one turn **CW** → `0xFFFFFFFFFFF0`（`-0x4000`）
- `Note: When moving relative to or absolutely according to coordinate values, use the encoder value as the coordinate.` —— 0xF4/0xF5 座標控制用此值當座標。

CRC 範例（CAN_ID=0x01）：`[01][02][31][CRC]` → CRC = `(0x01 + 0x31) & 0xFF` = **0x32**。PDF 標 `CRC(32)`。完整下行 `01 02 31 32`。

> 🔀 **vs 42D：** 方向符號與 42D **相反方向標示**：42ES 文件寫「CW: +0x4000 / CCW: -0x4000」；42D twin 寫「CW: +0x4000, CCW: -0x4000」於 0x35 段，但 0x31 段 twin 寫「CW: +0x4000/turn, CCW: -0x4000/turn」一致。**42ES CCW 範例算式為 `+0x4000`（0x3FF0→0x7FF0）**，與「CCW: -0x4000」的文字敘述自相矛盾 —— 以實機方向為準（cal 標定見 0x40）。42ES **無 0x35**（不受 0x92 影響的原始累積值）這條獨立指令；42D twin 有 0x35。

---

## 0x32 — Read Real-Time Speed (4.2.3)

Send: `[32, CRC]` (DLC=2)

Response DLC=4:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `32H` |
| 2..3 | motor speed (int16) | rpm |
| 4 | CRC | checksum |

- 資料型別 `int16_t`，unit RPM
- CCW: speed > 0；CW: speed < 0

CRC 範例（CAN_ID=0x01）：`[01][02][32][CRC]` → CRC = `(0x01 + 0x32) & 0xFF` = **0x33**。完整下行 `01 02 32 33`。

> 🔀 **vs 42D：** 完全一致（DLC=4、int16、CCW>0/CW<0）。

---

## 0x33 — Read Received Pulse Count (4.2.4)

Send: `[33, CRC]` (DLC=2)

Response DLC=6:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `33H` |
| 2..5 | pulse count (int32) | pulses received |
| 6 | CRC | checksum |

- 資料型別 `int32_t`

CRC 範例（CAN_ID=0x01）：`[01][02][33][CRC]` → CRC = `(0x01 + 0x33) & 0xFF` = **0x34**。完整下行 `01 02 33 34`。

> 🔀 **vs 42D：** 完全一致（DLC=6、int32）。

---

## 0x37 — Read Motor Alarm Status (4.2.5)

Send: `[37, CRC]` (DLC=2)

Response DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `37H` |
| 2 | state (uint8) | motor status（見下表） |
| 3 | CRC | checksum |

Motor status codes：

| Motor status | status |
|--------------|--------|
| motor running | 0 |
| Motor stopped | 1 |
| Over current error | 2 |
| Phase loss error | 3 |
| Overvoltage error | 4 |
| Undervoltage error | 5 |
| Position error | 6 |
| Encoder error | 7 |

CRC 範例（CAN_ID=0x01）：`[01][02][37][CRC]` → CRC = `(0x01 + 0x37) & 0xFF` = **0x38**。完整下行 `01 02 37 38`。

> 🔀 **vs 42D：** 42D twin 的 03-read-commands 檔**未列** 0x37 馬達警報狀態指令（其錯誤碼分散於別處 / 0x3D 系列）。42ES 將完整 8 狀態碼（含 over-current/phase-loss/over+under-voltage/position/encoder error）集中於 0x37 read 指令，DLC=3、single uint8。

---

## 0x39 — Read Position Angle Error (4.2.6)

Send: `[39, CRC]` (DLC=2)

Response DLC=6:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `39H` |
| 2..5 | angular error (int32) | position error |
| 6 | CRC | checksum |

- `Angle error is the difference between the position angle controlled by the command and the real-time angle position of the motor.`
- Unit: `0~51200 = 0°~360°`。例如 error = 1° → value = `51200 / 360 ≈ 142.22`
- 資料型別 `int32_t`

CRC 範例（CAN_ID=0x01）：`[01][02][39][CRC]` → CRC = `(0x01 + 0x39) & 0xFF` = **0x3A**。完整下行 `01 02 39 3A`。

> 🔀 **vs 42D：** 比例尺一致（51200 counts = 360°，≈142.22/deg）。**DLC 不同**：42ES 文件 uplink 表為 **DLC=6 + int32**（Byte2..5 errors）；42D twin 亦標 DLC=6、int32 —— 一致。

---

## 0x3A — Read Motor Enable Status (4.2.7)

Send: `[3A, CRC]` (DLC=2)

> `When using bus control, this command can be used to obtain the enable status of the driver board.`

Response DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `3AH` |
| 2 | enable state | status |
| 3 | CRC | checksum |

- `status = 01` → Motor is enabled
- `status = 00` → Motor not enabled

CRC 範例（CAN_ID=0x01）：`[01][02][3A][CRC]` → CRC = `(0x01 + 0x3A) & 0xFF` = **0x3B**。完整下行 `01 02 3A 3B`。

> 🔀 **vs 42D：** 完全一致（DLC=3、1=enabled / 0=disabled）。

---

## 0x3B — Read Return-to-Zero Status (4.2.8)

Send: `[3B, CRC]` (DLC=2)

Response DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `3BH` |
| 2 | return to zero state | status |
| 3 | CRC | checksum |

- `status = 00` → The motor has not returned to zero, **or the return-to-zero process failed**
- `status = 01` → The motor has returned to zero

CRC 範例（CAN_ID=0x01）：`[01][02][3B][CRC]` → CRC = `(0x01 + 0x3B) & 0xFF` = **0x3C**。PDF 標 `CRC(3C)`。完整下行 `01 02 3B 3C`。

> 🔀 **vs 42D：** **結構不同**。42D twin 回 **DLC=4**：`[3B, status1, status2, CRC]`，status1=single-turn 歸零態、status2=non-single-turn 歸零態，且 status 三值（0=going/1=success/2=fail）。42ES 簡化為 **DLC=3 單一 status**，僅 0/1 兩值（0 把「未歸零」與「歸零失敗」合併、無 going 中間態、無 single/non-single 拆分）。

---

## 0x01 — Auto-Return Read-Only Parameter at Intervals (4.2.9)

讓馬達每隔固定週期自動回傳某個 read-only 參數，免 host 反覆輪詢。

Send DLC=5:

| Byte | Field | Value |
|------|-------|-------|
| 1 | command | `01` |
| 2 | code | read-only 參數碼（e.g. `31`） |
| 3..4 | Timer (times) | period in ms (uint16)；`0` = 停用、不回傳 |
| 5 | CRC | checksum |

Response DLC=4:

| Byte | Field | Value |
|------|-------|-------|
| 1 | command | `01` |
| 2 | code | echo 參數碼 |
| 3 | state | `00`=Setup failed / `01`=Set successed |
| 4 | CRC | checksum |

Periodic uplink（DLC=n，視參數而定）：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | 參數碼 |
| 2..n-1 | param | parameter value（格式 = 該參數正常回應格式） |
| n | CRC | checksum |

**Worked example（PDF 原例，每 1000ms 讀 0x31 累積編碼器）：**
- Send: `01 01 31 03 E8 1E`
  - `03 E8` = 1000 (ms)；CRC = `(0x01 + 0x01 + 0x31 + 0x03 + 0xE8) & 0xFF` = `0x11E & 0xFF` = **0x1E** ✓
- Return: `01 01 31 01 34`
  - state=`01` success；CRC = `(0x01 + 0x01 + 0x31 + 0x01) & 0xFF` = **0x34** ✓
- 之後每 1000ms 自動回 `01 31 00 00 00 00 04 36`（8-byte，0x31 累積值格式）

> 🔀 **vs 42D：** 機制與 byte layout 一致（command=0x01、code、uint16 timer ms、0=disable）。worked example 數值 42ES 與 42D twin 相同邏輯。

---

## 0x00 — Read a Single Configuration Parameter (4.2.10)

讀回任一 **write-only** 配置參數的當前值（寫入後驗證用）。

Send DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | command | `00` |
| 2 | code | 參數碼（e.g. `82` = working mode、`83` = current、`96` = PID） |
| 3 | CRC | checksum |

Response DLC=n:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | echo 參數碼 |
| 2..n-1 | param | parameter value（格式須與設定該參數時一致） |
| n | CRC | checksum |

若該參數**不支援讀取**，回傳 `[code, FFFFH, CRC]`（param 全 FF）。

**Worked example（PDF 原例，讀 operating current 0x83）：**
- Send: `00 83 84`
  - CRC = `(0x01 + 0x00 + 0x83) & 0xFF` = **0x84** ✓（DLC=3）
- Return: `83 0C 80 10`
  - param `0C80` = 3200 mA；CRC = `(0x01 + 0x83 + 0x0C + 0x80) & 0xFF` = `0x110 & 0xFF` = **0x10** ✓

讀其他配置參數同理（`command=0x00` + 目標 config code），例如讀工作模式 `00 82 CRC`、讀細分 `00 84 CRC`。可讀的配置碼見 [[servo42es/04-config-commands|配置指令]]，工作模式定義見 [[servo42es/02-specs-and-modes|工作模式與規格]]。

> 🔀 **vs 42D：** 42D 可用 `0x00` 讀回 `0x96`/`0x97` PID 參數；**42ES 無 CAN 設/讀 PID 指令**（PID 由 CANGAROO 上位機管理，見 [[servo42es/07-pid-tuning|07-pid-tuning]]），故無 `00 96` 讀 PID 的用法。

> 🔀 **vs 42D：** 機制一致（command=0x00 + code）。不支援讀取時 42ES 回 `FFFFH`（兩 byte FF）；42D twin 寫 `FF FF`（同義）。**42ES-CRITICAL**：write-only 參數寫入後**非永久** —— 必須執行 **SAVE 指令 0x60** 才寫入 NVS（Part 4 note 3 + 4.5.1）；用 0x00 讀回可確認當前 RAM 值，但斷電保留與否取決於是否曾 0x60 save（見 [[servo42es/12-errors-recovery|0x60 save]]）。42D 文件無此顯式 save 指令。

---

## 0x40 — Read Version Information (4.2.11)

Send: `[40, CRC]` (DLC=2)

Response DLC=6:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `40H` |
| 2 (b7) | D/E distinction (series) | `1`=E series / `0`=D series |
| 2 (b5-b4) | Calibration status (cal) | `1`=encoder value **decreases** as motor rotates CW / `2`=encoder value **increases** as motor rotates CW |
| 2 (b3-b0) | Hardware version (hardVer) | board type（見下表） |
| 3..5 | Firmware version (firmVer[3]) | `firmVer[0..2]` |
| 6 | CRC | checksum |

- Firmware example: `firmVer[0]=1, firmVer[1]=0, firmVer[2]=0` → version **V1.0.0**
- Hardware version 對照（ES 系列專屬）：

| Board type | hardVer |
|------------|---------|
| S42ES_RS485 | 1 |
| S42ES_BUS | 2 |
| S57ES_RS485 | 3 |
| S57ES_BUS | 4 |
| S28ES_RS485 | 5 |
| S28ES_BUS | 6 |
| S35ES_RS485 | 7 |
| S35ES_BUS | 8 |

CRC 範例（CAN_ID=0x01）：`[01][02][40][CRC]` → CRC = `(0x01 + 0x40) & 0xFF` = **0x41**。完整下行 `01 02 40 41`。

> 🔀 **vs 42D：** **Byte2 nibble 語意大改**。42D twin：Byte2 b7-b4 = calibration status（0=未校正 / 1=已校正）、b3-b0 = hardware version；firmware 在 Bytes 3-4（uint16）。42ES：Byte2 **b7** = D/E series distinction（本品 series=1，E 系列）、**b5-b4** = cal 但語意改為**編碼器旋向標定**（cal=1 CW 時編碼值遞減 / cal=2 CW 時遞增，**非** 0/1 校正旗標）、b3-b0 = hardVer（ES 專屬 1~8 表，含 28/35/42/57ES × RS485/BUS）；firmware 為 **Bytes 3-5 三 byte 陣列** `firmVer[3]`（非 uint16）。CAN BUS 版對應 hardVer = 2(42ES_BUS)/4(57ES_BUS)。

---

## 0x42 — Read / Write User-Defined ID (4.2.12)

### Write user ID

Send DLC=6:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `42H` |
| 2..5 | Custom ID (uint32) | 使用者自訂 ID |
| 6 | CRC | checksum |

Response DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `42H` |
| 2 | Setting status | `0`=Write failed / `1`=Successfully written |
| 3 | CRC | checksum |

### Read user ID

Send DLC=2: `[42, CRC]`

Response DLC=6:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `42H` |
| 2..5 | Custom ID (uint32) | 已存的自訂 ID |
| 6 | CRC | checksum |

CRC 範例（read，CAN_ID=0x01）：`[01][02][42][CRC]` → CRC = `(0x01 + 0x42) & 0xFF` = **0x43**。完整下行 `01 02 42 43`。

> 🔀 **vs 42D：** 結構一致（write DLC=6 / read DLC=6、uint32 Custom ID、write 回 0/1 status）。**42ES-CRITICAL**：此 Custom ID 屬配置寫入，永久保留須配 **0x60 save**（[[servo42es/12-errors-recovery|0x60]]）。注意 user ID 與 CAN_ID（馬達位址，0x8B 設定）為**兩回事**（見 [[servo42es/01-protocol|CAN_ID/位址]]）。

---

## 相關章節

- [[servo42es/00-index|42ES 指令索引]]
- [[servo42es/01-protocol|幀格式 / CRC / CAN_ID]]
- [[servo42es/04-config-commands|寫入/配置指令]] · [[servo42es/12-errors-recovery|0x60 SAVE]]
- [[servo42es/02-specs-and-modes|工作模式與規格]] · [[servo42es/04-config-commands|配置參數]]
- [[servo42d/03-read-commands|42D 對應 Read Commands（twin）]]
- [[servo-can-hub|CAN 文檔總覽 / 對照矩陣]]
