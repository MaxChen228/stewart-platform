# Read Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/03-read-commands|42ES Read Commands]]

All read commands: send DLC=2 `[code, CRC]`, receive variable-length response.

**Encoder 注意**：MT6816 硬體為 14-bit 單圈絕對編碼器（0~16383）。多圈累積由馬達 MCU 韌體軟體追蹤 wrap-around 實現，非硬體功能。斷電後累積值丟失（除非有 NVS 保存機制）。

> 🔀 **vs 42ES：** 42ES 磁編碼器為 **MT6826**（非 42D 的 MT6816），文檔未明列位元數；幀格式與 CRC 位元組相容，但 0x40 / 0x31 / 0x3B 等多處數值語意不同，逐條標註於下（見 [[servo42es/03-read-commands|42ES]]）。

## 0x30 — Read Multi-Turn Encoder

Send: `[30, CRC]` (DLC=2)
Response DLC=8: `[30, carry(int32), value(uint16), CRC]`

- value: 0~0x3FFF（硬體編碼器單圈原始值）
- carry: MCU 軟體圈數計數器。value 從 0x3FFF→0 時 carry+=1，value 從 0→0x3FFF 時 carry-=1
- 組合座標 = carry × 0x4000 + value
- Example: carry|value = 0x3FF0 → after one turn CCW → 0x13FF0 (+0x4000)

## 0x31 — Read Cumulative Multi-Turn Encoder

Send: `[31, CRC]` (DLC=2)
Response DLC=8: `[31, value(int48), CRC]`

- MCU 軟體累積值，CW: +0x4000/turn，CCW: -0x4000/turn
- 用於 0xF4/0xF5 座標控制的座標值
- **受 0x92 零點設定影響**：0x92 後此值重置為 0

## 0x32 — Read Real-Time Speed

Send: `[32, CRC]` (DLC=2)
Response DLC=4: `[32, speed(int16), CRC]`

- CCW: speed > 0 (RPM)
- CW: speed < 0 (RPM)

## 0x33 — Read Received Pulse Count

Send: `[33, CRC]` (DLC=2)
Response DLC=6: `[33, pulses(int32), CRC]`

## 0x35 — Read Raw Accumulated Multi-Turn Encoder

Send: `[35, CRC]` (DLC=2)
Response DLC=8: `[35, value(int48), CRC]`

- 與 0x31 相同的 MCU 軟體累積值，但 **不受 0x92 零點設定影響**
- CW: +0x4000/turn, CCW: -0x4000/turn
- 用途：自行管理零點時用 0x35 取得不被 0x92 干擾的原始累積位置

> 🔀 **vs 42ES：** 42ES **無此 0x35 獨立指令**（不受 0x92 影響的原始累積值）。自管零點時 42ES 只能用 0x31，須自行扣掉 0x92 偏移。見 [[servo42es/03-read-commands|42ES]]。

## 0x39 — Read Position Angle Error

Send: `[39, CRC]` (DLC=2)
Response DLC=6: `[39, error(int32), CRC]`

- Unit: 0~51200 = 0°~360° (51200/360 ≈ 142.22 per degree)
- Difference between commanded position and actual position.

## 0x3A — Read Motor Enable Status

Send: `[3A, CRC]` (DLC=2)
Response DLC=3: `[3A, enable(uint8), CRC]`

- 1=enabled, 0=disabled

## 0x3B — Read Return-to-Zero Status

Send: `[3B, CRC]` (DLC=2)
Response DLC=4: `[3B, status1, status2, CRC]`

- status: 0=going, 1=success, 2=fail
- status1 = single-turn zero return state
- status2 = non-single-turn zero return state

> 🔀 **vs 42ES：** 結構不同。42ES 回 **DLC=3 單一 status**，僅 0/1 兩值（0 把「未歸零」與「歸零失敗」合併、無 going 中間態、無 single/non-single 拆分）。見 [[servo42es/03-read-commands|42ES]]。

## 0x40 — Read Version Info

Send: `[40, CRC]` (DLC=2)
Response DLC=5: `[40, cal|hardVer, firmVer(uint16), CRC]`

- Byte2 high nibble (b7-b4) = calibration status (0=not calibrated, 1=calibrated)
- Byte2 low nibble (b3-b0) = hardware version (see [[servo42d/02-specs-and-modes|specs]])
- Bytes 3-4 = firmware version (uint16)

> 🔀 **vs 42ES：** Byte2 nibble 語意大改。42ES：**b7** = D/E series distinction（E 系列=1）、**b5-b4** = cal 但語意改為**編碼器旋向標定**（cal=1 CW 時編碼值遞減 / cal=2 遞增，非 0/1 校正旗標）、b3-b0 = ES 專屬 hardVer（1~8 涵蓋 28/35/42/57ES × RS485/BUS）；firmware 為 **Bytes 3-5 三 byte 陣列** `firmVer[3]`（非 uint16）。規格亦不同：42ES MCU = STM32F302（42D = N32L40x）、磁編碼器 MT6826、電壓 20-48V，見 [[servo42es/02-specs-and-modes|42ES specs]]。

## 0x42 — Read/Write User ID

**Write**: DLC=6: `[42, ID(uint32), CRC]`
Response DLC=3: `[42, status, CRC]`

**Read**: DLC=2: `[42, CRC]`
Response DLC=6: `[42, ID(uint32), CRC]`

> 🔀 **vs 42ES：** 結構一致（write/read DLC=6、uint32 ID、write 回 0/1 status）。但 42ES 的 Custom ID 屬配置寫入，**永久保留須配 0x60 SAVE 指令**（42D 無此顯式 save）。見 [[servo42es/03-read-commands|42ES]]。

## 0x01 — Auto-Return Read-Only Parameter

Configures periodic automatic return of any read-only parameter.

Send DLC=5: `[01, code, timer_hi, timer_lo, CRC]`
- code: read-only parameter code (e.g., 0x31)
- timer: period in ms (uint16). 0 = disable.

Response DLC=4: `[01, code, status, CRC]` (0=fail, 1=success)

After setup, motor auto-sends the parameter at the set interval. Data format matches the parameter's normal response.

## 0x00 — Read Configuration Parameter

Read any write-only configuration parameter's current value.

Send DLC=3: `[00, code, CRC]`
- code: parameter code (e.g., 0x82 for working mode, 0x96 for PID)

Response: `[code, param_value..., CRC]`
If unsupported: `[code, 0xFF, 0xFF, CRC]`

PID read examples:
- Read vFOC Kp/Ki: `[00, 96, 00, CRC]`
- Read vFOC Kd/Kv: `[00, 96, 01, CRC]`

> 🔀 **vs 42ES：** 0x00 讀回機制一致（command=0x00 + code，不支援讀取回 `FF FF`）。但 42ES write-only 參數寫入後**非永久**——用 0x00 讀回確認的是當前 RAM 值，斷電保留與否取決於是否曾執行 **0x60 SAVE**（42D 無此顯式 save 指令）。見 [[servo42es/03-read-commands|42ES]]。
