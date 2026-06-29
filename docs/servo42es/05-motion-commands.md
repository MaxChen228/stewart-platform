# Motion Commands (SERVO42ES / 57ES)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/05-motion-commands|Motion Commands]]

All motion commands are only valid in **bus control mode** (SR_OPEN / SR_CLOSE / SR_vFOC). Manual V1.0.1 「Note 1」明示本章指令僅在 bus control mode 生效（Part 8/9/10 開頭重申）。產品線涵蓋 **SERVO42ES V1.0** 與 **SERVO57ES V1.0**（同韌體 V1.0.1、同協議，差在電流額定與 I/O：42ES 單 IN、57ES 有 IN_1+IN_2+IN_COM）。

**Prerequisites**: 發送運動指令前必須：
1. 設定工作模式為 SR_* → `0x82` (見 [[servo42es/04-config-commands|04-config-commands]])
2. Enable 馬達 → `0xF3` (見 [[servo42es/06-bus-control|06-bus-control]])
3. 座標模式 (F4/F5/FE) 需先設零點 → `0x92` 或 `0x91` (見 [[servo42es/08-homing-zeroing|08-homing-zeroing]])

> 🔀 **vs 42D：** 42ES 多一層強制存檔規則 —— write/config 類參數（含工作模式 `0x82`）**寫入後不持久化，直到送 SAVE 指令 `0x60`**（Manual Part 4 note 3 + §4.5.1）。SERVO42D 文檔組無此顯式存檔指令。**運動指令本身（F6/FD/FE/F4/F5）是動作觸發、非持久參數，不需 `0x60`**；但若要靠 `0xFF` 開機自動運行（見文末），其 dir/speed/acc 設定必須 `0x60` 存檔才會在斷電後保留。官方範例（§9.4 / §10.x）皆在 `82` 之後緊跟 `60 01 62` 存檔。

**CRC**：本章所有 frame 的校驗 = `(CAN_ID + 所有 data bytes except CRC) & 0xFF`，big-endian，與 SERVO42D **byte-identical**。細節見 [[servo42es/01-protocol|01-protocol]]。本檔每條指令至少附一個 worked CRC。

---

## Speed / Acceleration Parameter Encoding（F6/FD/F4/F5/FE/FF 共用）

Manual Part 8 §8.1 定義 speed 與 acc，所有運動指令共用此編碼。

### speed（轉速）

- 範圍 **0 – 3000**；值越大越快；`speed = 0` → 馬達停止。
- 設定值超過該控制模式最大轉速時，以該模式最大轉速運行。
- **subdivision 校準基準 = 16/32/64**：在 16/32/64 細分下設定值 = 實際 RPM；其他細分 `actual_RPM = speed × 16 / subdivision`。
  - 例：`speed = 1200` → 8 細分 = 2400 RPM、16/32/64 細分 = 1200 RPM、128 細分 = 150 RPM。
- 細分設定見 [[servo42es/02-specs-and-modes|02-specs-and-modes]]。

> 🔀 **vs 42D：** speed 範圍 0–3000、16/32/64 校準基準與 42D 完全一致（[[servo42d/05-motion-commands|42D Motion]] 同述）。無差異 —— 此處刻意確認 byte-compatible，不憑空捏造差異。

### acceleration（加減速 acc）

- 範圍 **0 – 255**；值越大、加/減速越快。
- `acc = 0`：**無加減速**，直接以設定 speed 運行（瞬時達速）。
- 加減速時間量化（Manual §8.1 公式）：相鄰兩個 1-RPM 步階之間隔 `t2 − t1 = (256 − acc) × 50 (µs)`。
  - 加速：`V_t2 = V_t1 + 1`（直到 `V_t2 <= speed`）。
  - 減速：`V_t2 = V_t1 − 1`（直到 `V_t2 >= speed`）。
  - 範例 `acc = 236, speed = 3000`：T=0ms→0 RPM、T=1ms→1 RPM、…、T=3000ms→3000 RPM（每 1ms 升 1 RPM，因 `(256−236)×50µs = 1000µs = 1ms`）。

> 🔀 **vs 42D：** acc 範圍 0–255 與量化公式 `(256−acc)×50µs/step` 與 42D 一致，byte-compatible。

### 速度欄位的位元打包（F6/FD/FF — 12-bit dir|speed）

F6 / FD / FF 把 **direction + 12-bit speed** 塞進 Byte2+Byte3：

| Bit field | 位置 | 意義 |
|-----------|------|------|
| Byte2 bit7 | `dir` | 0 = CCW、1 = CW |
| Byte2 bits[6:4] | reserve | `——`（保留，填 0） |
| Byte2 bits[3:0] | speed[11:8] | speed 高 4 bit |
| Byte3 bits[7:0] | speed[7:0] | speed 低 8 bit |

> 🔀 **vs 42D：** dir|speed 的位元佈局（Byte2 b7=dir、b3-b0+Byte3 = 12-bit speed、b6-b4 保留）與 42D 同款，F6/FD frame byte-identical。

### 速度欄位的位元打包（F4/F5/FE — 16-bit speed，無 dir bit）

座標/絕對脈衝模式（F4/F5/FE）**不在 speed 欄位放 dir bit**：方向由目標座標 vs 當前座標的差值符號決定。speed 為純 **uint16**（Byte2 = speed_hi、Byte3 = speed_lo），範圍 0–3000。

> 🔀 **vs 42D：** 同 42D —— F4/F5/FE 用 uint16 speed、靠目標符號定方向，無 dir bit。byte-compatible。

---

## Speed Control Mode (0xF6) — Part 9

速度模式可：(1) 以設定 acc/speed 連續運行；(2) 運行設定時長後自動停；(3) 開機即自動運行（見 `0xFF`）。

### Run Continuously (DLC=5)

| Byte | Field | Value |
|------|-------|-------|
| —    | CAN_ID | 馬達位址 (e.g. 0x01) |
| —    | DLC | 5 |
| 1    | code | `0xF6` |
| 2    | dir \| speed[11:8] | bit7=dir, bits[3:0]=speed_hi |
| 3    | speed[7:0] | speed_lo |
| 4    | acc | 0–255 |
| 5    | CRC | checksum |

Example（forward，p65）: `01 F6 01 40 02 3A` → CCW, speed=0x140(320 RPM), acc=2
　CRC = (01+F6+01+40+02) & 0xFF = 0x13A & 0xFF = **0x3A** ✓
Example（reverse，p65）: `01 F6 81 40 02 BA` → CW, speed=320, acc=2
　CRC = (01+F6+81+40+02) & 0xFF = 0x1BA & 0xFF = **0xBA** ✓

### Run for Duration (DLC=8)

| Byte | Field | Value |
|------|-------|-------|
| 1    | code | `0xF6` |
| 2    | dir \| speed_hi | 同上 |
| 3    | speed_lo | |
| 4    | acc | 0–255 |
| 5-7  | runTime (uint24) | 單位 10ms，range 0 – 0x190000 |
| 8    | CRC | checksum |

runTime 到期後馬達自動停止。

> 🔀 **vs 42D：** runTime 上限 42ES Manual §9.1 明示 `0~190000H`（= 0x190000，約 4.66 小時 @10ms）；42D 文檔僅寫 `0~0xFFFFFF`。**兩值並列：42ES = 0x190000、42D = 0xFFFFFF。** 單位（10ms）相同。

### Stop via 0xF6 (DLC=5) — §9.2

`[F6, dir, REV|0, speed=0, acc, CRC]`（dir/REV 不影響停止）：

Example（slow stop，acc≠0，p67）: `01 F6 00 00 02 F9` → 以 acc=2 減速停止
　CRC = (01+F6+00+00+02) & 0xFF = 0xF9 ✓
Example（immediate stop，acc=0，p67）: `01 F6 00 00 00 F7` → 立即停
　CRC = (01+F6+00+00+00) & 0xFF = 0xF7 ✓
- `acc ≠ 0`：減速停止。
- `acc = 0`：立即停止（**馬達轉速 > 1000 RPM 時不建議**）。
- 亦可用緊急停止 `0xF7`（見 [[servo42es/06-bus-control|06-bus-control]]）停止運行。

### Response (DLC=3)

`[F6, status, CRC]`，status 對照（§9.1）：

| status | 意義 |
|--------|------|
| 0 | Running failed（啟動失敗） |
| 1 | Start running（開始運行） |
| 2 | Running completed（運行完成，僅限定時模式） |
| 5 | Synchronous mode received（已收到、待同步旗標才啟動） |

> 🔀 **vs 42D：** F6 frame 佈局、停止語意、status 碼（0/1/2/5）與 42D 一致。byte-compatible。

---

## Pulse-Based Position Control — Part 10 §10.1 / §10.2

### 0xFD — Relative Pulse Motion (DLC=8) — §10.1

| Byte | Field | Value |
|------|-------|-------|
| 1    | code | `0xFD` |
| 2    | dir \| speed_hi | bit7=dir(0=CCW/1=CW), bits[3:0]=speed[11:8] |
| 3    | speed_lo | speed[7:0] |
| 4    | acc | 0–255 |
| 5-7  | relPulses (uint24) | 0 – 0xFFFFFF |
| 8    | CRC | checksum |

Example（forward，p71）: `01 FD 01 40 02 00 FA 00 3B` → CCW, speed=0x140(320), acc=2, 0xFA00=64000 pulses（16 細分下 20 圈）
　CRC = (01+FD+01+40+02+00+FA+00) & 0xFF = 0x23B & 0xFF = **0x3B** ✓
Example（reverse，p71）: `01 FD 81 40 02 00 FA 00 BB` → CW, 同上 20 圈反向
　CRC = (01+FD+81+40+02+00+FA+00) & 0xFF = 0x2BB & 0xFF = **0xBB** ✓

**Stop 0xFD (DLC=8)** — §10.1.2：`[FD, 0, 0, acc, 0,0,0, CRC]`
- Slow stop（acc≠0，p72）: `01 FD 00 00 04 00 00 00 02` → acc=4 減速停
　CRC = (01+FD+00+00+04+00+00+00) & 0xFF = 0x102 & 0xFF = **0x02** ✓
- Immediate stop（acc=0，p72）: `01 FD 00 00 00 00 00 00 FE`
　CRC = (01+FD+00×6) & 0xFF = **0xFE** ✓

Response: 0=fail, 1=Start running, 2=Position completed, 3=Stopped upon touching limit switch, 5=Synchronous received。
Note（§10.1.1）: 可用 `8CH` 設定是否在脈衝完成後回到運行狀態（見 [[servo42es/04-config-commands|04-config-commands]]）。

> 🔀 **vs 42D：** FD frame、relPulses 範圍、dir|speed 編碼、status（0/1/2/3/5）與 42D 一致。byte-compatible。

### 0xFE — Absolute Pulse Motion (DLC=8) — §10.2

> Note（§10.2）：使用絕對運動前**必須先做一次歸零** —— `0x91`（Return to Zero）或 `0x92`（92H command mode）標定 "zero point"；未設則開機預設位置即 "midnight"。見 [[servo42es/08-homing-zeroing|08-homing-zeroing]]。

| Byte | Field | Value |
|------|-------|-------|
| 1    | code | `0xFE` |
| 2-3  | speed (uint16) | 0 – 3000 RPM（無 dir bit） |
| 4    | acc | 0–255 |
| 5-7  | absPulses (int24) | −8388607 ～ +8388607（big-endian，二補數） |
| 8    | CRC | checksum |

Example（正向，p74）: `01 FE 02 58 02 00 40 00 9B` → speed=0x258(600 RPM), acc=2, 到絕對脈衝 0x4000
　CRC = (01+FE+02+58+02+00+40+00) & 0xFF = 0x19B & 0xFF = **0x9B** ✓
Example（負向，p74）: `01 FE 02 58 02 FF C0 00 1A` → 到 −0x4000
　CRC = (01+FE+02+58+02+FF+C0+00) & 0xFF = 0x31A & 0xFF = **0x1A** ✓

**Stop 0xFE (DLC=8)** — §10.2.2：`[FE, speed=0, acc, pulses=0, CRC]`
- Slow stop（acc≠0，p76）: `01 FE 00 00 04 00 00 00 03`
　CRC = (01+FE+00+00+04+00+00+00) & 0xFF = 0x103 & 0xFF = **0x03** ✓
- Immediate stop（acc=0，p76）: `01 FE 00 00 00 00 00 00 FF`
　CRC = (01+FE+00×6) & 0xFF = **0xFF** ✓

Response（uplink 標 "Driver board"）: 0=fail, 1=Start running, 2=Position completed, 3=limit switch stopped, 5=Synchronous received。Note: 可用 `8CH` 設回運行狀態。

> 🔀 **vs 42D：** FE frame、absPulses int24 範圍、必須先歸零、status 一致。byte-compatible。

---

## Coordinate-Based Position Control — Part 10 §10.3 / §10.4

座標 = 累積多圈編碼器值（**16384 / turn**），用 `0x31` 讀（見 [[servo42es/03-read-commands|03-read-commands]]）。

### 0xF4 — Relative Coordinate Motion (DLC=8) — §10.3

| Byte | Field | Value |
|------|-------|-------|
| 1    | code | `0xF4` |
| 2-3  | speed (uint16) | 0 – 3000 RPM |
| 4    | acc | 0–255 |
| 5-7  | relAxis (int24) | −8388607 ～ +8388607（相對座標增量） |
| 8    | CRC | checksum |

Example（正向，p78，current=0x8000）: `01 F4 02 58 02 00 40 00 91` → speed=600, acc=2, relAxis=+0x4000 → 目標座標 0xC000
　CRC = (01+F4+02+58+02+00+40+00) & 0xFF = 0x191 & 0xFF = **0x91** ✓
Example（負向，p78）: `01 F4 02 58 02 FF C0 00 10` → relAxis=−0x4000 → 0x8000 變 0x4000
　CRC = (01+F4+02+58+02+FF+C0+00) & 0xFF = 0x310 & 0xFF = **0x10** ✓
　⚠️ 勘誤：V1.0.1 PDF p78 此例把 checksum 誤印為 `09`（正確值 `0x10`）；本表其餘幀與 PDF 逐位元組一致，僅此 CRC 修正。

**Stop 0xF4 (DLC=8)** — §10.3.2：`[F4, speed=0, acc, axis=0, CRC]`
- Slow stop（acc≠0，p80）: `01 F4 00 00 04 00 00 00 F9`
　CRC = (01+F4+00+00+04+00+00+00) & 0xFF = 0xF9 ✓
- Immediate stop（acc=0，p80）: `01 F4 00 00 00 00 00 00 F5`
　CRC = (01+F4+00×6) & 0xFF = **0xF5** ✓

Response: 0=start failed, 1=start running, 2=position completed, 3=limit switch stopped, 5=Synchronous received。

> 🔀 **vs 42D：** F4 frame、relAxis int24、status 一致。byte-compatible。

### 0xF5 — Absolute Coordinate Motion (DLC=8) — §10.4 ★本平台使用

> Note 1（§10.4）：座標 = 累積多圈編碼器值（16384/turn），`0x31` 讀。
> Note 2（§10.4）：**支援即時更新（real-time updates）** —— 前一條 F5 運行中可再送新 F5，即時改 speed 與目標座標。
> Note 3（§10.4）：使用絕對運動前**必須先歸零一次**（`0x91` 或 `0x92`），否則開機預設位置即 "zero point"。

| Byte | Field | Value |
|------|-------|-------|
| —    | CAN_ID | 馬達位址 0x01–0x06（本平台六顆） |
| —    | DLC | 8 |
| 1    | code | `0xF5` |
| 2    | speed_hi (uint16 高位) | speed[15:8]，0 – 3000 RPM |
| 3    | speed_lo | speed[7:0] |
| 4    | acc | 0–255 |
| 5-7  | absAxis (int24) | −8388607 ～ +8388607（big-endian 二補數，絕對座標） |
| 8    | CRC | checksum |

Example（正向，p82）: `01 F5 02 58 02 00 40 00 92` → speed=0x258(600 RPM), acc=2, 到座標 0x4000
　CRC = (01+F5+02+58+02+00+40+00) & 0xFF = 0x192 & 0xFF = **0x92** ✓
Example（負向，p82）: `01 F5 02 58 02 FF C0 00 11` → 到座標 −0x4000（0xFFC000 = int24 −0x4000）
　CRC = (01+F5+02+58+02+FF+C0+00) & 0xFF = 0x311 & 0xFF = **0x11** ✓

**Real-time update（即時更新）—— §10.4.4 worked sequence**
運行中再送一條 F5 即可同時改 speed 與目標，不需先停：

1. `01 F5 01 2C 02 7F 80 00 24` → speed=0x12C(300 RPM), acc=2, 目標 0x7F8000
　CRC = (01+F5+01+2C+02+7F+80+00) & 0xFF = 0x224 & 0xFF = **0x24** ✓
2. （約 20 秒後、馬達仍在跑）`01 F5 02 58 02 02 80 00 D4` → speed=0x258(600 RPM), acc=2, 目標改 0x028000
　CRC = (01+F5+02+58+02+02+80+00) & 0xFF = 0x1D4 & 0xFF = **0xD4** ✓
　→ 馬達在運行途中即時換速並改奔向新目標。

> 🔀 **vs 42D：** **F5 即時更新行為與 42D 完全一致**（[[servo42d/05-motion-commands|42D Motion]] 同述「Supports real-time updates: send new F5 while previous is running to change speed and target instantly」），frame/byte/status 皆 byte-identical。本專案靠此特性做 20ms 軌跡協調器持續重發 F5（見 [[servo-can-hub|CAN 文檔總覽]] 之平台架構）。**未發現 F5 編碼或即時更新差異。**

**Stop 0xF5 (DLC=8)** — §10.4.2：`[F5, speed=0, acc, axis=0, CRC]`
- Slow stop（acc≠0，p84）: `01 F5 00 00 04 00 00 00 FA`
　CRC = (01+F5+00+00+04+00+00+00) & 0xFF = 0xFA ✓
- Immediate stop（acc=0，p84）: `01 F5 00 00 00 00 00 00 F6`
　CRC = (01+F5+00×6) & 0xFF = **0xF6** ✓

Response (DLC=3) `[F5, status, CRC]`:

| status | 意義 |
|--------|------|
| 0 | Start failed |
| 1 | Start running |
| 2 | Position completed |
| 3 | Stopped upon touching limit switch |
| 5 | Synchronous mode received（待同步旗標才啟動，多機同步見 [[servo42es/11-multi-motor-sync|11-multi-motor-sync]]） |

> 🔀 **vs 42D：** F5 stop frame、status（0/1/2/3/5）與 42D 一致。byte-compatible。

#### 完整 F5 絕對座標操作序列（§10.4.3 實機抓包）

| Step | Frame | 說明 |
|------|-------|------|
| 1. 設工作模式 | `01 82 05 88` | SR_* bus mode（已設可略） |
| 2. 存檔 | `01 60 01 62` | **0x60 SAVE**（已存可略） |
| 3. 設零點 | `01 92 00 93` | 0x92 標 zero point |
| 4. 絕對座標運動 | `01 F5 01 2C 02 02 80 00 A7` | speed=300 RPM, acc=2, 到座標 0x28000 |
| 5. 讀座標 | `01 31 32` | 0x31 回讀累積座標確認 |
| 6. 減速停（運行中） | `01 F5 00 00 02 00 00 00 F8` | acc=2 減速停 |
| 7. 緊急停 | `01 F7 F8` | 0xF7 緊急停止 |

Step4 CRC = (01+F5+01+2C+02+02+80+00) & 0xFF = 0x1A7 & 0xFF = **0xA7** ✓（與 §10.4.3 抓包一致）

> 🔀 **vs 42D：** 本序列在 42ES 多了 Step2 `0x60` 存檔（42D 無此步）。其餘 frame byte-identical。

---

## 0xFF — Auto-Start on Power-On — §9.3 / §9.5

設定後，每次上電馬達自動以設定 dir/speed/acc 運行。frame 與 F6 連續模式同形（dir|speed 12-bit 打包 + acc）：

| Byte | Field | Value |
|------|-------|-------|
| 1    | code | `0xFF` |
| 2    | dir \| speed_hi | bit7=dir(0–1, default 0), bits[3:0]=speed[11:8] |
| 3    | speed_lo | speed[7:0]（speed 0–3000, default 0） |
| 4    | acc | 0–255 (default 0) |
| 5    | CRC | checksum |

Example（設定，p68/p70）: `01 FF 01 2C 02 2F` → dir=CCW, speed=0x12C(300 RPM), acc=2
　CRC = (01+FF+01+2C+02) & 0xFF = 0x12F & 0xFF = **0x2F** ✓
Example（清除/停用，p70）: `01 FF 00 00 00 00` → speed=0 → 停用開機自動運行
　CRC = (01+FF+00+00+00) & 0xFF = **0x00** ✓
- **`speed = 0` 時停用開機自動運行**（§9.3 紅字 Note）。

Response (DLC=3): 0=Set failed, 1=set successed。

**啟用流程（§9.5）**：`82`(設模式) → `FF 01 2C 02 2F`(設 dir/speed/acc) → **`60 01 62`(SAVE)** → 斷電重啟即自動運行。
**取消流程（§9.5）**：`F6 00 00 02 F9`(停) → `FF 00 00 00 00`(清參數) → **`60 01 62`(SAVE)** → 重啟不再自動運行。

> 🔀 **vs 42D：** 啟用碼不同！**42ES 用 `0xFF` + `speed=0` 停用；speed≠0 啟用**（dir/speed/acc 直接編入 frame，與 F6 同形）。**42D 用 `0xFF` + enable byte：`0xC8`=啟用 / `0xCA`=停用**，且需先發 F6 再 enable auto-start。語意與 byte 佈局**不同**：
> - 42ES：`[FF, dir|speed_hi, speed_lo, acc, CRC]`（DLC=5，參數內嵌）
> - 42D：`[FF, enable(0xC8/0xCA), CRC]`（DLC=3，僅 enable byte）
> 並列兩值供比對矩陣。此外 42ES 設定後**必須 `0x60` 存檔**才會在斷電後生效，42D 無此存檔步驟。

---

## 共用提醒

- **MCP2515 雙緩衝**：本平台高頻重發 F5，回覆會塞爆 2-buffer RX，每 cycle 開頭須 `flushReceiveBuffer()`（見 [[servo-can-hub|CAN 文檔總覽]]、本專案 CLAUDE.md）。
- **緊急停止 `0xF7`** 可中止任何模式（見 [[servo42es/06-bus-control|06-bus-control]]）；> 1000 RPM 時不建議用立即停（acc=0）或 F7。
- **多機同步**：status=5 表示已收指令、待同步旗標；批次同步見 [[servo42es/11-multi-motor-sync|11-multi-motor-sync]]。
- **錯誤碼與恢復**：見 [[servo42es/12-errors-recovery|12-errors-recovery]]。
- **持久化鐵律**：任何要在斷電後保留的 bus 參數/工作模式/開機自啟設定，務必 `0x60` SAVE（42ES-critical，見 [[servo42es/04-config-commands|04-config-commands]]）。
