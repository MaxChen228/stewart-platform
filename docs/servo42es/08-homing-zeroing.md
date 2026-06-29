# Homing & Zero-Point Setting (SERVO42ES / 57ES)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/08-homing-zeroing|42D Homing & Zero-Point Setting]]

Manual Part 6 (return to zero) + Part 7 (left/right limit switches). All frames follow the [[servo42es/01-protocol|standard CAN frame]] layout — downlink `[CAN_ID][DLC][code][data...][CRC]`, [[servo42es/01-protocol|CRC]] = `(CAN_ID + all data bytes) & 0xFF`, big-endian. CRC is **byte-identical to [[servo42d/01-protocol|SERVO42D]]**.

> 🔀 **vs 42D：** 42ES 把歸零拆成 **五個獨立命令碼**（`0x95` 模式/方向/速度、`0x96` 力矩/偏移、`0x97` 觸發法/逾時、`0x92` 設零點、`0x91` 執行）。42D 把模式/方向/速度/EndLimit/hm_mode 全擠進一個 **`0x90`**，力矩/偏移走 `0x94`，無獨立的 `0x97` 觸發/逾時命令、無 `0x95`/`0x96`。**所以 42D 的 `0x90`/`0x94` 在 42ES 不存在；改用 `0x95`/`0x96`/`0x97`。**

> ⚠️ **42ES SAVE 規則：** Part 4 note 3 + §4.5.1 — 寫入型參數在下 **`0x60` SAVE**（見 [[servo42es/04-config-commands|0x60 SAVE]]）前**不落 NVS**。本章每個 example 末步都是 `60 01 62`。42D 文件無此明確 SAVE 命令。
> 🔀 **vs 42D：** 42D 歸零參數寫即生效/即存；42ES 必須 `0x60` SAVE 才持久（不存則斷電丟失）。

---

## Two Homing Concepts

| Concept | 中文 | 說明 |
|---------|------|------|
| **Origin homing** | 原點歸零 | 馬達主動搜尋實體基準（開關 / 機械限位 / 單圈零點），抵達後把當前位置標記為「coordinate zero point」。建立座標系。 |
| **Coordinate homing** | 座標歸零 | 已知零點後，直接高速跑回該座標零點，**無搜尋過程**。前提：先做過一次 origin homing 才有零點。 |

### Origin Homing — three search methods (Part 6.1)

1. **Origin switch (`HmMode=0x00`)** — 高精度。搜尋流程：高速朝設定方向找開關 → 遇上升緣後低速脫開開關 → 反向低速再搜上升緣 → 遇緣後高速跑到 origin offset → 標零。
2. **Mechanical limit switch (`HmMode=0x01`)** — 無需實體開關。先以 `0x96` 設好歸零電流（足以驅動負載但不可過大以免損傷機構），低速朝設定方向頂機械限位 → 堵轉停 → 跑到 origin offset → 標零。
3. **Single-lap zero (`HmMode=0x02`)** — 單圈內歸零。零點座標須先用 `0x92` 設定（見下）；方向由 `HmDir` 決定（forward/reverse/nearest）；上電後高速跑回單圈內零點 → 標零。

> 🔀 **vs 42D：** 概念與四方法一致，但 42ES 的 origin switch 流程明確描述為「上升緣 → 脫開 → 反向再搜上升緣 → offset」雙向精修；42D 文件只列方法不展開搜尋細節。命令碼完全不同（見上方 callout）。

### Origin switch access port

| 型號 | Origin switch port |
|------|--------------------|
| SERVO57ES | `IN_COM / IN1` |
| SERVO42ES | `IN+ / IN-` |

接線細節見 [[servo42es/10-io-ports|2.4 External switch wiring]]。

---

## 0x95 — Set Zero-Return Mode / Direction / Speed (Part 6.3.1)

Downlink **DLC=8**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x95` |
| 2 | HmMode | `00`=origin switch (default) / `01`=mechanical limit / `02`=single-lap / `03`=disable zero-reset |
| 3 | HmDir | `00`=positive/forward (default) / `01`=reverse / `02`=nearest (僅單圈零點適用) |
| 4–5 | HiSpeed (uint16) | high-speed homing RPM, **1~3000, default 100** |
| 6–7 | LoSpeed (uint16) | low-speed homing RPM, **1~100, default 10** |
| 8 | CRC | checksum |

Response **DLC=3** `[95, status, CRC]` — `status`: 0=fail, 1=success（[[servo42es/01-protocol|write-cmd 通用回覆]]）。

**Worked example** `01 95 00 00 00 64 00 0A 04`
HmMode=0 (switch), HmDir=0 (forward), HiSpeed=`0x0064`=100 RPM, LoSpeed=`0x000A`=10 RPM.
CRC = (01+95+00+00+00+64+00+0A) & 0xFF = 0x104 & 0xFF = **0x04** ✓

> 🔀 **vs 42D：** 42D 用 `0x90` DLC=8 `[90, homeTrig, homeDir, speed_hi, speed_lo, EndLimit, hm_mode, CRC]` — **單一速度欄、外加 homeTrig 與 EndLimit 欄、hm_mode 在 Byte6**。42ES 的 `0x95` **拆出 HiSpeed/LoSpeed 兩段速度**、HmMode 移到 Byte2、**無 homeTrig/EndLimit 欄**（這兩者改由 `0x97` 與 `0x9E` 各自承擔）。

---

## 0x96 — Set Zero-Return Torque & Origin Offset (Part 6.3.2)

Downlink **DLC=8**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x96` |
| 2–3 | HmATorque (uint16) | 機械限位歸零電流 (mA) |
| 4–7 | Orgoffset (int32) | 歸零後偏移位置, default `0x2000` |
| 8 | CRC | checksum |

- **HmATorque** — 僅機械限位模式有效。
  - SERVO42ES max 3000 mA, **default 300 mA**
  - SERVO57ES max 5200 mA, **default 600 mA**
- **Orgoffset** — `=0`：歸零後即停；`≠0`：歸零後續跑到 offset 位置才停。**單圈零點 (lap-back) 模式下此值無效。**

Response **DLC=3** `[96, status, CRC]`.

**Worked example** `01 96 02 58 00 00 20 00 11`
HmATorque=`0x0258`=600 mA, Orgoffset=`0x00002000`=8192 (= half turn).
CRC = (01+96+02+58+00+00+20+00) & 0xFF = 0x111 & 0xFF = **0x11** ✓

> 🔀 **vs 42D：** 同概念但**命令碼 `0x94`（非 `0x96`）**，欄位順序為 `[94, Hm_offset(int32), Hm_ma(uint16), CRC]` — **offset 在前、電流在後**，與 42ES `0x96`（電流在前 Byte2–3、offset 在後 Byte4–7）**相反**。預設電流也不同：42D 文件記 42D=400 mA / 57D=800 mA / 28D·35D=200 mA；42ES 文件記 **42ES=300 mA / 57ES=600 mA**。offset 預設兩者皆 `0x2000`。

---

## 0x97 — Configure Trigger Method & Timeout (Part 6.3.3)

Downlink **DLC=7**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x97` |
| 2 | HmTrig | `00`=command-triggered (default) / `01`=auto on power-on (僅 bus mode) / `02`=En-signal triggered (僅 pulse mode) |
| 3–6 | HmTimOut (uint32) | 歸零逾時 (ms), **default 120000** |
| 7 | CRC | checksum |

- **HmTrig=02 (pulse mode only)**：En 訊號產生 200 ms 寬的「失能電平」即觸發歸零（脈衝辨識窗 150~250 ms）。En 為低 → 使能 + 產生 200 ms 高電平觸發；En 為高 → 使能 + 產生 200 ms 低電平觸發。
- **HmTimOut**：逾時內未到零點 → 回 timeout failure（`0x91` status=3）。

Response **DLC=3** `[97, status, CRC]`（status uint8_t）。

**Worked example** `01 97 00 00 00 EA 60 E2`
HmTrig=0 (command), HmTimOut=`0x0000EA60`=60000 ms.
CRC = (01+97+00+00+00+EA+60) & 0xFF = 0x1E2 & 0xFF = **0xE2** ✓

> 🔀 **vs 42D：** **42ES 獨有命令。** 42D 把觸發法塞進 `0x90` 的 `homeTrig` 欄、**無獨立逾時設定命令、無 `0x97`**。42ES 把 trigger + timeout 抽成 `0x97`（power-on 自動歸零 / En 觸發 / 命令觸發 + 可調逾時）。

---

## 0x92 — Set Current Position as Zero (Part 6.3.4)

直接把當前位置設為「zero」。Downlink **DLC=3**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x92` |
| 2 | data | `0x00` |
| 3 | CRC | checksum |

Response **DLC=3** `[92, status, CRC]` — 0=fail, 1=success.

**Worked example** `01 92 00 93` → CRC = (01+92+00) & 0xFF = **0x93** ✓

> 此命令亦是 **single-lap 模式建立「單圈零點座標」** 的方式（§6.6 例 `01 92 00 93`）。
> 📝 手冊 §6.1 提到「單圈歸零方向用命令 93H 設定」，但所有實測 example（§6.6.1/6.6.2）的零點座標設定一律用 `0x92`，方向則由 `0x95` 的 `HmDir` 欄承載；`93H` 在範例中**只**出現於 `91 01 93` 這類 `0x91` 的 CRC，並非獨立命令。本章以實測 example 為準：用 `0x92` 設零點、`0x95.HmDir` 設方向。

> 🔀 **vs 42D：** 42D 的 `0x92` DLC=2 `[92, CRC]`（無 data byte）。42ES 的 `0x92` **DLC=3、Byte2 固定 `0x00` data**。語意相同（設當前位置為座標零點），但**幀長與 CRC 不同**：42D `[01 92 93]`（01+92=0x93）vs 42ES `[01 92 00 93]`（01+92+00=0x93，碰巧同 CRC 因 data=0）。

---

## 0x91 — Execute Zero-Return (Part 6.3.5)

Downlink **DLC=3**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x91` |
| 2 | goZeroMode | `00`=Return to Zero (origin search) / `01`=Coordinate Zeroing (run to known zero) |
| 3 | CRC | checksum |

Response **DLC=3** `[91, status, CRC]`：

| status | 意義 |
|--------|------|
| 0 | failed to return to zero |
| 1 | start to return to zero |
| 2 | return to zero complete |
| 3 | times out and failed |

**Worked examples**
- Origin search: `01 91 00 92` → CRC=(01+91+00)&0xFF=**0x92** ✓
- Coordinate zeroing: `01 91 01 93` → CRC=(01+91+01)&0xFF=**0x93** ✓

**Note**：執行 **Coordinate Zeroing (`goZeroMode=01`) 前馬達必須先 enable**（§6.7）。Coordinate zeroing 須先做過 origin return 以確立零點。

> 🔀 **vs 42D：** 命令碼/語意一致。42D 額外允許 DLC=2 `[91, CRC]` 預設為 origin return；42ES 文件僅列 DLC=3 形式。status 編碼兩者相同（0/1/2/3）。

---

## Single-Lap Zero-Return (Part 6.6)

單圈零點不另設專用命令，而是 `0x95.HmMode=02` + `0x95.HmDir`（forward/reverse/**nearest**）+ `0x92` 設零點座標的組合。`HmDir=02 (nearest)` 為單圈獨有：走最短路徑回零。

> 🔀 **vs 42D：** 42D 用獨立命令 **`0x9A`** DLC=6 `[9A, mode, enable, speed, dir, CRC]` 設單圈歸零參數（mode 0/1/2、enable 清/設/保留零點、speed 0~4 檔、dir CW/CCW）。**42ES 無 `0x9A`**；單圈參數併入 `0x95`（mode=02、dir 含 nearest）＋ `0x92` 設零點，速度走 `0x95` 的 Hi/Lo RPM（非 0~4 檔位）。**功能等價、機制完全不同。**

---

## Port Mapping for Right Limit (Part 6.x / Part 7)

42D 用獨立 `0x9E` 開「port mapping」把 Dir 腳當右限位。**42ES 把右限位映射併入 `0x9E` 的 `reMap` 欄**（見下方 §7.1）。

> 🔀 **vs 42D：** 42D `0x9E` DLC=3 `[9E, enable, CRC]`（純 0/1 開關，En→左限位、Dir→右限位，COM 須拉高）。42ES `0x9E` 是 **DLC=5 的完整限位設定命令**（EndLimit + EndLevel + reMap 三欄），右限位映射只是其中 `reMap` 一欄。

---

# Part 7 — Left & Right Limit Switches

## Default Limit Switch Wiring

| 型號 | Left limit port | Right limit port | Remark |
|------|-----------------|------------------|--------|
| SERVO57ES | `IN_COM / IN1` | `IN_COM / IN2` | 專用雙輸入 |
| SERVO42ES | `IN+ / IN-` | `EN+ / EN-` | **Bus mode：須 enable right limit mapping** |

> 🔀 **vs 42D：** 28/35/42D **只有 IN_1（左限位）**，右限位須靠 `0x9E` port mapping 借 **Dir 腳**。**42ES 改借 EN 腳**（`EN+/EN-`）當右限位（bus mode + reMap=01）。**57ES 兩代都有專用第二輸入**（57ES 為 `IN_COM/IN2`）。差異：42ES 右限位佔用的是 **EN 腳非 Dir 腳**。

## 7.1 — 0x9E Set Limit Switch Parameters

Downlink **DLC=5**：

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x9E` |
| 2 | EndLimit | `00`=disable (default) / `01`=enable / `FF`=keep unchanged |
| 3 | EndLevel | `00`=low when closed (default) / `01`=high when closed / `FF`=keep |
| 4 | reMap | `00`=disable right-limit mapping (default, **only valid for 42ES**) / `01`=enable / `FF`=keep |
| 5 | CRC | checksum |

Response **DLC=3** `[9E, status, CRC]` — 0=fail, 1=success.

**Limit enable 行為**（EndLimit=1, **bus mode only**）：
1. 左限位觸發 → 馬達停止向左運動；右限位觸發 → 停止向右運動。
2. **首次啟用或變更限位參數後，必須執行一次「return to zero」(`0x91`)。**
3. **脈衝控制模式下限位功能無效**（僅 bus mode 有效）。

**Worked examples**
- **57ES**（無需右限位映射）：`01 9E 01 00 FF 9F`
  EndLimit=1, EndLevel=0, reMap=`FF` (keep — 57ES 有專用 IN2).
  CRC = (01+9E+01+00+FF) & 0xFF = 0x19F & 0xFF = **0x9F** ✓
- **42ES**（需右限位映射）：`01 9E 01 00 01 A1`
  EndLimit=1, EndLevel=0, reMap=`01` (enable EN→right limit).
  CRC = (01+9E+01+00+01) & 0xFF = 0x1A1 & 0xFF = **0xA1** ✓

> 🔀 **vs 42D：** 命令碼同為 `0x9E` 但**功能/幀長不同**：42D=DLC=3 純 enable 開關（見上）；42ES=DLC=5 三欄完整限位設定（觸發電平 EndLevel + 右限位映射 reMap 皆可調），且 `reMap` 欄**僅 42ES 有效**（57ES 用專用 IN2，故 example 帶 `FF` 保持）。

> ⚠️ 變更後記得 `0x60` SAVE（[[servo42es/04-config-commands|SAVE]]）才落 NVS。

---

## Homing Procedures — Worked Frame Sequences

所有序列先設工作模式 `01 82 05 88`（SR_vFOC，見 [[servo42es/02-specs-and-modes|0x82 working mode]]），末步 `01 60 01 62` SAVE。

### 6.4.1 — Origin Switch, Command-Triggered

| # | Step | Frame |
|---|------|-------|
| 1 | Set working mode | `01 82 05 88` |
| 2 | Set switch trigger level (limit param) | `01 9E FF 00 FF 9D` |
| 3 | Set mode/dir/HiSpeed/LoSpeed | `01 95 00 00 00 64 00 0A 04` |
| 4 | Set origin offset | `01 96 02 58 00 00 20 00 11` |
| 5 | Set trigger method & timeout | `01 97 00 00 00 EA 60 E2` |
| 6 | Save | `01 60 01 62` |
| 7 | Execute return to zero | `01 91 00 92` |

觀察：高速正轉 (100 RPM) → 觸開關 → 低速反轉 (10 RPM) → 低速正轉 (10 RPM) → 再觸開關 → 高速跑 offset → 歸零完成。
CRC check step 2: (01+9E+FF+00+FF)&0xFF=0x29D&0xFF=**0x9D** ✓

### 6.4.2 — Origin Switch, Auto-on-Power-On

同 6.4.1，但 **step 5 改 `HmTrig=01`**：`01 97 01 00 00 EA 60 E3`（CRC=(01+97+01+00+00+EA+60)&0xFF=0x1E3&0xFF=**0xE3** ✓），**省略 step 7**（下次上電自動歸零）。逾時內未觸開關回 timeout failure。

### 6.5.1 — Mechanical Limit, Command-Triggered

| # | Step | Frame |
|---|------|-------|
| 1 | Working mode | `01 82 05 88` |
| 2 | Mode=01(limit)/dir/Hi/Lo | `01 95 01 00 00 64 00 0A 05` |
| 3 | Current + origin offset | `01 96 02 58 00 00 20 00 11` |
| 4 | Trigger & timeout | `01 97 00 00 00 EA 60 E2` |
| 5 | Save | `01 60 01 62` |
| 6 | Execute | `01 91 00 92` |

觀察：低速正轉 (10 RPM) → 頂機械限位 → 停 → 高速反向跑 offset → 歸零完成。
CRC step 2: (01+95+01+00+00+64+00+0A)&0xFF=0x105&0xFF=**0x05** ✓

### 6.5.2 — Mechanical Limit, Auto-on-Power-On
同 6.5.1 但 `0x97` 帶 `HmTrig=01`（`01 97 01 00 00 EA 60 E3`），省略 execute step。

### 6.6.1 — Single-Lap, Command-Triggered
先手動把軸轉到合適零點位置：

| # | Step | Frame |
|---|------|-------|
| 1 | Working mode | `01 82 05 88` |
| 2 | Mode=02(single-lap)/dir=02(nearest)/Hi/Lo | `01 95 02 02 00 64 00 0A 08` |
| 3 | Trigger & timeout | `01 97 00 00 00 EA 60 E2` |
| 4 | Set zero-point coords | `01 92 00 93` |
| 5 | Save | `01 60 01 62` |
| 6 | Execute | `01 91 00 92` |

CRC step 2: (01+95+02+02+00+64+00+0A)&0xFF=0x108&0xFF=**0x08** ✓
存檔後可斷電/鬆軸離開零點；執行 return-to-zero 後馬達高速 (100 RPM) 跑回單圈零點。

### 6.6.2 — Single-Lap, Auto-on-Power-On
同 6.6.1 但 `0x97` 帶 `HmTrig=01`（`01 97 01 00 00 EA 60 E3`），省略 execute step；上電自動高速回零。

### 6.7 — Coordinate Zeroing (after origin established)
零點已知後，**馬達須先 enable**，直接執行：
`01 91 01 93` → 高速跑回座標零點（無搜尋）。CRC=(01+91+01)&0xFF=**0x93** ✓

### 7.2 — Limit Switch Config

| # | Step | 57ES | 42ES |
|---|------|------|------|
| 1 | Working mode | `01 82 05 88` | `01 82 05 88` |
| 2 | EndLimit/level/reMap | `01 9E 01 00 FF 9F` (no right-map) | `01 9E 01 00 01 A1` (right-map) |
| 3 | Mode/dir/Hi/Lo | `01 95 00 00 00 64 00 0A 04` | 同左 |
| 4 | Return to zero | `01 91 00 92` | 同左 |
| 5 | Save | `01 60 01 62` | 同左 |

首次啟用限位 / 變更參數後**必須做一次歸零**（step 4）。

### 7.3 — Left Limit Switch Test
運動指令 `01 FD 81 2C 02 70 00 00 1D`（[[servo42es/05-motion-commands|0xFD speed/position run]]），馬達運轉中觸發**左**限位即停。
CRC=(01+FD+81+2C+02+70+00+00)&0xFF=0x21D&0xFF=**0x1D** ✓

### 7.4 — Right Limit Switch Test
運動指令 `01 FD 01 2C 02 70 00 00 9D`，馬達運轉中觸發**右**限位即停。
CRC=(01+FD+01+2C+02+70+00+00)&0xFF=0x19D&0xFF=**0x9D** ✓
(兩例差 Byte2 方向位元：`81` vs `01`，故觸發的限位側不同。)

---

## Quick Reference — 42ES Homing Command Map

| Code | DLC | 功能 | 42D 對應 |
|------|-----|------|----------|
| `0x95` | 8 | mode/dir/HiSpeed/LoSpeed | `0x90`（單速 + homeTrig + EndLimit + hm_mode） |
| `0x96` | 8 | torque(mA) + origin offset | `0x94`（offset 在前、電流在後） |
| `0x97` | 7 | trigger method + timeout | **無對應**（42D 併入 `0x90.homeTrig`，無逾時命令） |
| `0x92` | 3 | set current pos as zero | `0x92` DLC=2 |
| `0x91` | 3 | execute (origin / coord) | `0x91` |
| `0x9E` | 5 | limit: EndLimit+EndLevel+reMap | `0x9E` DLC=3（純 enable port-map） |
| `0x60` | — | SAVE（持久化前述全部） | **無對應** |
| (單圈) | — | `0x95.HmMode=02`+`0x92` | 獨立 `0x9A` |

相關章節：[[servo42es/04-config-commands|config/SAVE]] · [[servo42es/05-motion-commands|motion (0xFD)]] · [[servo42es/10-io-ports|IO ports/wiring]] · [[servo42es/09-protection|protection/endstop]] · [[servo42es/00-index|42ES index]]。
