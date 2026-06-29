# Bus Control General Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/06-bus-control|Bus Control General Commands]]

Manual section **8.2 Bus control general instructions**. These commands are only valid in **bus control mode** (the ES product line's SR-series modes — see [[servo42es/02-specs-and-modes|工作模式]]). In bus control mode the EN pin no longer gates enable; the host owns enable / status / stop over CAN.

涵蓋指令：`0xF1` 讀運行狀態、`0xF3` 使能/禁用、`0xF7` 緊急停止。Frame shape、CRC 與 big-endian 規則見 [[servo42es/01-protocol|協議與 CRC]]。本章三條指令皆為 **read / runtime control**，不寫持久參數，故與 `0x60` SAVE 無關（持久化規則見 [[servo42es/04-config-commands|設定指令]]）。

> 🔀 **vs 42D：** 42ES 全線涵蓋 **SERVO42ES V1.0 + SERVO57ES V1.0**（同韌體/協議，差別僅電流額定與 I/O：42ES 單 IN，57ES IN_1+IN_2+IN_COM）。本章三條指令對兩型號位元組完全相同。

---

## 0xF1 — Read Motor Operating Status

讀馬達當前運轉狀態機。

### Downlink (PC → SERVOxxES) — DLC=2

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01`（馬達位址，預設 0x01） |
| Byte1 | code | `0xF1` |
| Byte2 | CRC | checksum |

### Uplink (PC ← SERVOxxES) — DLC=3

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01` |
| Byte1 | code | `0xF1` |
| Byte2 | status | running status（見下表） |
| Byte3 | CRC | checksum |

### status 碼

| status | Motor operating status | 說明 |
|--------|------------------------|------|
| 0 | Query failed | 查詢失敗 |
| 1 | Stop running | 停止 |
| 2 | Speed up | 加速中 |
| 3 | Slow down | 減速中 |
| 4 | Run at full speed | 全速運行 |
| 5 | Reset to zero | 回零 / 歸零中 |

> 🔀 **vs 42D：** 本專案 42D 文檔的 0xF1 狀態表多出 `6 = Calibrating`（校準），且 status 5 標為 `Homing`。**42ES 手冊 V1.0.1 僅列到 status 5，名稱為 "Reset to zero"，並無 6（校準）這一列。** 兩值並陳供比對：42D `{5:Homing, 6:Calibrating}` vs 42ES `{5:Reset to zero}`。

### Worked example — 讀 ID 0x01 狀態

Downlink: `01 F1 F2`
CRC = `(0x01 + 0xF1) & 0xFF = 0xF2`。

Uplink（假設正全速運行 status=4）: `01 F1 04 F6`
CRC = `(0x01 + 0xF1 + 0x04) & 0xFF = 0xF6`。

> 📌 **Note（手冊）：** This instruction is only valid in **bus control mode**（CR 系列脈衝模式下不適用此匯流排狀態查詢）。

---

## 0xF3 — Set Motor Enable State

In bus control mode, the motor's enable state is **no longer controlled by the level of the EN pin**, but is controlled by this command.

### Downlink (PC → SERVOxxES) — DLC=3

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01` |
| Byte1 | code | `0xF3` |
| Byte2 | enable | `0x00` / `0x01` |
| Byte3 | CRC | checksum |

- `enable = 0x00` → Motor **disabled**（loose shaft，可自由轉動，無保持力矩）
- `enable = 0x01` → Motor **enabled**（shaft lock，有保持力矩）

### Uplink (PC ← SERVOxxES) — DLC=3

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01` |
| Byte1 | code | `0xF3` |
| Byte2 | status | setting status |
| Byte3 | CRC | checksum |

| status | 說明 |
|--------|------|
| 0 | Set failed |
| 1 | Set succeeded |

### Worked example — enable ID 0x01

Downlink（使能）: `01 F3 01 F5`
CRC = `(0x01 + 0xF3 + 0x01) & 0xFF = 0xF5`。

Downlink（禁用）: `01 F3 00 F4`
CRC = `(0x01 + 0xF3 + 0x00) & 0xFF = 0xF4`。

Uplink（成功）: `01 F3 01 F5` → CRC = `(0x01 + 0xF3 + 0x01) & 0xFF = 0xF5`。

> 📌 **Note（手冊）：** This instruction is only valid in **bus control mode**。EN 接腳準位在匯流排模式下不再決定使能；脈衝模式下的 EN 行為與 `0x85`（設 EN 準位）見 [[servo42es/04-config-commands|設定指令]]。

> 🔀 **vs 42D：** 行為與位元組布局與 [[servo42d/06-bus-control|42D 0xF3]] 完全一致（enable 00/01，回 status 0/1）。無差異 — 列此僅供 hub 確認對等。

---

## 0xF7 — Emergency Stop

立即停轉（急停）。

### Downlink (PC → SERVOxxES) — DLC=2

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01` |
| Byte1 | code | `0xF7` |
| Byte2 | CRC | checksum |

### Uplink (PC ← SERVOxxES) — DLC=3

| Byte | Field | Value |
|------|-------|-------|
| ID | CAN_ID | `0x01` |
| Byte1 | code | `0xF7` |
| Byte2 | status | stopped state |
| Byte3 | CRC | checksum |

| status | 說明 |
|--------|------|
| 0 | Emergency stop failed |
| 1 | Emergency stop successful |

### Worked example — 急停 ID 0x01

Downlink: `01 F7 F8`
CRC = `(0x01 + 0xF7) & 0xFF = 0xF8`。

Uplink（成功）: `01 F7 01 F9`
CRC = `(0x01 + 0xF7 + 0x01) & 0xFF = 0xF9`。

### ⚠️ 安全須知

> **Note（手冊原文，紅字）：** Emergency stop commands are **not recommended when the motor speed exceeds 1000 RPM!**

0xF7 為硬急停（無減速斜坡），高速下強行剎停會對機構與驅動造成衝擊。需受控減速時，改用帶加速度的停止指令（`0xF6` 速度模式以 `acc≠0`、或 `0xFD/0xFE` 位置/相對指令的停止形式）做斜坡減速 — 詳見 [[servo42es/05-motion-commands|運動控制指令]]。

> 🔀 **vs 42D：** 1000 RPM 安全閾值與位元組布局與 [[servo42d/06-bus-control|42D 0xF7]] 一致。無數值差異。

---

## 持久化提醒（42ES-critical）

本章三條指令皆為 **runtime control / read**，狀態不入 NVS、開機不保留，因此**不需要** `0x60` SAVE。

> 🔀 **vs 42D：** 此為通則差異而非本章差異 — 42ES 全線「write-only 參數須由 `0x60` SAVE 顯式存檔才持久」（手冊 Part 4 note 3 + 4.5.1），SERVO42D 文檔集無對應顯式 save 指令。涉及寫入/設定的章節（[[servo42es/04-config-commands|設定指令]] 等）須留意；F1/F3/F7 不受此規範影響。

---

## 相關章節

- [[servo42es/01-protocol|協議與 CRC]] — frame 結構、checksum 演算法、big-endian
- [[servo42es/02-specs-and-modes|工作模式]] — bus control（SR 系列）vs 脈衝（CR 系列）
- [[servo42es/03-read-commands|讀取指令]] — 編碼器/位置/速度等量測查詢
- [[servo42es/04-config-commands|設定指令]] — EN 準位 `0x85`、write-only 持久化、`0x60` SAVE
- [[servo42es/05-motion-commands|運動控制指令]] — `0xF6/0xFD/0xFE` 受控減速停止
- [[servo42d/06-bus-control|42D 對應章]] — 雙生對照
