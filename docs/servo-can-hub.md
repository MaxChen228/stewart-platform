# 🗺️ Makerbase Servo CAN — 文檔總覽 (MOC)

This vault documents the **MKS SERVO42D/57D** and **MKS SERVO42ES/57ES** CAN protocols in one graph. The two product lines are **byte-compatible at the CAN layer** — same standard frame, same 8-bit CHECKSUM CRC, and identical motion/zero/read opcodes (`0xF5` / `0xF6` / `0xF4` / `0xFD` / `0xFE` / `0x92` / `0x30` / `0x31`) — but they differ in **hardware** (MCU, encoder, MOSFET, voltage/current, no OLED on ES) and in a handful of **commands** (PID over CAN, the `0x60` SAVE step, subdivision set, RS485/MODBUS). This page is the graph center: every chapter is linked to **both twins** so the Obsidian graph renders as a dense mesh.

> **Shared CAN facts (both lines).** Standard CAN frame, data field ≤ 8 bytes. Downlink `[CAN_ID][DLC][Byte1=code][data…][CRC]`; uplink has the same shape. `CRC = CHECKSUM 8-bit = (CAN_ID + all data bytes except CRC) & 0xFF`, **byte-identical across 42D/42ES**, data **big-endian**. CAN_ID = motor address `0x00–0x7FF`, default `0x01`; `0x00` = broadcast (slaves do **not** respond); `0x01–0x10` settable on OLED, `>0x10` via `0x8B`. Write commands respond `DLC=3 [code, status, CRC]`, status `0=fail / 1=success`; motion may add `2=completed / 3=endstop-stopped / 5=sync-received`.

---

## Primary nodes — the two index hubs

- 🟦 [[servo42d/00-index|SERVO42D / 57D 索引]] — N32L40x MCU, MT6816 encoder, 12–24 V, CAN-native PID (`0x96`/`0x97`), OLED faults. Source: *MKS SERVO42&57D_CAN User Manual V1.0.9*.
- 🟩 [[servo42es/00-index|SERVO42ES / 57ES 索引]] — STM32F302 MCU, MT6826 encoder, 20–48 V, **no CAN-PID** (CANGAROO-only), explicit `0x60` SAVE, LED-blink faults. Source: *MKS SERVO42ES/57ES manual V1.0.1, firmware V1.0.1*. Covers **both** SERVO42ES V1.0 (single `IN`) and SERVO57ES V1.0 (`IN_1`+`IN_2`+`IN_COM`); same firmware/protocol, differ only in current rating & I/O count.

---

## Chapter-pair map (the mesh)

每列同時連 42D ↔ 42ES 雙生章節 + 差異摘要。

| # | 主題 | SERVO42D / 57D | SERVO42ES / 57ES | 差異摘要 |
|---|------|----------------|------------------|----------|
| 01 | Protocol & CRC | [[servo42d/01-protocol\|01-protocol]] | [[servo42es/01-protocol\|01-protocol]] | **位元組相容**：同 frame、同 8-bit CHECKSUM CRC、big-endian。ES 文檔明標「42ES/57ES 雙生」。 |
| 02 | Specs & working modes | [[servo42d/02-specs-and-modes\|02-specs-and-modes]] | [[servo42es/02-specs-and-modes\|02-specs-and-modes]] | MCU/編碼器/MOSFET/電壓全異（見下方矩陣）；**mode 數字映射不同**（42D `02H`=CR_vFOC default / `05H`=SR_vFOC；42ES `03H`=Pulse+Dir CL default / `04H·05H`=bus open/closed + No-encoder `10/11/14H`）。 |
| 03 | Read commands | [[servo42d/03-read-commands\|03-read-commands]] | [[servo42es/03-read-commands\|03-read-commands]] | `0x30`/`0x31` 讀碼器位元組相同；42ES **無 `0x35`**（原始累積讀）；ES 編碼器硬體 MT6826（multi-turn native）。 |
| 04 | Config / write commands | [[servo42d/04-config-commands\|04-config-commands]] | [[servo42es/04-config-commands\|04-config-commands]] | **ES 新增 `0x60` SAVE**：所有 write-only 參數下電前未 `0x60` 即丟失；42D 無顯式 SAVE。ES subdivision 多十進位非 2 冪值。 |
| 05 | Motion commands | [[servo42d/05-motion-commands\|05-motion-commands]] | [[servo42es/05-motion-commands\|05-motion-commands]] | `F6`/`FD`/`FE`/`F4`/`F5` **完全相容**；speed 0–3000、`actual_RPM = speed×16/subdivision`、acc step `(256−acc)×50µs` 兩線一致。 |
| 06 | Bus control | [[servo42d/06-bus-control\|06-bus-control]] | [[servo42es/06-bus-control\|06-bus-control]] | `F1`/`F3`/`F7` 相容；ES 配置寫入仍受 `0x60` 持久化規則。 |
| 07 | PID tuning | [[servo42d/07-pid-tuning\|07-pid-tuning]] | [[servo42es/07-pid-tuning\|07-pid-tuning]] | **關鍵分歧**：42D 走 CAN `0x96`(vFOC)/`0x97`(CLOSE)；**42ES 兩碼皆無**，只能用 CANGAROO + USB-CAN dongle 調，CAN frame 無法寫 PID。 |
| 08 | Homing & zeroing | [[servo42d/08-homing-zeroing\|08-homing-zeroing]] | [[servo42es/08-homing-zeroing\|08-homing-zeroing]] | `0x92` 設零相容；ES MT6826 原生 multi-turn 改變 wrap/zero 語意。 |
| 09 | Protection | [[servo42d/09-protection\|09-protection]] | [[servo42es/09-protection\|09-protection]] | 過流/超差/心跳/堵轉概念對應；ES 故障由 LED 紅閃碼呈現，非 OLED 文字。 |
| 10 | I/O ports | [[servo42d/10-io-ports\|10-io-ports]] | [[servo42es/10-io-ports\|10-io-ports]] | 42ES 單 `IN`、57ES `IN_1`+`IN_2`+`IN_COM`；ES 新增 RS485/MODBUS-RTU 物理介面。 |
| 11 | Multi-motor sync | [[servo42d/11-multi-motor-sync\|11-multi-motor-sync]] | [[servo42es/11-multi-motor-sync\|11-multi-motor-sync]] | 廣播 `0x00`、group CAN ID `0x8D`、sync flag `0x4A`/`0x4B` 相容；status `5`=sync-received。 |
| 12 | Errors & recovery | [[servo42d/12-errors-recovery\|12-errors-recovery]] | [[servo42es/12-errors-recovery\|12-errors-recovery]] | `0x3F` 復原 / `0x41` 重啟相容；**ES 多 `0x60` SAVE**；42D OLED 文字 ↔ 42ES「1 Green + N Red」閃碼（非 1:1）。 |

---

## ⭐ 42D vs 42ES — 比對矩陣（centerpiece）

### Hardware

| 項目 | SERVO42D / 57D | SERVO42ES / 57ES |
|------|----------------|------------------|
| MCU | N32L40x (42D=N32L403 / 57D=N32L406) | STM32F302CBT6 (42ES, 48-pin) / STM32F302RBT6 (57ES, 64-pin)，Cortex-M4 |
| 磁編碼器 | **MT6816** (14-bit) | **MT6826** (原生 multi-turn) |
| MOSFET | 40V / 20A 級 | HYG090ND06LS1C2 **60V / 56A** |
| 工作電壓 | **12–24 V** | **20–48 V** |
| 工作電流 | 42D 0–3000mA / 57D 0–5200mA | 42ES 0–3000mA (default 1600mA) / 57ES 0–5200mA (default 3200mA) |
| 控制環頻率 | torque 20kHz / speed 10kHz / position 10kHz | **相同** torque 20kHz / speed 10kHz / position 10kHz |
| Max pulse freq | 160 kHz | 300 kHz (標稱) / port 電氣上限 400 kHz |
| CAN transceiver | (42D 板載) | TJA1051T |
| CAN bit rate | 125K/250K/500K/1M | 125K/250K/500K/1M |
| 顯示/故障介面 | OLED 文字訊息 | **無 OLED**：LED「1 Green + N Red」閃碼 |
| RS485 / MODBUS-RTU | 無（線上文檔純 CAN） | **新增** CAN + RS485 (MODBUS-RTU) 共存 |

### Protocol（位元組層 — 相容）

| 項目 | 42D | 42ES | 結論 |
|------|-----|------|------|
| CRC | `(CAN_ID + data) & 0xFF` | 同 | **byte-identical** |
| Frame / DLC / big-endian | `[ID][DLC][code][data][CRC]` | 同 | 相容 |
| Motion opcodes | `F5`/`F6`/`F4`/`FD`/`FE` | 同 | 相容 |
| Zero / read | `0x92`/`0x30`/`0x31` | 同（42ES 無 `0x35`） | 相容 |
| Enable / status / e-stop | `F3`/`F1`/`F7` | 同 | 相容 |
| Write 回覆 | `DLC=3 [code,status,CRC]` | 同 (motion 加 2/3/5) | 相容 |

### Command-level deltas（關鍵差異）

| 能力 | SERVO42D | SERVO42ES | 影響 |
|------|----------|-----------|------|
| 設 vFOC PID over CAN | `0x96` (default Kp/Ki/Kd/Kv=220/100/270/320) | **無此碼** | 只能 CANGAROO + USB-CAN |
| 設 CLOSE-loop PID over CAN | `0x97` (default 200/80/250/300) | **無此碼** | 同上 |
| 讀 PID over CAN | `0x00` read-config of `0x96`/`0x97` | **無** | 無 PID telemetry 來源 |
| SAVE 參數 | **無顯式 SAVE**（write 即視為已存） | **`0x60`** (control word `0x01`，frame `01 60 01 62`) | ES write-only 參數**未 `0x60` 下電即丟** |
| Subdivision | 1–256 任意 | 2/4/8/16/32/64/128 + 5/10/20/25/40/50/100/200/256 (0→256) | ES 多十進位非 2 冪值 |
| RS485 / MODBUS-RTU | 無 | **新增**（物理介面共存，本 vault 不展開暫存器） | ES 多一條控制總線 |
| 故障呈現 | OLED 文字（`Phase Line Error!` / `Wrong2…` / `Low Voltage Error!`…） | LED 1G+N Red（1=OverCurrent / 2=Open-Phase / 3=V-High / 4=V-Low / 5=Position Error / 6=Encoder Error） | 概念對應但非 1:1，新增「3 Red 過壓」 |
| **Opcode 重映射** | `0x95`=pos-reach門檻 · `0x96`=vFOC PID · `0x97`=CLOSE PID · `0x98`=heartbeat · `0x99`=脈衝分頻 · `0x9F`=IN_1 mode | `0x95`/`0x96`/`0x97`=**歸零三參數** · `0x98`=pos-reach門檻 · `0x89`=heartbeat · `0x9F`=脈衝分頻 · (無 `0x99`/IN_1-mode) | **移植大坑**：六碼整體洗牌，每碼語意改變。完整對照見 [[servo42es/00-index\|42ES 索引「跨產品碼重映射」表]] |

---

## 🔧 Firmware impact — this Stewart platform

本平台 ESP32 + MCP2515 橋接目前跑 42D。若遷移到 42ES：

1. **CAN layer 不動。** Frame/CRC/`F5`/`0x92`/`0x30`/`0x31` 位元組相容 → ESP32 的 CAN 收發與 IK→`0xF5` 軌跡路徑**無需重寫**。
2. **CAN-PID 路徑廢除。** 42ES 無 `0x96`/`0x97` → 序列指令 **`K` / `KS` / `KRESET` 失效**（底層 frame 被拒、write status `0`），`spid`/`pidn` PID telemetry 無來源。調參改用 **USB-CAN dongle (MKS CANable / candle0) + CANGAROO**（必買）。當前「純P 工作點 `[1024,0,0,0]`」是 `0x96` override，**不可轉移** → 須在 CANGAROO 內調定再 `0x60` 存盤。
3. **持久化需 `0x60`。** 42ES 所有 write-only 配置（mode/current/subdivision/speed/acc…）下電前未送 `0x60` 即丟失；ESP32 開機**不再** re-push PID（馬達自己擁有 tuning）。
4. **編碼器處理變更。** MT6826 原生 multi-turn → 取代 42D 的「單圈絕對 + ±180° wrap」假設，影響歸零/wrap 邏輯（搭配減速器解 wrap，見 mega-upgrade 評估）。

> 詳見 [[servo42es/07-pid-tuning|07-pid-tuning（CANGAROO 工作流 + firmware implication）]] 與 [[servo42es/02-specs-and-modes|02-specs（硬體矩陣 + 0x82 mode 映射）]]。

---

## See also

- [[servo42d/00-index|42D / 57D 索引]] · [[servo42es/00-index|42ES / 57ES 索引]]
- 雙生關鍵章節：[[servo42d/07-pid-tuning|42D PID (0x96/0x97 over CAN)]] ↔ [[servo42es/07-pid-tuning|42ES PID (CANGAROO-only)]]
- 持久化分歧：[[servo42es/04-config-commands|42ES 0x60 SAVE]] · [[servo42es/12-errors-recovery|42ES recovery (0x60/0x41/0x3F)]]
