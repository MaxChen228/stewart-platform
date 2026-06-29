# Product Specs & Working Modes

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/02-specs-and-modes|42D Product Specs & Working Modes]]

涵蓋 **SERVO42ES V1.0** 與 **SERVO57ES V1.0**（同韌體 V1.0.1、同協議，差在電流額定與 I/O 數量）。設定/寫入類指令的 frame 結構、CRC 與 big-endian 規則見 [[servo42es/01-protocol|01-protocol]]；working-mode (0x82) 與 subdivision (0x84) 屬 write-only 配置指令，完整收錄在 [[servo42es/04-config-commands|04-config-commands]]，本章只給 spec-level 語意。

---

## Hardware Parameters

| Parameter | SERVO42ES V1.0 | SERVO57ES V1.0 |
|-----------|----------------|----------------|
| Motherboard model | MKS SERVO42ES V1.0 | MKS SERVO57ES V1.0 |
| MCU | STM32F302CBT6 (Cortex-M4) | STM32F302RBT6 (Cortex-M4) |
| MOSFET | HYG090ND06LS1C2 (60V, 56A) | HYG090ND06LS1C2 (60V, 56A) |
| Magnetic encoder | MT6826 | MT6826 |
| CAN transceiver | TJA1051T | TJA1051T |
| Operating voltage | 20V-48V | 20V-48V |
| Operating current | 0-3000mA (default 1600mA) | 0-5200mA (default 3200mA) |
| Torque loop | 20 kHz | 20 kHz |
| Speed loop | 10 kHz | 10 kHz |
| Position loop | 10 kHz | 10 kHz |
| Maximum speed | 3000+ RPM | 3000+ RPM |
| Subdivisions | 2/4/8/16/32/64/128 + 5/10/20/25/40/50/100/200/256 (0→256) | same |
| Wiring method | Common positive / common negative / difference | same |
| Input signal level | 3.3V-24V | 3.3V-24V |
| Max pulse frequency | 300 kHz | 300 kHz |
| CAN bit rate | 125K/250K/500K/1M | 125K/250K/500K/1M |
| CAN ID range | 1 broadcast + 2047 slave IDs | 1 broadcast + 2047 slave IDs |
| Onboard interface | CAN + RS485 (MODBUS-RTU) | CAN + RS485 (MODBUS-RTU) |

註：MCU 命名 `STM32F302C*` = 48-pin LQFP (42ES)、`STM32F302R*` = 64-pin LQFP (57ES)；`B` = 128KB flash。MOSFET `HYG090ND06LS1C2` 由 PDF 標稱 60V/56A（特性 §1.2 #16）。Operating current 兩型不同（見 [[servo42es/04-config-commands|0x83 set current]]）。

> 🔀 **vs 42D：** MCU 從 N32L40x（國民技術，42D=N32L403／57D=N32L406）改為 **STM32F302**（ST Cortex-M4），韌體編譯目標不同但 CAN 應用層協議位元組相容。
> 🔀 **vs 42D：** 磁編碼器 **MT6826**（42D/57D=MT6816）。MT6816 為 14-bit；42ES 文檔僅以 §1.2 「industrial-grade high-precision magnetic encoder」描述，未列位元數。讀位置/編碼器指令見 [[servo42es/03-read-commands|03-read-commands]]。
> 🔀 **vs 42D：** 功率級全面上拉——MOSFET 40V/20A (42D) → **60V/56A**；voltage 12-24V → **20-48V**；42ES current 上限 3000mA 同 42D，但 57ES 5200mA 同 57D（差在 voltage/MOSFET 而非電流上限）。
> 🔀 **vs 42D：** subdivision 集合多了 **十進位非 2 冪值** 5/10/20/25/40/50/100/200（42D 文檔記為「1-256 arbitrary」，未列舉這組離散小數值）。
> 🔀 **vs 42D：** max pulse frequency **300 kHz**（42D=160 kHz）。
> 🔀 **vs 42D：** 新增 **RS485 + MODBUS-RTU**（§1.2 #8, #17）；42D 文檔線上版為純 CAN。本 vault 不展開 MODBUS 暫存器，僅標明物理介面共存。

---

## Working Modes — 0x82 (set working mode)

write-only。Downlink `[ID][DLC=3][82H][mode][CRC]`，回 `[ID][DLC=3][82H][status][CRC]`（status 0=fail / 1=success）。

| mode | Group | Work mode |
|------|-------|-----------|
| 00H | With encoder | Pulse + Pulse Open-Loop FOC |
| 01H | With encoder | Pulse + Direction Open-Loop FOC |
| 02H | With encoder | Pulse + Pulse Closed-Loop FOC |
| 03H | With encoder | Pulse + Direction Closed-Loop FOC **(Default)** |
| 04H | With encoder | Bus (CAN) open-loop FOC |
| 05H | With encoder | Bus (CAN) closed-loop FOC |
| 10H | No encoder | Pulse + Pulse Open-Loop FOC |
| 11H | No encoder | Pulse + Direction Open-Loop FOC |
| 14H | No encoder | Bus open-loop FOC |

- **Note 1**：`With encoder` = 軸上有磁鐵、驅動板裝於背面可讀編碼器值；`No encoder` = 軸無磁鐵、驅動板可任意安裝、無法讀編碼器值。
- **Note 2**：Pulse 控制模式 max input 300 kHz；Bus 控制模式 max 3000 RPM。
- 本專案 Stewart 平台走 **bus 模式**（CAN 下發位置/速度），對應 04H / 05H。bus 控制指令（0xF5 等，見 [[servo42es/05-motion-commands|05-motion-commands]] / [[servo42es/06-bus-control|06-bus-control]]）僅在 bus 模式生效。

#### Worked CRC — set mode = 05H (SR_vFOC bus closed-loop), ID = 0x01
```
frame = [01] [82 05] [CRC]
CRC = (0x01 + 0x82 + 0x05) & 0xFF = 0x88
→ 01 82 05 88
```
回覆 success：`(0x01 + 0x82 + 0x01) & 0xFF = 0x84` → `01 82 01 84`。

> 🔀 **vs 42D：** 42D 用 **CR_/SR_ 命名**（CR_OPEN/CLOSE/vFOC = 00/01/02、SR_OPEN/CLOSE/vFOC = 03/04/05）。42ES 改用 **encoder-presence 分組 + 描述式命名**，且 mode 值映射不同：42ES `03H`=Pulse+Direction Closed-Loop **(default)**、`04H/05H`=bus open/closed；42D `02H`=CR_vFOC (default)、`05H`=SR_vFOC。**設值前務必對照本表，勿沿用 42D 的 mode 數字。**
> 🔀 **vs 42D：** 42ES 增加 `10H/11H/14H` 的 **No-encoder 群組**（軸無磁鐵、可作純開環脈衝/bus 用），42D 文檔無此分組。
> 🔀 **vs 42D：** 42D 文檔每個 mode 附 Max RPM/Work Current 欄（OPEN=400 / CLOSE=1500 / FOC=3000，FOC 自適應電流）。42ES 0x82 表未逐 mode 列 RPM/電流，僅以 Note 2 給「pulse 300kHz / bus 3000RPM」總值；電流由 [[servo42es/04-config-commands|0x83]] 獨立設定。

---

## Speed Parameter（§8.1，bus 模式專用）

- 範圍 **0-3000**。值越大轉越快；`speed = 0` → 停轉。
- 設定值若 **超過控制模式上限**，馬達以該模式上限運轉。
- 速度值以 **16/32/64 subdivision 校準**；其他 subdivision 須以 16 細分為基準換算：
  `actual_RPM = speed × 16 / subdivision`
  PDF worked example（speed = 1200）：
  | subdivision | actual RPM |
  |-------------|-----------|
  | 8 | 2400 |
  | 16 / 32 / 64 | 1200 |
  | 128 | 150 |

> 🔀 **vs 42D：** 公式與 0-3000 範圍一致（位元組相容）。差異僅在「控制模式上限」的成因——42D 文檔明列 OPEN=400/CLOSE=1500/FOC=3000；42ES §8.1 只說「runs at the maximum speed of the control mode」未在此節重列各 mode 上限數字（總上限 3000 RPM 見 0x82 Note 2）。

---

## Acceleration Parameter（§8.1，bus 模式專用）

- 範圍 **0-255**。值越大加/減速越快。
- `acc = 0` → 無加減速斜坡，直接跳到設定速度。
- 每遞增/遞減 **1 RPM** 的時間步：`t2 − t1 = (256 − acc) × 50 µs`。
  - 加速：`Vt2 = Vt1 + 1`（直到 `Vt2 <= speed`）。
  - 減速：`Vt2 = Vt1 − 1`（直到 `Vt2 >= speed`）。
- PDF worked example（acc = 236, speed = 3000）：每步 `(256−236)×50µs = 1000 µs = 1 ms`，故 T=0→0RPM、T=1ms→1RPM、…、T=3000ms→3000RPM（線性 1RPM/ms 爬升）。

> 🔀 **vs 42D：** 公式 **完全一致**（`(256 − acc) × 50 µs` per 1 RPM step、acc=0 即時切換）。位元組與語意相容，可直接沿用 42D 的 acc 調參經驗。

---

## ⚠️ Persistence — 0x60 SAVE（42ES-critical）

§4.3 開宗明義：**write-only 參數寫入後不會立即保存；必須執行 `0x60` 指令一次性存盤**（Part 4 note 3 + §4.5.1）。本章涉及的 0x82 (working mode)、0x83 (current)、0x84 (subdivision) 及 §8.1 的 speed/acc 設定皆屬此類——下電後若未 0x60，改動丟失。

> 🔀 **vs 42D：** 42D 文檔集 **沒有顯式 SAVE 指令**（多數設定即時生效/各自存盤語意不同）。42ES 統一改為「先 write、後 0x60 commit」模型。完整 0x60 frame 與行為見 [[servo42es/04-config-commands|04-config-commands]]，任何 write/config 章節都會重申此鐵律。

---

## I/O Ports（§1.5 Port Function）

| PIN | I/O | Function | Notes |
|-----|-----|----------|-------|
| STP+/STP- (CW+/CW-) | Input | Pulse signal (or CW pulse) ± | Optocoupler, falling-edge trigger; one step per high→low |
| DIR+/DIR- (CCW+/CCW-) | Input | Direction signal (or CCW pulse) ± | Single-pulse (Pulse+Dir) or double-pulse (CW+CCW) 由指令設定 |
| EN+/EN- | Input | Enable signal ± | Optocoupler, **low-level effective**；失效時 motor released + 清警報 |
| ALM+/ALM- | Output | Alarm signal ± | 警報時 optocoupler ON；max +35V / 50mA |
| PEND+/PEND- | Output | In-Position signal ± | 完成指定脈衝時 optocoupler ON；max +35V / 50mA |
| A-/A+/B-/B+ | Output | Motor phase A-/A+/B-/B+ | 相序接錯馬達會報警 |
| VDC / GND | Input | Drive power supply +/- | Operating Voltage +20-48V |

脈衝輸入電氣規格：positive voltage 3.3-28V；negative high 3.3-28V、low 0-0.5V；max input frequency 400 kHz、pulse duration > 2.5 µs。完整 I/O 重映射（limit/home）見 [[servo42es/10-io-ports|10-io-ports]]。

- **SERVO42ES**：單組輸入（single IN）。
- **SERVO57ES**：IN_1 + IN_2 + IN_COM（雙輸入 + 共腳），對應左右限位 / home 擴充。

> 🔀 **vs 42D：** 42D 文檔以 IN_1/IN_2/OUT_1/OUT_2 的「預設端口功能 + 機型有無」表呈現（57D 全有、28/35/42D 部分）。42ES §1.5 改以 **完整 pin-out 表**（STP/DIR/EN/ALM/PEND/相線/電源）描述，語意對應：ALM≈stall/alarm 輸出、PEND≈in-position 輸出。
> 🔀 **vs 42D：** 脈衝電氣 spec 在 §1.5 標 max input frequency **400 kHz / pulse > 2.5µs**（注意：與 §1.2/§1.3 的「pulse 300 kHz」並存——300kHz 為 product-parameter 標稱、400kHz 為 port 電氣上限）；42D 對應值為 160 kHz。
> 🔀 **vs 42D：** 57ES = IN_1+IN_2+IN_COM、42ES = 單 IN，與 42D/57D 的 IN_1/IN_2 分歧邏輯一致但腳位命名不同。

---

## LED Status（§1.6 Indicator Light）

| LED | Meaning |
|-----|---------|
| Green stays on | Motor Running |
| Green flash | Motor Stop |
| 1 Green + 1 Red | Over Current |
| 1 Green + 2 Red | Open-Phase |
| 1 Green + 3 Red | Supply Voltage High |
| 1 Green + 4 Red | Supply Voltage Low |
| 1 Green + 5 Red | Position Error |
| 1 Green + 6 Red | Encoder Error |

Tip（PDF）：相線接錯，收到脈衝後會出現 tracking error 警報（**5 red + 1 green** = Position Error）。錯誤碼/復位流程見 [[servo42es/12-errors-recovery|12-errors-recovery]]；保護機制見 [[servo42es/09-protection|09-protection]]。

> 🔀 **vs 42D：** LED 閃碼語意與 42D 共用同一套「1 Green + N Red」family；42ES 明確列出 6 種紅閃級別（Over Current / Open-Phase / V-High / V-Low / Position Error / Encoder Error）。對照 [[servo42d/02-specs-and-modes|42D 02 章]] 與 [[servo42d/12-errors-recovery|42D 12 章]] 的錯誤碼確認逐位元對應。
