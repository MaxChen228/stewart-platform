# Stewart Platform — Mega 進化設計

> 調查日期 2026-06-29｜狀態：**評估／設計定案，未採購**
> 三大主軸：**ESP32-S3 控制電子** + **SERVO42ES 馬達** + **1:3 行星減速器**

設計圖（獨立 SVG，瀏覽器可直接開）：
- [`power-tree.svg`](power-tree.svg) — 電源樹（市電→24V→馬達/邏輯 UPS）
- [`can-topology.svg`](can-topology.svg) — CAN 鏈＋隔離＋42ES 菊鏈拓樸
- [`star-ground.svg`](star-ground.svg) — 單點星接地

參考：[`../servo42es/refs/MKS_SERVO42ES_CAN_Manual_V1.0.1.pdf`](../servo42es/refs/MKS_SERVO42ES_CAN_Manual_V1.0.1.pdf)（firmware V1.0.1 / 2025-11；整理版文檔集見 [`../servo42es/`](../servo42es/00-index.md)）

---

## 0. TL;DR 決策

| 項目 | 現狀 | Mega 進化 | 理由 |
|---|---|---|---|
| 主控 | ESP32 WROOM | **ESP32-S3 N16R8** | 內建 TWAI、原生 USB、16MB/8MB |
| CAN 控制 | MCP2515(SPI) + TJA1050 | **S3 內建 TWAI + 隔離收發 CTM1051M** | 消除 SPI 延遲與 2-buffer 瓶頸、斷馬達地迴路 |
| 馬達 | SERVO42D（vFOC 手調純P） | **SERVO42ES**（工業三環 FOC） | 出廠整定、原生 multi-turn、協議相容 |
| 傳動 | 直驅 | **1:3 行星減速** | 力矩 ×2.7、解析度 ×3、配 multi-turn |
| 邏輯供電 | USB | **24V→buck→BQ24074 power-path + 18650 緩衝** | 拔電不斷電、可偵測市電有無 |
| 調參工具 | K/KS 指令 | **USB-CAN dongle + cangaroo（必買）** | 42ES PID 只能上位機調 |

**判定**：方向正確且划算。協議相容 → 韌體不重寫；三環 FOC 很可能直接解決舊的 M4 極限環/手調 PID；multi-turn 正好配減速器。最大新變數＝**減速器背隙**與**調參改 cangaroo**。

---

## 1. 控制電子升級

- **板**：ESP32-S3-DevKitC-1 **N16R8**（16MB Flash / 8MB Octal PSRAM）。
- **CAN**：丟掉 MCP2515，用 S3 內建 TWAI（CAN 2.0）+ 收發器。建議 **CTM1051M 隔離 CAN**（內含隔離 DC-DC，斷開 ESP32 地與馬達側 CAN 地）；預算版退回裸 VP230(SN65HVD230)。
- **邏輯 UPS**：`24V → buck 5V → BQ24074 power-path → ESP32 5V`；18650 掛 BQ24074 BAT，斷 AC 無縫切換。`MAX17048` I²C 燃料計回報 SoC%。`24V sense` 分壓進 ADC → ESP32 能主動偵測市電消失並回報/failsafe。
- **Host**：先用 USB-C UART 橋孔（drop-in 現有 server.js）；WiFi(TCP:3333) 已有；原生 USB CDC 留未來。

### GPIO 腳位（ESP32-S3-N16R8）
| 功能 | 腳位 | 接到 |
|---|---|---|
| TWAI_TX | GPIO5 | CTM1051M TXD |
| TWAI_RX | GPIO4 | CTM1051M RXD |
| I²C SDA / SCL | GPIO8 / GPIO9 | MAX17048 |
| ADC（24V sense） | GPIO1 | 24V 分壓中點（ADC1_CH0, 11dB） |
| 狀態 LED | GPIO48 | 板載 RGB |
| 5V in / GND | 5V / GND 腳 | BQ24074 SYS / 星點 |

**禁用腳**：GPIO26–32（封裝內 flash）、**GPIO33–37（N16R8 octal PSRAM）**、strap 0/3/45/46、GPIO19/20（保留原生 USB）、GPIO43/44（UART0 console）。

---

## 2. SERVO42D → SERVO42ES

### 2.1 規格對比（查證自 CAN Manual V1.0.1）
| 項目 | SERVO42D（現用） | **SERVO42ES** |
|---|---|---|
| MCU | — | STM32F302CBT6 |
| 編碼器 | 14-bit **單圈**（韌體自行 wrap） | MT6826，**原生 multi-turn 累積坐標**（0x31, int48, 16384/圈） |
| 控制 | vFOC（手調純P[1024,0,0,0]） | **三環 FOC**：力矩 20kHz / 速度 10kHz / 位置 10kHz，出廠整定 |
| 電壓 | — | **20–48V**（注意非 42E 的 60V）→ 24V 安全 |
| 電流 | — | 0–3000mA（預設 1600） |
| CAN | 500k | 125k/250k/500k/1M；收發器 TJA1051T；1 broadcast + 2047 ID |
| MOSFET | — | HYG090ND06LS1C2（60V/56A） |
| 介面 | — | **兩組 CAN 並聯 + 兩組 RS485 並聯**（BUS 版 2×11 排針）；馬達相線 A-/A+/B-/B+；5V 輸出 100mA |
| CRC | `(ID+bytes)&0xFF` | **完全相同** |

### 2.2 CAN 協議相容性（韌體影響）
**位元組級相容，主力指令碼全保留：**
- `0xF5` 絕對坐標 + 即時更新：`ID F5 speed(2) acc abs(int24) CRC`，範例 `01 F5 02 58 02 00 40 00 92`（600RPM, acc2, 0x004000=1圈）。→ followTgt/PF 串流直接沿用。
- `0x92`/`0x91` 設零、`0xF7` 急停、`0x30`(單圈+carry)/`0x31`(multi-turn 累積)、`0x60` 存 NVS、`0x82` 工作模式、`0x83` 電流、`0x84` 細分 — 全在、語意一致。
- bus 控制啟動序列：`82 05 88`(工作模式=05H bus 閉環FOC) → `60 01 62`(存) → `92 00 93`(設零) → `F5 …`(動) → `31`(讀坐標) → 停用 `F5 …acc 0…` 或 `F7`。

**重大改變 — 42ES 不開放用 CAN 設 PID**：4.3 write 全節（4.3.1–4.3.14）無任何 set-PID 命令；手冊所有調參示範一律走 **cangaroo 上位機**。→ `K`/`KS`/`KRESET` 指令在 42ES 無對應，**移除**。（純P 本是為壓 42D M4 Ki 極限環的 workaround，三環 FOC 出廠整定理論上不需要。）

**其餘小改**：讀編碼器 `0x35`(raw)→`0x30`/`0x31`；校正改 multi-turn 原生；工作模式設 05H+存。

---

## 3. 1:3 行星減速器（運動學大事）

- **角度映射**：IK 算輸出/腿角度，馬達端轉 ×3 → `angleToCoord` 乘 3。三處真相源（`kinematics.h`、`web/index.html`、`sysid/kin.js`）加 `GEAR_RATIO=3`。
- **多圈天作之合**：1:3 後輸出 ±60° = 馬達 ±180°，**正好撞 42D 的 ±180° wrap 限制**；而 42ES 原生 multi-turn 坐標連續不 wrap → **換 42ES 與上減速器互相成全**（換它的關鍵理由之一）。
- **解析度 ×3**（輸出端 ~0.0073°/count）、**力矩 ≈ ×2.7**（含效率）→ 剛性/承載大增，利於死咬/推擾回正。
- **唯一代價 — 背隙**：編碼器在馬達軸（減速器**前**），閉環測不到減速器背隙（經濟級 ~0.5–1°、精密級 <15 arcmin）。此為死咬精度物理上限 → 要回正精度就選**低背隙**等級。

---

## 4. 接線

- **馬達 CAN**：42ES 每顆**兩組並聯 CAN** → 直接 in→out **菊鏈**（M1→M2→…→M6），**免外部 CAN busbar**。**120Ω 只放匯流排兩物理端**（ESP32/CTM 端 + 末端 M6）。CANH/CANL 用一對絞線。
- **馬達電**：24V power busbar → 各馬達 VIN(20–48V)；相線走 A-/A+/B-/B+ 端子。
- **單點星接地**：PSU、buck、UPS、ESP32 的 GND 全回 **PSU 負極一點**；CAN 匯流排地走馬達側、靠 CTM 隔離不另成迴路（見 `star-ground.svg`）。
- **線徑**：24V 主幹（350W≈15A 尖峰）≥14AWG；馬達分支 18–20AWG；CAN/訊號 24–26AWG。
- **24V sense 分壓**：R1=100k / R2=12k → 24V→2.57V，並聯 3.3V 齊納箝位。

---

## 5. BOM（相對現行的增刪）
| 動作 | 料 |
|---|---|
| 換 | ESP32-S3-N16R8 |
| 刪 | MCP2515 模組（整顆） |
| 換 | TJA1050 → **CTM1051M 隔離 CAN**（預算版 VP230） |
| 換 | SERVO42D ×6 → **SERVO42ES ×6 + 1:3 行星減速器 ×6** |
| 加 | BQ24074 power-path + 24V→5V buck + 18650×1(含保護) |
| 加 | MAX17048 燃料計 |
| 加 | **USB-CAN dongle（CANable 2.0 / SLCAN，cangaroo 相容）** |
| 加 | 120Ω ×2、24V 分壓電阻 |
| 留 | 24V/350W PSU |

---

## 6. USB-CAN（必買，非選配）
手冊全程上位機 = **cangaroo + CANable/candle(SLCAN)**。理由排序：
1. **42ES 的 PID 只能用 cangaroo 調 → 無 USB-CAN 無法整定馬達**。
2. 逐顆驗 CAN 通訊、抓包除錯。
3. 韌體未完成前先 PC 直驅馬達跑 0xF5 驗證。
與 ESP32 的收發器並存（CAN 多節點）；掛 dongle 時注意 bus 終端。

---

## 7. 韌體遷移清單（MCP2515→TWAI + 42D→42ES）
1. `platformio.ini`：env 改 `esp32-s3-devkitc-1`。
2. CAN 層：MCP2515 lib → `driver/twai.h`（500k，RX queue≥32）；**刪 `flushReceiveBuffer()` 每-cycle hack**。
3. 腳位重映（見 §1 表）；序列走 S3 UART 橋。
4. 移除 `K`/`KS`/`KRESET`（PID 改 cangaroo）。
5. 啟動序列改 42ES：工作模式 05H + `0x60` 存。
6. 編碼器：`0x35`→`0x30`/`0x31`，校正改 multi-turn 原生坐標（可廢自行 wrap 邏輯）。
7. 運動學三處加 `GEAR_RATIO=3`。
8. telemetry 加 `batt_pct`/`batt_v`/`ac_present`。

---

## 8. 待辦 / 未決
- 行星減速器**背隙等級**選型（精度 vs 成本）。
- 隔離 CAN（CTM1051M）vs 裸 VP230 最終取捨。
- 18650 ×1 vs ×2P。
- host link 主走 USB 還 WiFi。

---

## 9. 來源
- [MKS-SERVO42ES-57ES (repo)](https://github.com/makerbase-motor/MKS-SERVO42ES-57ES) — 規格 + 本資料夾的 CAN Manual V1.0.1
- [MKS-SERVO42E-57E (repo)](https://github.com/makerbase-motor/MKS-SERVO42E-57E) — 對比
- [MKS-SERVO42D-57D (repo)](https://github.com/makerbase-motor/MKS-SERVO42D-57D) — 現用對照
- ESP32-S3 N16R8 開發板（傑森創工，蝦皮，NT$295）
