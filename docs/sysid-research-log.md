# Stewart Platform — 系統辨識研究記錄

> 可持續記錄。每次有新數據/決策就**往下追加一筆 dated entry**，不要改舊的（保留歷程）。
> 數據檔在 `sysid/data/`（git ignore，本機保存）。工具：`sysid/analyze.py`、`sysid/excite.js`、`server.js` 記錄器。

## 背景與動機

傳統 PID/Task-space PD 無法穩定平台：稍大擾動即發散、不可恢復。轉向前先做系統辨識，
判別問題是 **control-law-limited（模型對、控制律不夠）** 還是 **bandwidth-limited（迴圈太慢/通訊太髒）**。
兩者建議相反：前者 → LQR/MPC 解；後者 → 先修硬體迴圈，RL/MPC 都救不了。

## 工具鏈（如何重現）

- `npm start` 啟動 server（含 JSONL 記錄器）
- 記錄：`curl "localhost:3000/api/rec/start?name=XXX"` → 動作 → `curl localhost:3000/api/rec/stop`
- 分析：`uv run sysid/analyze.py sysid/data/XXX.jsonl` → 產出 `.fingerprint.json` + `.png`
- 送指令：WebSocket ws://localhost:3000（server 轉發到序列埠）
- 韌體遙測欄位：`a[]`角度 `r[]`原始 `ok`讀成功數 `ef`MCP2515錯誤旗標 `tx/rx`TEC/REC錯誤計數
  `t`esp毫秒 `dt`迴圈週期 `cus`讀6編碼器µs（auto-return 下=0）

---

## 2026-06-24 — Phase 0：感測管線特性化

### 關鍵教訓（最高準則）
**真實數據說話，永不臆測。** 已被坑兩次：(1) 把 tx/rx 誤判為封包數（實為 CAN 錯誤計數器）；
(2) 推理「M6 是壞硬體」被輪轉實驗推翻。任何改動 → 燒錄 → 量測 → 才下結論。

### 發現 1：靜止時系統乾淨，問題在運動/負載
idle 20s 量測：迴圈 50Hz std 0、編碼器雜訊 ~1 LSB(0.02°)、CAN 錯誤(tec/rec/eflg)全 0、掉幀 ~0%。
→ 不穩定不來自 idle 感測/通訊雜訊。

### 發現 2：感測延遲是最大嫌疑（6.3ms/cycle）
韌體加 `cus`（micros 量讀取）後：**讀 6 顆編碼器 = mean 6.3ms / p99 7.7ms**，佔 20ms 週期 32%，
且為 idle（未疊加 F5）。根因：`readAllRawEncoders` 逐顆阻塞（送 0x35→等回覆→換下一顆）。
含義：回授至少落後 6.3ms + 20ms 離散化 → 閉迴圈頻寬卡在個位數 Hz → **強烈指向 bandwidth-limited**。

### 改進嘗試（編碼器讀取，idle 對比）

| 版本 | cus 平均 | cus 中位 | cus p99 | 掉幀率 | 備註 |
|------|---------|---------|---------|--------|------|
| 逐顆阻塞（原始）| 6311µs | 6318 | 7726 | 0% | 慢但穩 |
| naive burst（先全送再收）| 4375 | 2703 | 12093 | **18%** | 溢位，退步 |
| interleaved drain（邊送邊清）| 2965 | 2554 | 7970 | 5.8% | 折衷 |
| **auto-return（馬達主動上報）** | **0** | 0 | — | **0%** | 見發現 4 |

### 發現 3：掉幀隨「查詢位置」走，非馬達身分（M6 洗清）
固定順序時掉幀集中 M6（疑似壞硬體）；改每 cycle 輪轉查詢起始順序後，失敗均勻
分散 [23,11,14,15,13,14] → 證明是「最後查詢的回覆撞滿 2-buffer MCP2515 而丟」，非 M6 故障。
輪轉永久保留（公平分攤 staleness）。

### 發現 4：auto-return（item 1）靜態驗證成功
做法：0x01 指令令 6 顆每 3ms 主動上報 0x35 位置；ESP32 在 loop 高速空轉中連續排空（無需 ISR）。
控制環只取 RAM 快照 → `cus`=0。靜態 15s 結果：**掉幀 0%、6 顆永遠齊全**。
微妙處：`ef:64`（RX overflow）100% 亮 → buffer 確實一直溢位、確實丟幀，**但因每 3ms 重送、
控制每 20ms 才取樣，冗餘使單幀丟失無害**。
⚠️ 未驗：(a) Enable 發 F5 後 bus 加載撐不撐得住；(b) 新鮮度 ≤3-6ms 為推算非實測。

### CAN 頻寬天花板（協定數學，500kbps，每訊息~0.25ms，上限~4000/s）

| 迴圈 | F5/秒 | +auto-return(3ms)/秒 | 合計 | 狀況 |
|------|-------|---------------------|------|------|
| 20ms | 300 | 2000 | 2300 | 輕鬆 |
| 10ms | 600 | 2000 | 2600 | OK |
| 5ms | 1200 | 2000 | 3200 | 偏緊 |
| 3ms | 2000 | 2000 | 4000 | 塞爆 |

→ 500k 下迴圈甜蜜點約 5~10ms（100~200Hz）。要 3ms（333Hz）需 1M baud（item 3）或換 TWAI（item 2）。
另注意：序列埠 115200 baud 無法承載 333Hz 全遙測 → 遙測需與控制率解耦（固定 ~30ms 輸出）。

---

## 2026-06-24 — 可調速率 + 負載矩陣

新增韌體能力：控制迴圈週期可調（`L <ms>`）、上報週期可調（`AR <ms>`）、遙測與控制解耦
（固定 ~33Hz 輸出，免塞爆 115200 序列埠）、遙測加 `lhz`（實測控制 Hz）、`ar`（上報週期）。

負載矩陣（auto-return AR=3ms，掃迴圈週期 × {disable, hold}；hold=馬達鎖位純送 F5）：

| loop | state | 實測Hz | 掉幀% | RXovf% | TECmax | RECmax | cus |
|------|-------|--------|-------|--------|--------|--------|-----|
| 3  | dis  | 250 | 0 | 100 | 205 | 0 | 0 |
| 3  | hold | 125 | 0 | 99 | **251** | 122 | 0 |
| 5  | dis  | 156 | 0 | 100 | 181 | 0 | 0 |
| 5  | hold | 91 | 0 | 99 | 249 | 104 | 0 |
| 10 | dis  | 97 | 0 | 100 | 173 | 0 | 0 |
| 10 | hold | 61 | 0 | 99 | 252 | 0 | 0 |
| 20 | dis  | 50 | 0 | 100 | 198 | 0 | 0 |
| 20 | hold | 47 | 0 | 100 | 249 | 1 | 0 |

### 結論
1. **迴圈確實能大幅加速**：50Hz → disable 達 250Hz、hold 達 125Hz。但**達不到設定目標**
   （L3 目標 333、disable 只 250、hold 只 125）→ 高速時瓶頸換成「每 cycle 發 6 個 F5 的耗時」，
   hold 比 disable 砍半。
2. **掉幀率全 0%**：即便 L3 hold 最大負載，auto-return 冗餘仍讓 6 顆永遠齊全。
3. **🚩 紅旗：auto-return 下 MCP2515 持續 RX 溢位（~100%），且 hold 時 TEC 衝到 249-251 / 255**
   （255=bus-off 全斷）。在最大負載下幾乎把 CAN 控制器推到 bus-off 邊緣。一退回輪詢 ef/tx 立刻歸 0
   → 證實是 auto-return 把 2-buffer MCP2515 操爆。**掉幀 0 是冗餘救的，但 bus 電氣上處於瀕死狀態，
   不可長期這樣跑。** TEC 為何在 disable（ESP32 不發送）也偏高、hold 衝頂的確切機制未明，待查
   （勿臆測）。
4. → 強力佐證 **item 2（換 ESP32 原生 TWAI 深 FIFO）** 才是「同時要低延遲 + 高迴圈 + 乾淨 bus」的正解。
   auto-return + MCP2515 可當過渡，但不能在 hold/高速長跑。

### 安全狀態
測試後已 `A0`（回輪詢）+ `L 20`，ef/tx 歸 0。預設開機亦為輪詢。

## 2026-06-25 — 負面結果：F5 回覆不是 bus-off 元凶

假設：hold 時 TEC 逼近 bus-off(251) 是 F5 指令回覆（0x8C 預設每指令回 1-2 幀）灌爆 bus。
實驗：auto-return + hold，設 0x8C=0（馬達完全不回覆），比對 TEC。

| loop (hold) | TEC 有回覆 | TEC 關回覆(C00) |
|------|------|------|
| 10ms | 252 | 242 |
| 5ms | 249 | 235 |
| 3ms | 251 | 245 |

**結果：關 F5 回覆幾乎沒降 TEC（251→245）。假設錯誤（第二次被數據打臉，謹記準則）。**

真凶 = **auto-return 串流本身**（2000 幀/秒灌進 2-buffer MCP2515、ef 持續 100% 溢位）把控制器操到 redline，
與 F5 回不回覆無關。佐證：一退回**輪詢模式 TEC 立即歸 0、ef=0**（輪詢不串流、不溢位）。

附帶證實：**0x8C=0 不影響 auto-return**（push 機制獨立於指令回覆）→ 「位置用 auto-return 推送 +
指令 fire-and-forget」架構可行，只是在 MCP2515 上 auto-return 本身就會 redline。

### 更新的權衡認知（MCP2515 上，二選一都不完美）
- **輪詢(pipelined)**：bus 乾淨 TEC=0，但讀取延遲 2.7ms + 掉幀 5.8%
- **auto-return**：延遲~0 + 掉幀 0%，但 bus redline（TEC~240，瀕臨 bus-off）

→ 同時要「0 延遲 + 0 掉幀 + 乾淨 bus」在 2-buffer MCP2515 上做不到，需 TWAI 深 FIFO。
**未測的最後免費槓桿**：調慢 auto-return（AR 5/8/10ms）能否把串流降到 2 buffer 跟得上、TEC 退出 redline。

## 2026-06-25 — Gate A 已儀器化（待人在硬體旁執行）

**安全決策（嚴謹紀律）**：不盲目通電跑閉迴圈。平台斷電被手轉過、現處未知/可能極端姿態
（曾見 a[2]=253°），且已知「任何擾動即發散」，又無人監看 → 三者疊加下盲 enable 是賭硬體，不跑。

已把 Gate A 做成 turnkey（`sysid/gate_a.js` + `sysid/gate_a_analyze.py`）：

**執行前置（人在硬體旁親手）**：① 平台置中位（下腿朝上≈90°）② 送 `Z` 重新歸零
③ 周圍淨空、手在電源開關旁 ④ server 在跑、ok:6。

**協定**：輪詢模式（乾淨 bus，不用 redline 的 auto-return）→ 在 50/100/167Hz 三種迴圈下
enable（Task-space PD，target=當前姿態不暴衝）→ 注入 z +4mm 階躍 → 記錄 pose error
收斂/發散。每段間斷電。`gate_a_analyze.py` 用「後1/3 vs 中1/3 error RMS 比值」判定收斂/發散。

**這實驗回答整個專案的核心**：迴圈加速到底治不治發散？
- 100-167Hz 收斂 → 不用 TWAI/1M/auto-return，問題解決（bandwidth-limited 確認且夠用）。
- 仍發散 → 才證明需要更高速或非控制律問題，TWAI 才有意義。

## 決策與待辦（現況摘要）

- **🎯 Gate A（核心未答，下一步）**：迴圈加速到底有沒有讓平台變穩？已儀器化 turnkey
  （`sysid/gate_a.js` + `gate_a_analyze.py`），用乾淨輪詢即可測。**等人在硬體旁：置中位 → 確認姿態對稱
  （多半免重 Z）→ 手在電源旁 → 執行**。結果分流：100-167Hz 收斂→不用換硬體；仍發散→才需更高速。
- **item 1（auto-return）**：✅ 實作+靜態驗證（cus→0、掉幀0%）。但 hold 時 MCP2515 redline（TEC~250）。
  預設關閉，A1 啟用、A0 退回。**結論：不能在高速/hold 長跑**。
- **0x8C 關 F5 回覆**：✅ 實作（C 指令）。實測**不是** bus-off 元凶（251→245），auto-return 串流本身才是。
- **item 3（1M baud）**：⏸ 不在現有 8MHz 晶振盲做（出規格、改馬達鮑率不可逆）。替代：16/20MHz MCP 模組或併 TWAI。
  注意：1M 加頻寬、TWAI 只解溢位不加頻寬——兩者正交。
- **item 2（換 TWAI）**：深 FIFO 同時拿低延遲+0溢位+乾淨bus。**僅在 Gate A 證明輪詢不夠快時才做**，別超前投資。

## 硬體接線速查
**現用**：ESP32→(SPI: GPIO18 SCK/23 MOSI/19 MISO/5 CS/17 INT)→MCP2515→(內建)→TJA1050→(CANH/CANL,120Ω×2,500k)→6×SERVO42D(0x01-0x06)。
**TWAI 改裝（若做）**：拔掉整塊 MCP2515，改用 **3.3V 收發器 SN65HVD230**（勿用 5V TJA1050，會燒 ESP32 RX）：
GPIO21→CTX、GPIO22→CRX、3.3V→VCC、GND→GND；收發器 CANH/CANL 接原匯流排。CAN 線、馬達、終端電阻全不動。
邏輯電(3.3V) 與 馬達動力電(12-24V) 分離、共地。量 CANH-CANL 應 ~60Ω（兩顆 120Ω 並聯）。

## 2026-06-27 — TWAI 到貨前準備：可觀測性與頻寬預算

背景更新：已下單 SN65HVD230 類 3.3V CAN transceiver，目標從「讓 MCP2515 勉強可用」轉為
「ESP32 TWAI + 可能 1M bitrate，挑戰 200Hz，300Hz 作為 stretch goal」。

### 新增工具

- `sysid/can_budget.js`：用控制頻率、encoder 上報頻率、ACK 開關、bitrate 估算 CAN frame/s 與 bus load。
  之後所有 200-300Hz 設計先跑預算，不憑感覺。
- `sysid/can_benchmark.js`：HOLD/CAN 條件矩陣 benchmark 腳本。預設 dry-run，需 `--live` 才送命令；
  使用現有 `/api/rec/start` JSONL 記錄格式，輸出 `ef/tx/ovr/bus.rx/lhz/per/hmax` 摘要。
- `docs/can-upgrade-plan.md`：MCP2515→TWAI、500K→1M 的分階段計劃與驗證判準。

### 修正 6/25 C00 結論的適用範圍

6/25 在舊條件下觀察到「關 F5 回覆幾乎沒降 TEC」。6/26-27 在 core0 drain + 最新 telemetry 下，
HOLD 監控看到：

- `C 1 0`, `L 10`：`ovr_sum=29/30`，`tx` 最高約 219。
- `C 0 0`, `L 10`：`ovr_sum=9/30`，有改善但不根治。
- `C 0 0`, `L 20`：`ovr_sum=4/20`，目前 MCP2515 HOLD 下較穩。

因此更新判斷：**F5/命令回覆不是唯一元凶，但在 core0 drain 後確實是可量測壓力來源之一**。
主因仍是 MCP2515 2-buffer + SPI 架構對高頻 auto-return/F5 流量太脆。

### 目前決策

- 等待 TWAI transceiver 到貨期間，不再重度投資 MCP2515 修補。
- 短期 HOLD 實驗優先使用 `C 0 0` + 較保守 loop，例如 `L 20`。
- 到貨後先做 TWAI 500K，確認上層行為一致，再做 1M migration。
- 1M 只解 bus bandwidth；TWAI 解 host RX queue/latency。兩者正交，200-300Hz 需要一起評估。

## 2026-06-27 — MPC 可行性判斷與耦合辨識工具

背景：Z/heave 動作截圖顯示能到位但動態過程有明顯 cross-axis wobble；`Ki=20` 能改善靜態貼近目標，
`Ki=0` 不一定改善動態振盪。結論修正：問題不是單純 PID 某一項，而是 **平台層 MIMO 耦合 + 命令/回授頻寬 +
馬達黑盒內環** 的綜合問題。

### SERVO42D 對 MPC 的硬體條件

本地手冊確認 SERVO42D 支援外層 MPC 所需的大部分介面：

- `0xF5` 絕對座標位置指令支援 real-time updates，可作為外層控制輸入。
- `0x35` raw accumulated encoder 可作為位置回授；`0x01` auto-return 可串流唯讀參數。
- `0x32` real-time speed、`0x39` position angle error 可作為後續狀態觀測補強。
- `0x4A/0x4B` 支援多馬達同步觸發，將來可減少六顆 F5 sequential skew。
- `0x8A` 支援 1M CAN bitrate。
- 內部 position/speed loop 10kHz、torque loop 20kHz，但這些是黑盒，外部不能直接 command torque。

因此可做的是 **outer-loop position/pose MPC**：MPC 輸出 target pose/motor target deltas，SERVO42D 內環負責追蹤。
不應把它當成 torque-mode 工業伺服來做力矩 MPC。

### 新增工具

- `docs/mpc-readiness.md`：硬體能力、瓶頸、MPC 分階段架構與判斷。
- `sysid/coupling_probe.js`：預設 dry-run 的耦合辨識動作腳本；`--live` 才會實際送 H/P/FOLLOW/PF 並錄 JSONL。
- `sysid/kin.js` 新增 Node 版 FK `solveFK()`，與前端 FK 同源，用於離線分析錄檔。
- `sysid/coupling_summary.js`：把 JSONL 轉成 FK pose 統計、target error、cross-axis coupling ratio。

### CAN 預算更新

ACK off 估算：

- 500K, 200Hz F5 + 200Hz encoder：~63% bus load，理論可用但在 MCP2515 上偏緊。
- 1M, 200Hz F5 + 200Hz encoder：~31%，舒服。
- 1M, 300Hz F5 + 300Hz encoder：~47%，可用。

結論：**MPC 不被 SERVO42D 擋住；擋住的是目前 MCP2515/500K 的穩定低延遲資料面。**

### 下一步

1. 先用 `coupling_probe` 取得中心姿態、小幅、可重複資料，避免拿舊的激烈擾動資料硬 fit。
2. 用 `coupling_summary` 量化 Z→Y/roll/pitch/yaw 等交叉耦合。
3. 先做 decoupling feedforward 或 pose-space MIMO PD/LQI，再進 LQR/MPC。
4. TWAI + 1M 到位後再挑戰 200-300Hz MPC。

### Safe landing convention

使用者提供可重複安全卸力流程：先回 UI 可設定的 relative home（目前 `[0,0,+28,0,0,0]`），
再到支架 landing relative pose `[0,0,+10,0,0,0]`（絕對 pose 約 `z=NEUTRAL_Z+10`），
最後 `D` release；平台會落在已放好的支架上。

最佳實踐修正：Home 不應在腳本硬寫第二份。UI 的「存Home」同步寫 server `/api/home`；
`sysid/safe_land.js` / `coupling_probe --live` 從同一 API 讀取 home，避免雙真相源。

## 2026-06-27 — FreeRTOS 分層後 L10 真 100Hz，但 MCP2515 仍是 CAN 錯誤瓶頸

使用者重構韌體排程：Arduino `loop()` 不再混控制、命令、telemetry、OTA；新增 FreeRTOS 分層。
控制 task 高優先，只做控制/CAN；telemetry/net 低優先，透過 bounded queue / snapshot 溝通。

### 現場確認

`L10` 下，hold/off 都能穩定 `lhz≈100`，`dt≈9-10ms`，`cus=0`。這表示控制週期已不再被
USB/TCP/JSON telemetry/OTA 慢路徑拖住；telemetry queue 採 newest-wins，慢了只丟觀測資料。

### 非 enable CAN 矩陣

固定 `L10`，不 enable 馬達，只掃 auto-return 週期與 response 設定：

| 條件 | lhzAvg | busRxAvg/30ms | ovrSum/8s | txMax | efMax |
|---|---:|---:|---:|---:|---:|
| C00 L10 AR30 | 100 | 6.0 | 23 | 149 | 85 |
| C00 L10 AR20 | 100 | 9.1 | 62 | 131 | 85 |
| C00 L10 AR10 | 100 | 17.5 | 169 | 113 | 69 |
| C00 L10 AR5 | 100 | 33.6 | 265 | 127 | 69 |
| C10 L10 AR30 | 100 | 6.0 | 17 | 109 | 69 |
| C10 L10 AR20 | 100 | 9.1 | 50 | 91 | 64 |
| C10 L10 AR10 | 103 | 16.7 | 224 | 227 | 85 |
| C10 L10 AR5 | 100 | 29.1 | 267 | 247 | 96 |

另測 `A0` 輪詢 + `L10`：控制仍約 100Hz，但 `cusAvg≈2.9ms`、`cusMax≈8.9ms`，
且輪詢每秒約 600 queries + 600 replies，也造成 MCP2515 RX overflow。

### 判斷

FreeRTOS 分層修掉的是「控制週期被 I/O 拖慢」；這點已成功。  
TEC/EFLG 高不是同一層問題，而是 500K + MCP2515 2 RX buffer 在 100Hz 六軸資料面下仍會被 burst 壓力打到。

AR 越快，`ovrSum` 越高；`A0` 在 L10 也不乾淨，代表問題不是 auto-return 唯一錯，而是
MCP2515 對「高頻六軸回授 + F5/配置命令」整體資料面不夠 robust。

目前較保守暫態設定：`C 1 0` + `L 10` + `AR 20` + `A1`。最新觀察 `pos=0`、`lhz=100`、
`ar=20`、`ovr=0`，但 TEC/EFLG 仍可能殘留，需要靠後續成功傳輸慢慢恢復或重啟 CAN。

## 2026-06-27 — WiFi 每 ~1.5s 掉線根因：第二個 worktree dashboard 搶 ESP32 single-client TCP

### 症狀

Dashboard 顯示 WiFi transport 規律 `connected → down closed → reconnect`，週期約 1.5s。
一開始懷疑 RSSI、家用網路不穩、TCP backpressure 或 telemetry 太大。

### 排查

- ping ESP32 `192.168.50.107`：0% packet loss，RSSI 約 `-51~-56 dBm`，不像 RF 訊號差。
- 將 TCP telemetry 降到約 20Hz 並加診斷：`out_short=0`、`out_bp=0`、`out_drop=0`，
  排除「ESP32 write 寫不出去所以主動斷線」作為主因。
- 暫停 dashboard transport 後，用 raw TCP client 直接連 ESP32，不送任何命令仍在 `~1.58s` 被 close。
- `lsof -nP -iTCP@192.168.50.107:3333` 發現另一個 `node server.js`
  從 `/Users/chenliangyu/project/stewart-platform-dedup-sot`（PID 8468，HTTP 3001）持有同一個 ESP32 TCP 連線。

### 結論

ESP32 `:3333` 是 single-client 資源。韌體採 newest-wins，第二個 dashboard/server 連上時會踢掉舊 client。
兩個 worktree server 同時自動重連，就形成互相踢掉的振盪，看起來像 WiFi 固定掉線。

停掉舊 worktree server 後，raw TCP 連續 45s 穩定讀取 `749` 筆 telemetry，沒有 1.5s close。
因此這次 WiFi 掉線不是家裡網路不穩，而是本機多 server 搶同一個 ESP32 TCP endpoint。

### 預防措施

- `server.js` 新增系統 temp dir 內的 `stewart-platform-esp32-3333.lock` 全域 transport lock；同一台 Mac 第二個 server 啟動會拒絕連 ESP32。
- `npm run upload:ota` 會先呼叫 `/api/release?ms=90000`，避免 dashboard TCP 與 OTA 搶 WiFi。
- 多 worktree 實驗前先檢查：

  ```bash
  lsof -nP -iTCP@192.168.50.107:3333
  ```

- 若刻意要跑第二個 UI，只能做靜態/前端開發，不應同時持有 ESP32 transport。

## 2026-06-27 — WiFi 魯棒性 Phase 2：ping 活著但 TCP application reset

### 新症狀

使用者離開房間後 dashboard 再次斷線。這次不同於 worktree 搶線：

- `ping 192.168.50.107` 穩定，0% packet loss。
- 沒有第二個 worktree 持有 `:3333`。
- server 連 `:3333` 後收到 `ECONNRESET`；raw client 送 `WIFI?` 也立刻 reset，沒有任何 JSON 回覆。
- `/api/latest` 停在舊資料或清空，表示 TCP telemetry 沒恢復。

判斷：ESP32 WiFi 層仍在線，但應用層 TCP server/client 狀態進入半死狀態。

### 修正方向

韌體端：

- TCP telemetry 是觀測路徑，遇到 `availableForWrite() <= 0` 直接丟資料，不嘗試 write、不關 socket。
- 健康 client 受到保護：新的 accidental TCP probe 不再踢掉 dashboard client。
- 半死 client 有 stale watchdog：若長時間沒有成功 TX，主動 stop，讓 server 重新接入。
- `WIFI?` 增加 `reject/preempt/stop/wifi_drop/close` 計數，用於判斷是 probe、stale、WiFi drop 還是 short write。

Server 端：

- TCP connect 有明確 timeout/error reason。
- 連續失敗採 exponential backoff，避免 ESP32 半死時每 1.5s 被猛烈重連敲打。
- 新增 `npm run net:health`，只讀 dashboard API、ping、lsof，不直接連 `:3333`，避免診斷工具本身踢掉 single-client。

### 驗證狀態

- `npm run build` 通過。
- server 端修正已可直接生效。
- 韌體端修正尚需 OTA/USB 燒錄；目前 OTA port `3232` 拒絕，需按 EN 或接 USB 讓 OTA/serial 恢復後再上傳。

## 2026-06-27 — WiFi 魯棒性 Phase 3：修掉 ghost TCP client

### 觸發

平台已安全躺在支架上，姿態約 landing `+8.5mm`、其他軸接近 0。WiFi 再次間歇性斷線。
這次 dashboard server 不是卡死：ESP32 ping 穩定，server 每次連 `192.168.50.107:3333` 後立即收到
`ECONNRESET`。

### 第一性原理結論

之前韌體用「ESP32 最近有成功寫 telemetry」判斷 client 健康，這是錯的。TCP 半開時，
`client.write()` 可能只是寫進本地/lwIP buffer，不能證明 Mac 端真的還活著。結果 ESP32 會保護一個
ghost client，並拒絕真正 dashboard server 的新連線，看起來像 WiFi application layer 一直 reset。

另外，`nc -vz 192.168.50.107 3333` 這類 port probe 會真的佔用 ESP32 single-client TCP socket。
健康檢查如果預設碰 `:3333`，會自己製造干擾。

### 修正

- `src/net_transport.cpp`：client stale 判斷改用 `lastNetRxMs`，也就是「host 最近有送心跳/指令進來」
  才算健康。Node transport 本來就每秒送 newline heartbeat，因此真正 dashboard 會持續保活；
  `nc`、斷線 socket、ghost client 不會被長期保護。
- `server.js`：新增 recovery watchdog，若 transport 停在 down/connecting 且沒有 pending reconnect，
  自動重新選路；新增 `/api/transport/reconnect`，且可取消 OTA release 等待。
- `sysid/net_health.js`：預設不再觸碰 `:3333`，只用 dashboard API、ping、lsof 判斷；只有明確
  `--probe` 才做 TCP port 探測。

### 驗證

- `pio run` 通過。
- USB serial upload 失敗是因為 macOS 只看到 Bluetooth 虛擬序列埠，未看到 ESP32 USB port。
- `npm run upload:ota -- --host 192.168.50.107` 成功，OTA result `OK`。
- OTA 後重啟 server，立即進入 `connected wifi (192.168.50.107:3333)`。
- `npm run net:health` verdict `ok`，且顯示只有一個 node owner 持有 ESP32 `:3333`。
- 30 秒 API 監控：`31` 次讀取、`stale=0`、`ok=6`、`lhz≈100`、`pos=0`。

### 新操作規則

不要用一般 TCP port probe 當健康檢查預設步驟。ESP32 `:3333` 是 single-client 控制資源；
診斷工具預設只能被動查 dashboard state、ping 與本機 owner。只有在 dashboard 已停且明確要測 port
時才加 `--probe`。

## 2026-06-27 — Clean heave sysid：Ki=20 vs pure-P 的真實取捨

### 工具修正

`sysid/research_trial.js` 新增 `--record-main-only`。錄製只包含：

1. `H`
2. 回 configured home
3. 主 heave target + settle

然後才停止 recorder，再執行 `home → landing → D`。這避免 safe landing / release 汙染 MIMO/sysid 指標。

新增 `sysid/heave_step_metrics.js` / `npm run heave:metrics`：用最後一個 `P` target 前的短窗當 baseline，
只分析 main target window，輸出 Z 響應、X/Y/R/P/Yaw 交叉峰值、CAN health。

### 實驗

同一個 clean heave +10mm / 5s / settle 1.8s：

| 條件 | zFinal | zPeak | crossPeak | cross/Z | 主要交叉 | hmaxMax | herrP95 | CAN efOr / rxDrop |
|---|---:|---:|---:|---:|---|---:|---:|---:|
| `K 1024 20 0 0` | +10.46mm | 10.47mm | 2.12mm | 20.2% | X/Y | 4.42° | 2.03° | 85 / 153 |
| `K 1024 0 0 0` | +9.75mm | 9.76mm | 0.84mm | 8.6% | Y | 6.94° | 4.34° | 85 / 146 |

資料：

- `sysid/data/clean_heave10_ms5000_heave-step_h38_ms5000_2026-06-27T05-12-32.jsonl`
- `sysid/data/clean_heave10_ms5000_pureP_heave-step_h38_ms5000_2026-06-27T05-13-43.jsonl`

### 判斷

Ki=20 的價值是補靜態負載：高度更接近目標、hold error 較小。  
Ki=20 的代價是 MIMO 耦合顯著變大：同樣 +10mm heave，X/Y 交叉峰值從約 `0.84mm` 升到 `2.12mm`。

pure-P 比較乾淨，但高度少到約 `9.75mm`，且 joint hold error 較大。這表示問題不該靠馬達內部 per-joint
積分硬補，因為每顆馬達只看自己的角度誤差，不知道平台 pose 交叉耦合；積分會把重力/摩擦/模型誤差轉成
六顆馬達彼此競爭的 hidden state。

下一步控制主線：保留 pure-P 或低 Ki 作為較乾淨的內環，將高度/重力補償提升到 pose-space：

1. 先做 heave feedforward / command bias（例如 pure-P 下 Z 目標加一個小補償，而不是開 joint Ki）。
2. 再做 pose-space PI/LQI，integral state 只積分平台 `z/x/y/姿態` 誤差，避免六顆 joint Ki 各自亂積。
3. 取得足夠 clean sysid 後再進 LQR/MPC；目前資料已能支持建模，但 CAN `efOr=85` / `rxDrop≈150` 仍提醒
   MCP2515/500K 是高頻控制與觀測瓶頸。

## 2026-06-27 — Heave feedforward：pure-P + Z bias 贏過 joint Ki 的方向

### 實驗

在 pure-P `[1024,0,0,0]` 下，不使用 joint Ki，只把錄製段的 home/target 命令一起加 Z bias；
safe landing 仍回原本 configured home/landing，不改安全流程。

同一個 +10mm / 5s heave：

| 條件 | cmd home/target | actual home | actual final | final err vs 143 | actual step | crossPeak | cross/Z |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ki=20 | 133 / 143 | 132.62 | 143.08 | +0.08 | 10.46 | 2.12 | 20.2% |
| pure-P | 133 / 143 | 129.10 | 138.85 | -4.15 | 9.75 | 0.84 | 8.6% |
| pure-P + zBias 4.0 | 137 / 147 | 132.84 | 142.49 | -0.51 | 9.65 | 0.84 | 8.7% |
| pure-P + zBias 4.5 | 137.5 / 147.5 | 133.25 | 142.93 | -0.07 | 9.68 | 0.78 | 8.0% |

資料：

- `sysid/data/clean_heave10_ms5000_pureP_zbias4_heave-step_h38_ms5000_2026-06-27T05-18-57.jsonl`
- `sysid/data/clean_heave10_ms5000_pureP_zbias4p5_heave-step_h38_ms5000_2026-06-27T05-19-39.jsonl`

### 結論

`pure-P + zBias≈4.5mm` 幾乎達到 Ki=20 的絕對高度準度，但交叉耦合仍維持 pure-P 的乾淨程度。
這強烈支持目前控制架構下一步：

- 內環採 pure-P/低 Ki，避免 joint-level integrator 互相打架。
- 用上層 pose-space feedforward / LQI 來補重力 sag 與慢誤差。
- 後續 LQR/MPC 的模型應基於「乾淨內環 + 上層補償」而不是「joint Ki=20」。

`sysid/research_trial.js` 新增 `--z-bias`，且 `sysid/platform_sot.js` 的 `trialDefaults.zBias`
納入 SoT，避免 CLI/UI 後續各存一份補償值。

### 重要修正：固定 zBias 只是 anchor，不是最終控制器

固定 `zBias=4.5mm` 看起來有效，但它不夠專業，也不會泛化。它只證明一件事：
**靜態高度誤差應該在上層補，不應藏在六顆馬達的 joint Ki 裡。**

專業路線應改成：

1. 量測多個 operating points：不同 heave、高度、tilt、payload/支撐狀態。
2. 擬合靜態補償模型 `bias = f(pose, load)`，先可用低階多項式/lookup table。
3. 把模型當 feedforward；殘餘慢誤差才交給 pose-space LQI。
4. LQR/MPC 使用「pure-P/低 Ki + compensation model」的乾淨資料建模，而不是使用 joint Ki=20 的耦合資料。
