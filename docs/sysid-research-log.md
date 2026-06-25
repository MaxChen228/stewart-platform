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

## 決策與待辦

- **item 1（auto-return）**：✅ 已實作、靜態驗證。預設關閉，A1 啟用。
- **item 3（1M baud）**：⏸ 不在現有 8MHz 晶振盲做（出規格、改馬達鮑率不可逆）。
  替代：換 16/20MHz MCP2515 模組，或併入 item 2 TWAI。
- **item 2（換 ESP32 原生 TWAI）**：深 FIFO 可同時拿低延遲+0溢位，未做。
- **Gate A（核心未答）**：迴圈加速到底有沒有讓平台變穩？需馬達動力電 + Enable + chirp 實測。

## 硬體接線速查
ESP32→(SPI: GPIO18 SCK/23 MOSI/19 MISO/5 CS/17 INT)→MCP2515→(內建)→TJA1050→(CANH/CANL,120Ω×2,500k)→6×SERVO42D(0x01-0x06)。
邏輯電(3.3V) 與 馬達動力電(12-24V) 分離、共地。
