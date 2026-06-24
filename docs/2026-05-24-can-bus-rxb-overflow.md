# CAN 匯流排 RXB Overflow 根因分析

**日期**：2026-05-24
**症狀**：normal cycle 每顆馬達 `readRawEncoder` 成功率 ~68-78%，ok=2-4/6 而非 6/6。

## 一、排除過的假設

| 假設 | 結論 | 證據 |
|------|------|------|
| ESP32 SPI 不穩 | 排除 | raw SPI 從 10MHz 降到 1MHz 後，register 讀寫 100% 一致 |
| MCP2515 mode 切換異常（ABAT 被誤設） | 已修 | `rawSetMode` 改用絕對寫入 CANCTRL，不再 R-M-W |
| MCP2515 硬體 filter 壞掉 | 部分釐清 | init 時設 filter=3：只有 M3 回應，其他全擋 → init-time filter 工作正常。filter register 在 normal mode 讀回 0 是 chip quirk，不影響實際行為 |
| CAN bus 電氣層異常 | 排除 | EFLG / TEC / REC 全程為 0 |
| 馬達自主廣播（不請自來） | **排除** | silent_boot test：5 秒完全不 TX，馬達 0 frame |
| 馬達收到指令後一直自發 spam | **修正** | 是 0x8C 預設行為（見下） |

## 二、真正根因：RXB 容量 vs frame 流量

### MCP2515 接收能力

- 只有 2 個 RX buffer（RXB0、RXB1）
- BUKT rollover 後仍是 2 個
- ESP32 主 loop 20ms cycle，cycle 間隔內若進來 > 2 個 frame，後到者全部 overrun 被丟

### SERVO42D 0x8C 預設回應策略

`docs/servo42d/04-config-commands.md:91-105`：

| XX | YY | 每個命令觸發的回應 frame 數 |
|----|----|---|
| 1 | 1（**出廠預設**） | immediate (01 start) + completion (02 done / 03 endstop) = **2** |
| 1 | 0 | immediate only = **1** |
| 0 | x | **0**（要靠 0xF1 主動查詢） |

### 我們 cycle 內的 frame 流量

每 20ms cycle 對 6 顆馬達送 `0x35`（讀編碼器）+ `0xF5`（位置命令）：

- 0x35 reply：每顆 1 frame
- 0xF5 reply（預設 XX=1/YY=1）：每顆 2 frame
- **總計：6 × (1 + 2) = 18 frame / cycle**

匯流排上 18 frame 瞬間湧入 2 個 RXB → 必然 overrun。表面看像「某顆馬達搶到 reply」，實際是 RXB 溢位的 victim 是隨機的。

### 之前誤判的「80-100Hz spam」

不是真的 spam。是 0xF5 即時更新時：

1. 馬達還在執行上個 0xF5（completion 02 尚未發）
2. 我們又送下一個 0xF5（新的 start 01 reply）
3. 舊的 completion + 新的 start 疊在同一個 cycle 窗口

→ 是「documented 行為被我們的用法放大」，不是馬達 firmware bug。

## 三、解法（依激進度排序）

### 方案 A：0x8C XX=1, YY=0（保留 start/fail，關 completion）

- 每 cycle frame：6 × (1+1) = **12 frame**
- 仍可能 overrun（>2 RXB），但不會跨 cycle 疊加
- 保留 immediate 0x00 可診斷拒命令（如 disable 失敗 status=0）

### 方案 B：0x8C XX=0（完全不回）

- 每 cycle frame：6 × 1 = **6 frame**（只剩 0x35 reply）
- 仍 > 2 RXB，但 ESP32 在每個 0x35 query 之間都會 receive→消化，實務上不會卡
- 不需要 motor completion，因為我們有自己的 FK + encoder loop

**推薦 B。** 0x8C 不像 0x3F 會 reset CAN ID / PID，副作用低。

### 方案 C（補強）：每顆 query 前切換 hardware filter

正交於 A/B：硬體層把不相干馬達的 reply 直接擋在 MCP2515 外。dyn filter 是否可在 normal mode 切換尚未驗證完，待測。

## 四、待辦

- [ ] 上傳測試韌體驗證 `rawSetHardwareFilter` 在 normal mode 動態切換是否有效
- [ ] 對 6 顆馬達寫入 0x8C `XX=0`（一次性，存 flash）
- [ ] 重測 normal cycle ok 率，目標 6/6 穩定

## 五、學到的事

1. **silent test 是黃金診斷**：完全不 TX 看 RX，一次釐清「主動 vs 被動」哪邊出問題
2. **MCP2515 RXB = 2 是硬天花板**，任何 > 2 frame/cycle 的 protocol 設計都要在「源頭減量」或「filter 擋掉」
3. **0x8C 是高槓桿開關**：一行 config 改變整個 bus 流量數量級，但出廠預設不適合 6-node high-rate 應用
4. raw SPI 速度不是越快越好，autowp 預設 10MHz 對長線/雜訊敏感，1MHz 換到 100% 穩定
