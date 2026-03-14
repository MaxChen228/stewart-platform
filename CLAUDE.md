# Stewart Platform — ESP32 + MCP2515 + SERVO42D

## 硬體配置

| 組件 | 規格 |
|------|------|
| 主控 | ESP32 WROOM (3.3V) |
| CAN 橋接 | MCP2515 (SPI) + TJA1050 |
| 致動器 ×6 | MKS SERVO42D, SR_vFOC 模式 |
| CAN 鮑率 | 500 Kbps（後續升 1Mbps） |
| 馬達位址 | 0x01 ~ 0x06 |

### SPI 腳位

| ESP32 | MCP2515 | 功能 |
|-------|---------|------|
| GPIO 18 | SCK | SPI 時脈 |
| GPIO 23 | SI | MOSI |
| GPIO 19 | SO | MISO |
| GPIO 5 | CS | 片選 |
| GPIO 17 | INT | 中斷 |

## 架構決策

- **集中式控制**：ESP32 統一做位置閉環，馬達只跑速度模式 (0xF6)
- **不使用** 0xFE/0xFD（馬達內建位置控制），避免六軸獨立 PID 互相激盪
- 逆向運動學參照 `/Users/chenliangyu/Desktop/Development/Physics-Sim/stewart_platform`

## 關鍵 CAN 指令

| 指令 | 用途 |
|------|------|
| 0x30 | 讀編碼器位置 |
| 0xF6 | 速度模式（方向 + 速度 + 加速度） |
| 0xF3 | 使能/禁用 |
| 0xF7 | 緊急停止 |

## 逆向運動學

輸入 `[x, y, z, roll, pitch, yaw]`，輸出 6 個馬達目標角度。
公式中的幾何參數需量測實際機構後填入。

```
Q = R·P(i) + T
L = |Q - B|² - (upper² - lower²)
M = 2·lower·(Q_z - B_z)
N = 2·lower·(cos(θ)·ΔX + sin(θ)·ΔY)
angle(i) = asin(L/√(M²+N²)) - atan(N/M)
```

## 建置與燒錄

```bash
pio run                 # 編譯
pio run -t upload       # 燒錄
pio device monitor      # 序列監控
```
