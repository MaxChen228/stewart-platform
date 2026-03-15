# Stewart Platform — ESP32 + MCP2515 + SERVO42D

## 硬體配置

| 組件 | 規格 |
|------|------|
| 主控 | ESP32 WROOM (3.3V) |
| CAN 橋接 | MCP2515 (SPI, 8MHz 晶振) + TJA1050 |
| 致動器 ×6 | MKS SERVO42D, SR_vFOC 模式, 14-bit 磁編碼器 |
| CAN 鮑率 | 500 Kbps |
| 馬達位址 | 0x01 ~ 0x06 |

### SPI 腳位

| ESP32 | MCP2515 | 功能 |
|-------|---------|------|
| GPIO 18 | SCK | SPI 時脈 |
| GPIO 23 | SI | MOSI |
| GPIO 19 | SO | MISO |
| GPIO 5 | CS | 片選 |
| GPIO 17 | INT | 中斷 |

## 架構

### 控制哲學

**馬達內部位置控制**：ESP32 做軌跡協調，馬達用 0xF5 位置模式 + 內部 10kHz 三環 PID。
ESP32 不做 PID — 只算 IK 目標角度 → 轉換為馬達坐標 → 發 0xF5。
馬達的位置環(Kp/Ki/Kd) + 速度環(Kv) 提供力矩級控制，解決了舊 0xF6 速度模式整數 RPM 量化問題。

**自適應追蹤**：ESP32 監測六軸總動能 `Σ velocity²`，震動時自動降低追蹤增益（放棄精度換穩定）。

### 系統拓撲

```
Web UI (target pose sliders + μ/speed/acc/motor PID)
    │ WebSocket
    ▼
Node.js server (server.js:3000)  ←→  REST /api/latest
    │ Serial
    ▼
ESP32 (20ms trajectory coordinator)
    │ CAN bus (MCP2515, 500kbps)
    ▼
SERVO42D ×6 (0xF5 position mode, internal 10kHz PID)
```

### 開發迴圈

- **改網頁**：存檔 → 瀏覽器 F5 → 即時生效（server 持續連線）
- **改韌體**：`npm run upload`（自動釋放序列埠 → 編譯燒錄 → 30 秒後 server 自動重連）
- **Agent 讀資料**：`curl localhost:3000/api/latest`

## 幾何定義 — CW 排列

實體馬達從上方俯視為**順時針** M1→M6。程式碼的 base/platform joint 角度已配合 CW：

```
Pair 1 (M1,M2) @ 0°
Pair 2 (M3,M4) @ -120° (= 240°)
Pair 3 (M5,M6) @ +120°
```

三個陣列（韌體 `kinematics.h` + 前端 `index.html` 同步）：
```
BASE_ANGLES:          [-9.46, 9.46, -129.46, -110.54, 110.54, 129.46]
PLATFORM_ANGLES:      [-14.035, 14.035, -134.035, -105.965, 105.965, 134.035]
MOTOR_PLANE_ANGLE:    [-90, 90, -210, -30, 30, 210]
MOTOR_SIGN:           [1, -1, 1, -1, 1, -1]
```

## CAN 指令

完整 CAN 指令手冊：`docs/servo42d/00-index.md`（按指令碼查表 + 歧義消除 + 常見任務流程）

本專案常用指令摘要：

| 指令 | 用途 |
|------|------|
| 0x35 | 讀 RAW 累積編碼器（不受 0x92 影響） |
| 0x92 | 設當前位置為坐標零點 |
| 0x96 | 設 vFOC PID（Kp/Ki/Kd/Kv, 0-1024） |
| 0xF3 | 使能(01)/禁用(00) |
| 0xF5 | 絕對坐標位置模式（支援即時更新） |
| 0xF7 | 緊急停止 |

CRC = `(CAN_ID + 所有 data bytes) & 0xFF`

### MCP2515 注意事項

- 只有 2 個接收 buffer，F5 回覆會塞爆 → 每 cycle 開頭必須 flushReceiveBuffer()
- encoder 讀取失敗時保持上一次角度值（不回退到 neutralAngle）

## 編碼器與校正

- 14-bit 絕對磁編碼器 (0-16383 = 一圈)
- 零點校正值 `zeroRaw[6]` 存 ESP32 NVS，重開機不丟失
- 角度公式：`angle[i] = MOTOR_SIGN[i] * (raw - sessionZeroRaw) * 360/16384 + 90`
- 校正假設：Zero All 時所有下腿朝上 = 90°
- ±180° wrapping 限制：馬達不可轉超過半圈

### zeroRaw vs sessionZeroRaw

- `zeroRaw`：存 NVS 的校正值，只有 Z/Z0~Z5 指令會改
- `sessionZeroRaw`：運行時使用的值，開機時 = zeroRaw
- Enable 時 0x92 可能改變 0x30 讀數 → sessionZeroRaw 自動補償偏移
- **只有 Z/Z0~Z5 會改變校正映射**，其他操作不影響

## 逆向運動學 (IK)

輸入 `[x, y, z, roll, pitch, yaw]`，輸出 6 個馬達目標角度。

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
Q(i) = R · P(i) + T
L = |Q-B|² - (upper² - lower²)
M = 2·lower·(Q_z - B_z)
N = 2·lower·(cos(θ)·ΔX + sin(θ)·ΔY)
angle(i) = asin(L/√(M²+N²)) - atan2(N, M)
```

Matlab 參考：`/Users/chenliangyu/Desktop/Development/Physics-Sim/stewart_platform`

## 正向運動學 (FK)

**不使用 IK 反解**（asin 在 90° 有 clamp 奇異）。改用距離約束 Newton-Raphson + 解析 Jacobian：

```
f_i = |R·P_i + T - A_i|² - UPPER² = 0   (6 方程, 6 未知)
Jacobian: ∂f/∂T = 2v,  ∂f/∂angle = 2(v · dR·P) · DEG
```

無 asin、無奇異、任何構型都收斂。

## 序列協議

| 指令 | 說明 |
|------|------|
| `Z` | Zero All（0x92 設零 + 存 NVS） |
| `Z0`~`Z5` | 單顆歸零 |
| `D` | Disable 所有馬達（斷電，可自由轉動） |
| `E` | 啟動位置控制（0x92 + enable + 開始發 F5） |
| `S` | 停止追蹤（馬達保持使能，有保持力矩） |
| `P x y z roll pitch yaw` | 設定目標姿態 |
| `K kp ki kd kv` | 設定馬達內部 vFOC PID（0x96，範圍 0-1024） |
| `V speed acc` | 設定位置模式速度(1-200 RPM)和加速度(1-255) |
| `M mu` | 設定自適應追蹤震動懲罰係數 |
| `T0`~`T5` | 單馬達方向測試（以 3 RPM 轉 0.5 秒，回報角度變化） |

## 位置控制 — 現狀

### 架構

```
targetPose → smoothPose → IK → idealAngles[6]
                                    ↓
                          自適應追蹤 (gain based on Σvel²)
                                    ↓
                          adjustedAngles[6] → angleToCoord → F5
                                    ↓
                          馬達內部 10kHz PID → 力矩 → 運動
```

### 自適應追蹤

```
kinetic = Σ (angle[i] - prevAngle[i])²   // 度/cycle，不除以 dt
gain = 1 / (1 + μ × kinetic)             // 0~1
adjustedAngle = current + gain × (ideal - current)
```

- μ=0：純追蹤，不做自適應
- μ=0.5（預設）：輕度自適應
- μ>1：強力抑震，犧牲追蹤精度

### 已知問題與待調整

1. **F5 方向是否正確**：尚未 100% 驗證 angleToCoord 的 coord 正負是否對應正確的馬達物理方向。若推動後暴走，可能是 coord 符號需翻轉。
2. **posSpeed/posAcc 調參**：預設 speed=30, acc=5，可能太快導致過沖。建議先用 speed=5, acc=2 測試。
3. **馬達內部 PID 調參**：降 Kp(→100) + 升 Kd(→400) = 柔順阻尼感。

### 進化路徑

- **Phase 1**（當前）：Joint-space 位置控制 + 自適應追蹤
- **Phase 2**：Task-space PID（需要 FK 移植到 ESP32）
- **Phase 3**：Computed torque 前饋（需要動力學模型）

## 建置與燒錄

```bash
npm start              # 啟動 WebSocket server (localhost:3000)
npm run upload         # 自動釋放序列埠 → 編譯燒錄 → server 自動重連
pio run                # 僅編譯
curl localhost:3000/api/latest  # 讀取即時資料
```
