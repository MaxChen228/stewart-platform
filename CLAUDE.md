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

**集中式控制**：ESP32 統一做位置閉環，馬達只跑速度模式 (0xF6)。
不使用馬達內建位置控制 (0xFE/0xFD)，因為 Stewart platform 六軸高度耦合——六個獨立 PID 會互相打架。

### 系統拓撲

```
Web UI (target pose sliders)
    │ WebSocket
    ▼
Node.js server (server.js:3000)  ←→  REST /api/latest (供 agent 讀取)
    │ Serial
    ▼
ESP32 (20ms PID loop)
    │ CAN bus (MCP2515, 500kbps)
    ▼
SERVO42D ×6 (0xF6 speed mode)
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

| 指令 | DLC | 格式 | 用途 |
|------|-----|------|------|
| 0x30 | 2 | `[0x30, CRC]` | 讀編碼器（回覆 8 bytes: carry×4 + val_hi + val_lo + CRC） |
| 0xF6 | 5 | `[0xF6, dir\|speed_hi, speed_lo, acc, CRC]` | 速度模式（speed 0-3000 RPM, dir b7, acc 0-255, 0=instant） |
| 0xF3 | 3 | `[0xF3, enable, CRC]` | 使能(01)/禁用(00) |
| 0xF7 | 2 | `[0xF7, CRC]` | 緊急停止 |

CRC = `(CAN_ID + 所有 data bytes) & 0xFF`

手冊路徑：`/Users/chenliangyu/Downloads/MKS SERVO42&57D_CAN User Manual V1.0.9.pdf`

## 編碼器與校正

- 14-bit 絕對磁編碼器 (0-16383 = 一圈)
- 零點校正值 `zeroRaw[6]` 存 ESP32 NVS，重開機不丟失
- 角度公式：`angle[i] = MOTOR_SIGN[i] * (raw - zeroRaw) * 360/16384 + 90`
- 校正假設：Zero All 時所有下腿朝上 = 90°
- ±180° wrapping 限制：馬達不可轉超過半圈

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
| `Z` | Zero All（存 NVS） |
| `Z0`~`Z5` | 單顆歸零 |
| `D` | Disable 所有馬達 |
| `E` | 啟動 PID（先 0xF3 enable） |
| `S` | 停止 PID（0xF6 speed=0 → 0xF3 disable） |
| `P x y z roll pitch yaw` | 設定目標姿態 |
| `K kp ki kd` | 設定 PID 增益 |
| `T0`~`T5` | 單馬達方向測試（以 3 RPM 轉 0.5 秒，回報角度變化） |

## 六軸協作 PID — 現狀與下一步

### 已完成

- PID 框架：20ms loop, 讀 encoder → IK(target) → error → PID → 0xF6
- 方向測試工具 (`T` 指令)
- 安全限制：maxError=20° 急停, maxRPM=10
- 死區 1.5°（防低速抖動）
- Web UI：target pose 滑桿 + PID gains + enable/stop

### 已驗證

- 馬達確實會動，PID 指令鏈完整
- 六顆馬達方向測試結果一致：奇數馬達 CCW=angle-, 偶數 CCW=angle+
- MOTOR_SIGN 映射正確

### 待解決：為什麼 PID 還在振盪

1. **幾何排列剛修正為 CW**——修正後尚未實際測試 PID。之前 PID 振盪/跑飛的根因就是 CCW/CW 不匹配，IK 目標角度發給錯的馬達。

2. **參數初始值**：Kp=3, Ki=0, Kd=1, maxRPM=10。修正排列後預期可以正常收斂，但仍需調參：
   - 先 P-only (Ki=0, Kd=0) 確認方向全對
   - 加 Kd 壓過衝
   - 最後加微量 Ki 消除穩態誤差

3. **0xF6 速度解析度限制**：speed 參數為整數 RPM (0-3000)。1 RPM = 6°/s，低速控制的最小單位就是 6°/s。使用 `roundf()` + 死區 1.5° 處理。

### 六軸聯動 PID 的宏大理念

Stewart platform 的本質困難：**六軸耦合**。移動一個馬達會同時改變所有六條腿的受力。

目前的架構是「IK 空間解耦」：
1. 在 task space 定義目標 `[x, y, z, roll, pitch, yaw]`
2. IK 轉換為 6 個獨立的馬達目標角度
3. 每個馬達獨立 PID 追自己的目標角度

這個架構在以下條件下可以工作：
- **低速**：耦合效應被忽略（靜態假設）
- **保守增益**：不追求快速響應
- **目標變化平滑**：避免六個馬達同時大幅移動

如果未來需要更高性能（快速、大幅度運動），需要進化到**task-space 控制**：
- 在 task space 直接做 PID（誤差在 x,y,z,r,p,y 空間）
- 透過 Jacobian 將 task-space 力/速度映射到 joint-space
- 加入 feedforward（重力補償、慣性前饋）
- 考慮耦合動力學

但現階段，IK 解耦 + 保守增益 + 低速運動就足夠了。先讓它穩定地動起來。

## 建置與燒錄

```bash
npm start              # 啟動 WebSocket server (localhost:3000)
npm run upload         # 自動釋放序列埠 → 編譯燒錄 → server 自動重連
pio run                # 僅編譯
curl localhost:3000/api/latest  # 讀取即時資料
```
