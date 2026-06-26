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

### 控制分層

**馬達內部位置控制**：ESP32 做軌跡協調，馬達用 0xF5 位置模式 + 內部三環 PID。
ESP32 端不跑 PID，只算 IK 目標角度 → 轉成馬達坐標 → 發 0xF5。
0xF5 走脈衝量，避開 0xF6 速度模式 1 RPM 的整數量化邊界（量化問題本身仍存在於底層速度環，僅是不再由 ESP32 直接設定）。

**控制模式**：main.cpp 同時保留兩種模式，預設 `controlMode = 1`：
- `0` — Joint-space：IK → 直接餵馬達角度 + 自適應追蹤增益
- `1` — Task-space PD：FK 回算當前 pose → 與 target pose 算誤差 → PD → IK

Task-space PD 連續 FK 失敗 5 次會自動降回 joint-space。

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

實體馬達從上方俯視為**順時針** M1→M6。base/platform joint 角度已配合 CW：

```
Pair 1 (M1,M2) @ 210°
Pair 2 (M3,M4) @ 90°
Pair 3 (M5,M6) @ -30°
```

奇數馬達在 `+ANGLE/2` 側、偶數在 `-ANGLE/2` 側（`BASE_ANGLE=18.92` / `PLATFORM_ANGLE=28.07`）。
`MOTOR_PLANE_ANGLE=[300,120,180,0,60,240]`、`MOTOR_SIGN=[1,-1,1,-1,1,-1]`。

**真相源三處須同步**（改幾何時一起改）：韌體 `kinematics.h`、前端 `web/index.html`（本地 `ik()`+常數）、共用模組 `sysid/kin.js`。實際數值以這三處為準，別在本文件重列（易腐）。

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

沒有 IK 的 asin clamp 奇異；Jacobian 在工作空間邊緣仍會病態。實測在 ±15° 操作範圍內收斂良好，邊緣外未驗證。

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

### 現狀（純P 工作點穩定）

**已找到穩定工作點：vFOC 純P `[Kp,Ki,Kd,Kv]=[1024,0,0,0]` + HOLD 死咬。** `Ki=0` 滅掉 M4 2Hz 自持極限環的根因（積分撞死區）、`maxKp` 補剛性、F5 posSpeed 提供阻尼；battery 壓測下手推擾動穩定回正。完整決策記錄見記憶 `project_pid_working_point` / `project_m4_limit_cycle`。

**主要運行模式 = HOLD（純P 死咬）**：`H` 鎖住當前姿態 snapshot（holdAngles），馬達對該凍結目標做純P 保持；姿態操作（`G` 指令）平滑移動鎖定目標。下方 mode 0/1 仍存在於碼中、作架構備援，非當前主運行路徑。

注意：BOOT 預設已改 `[1024,0,0,0]`（`main.cpp`），但若跑舊 binary（未 `npm run upload`）開機仍是出廠 `220/30/270/320`，需 K 指令或重燒。噪音（Kd 致嘯叫 / 馬達保持電流固有聲）未結案、使用者選擇忽略。

### Joint-space 模式（mode 0，備援）

```
targetPose → smoothPose → IK → idealAngles[6]
                                    ↓
                          自適應追蹤 (gain based on Σvel²)
                                    ↓
                          adjustedAngles[6] → angleToCoord → F5
```

自適應追蹤公式：
```
kinetic = Σ (angle[i] - prevAngle[i])²   // 度/cycle，不除以 dt
smoothKinetic = 0.9·smoothKinetic + 0.1·kinetic
gain = min(maxGain, 1 / (1 + μ × smoothKinetic))
adjustedAngle = current + gain × (ideal - current) - Kd × vel × dampRatio
```

`dampRatio = 1.0`（vel 與 error 反向，幫助減速）或 `0.3`（同向，避免阻尼造成的相位滯後）。

### Task-space PD 模式（mode 1，預設）

`enc.angles → FK → currentPose → poseError = target - current → PD → ΔPose → IK → motorTargets`

FK 連續失敗 5 次自動 fallback 到 mode 0。

### 待驗證 / 未結案項

- 噪音源：純P 下仍有殘響，疑馬達 vFOC 保持電流 PWM 固有聲（已排除 F5 指令流與各 PID 項；使用者選擇忽略）
- F5 方向：`angleToCoord` 的 coord 正負對應每顆馬達物理方向（單軸 T 過、六軸耦合下未交叉驗）
- heave→yaw 運動學疑點：heave+ 實體微順時針旋轉（使用者判 `MOTOR_SIGN` 標定無誤、暫不糾結）
- 終極目標：無支架推擾回正（pose 調節），見記憶 `project_control_plan`

## 建置與燒錄

```bash
npm start              # 啟動 WebSocket server (localhost:3000)
npm run upload         # 自動釋放序列埠 → 編譯燒錄 → server 自動重連
pio run                # 僅編譯
curl localhost:3000/api/latest  # 讀取即時資料
```
