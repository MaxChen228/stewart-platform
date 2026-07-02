# Stewart Platform — ESP32 + TWAI/VP230 + SERVO42D

## 硬體配置

| 組件 | 規格 |
|------|------|
| 主控 | ESP32 WROOM (3.3V) |
| CAN 收發 | **SN65HVD230 (VP230) + ESP32 原生 TWAI**（2026-07-02 起現役；前代 MCP2515+TJA1050 退役備援） |
| 致動器 ×6 | MKS SERVO42D, SR_vFOC 模式, 14-bit 磁編碼器 |
| CAN 鮑率 | 500 Kbps |
| 馬達位址 | 0x01 ~ 0x06 |

### CAN 接線（TWAI + VP230）

| ESP32 | VP230 | 功能 |
|-------|-------|------|
| GPIO 21 | CTX/D | TWAI TX |
| GPIO 22 | CRX/R | TWAI RX |
| 3V3 | VCC | ⚠ 3.3V 元件，接 5V 會燒 |
| GND | GND | 與馬達側 bus 共地（非隔離） |

**後端雙軌**（`src/servo42d.h` 之 `USE_TWAI_CAN`，主 env 已開）：現役=TWAI（`src/can_twai_compat.h`，RX queue 64、無 SPI；telemetry `can.backend` 如實報 `"twai"`）；退役備援=MCP2515(SPI)（`pio run -e esp32_mcp2515 -t upload` 即回，腳位 SCK18/MOSI23/MISO19/CS5/INT17）。帶機探測：`pio run -e twai_probe -t upload`（唯讀 0x30 掃 0x01–0x06，不驅動）。

## 架構

### 控制分層

**馬達內部位置控制**：ESP32 做軌跡協調，馬達用 0xF5 位置模式 + 內部三環 PID。
ESP32 端不跑 PID，只算 IK 目標角度 → 轉成馬達坐標 → 發 0xF5。
0xF5 走脈衝量，避開 0xF6 速度模式 1 RPM 的整數量化邊界（量化問題本身仍存在於底層速度環，僅是不再由 ESP32 直接設定）。

**控制模式**：main.cpp 同時保留兩種模式，BOOT 預設 `controlMode = 0`（主運行走 HOLD，0/1 為架構備援）：
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
    │ CAN bus (TWAI+VP230, 500kbps)
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

**mm 域幾何真相源二處須同步**（改幾何時一起改）：韌體 `src/kinematics.h`、共用模組 `sysid/kin.js`（前端 `web/index.html` 已委派 `window.Kin`，不再是獨立真相源）。`MOTOR_SIGN` 另在韌體 `src/servo42d.h` ↔ JS `sysid/disturb_modes.js`。`npm test`（`sysid/check_kin_sync.js`）自動比對這兩對常數 parity。實際數值以碼為準，別在本文件重列（易腐）。

## CAN 指令

完整 CAN 指令手冊（Obsidian 圖譜雙產品線整合）：
- **總覽中樞**：`docs/servo-can-hub.md`（42D↔42ES 雙生章節對照 + 硬體/協議/指令差異矩陣 + 移植韌體影響）
- **本機在用 SERVO42D/57D**：`docs/servo42d/00-index.md`（按指令碼查表 + 歧義消除 + 任務流程）
- **Mega 升級候選 SERVO42ES/57ES**：`docs/servo42es/00-index.md`（源 V1.0.1 PDF `docs/servo42es/refs/`）。⚠️ 與 42D 位元組相容但**重映射多個碼**：心跳 `0x98→0x89`、位置到位門檻 `0x95→0x98`、脈衝分頻 `0x99→0x9F`、歸零參數改 `0x95/0x96/0x97`；**無 CAN PID（無 0x96/0x97 PID）→ `K/KS/KRESET` 失效，須 CANGAROO**；config 寫入須補 `0x60` SAVE 才落 NVS。

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

### CAN 後端注意事項

- ⚠️ **同 ID 相剋鐵律（2026-07-02 實症定案）**：42D 的指令幀、回覆 ack 幀、0x35 主動上報幀**全部同 ID（=馬達地址）**。CAN 仲裁對同 ID 同時發送=保證 bit error；TWAI 無限重傳下 TEC 破表→bus-off，曾把六顆馬達全轟進 bus-off（只能斷電馬達救）。**BOOT 定案 = no-ack（0x8C XX=0，讀取類 0x30/0x35 不受影響仍回）+ 輪詢（不 arm 上報）**→ TX 恆零錯。`C 1 0`/`A<hz>` 可 runtime 切回實驗，但高頻上報/ack + 驅動流必炸。實測：ack 開 12 幀 TX = TEC+134；no-ack+輪詢 = 恆 0
- 防護（`can_twai_compat.h`）：bus-off 自動 recovery + error-passive TX 死鎖斬斷（TX queue 卡 >500ms → stop/start 清卡死幀）。TWAI 進 bus-off 不自動恢復（MCP2515 硬體會），缺防護會一次瞬態永久卡死
- **馬達 power-cycle 後 ESP32 必須重開機**（sessionZeroRaw/座標映射建立於 boot；馬達重啟後映射失效 → F5 有 ack 但不動）
- 每次輪詢讀取前 flushReceiveBuffer()：語意=排空舊回覆，兩後端皆保留。TWAI RX queue 64 深，MCP2515 的 2-buffer 塞爆問題已消失（僅餘 fallback 路徑）
- **輪詢讀取率與迴圈率解耦（2026-07-02，`POLL` 指令）**：舊「每 cycle 全讀 6 顆」把 bus 佔用綁死迴圈率（355Hz→81%、讀等待 ~2.7ms 反鎖迴圈率天花板）。現讀取按排程（預設 100Hz/顆，bus ~23%），vel/積分類估計改走 encoder 時基（encUpdated/encDt）；主頁「讀取 Hz」滑桿=此值（原 auto-return 滑桿退役，`AR`/`A1` 僅存 console 實驗路徑）
- encoder 讀取失敗時保持上一次角度值（不回退到 neutralAngle）
- main.cpp 的 MCP 暫存器深診斷指令（rawReadReg/rawSetMode 路徑）在 TWAI 後端是 no-op stub，回值無意義——只在 fallback 硬體上有效
- ⚠️ **1M bitrate（2026-07-02 M5 白老鼠實測）**：42D@1M 有「閒置≥~5s → CAN 全聾 ~4.2s」韌體缺陷（500k 無）；醒時 1M 乾淨（0.6ms/TEC=0）=類比裕度 OK。黑屏可工程繞開（100Hz 輪詢=天然 keepalive 永不閒置；僅 ESP32 重開機後暴露 → boot 暖機敲 5s 可解）。**重啟條件**：迴圈需求 >500Hz 且 F5 率解耦不夠用、或主動控制需 >200Hz/顆感測；屆時須全隊六顆+F5 負載 soak 驗證（本次僅驗 M5 讀路徑）。已定案知識：0x8A 即刻生效、混速過渡自解（旁觀者 error-passive 自閉嘴）、復原=敲醒趁暖發。工具 `pio run -e bitrate_probe`；詳見記憶 `project_1m_bitrate_verdict`。註：500k 迴圈天花板 ≈ F5 六顆 1.46ms/cycle + 輪詢 → ~450-500Hz；先用「F5 率與迴圈率解耦」（同輪詢解耦手法，F5>400Hz 物理收益≈0）再考慮 1M

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
| `P x y z roll pitch yaw [ms]` | 設定絕對目標姿態（z 絕對 mm）；HOLD 下帶 ms（軌跡時長，預設 1500）→ 韌體 IK→smoothstep 平滑死咬到該絕對 pose；非 HOLD 僅更新 targetPose 供 mode0/1 |
| `FOLLOW 1`/`FOLLOW 0` | 進/出跟隨模式（僅 HOLD）。進入用 FK(當前角度) 當濾波器起點不跳，回 `{"status":"follow on","pose":[...]}` 供前端同步滑桿。離開凍結當前姿態續純P 死咬 |
| `PF x y z roll pitch yaw` | 跟隨目標串流（絕對 pose）：高頻、僅更新 followTgt、不回 ack；韌體速限濾波器每 cycle 把關 IK 有效性（超工作空間維持上一有效點）。非 followMode 忽略。host 動作庫亦走此指令。server 端另有安全殼 soft-knee 投影主閘（manualPfUpdateTarget，env `PF_SAFE_KNEE`/`PF_SAFE_CLAMP`，見 docs/phone-gyro-control.md） |
| `VF vmaxT vmaxR` | 跟隨速度上限（mm/s, deg/s）：濾波器追蹤速度的硬上限 = 所有 host 動作的硬體安全閥。預設 60/45 |
| `K kp ki kd kv` | 暫時套用馬達內部 vFOC PID（0x96，範圍 0-1024；重開機/重燒錄後不保證保留） |
| `KS [kp ki kd kv]` | 保存 vFOC PID 到 ESP32 NVS；帶參數時先套用再保存，不帶參數時保存目前韌體真值 |
| `KRESET` | 清除 ESP32 NVS 內保存的 vFOC PID，套用韌體 BOOT 預設 |
| `KI val` / `KI axis val` | 設 HOLD task-space 角度域積分增益（路線甲消重力 droop）：全軸或單軸(0-5)，範圍 0-200。設 0 清該軸積分態。僅 HOLD 純死咬靜止時作用 |
| `KIC clampDeg deadbandDeg settleDps` | HOLD 積分護欄：輸出角度硬上限(0-10)/誤差死區(0-5)/安定速度閾值(0.1-100) |
| `KIS` | 保存 HOLD 積分組態（Ki+護欄）到 ESP32 NVS |
| `KIRESET` | 清除 NVS HOLD 積分組態，回 BOOT 預設（Ki=0 關閉） |
| `V speed acc` | 設定位置模式速度(1-200 RPM)和加速度(1-255) |
| `POLL hz` | 編碼器輪詢讀取率（5-500Hz/顆，due 時全讀 6 顆保時間相干），與迴圈率解耦，存 NVS。boot 預設 100Hz（bus ~23%）；mode 0/1 備援強制回每 cycle 讀 |
| `M mu` | 設定自適應追蹤震動懲罰係數 |
| `T0`~`T5` | 單馬達方向測試（以 3 RPM 轉 0.5 秒，回報角度變化） |
| `WIFI ssid pass` | 設定 WiFi 憑證存 NVS（namespace netcfg；naive space-split，不支援含空格） |
| `WIFION` / `WIFIOFF` | 啟用／停用 WiFi 並存 NVS；WIFION 即時連線回報 IP |
| `WIFI?` | 查 WiFi 狀態（enabled/connected/ssid/ip/rssi/heap，不回 pass 明文） |
| `FS 0`/`FS 1` | 斷線失效保護策略（0=HOLD-current 預設保形 / 1=斷電），存 NVS |
| `HB ms` | 心跳逾時(ms)，0=停用心跳檢查（僅靠 socket-close 偵測），存 NVS |

### WiFi 遠端控制（TCP :3333，與 USB 並存）

- TCP 與 USB 共用同一條 dispatch，指令語意一致、ack 經 DualPrint 鏡像回兩路。
- **TCP 禁 `Z`/`Z0~Z5`（校正）**：寫 NVS 不可逆、且校正須實體 home 姿態（手邊操作）→ 僅 USB。
- **失效保護恢復**：HOLD-current 觸發後（凍結當前姿態、holdMode=true），TCP 重連送 `P`（絕對 pose）即啟動 IK 軌跡恢復驅動。若要完全重置到 mode 0/1 正常控制，送 `E`（或 `S`→`E`）。
- **heartbeat caveat**：`HB ms`>0 時，client 須週期性「上行」（server tcp 模式每秒送 `\n`）；只收不送的 client 會在逾時後誤觸 failsafe，故預設 `HB 0`（僅靠 socket-close/WiFi-down 偵測）。

## 位置控制 — 現狀

### 現狀（純P 工作點穩定）

**已找到穩定工作點：vFOC 純P `[Kp,Ki,Kd,Kv]=[1024,0,0,0]` + HOLD 死咬。** `Ki=0` 滅掉 M4 2Hz 自持極限環的根因（積分撞死區）、`maxKp` 補剛性、F5 posSpeed 提供阻尼；battery 壓測下手推擾動穩定回正。完整決策記錄見記憶 `project_pid_working_point` / `project_m4_limit_cycle`。

**主要運行模式 = HOLD（純P 死咬）**：`H` 鎖住當前姿態 snapshot（holdAngles），馬達對該凍結目標做純P 保持；姿態操作（`P` 絕對 pose）由韌體 IK 解→smoothstep 平滑移動到目標絕對姿態。下方 mode 0/1 仍存在於碼中、作架構備援，非當前主運行路徑。

注意：BOOT 預設已改 `[1024,0,0,0]`（`main.cpp`），但若跑舊 binary（未 `npm run upload`）開機仍是出廠 `220/30/270/320`，需 K 指令或重燒。已確認的調參值用 `KS` 存 ESP32 NVS；之後開機/一般重燒錄會先讀保存值並下發到六顆馬達，無保存值才回 BOOT 預設。噪音（Kd 致嘯叫 / 馬達保持電流固有聲）未結案、使用者選擇忽略。

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
