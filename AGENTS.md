# Stewart Platform - ESP32 CAN Bus 6DOF Controller

## 硬體架構

- **MCU**: ESP32 DevKit V1 (Arduino framework, PlatformIO)
- **CAN**: MCP2515 + TJA1050 模組 (8MHz crystal, 500Kbps)
  - SPI: GPIO18=SCK, GPIO23=MOSI, GPIO19=MISO, GPIO5=CS, GPIO4=INT
- **馬達**: 6× MKS SERVO42D 閉環步進馬達 (CAN ID 1~6)
  - 工作模式: SR_vFOC (0x05)，必須用此模式才能接受運動指令
  - 編碼器: 16384 counts/rev
  - 終端電阻: 第1顆和第6顆各接 120Ω
- **電源**: 24V PSU 供馬達，ESP32 由 USB 獨立供電

## CAN 協議 (SERVO42D)

- CAN 2.0A, 標準 11-bit ID, 500Kbps
- CRC 計算: `CRC = (CAN_ID + Byte1 + ... + Byte(n-1)) & 0xFF` — 必須包含 CAN ID
- 關鍵指令:
  - `0x30` 讀取編碼器 (回傳 carry + encoder value)
  - `0xF3` 使能/失能馬達
  - `0xF6` 速度模式
  - `0xFD` 相對位置模式 (用於精確定位)
  - `0x80~0x82` 讀取配置

## 專案結構

- `src/main.cpp` — ESP32 韌體 (掃描馬達、JSON狀態輸出、接受Serial指令)
- `dashboard.py` — Python USB Serial dashboard (localhost:8080, 自動重連)
- `platformio.ini` — PlatformIO 配置 (esp32dev, mcp_can library)

## 目標

實現 6DOF 反向運動學控制: 輸入 (roll, pitch, yaw, x, y, z) → 計算 6 個曲柄角度 → SERVO42D 位置模式移動到目標。

## IK 公式 (rotary crank-rod 拓撲)

參考 `/Users/chenliangyu/Desktop/Development/Physics-Sim/stewart_platform/Matlab Code/StewartPlatform.m`

```
Q(i) = Rz(yaw) * Ry(pitch) * Rx(roll) * P(i) + [x; y; z]
L = |Q(i) - B(i)|² - (upper_leg² - lower_leg²)
M = 2 * lower_leg * (Qz - Bz)
N = 2 * lower_leg * (cos(θi)*(Qx - Bx) + sin(θi)*(Qy - By))
angle(i) = asin(L / √(M² + N²)) - atan(N/M)
```

幾何參數 (待量測填入):
- `base_radius` — 基座圓心到馬達軸心距離
- `base_angle` — 基座同一對馬達之間的夾角
- `platform_radius` — 平台圓心到球頭接點距離
- `platform_angle` — 平台同一對接點之間的夾角
- `lower_leg` — 曲柄臂長 (馬達軸到連桿鉸接點)
- `upper_leg` — 連桿長度 (鉸接點到平台球頭)
- `stepper_plane_angles` — 6 個馬達的旋轉平面角度

## 注意事項

- 用繁體中文溝通
- VS Code IntelliSense 的 clang 錯誤可忽略，PlatformIO 編譯正常
- CAN bus 佈線: 雙絞線，電源線遠離信號線，避免接觸不良
- 燒錄前必須關閉 Serial Monitor，否則 port busy
