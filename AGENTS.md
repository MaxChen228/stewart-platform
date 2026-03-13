# Stewart Platform - ESP32 CAN 6DOF Controller

## 溝通與工作原則

- 用繁體中文溝通
- 先做、先驗證、再報告
- 優先把重複操作固化成腳本，而不是反覆手動執行
- 如果修改 `src/main.cpp`，必須明確提醒「需要重新燒錄 ESP32」
- 燒錄前必須關閉 Serial Monitor / 任何占用 serial port 的程式

## 專案目的

此專案的目標不是單純做一個 dashboard，而是建立一套：

- `pose -> IK -> target actuator -> hardware move -> actual actuator -> actual pose`

的完整 Stewart Platform 控制與驗證鏈。

## 專案結構

- `src/main.cpp`
  - ESP32 韌體
  - CAN 命令執行、encoder telemetry、硬體校正
- `host/kinematics.py`
  - 共用幾何、IK、actual FK
- `host/app.py`
  - serial bridge、HTTP API、runtime state、校正持久化
- `web/*`
  - 操作 UI 與 3D 視覺化
- `ops.py`
  - bring-up / calibrate / validate / goto-cal / watch
- `dashboard_ctl.py`
  - start / stop / restart / status

## 硬體摘要

- MCU: `ESP32 DevKit V1`
- CAN: `MCP2515 + TJA1050`, `500 kbps`, `8 MHz crystal`
- Motor: `6x MKS SERVO42D`
- Power: motors `24V`, ESP32 by USB

SPI:

- `GPIO18 = SCK`
- `GPIO23 = MOSI`
- `GPIO19 = MISO`
- `GPIO5 = CS`
- `GPIO4 = INT`

## SERVO42D 關鍵事實

- CRC 必須包含 CAN ID
- encoder telemetry 與 motion pulse 不是同一個單位系統
- `0xFD` relative position mode 目前已驗證要用專案內現行方向位與 `3200 pulses/rev`
- 目前 project 仍以 host 為 IK 真相來源，firmware 只做 motion executor + telemetry

更多細節見：

- [docs/servo42d-and-firmware.md](/Users/chenliangyu/Documents/PlatformIO/Projects/stewart-platform/docs/servo42d-and-firmware.md)
- [docs/calibration-and-operations.md](/Users/chenliangyu/Documents/PlatformIO/Projects/stewart-platform/docs/calibration-and-operations.md)

## 校正與 Home 概念

這是本專案最重要的操作知識：

- `Calibrate` 不是工作原點
- `Calibrate` = 六顆曲柄全朝上，用來建立 encoder reference
- `Home` = 穩定工作位，不應該等於 calibration 高點

目前已知：

- calibration height 約為 `222.239`
- operating home height 約為 `143.174`

更多細節見：

- [docs/calibration-and-operations.md](/Users/chenliangyu/Documents/PlatformIO/Projects/stewart-platform/docs/calibration-and-operations.md)

## 穩定性結論

目前已知重要經驗：

- 全朝上姿態在理論模型裡是唯一解
- 但它是近奇異 / 壞條件點
- 不適合當正常工作點
- 結構剛性不足、幾何誤差、保持力不足、閉鏈耦合，都會在這附近被放大

更多細節見：

- [docs/stability-and-known-issues.md](/Users/chenliangyu/Documents/PlatformIO/Projects/stewart-platform/docs/stability-and-known-issues.md)

## 架構與資料模型

不要混淆這三層：

1. `theory pose`
2. `target actuator`
3. `actual actuator`

模擬與真機必須透過這三層明確分開，否則會出現「看起來同步，實際是假同步」。

更多細節見：

- [docs/architecture.md](/Users/chenliangyu/Documents/PlatformIO/Projects/stewart-platform/docs/architecture.md)

## 目前預設幾何

- `base_radius = 152`
- `base_angle = 18.92`
- `platform_radius = 103`
- `platform_angle = 28.07`
- `lower_leg = 65`
- `upper_leg = 165`
- `stepper_plane_angles = [-90, 90, 30, 210, -210, -30]`
- `motor_signs = [1, -1, 1, -1, 1, -1]`
- `servo_pulses_per_rev = 3200`

這些值是目前工程上的工作值，不代表已完成最終量測定版。

## 參考運動學

rotary crank-rod IK 公式仍以：

- `/Users/chenliangyu/Desktop/Development/Physics-Sim/stewart_platform/Matlab Code/StewartPlatform.m`

為主要參考來源。

## 開發時優先事項

當問題很多時，優先順序應該是：

1. 先確認 runtime state / calibration state 是否一致
2. 再確認 target actuator 與 actual actuator 是否一致
3. 再確認幾何是否合理
4. 最後才調 UI 細節

## 操作提醒

- 日常運動從 `Home` 附近開始，不要停在 calibration 高點工作
- 如果 dashboard 重啟，先確認 runtime state 沒丟失
- 如果動到 firmware，先重新燒錄再驗證
- 若懷疑結構柔性主導問題，優先驗證底座與上平台剛性
