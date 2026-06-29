# 參考設計 — Liam Clark Stewart Platform (2018)

> 本專案 6-RSS 旋轉式 Stewart platform 的**直系設計來源**。3D CAD、運動學模型、設計報告與原始韌體的整理歸檔（已剔除 IDE 暫存/build 產物/廠商庫）。

## 出處

- **Repo**：[liamclarkza/stewart-platform](https://github.com/liamclarkza/stewart-platform)
- **作者**：Liam Clark（2018-10，University of Cape Town 畢業專案）
- **歸檔 commit**：`7ab9347`（2018-11-10「Add report」）
- **授權**：原 repo **無 LICENSE 檔** → 視為保留全部權利。此處僅作**私人工程參考**，勿再散布。
- 原始 clone 72 MB（含 16 MB Eclipse `.metadata/` + build 產物 + 20 MB ST HAL/CMSIS）→ 整理後 **23 MB**，只留設計實體。

## 目錄導覽

| 路徑 | 內容 |
|------|------|
| `Report.pdf` | 完整設計報告（機構設計、運動學推導、測試結果、渲染圖）13 MB |
| `solidworks/` | 全套 SolidWorks 3D（**零件檔名含空格，勿改名**——`.SLDASM` 靠檔名連結零件） |
| `solidworks/Assemblies/Full Assembly.SLDASM` | 總裝（從這顆開最快看懂機構） |
| `matlab/StewartPlatform.m` | 運動學 class（IK + 正運動繪圖 + serial 控制）——**幾何/IK 真相源** |
| `matlab/Tester.m` · `GraphsForReport.m` | 帶**實際幾何數值**的呼叫端 + 工作空間掃描 |
| `firmware/main.c` | 原始 STM32F407 韌體核心（TIM6 Bresenham 步進插補 + USB-CDC 指令解析）|
| `firmware/Controller.ioc` | CubeMX 腳位/時鐘配置 |

> ST HAL/CMSIS/USB middleware 廠商樣板碼、Eclipse 工作區、編譯產物**已剔除**（對理解機構零價值，需要可回原 repo 取）。

## 機構解剖 — 6-RSS 旋轉式

從 `solidworks/` 零件樹可讀出完整運動鏈：

```
Stepper(馬達) ─ Stepper Shaft Rod Mount(曲柄/crank = lower leg)
                      │  繞馬達軸轉 → 驅動變數 = 角度 θ
                 Arm Bearing Joint(球/軸承副) ── Arm Rod(連桿/rod = upper leg, 固定長)
                                                      │
                                                 End-Effector Platform(動平台)
```

- **6 顆馬達成 3 對**：`Stepper Pair Assembly` + `Stepper Mount Dual` 證實「兩顆共一座、鏡像安裝」→ 對應本專案 pairs @ `210°/90°/-30°`。
- **致動變數是馬達轉角**（非伸縮腿長）→ 這是 **6-RSS**（rotary，曲柄連桿），不是 6-UPS 線性伸縮 hexapod。
- `Base Platform/Leg/Center Beam` = 固定基座；`Calibration Block/Rod/Plate` = 校正治具（對應本專案 `zeroRaw` 校正概念）。

## 實際幾何數值（最終 config，`Tester.m` / `GraphsForReport.m`）

`StewartPlatform(base_radius, base_angle, platform_radius, platform_angle, lower_leg, upper_leg)`：

| 參數 | 值 | 說明 |
|------|----|------|
| `base_radius` | **152 mm** | 基座 joint 圓半徑 |
| `base_angle` | **18.92°** | 同對兩 joint 夾角（對 ±9.46°）|
| `platform_radius` | **103 mm** | 動平台 joint 圓半徑 |
| `platform_angle` | **28.07°** | 平台側同對夾角 |
| `lower_leg`（曲柄）| **73 mm** | 馬達曲柄臂（設計中 62→73 演進，見 `ReportTests.m`）|
| `upper_leg`（連桿）| **165 mm** | 固定長連桿 |
| home `z` | 105 mm（建構預設）/ 運行 ~140 mm | |
| 工作空間（測試）| x,y `±20 mm`、z `120–200 mm`、roll/pitch/yaw `±30°` | |

> ⚠️ 這正是本專案 CLAUDE.md「數值以三處程式為準、別在本文件重列」所指的 mm 真值來源。本專案 FK 僅在 **±15°** 內驗證收斂（比參考測的 ±30° 保守）。

## 運動學

IK（`StewartPlatform.m::set_position`，逐軸獨立解曲柄角）：

```
R = Rz(yaw)·Ry(pitch)·Rx(roll)
Q_i = R·P_i + [x;y;z]                       # 平台 joint 的基座座標
L = |Q_i − B_i|² − (upper² − lower²)
M = 2·lower·(Q_iz − B_iz)
N = 2·lower·(cosθ_i·(Q_ix−B_ix) + sinθ_i·(Q_iy−B_iy))    # θ = stepper_plane_angle
θ_motor(i) = asin(L/√(M²+N²)) − atan2(N, M)
```

`stepper_plane_angle = [-90, 90, 30, 210, -210, -30]`（規範化 `[270,90,30,210,150,330]`）。
**此 IK 與本專案 CLAUDE.md 的公式逐符號相同** → 確認本專案 IK 移植自此。FK 參考用無（本專案另寫距離約束 Newton-Raphson 解 asin 90° 奇異）。

## 血緣對照 — 此參考 ↔ 本專案

| 面向 | Liam Clark 參考 (2018) | 本專案 (stewart-platform) |
|------|----------------------|--------------------------|
| 拓樸 | 6-RSS 旋轉曲柄連桿 | **相同** |
| `base_angle` / `platform_angle` | 18.92° / 28.07° | **完全相同**（CLAUDE.md `BASE_ANGLE`/`PLATFORM_ANGLE`）|
| 幾何尺寸 | 152/103/73/165 mm | 衍生（以 `kinematics.h`/`web/index.html`/`sysid/kin.js` 為準）|
| IK 公式 | `asin(L/√(M²+N²))−atan2(N,M)` | **逐符號相同** |
| 馬達正負交替 | `move()` 送 `a1,-a2,a3,-a4,a5,-a6` | **相同**（`MOTOR_SIGN=[1,-1,1,-1,1,-1]`，成對鏡像安裝）|
| 馬達平面角 | `[270,90,30,210,150,330]` | **重排** `MOTOR_PLANE_ANGLE=[300,120,180,0,60,240]`（本專案改 CW M1→M6）|
| 致動器 | NEMA 步進 + A4988 類，**6400 步/圈**（1.8°×32 microstep）| **MKS SERVO42D**，14-bit 編碼器 **16384/圈**、閉環 vFOC |
| 通訊 | USB-CDC virtual COM | **CAN**（MCP2515）+ WiFi TCP |
| 軌跡執行 | STM32 TIM6 ISR + **Bresenham 多軸插補** | ESP32 算 IK → `0xF5` 位置模式（馬達內部三環 PID）|
| Serial 協議 | `m <t> m1..m6;`(移動) / `r`(歸零) / `s`(狀態) | 祖先 → 演進為 `P`/`E`/`S`/`Z`/`PF`/`FOLLOW`… |
| 校正 | `calibrate()` 全轉 590 步(≈33.2°)再 reset | 「下腿朝上=90°」`zeroRaw` 快照 |
| 主控 | STM32F407 | ESP32 WROOM（升級候選 ESP32-S3，見 mega-upgrade）|

**結論**：本專案 = 此參考設計的**再致動版**——同機構、同 IK、同幾何角，把開環步進+USB 換成閉環 SERVO42D+CAN，並加上 WiFi/跟隨/監控等。

## 關聯文檔（本專案）

- 運動學/幾何真相源：`src/kinematics.h`、`web/index.html`、`sysid/kin.js`（CLAUDE.md 鐵律：三處同步）
- CAN 指令文檔圖譜：[`../../servo-can-hub.md`](../../servo-can-hub.md)
- Mega 升級（ESP32-S3 + SERVO42ES + 1:3 行星，背隙=新精度上限）：[`../../mega-upgrade/README.md`](../../mega-upgrade/README.md)
