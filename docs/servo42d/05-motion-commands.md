# Motion Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/05-motion-commands|Motion Commands (42ES/57ES)]]

All motion commands only valid in bus control mode (SR_OPEN/SR_CLOSE/SR_vFOC).

**Prerequisites**: 發送運動指令前必須：
1. 設定工作模式為 SR_* → `0x82` (見 [[servo42d/04-config-commands|04-config-commands]])
2. Enable 馬達 → `0xF3` (見 [[servo42d/06-bus-control|06-bus-control]])
3. 座標模式 (F4/F5/FE) 需先設零點 → `0x92` 或 `0x91` (見 [[servo42d/08-homing-zeroing|08-homing-zeroing]])

> 🔀 **vs 42ES：** F6/FD/FE/F4/F5 frame、CRC、status 碼皆 byte-identical。但 42ES 多一層持久化規則 —— 工作模式 `0x82` 等 config 參數寫入後不持久化，須額外送 SAVE 指令 `0x60` 才在斷電後保留（見 [[servo42es/05-motion-commands|42ES]]）。42D 文檔組無此顯式存檔步驟。

**Speed 與 subdivision 關係**: speed 值在 16/32/64 subdivision 下 = 實際 RPM。其他 subdivision: `actual_RPM = speed × 16 / subdivision`。見 [[servo42d/02-specs-and-modes|02-specs-and-modes]]。

## Speed Control Mode (0xF6)

### Run Continuously (DLC=5)

`[F6, dir|speed_hi, speed_lo, acc, CRC]`

- Byte2 bit7 = direction: 0=CCW, 1=CW
- Byte2 bits[3:0] + Byte3 = speed (12-bit, 0-3000 RPM)
- Byte4 = acceleration (0-255)

Example: `F6 01 40 02` → CCW, speed=0x140(320 RPM), acc=2
Example: `F6 81 40 02` → CW, speed=320, acc=2

### Run for Duration (DLC=8)

`[F6, dir|speed_hi, speed_lo, acc, runTime(uint24), CRC]`

- runTime unit = 10ms. Range 0~0xFFFFFF.

### Stop via 0xF6 (DLC=5)

`[F6, 0, 0, acc, CRC]`
- acc≠0: decelerate and stop
- acc=0: immediate stop (**not recommended >1000 RPM**)

Response DLC=3: `[F6, status, CRC]`
- 0=fail, 1=start running, 2=completed, 5=sync mode received

---

## Pulse-Based Position Control

### 0xFD — Relative Pulse Motion (DLC=8)

`[FD, dir|speed_hi, speed_lo, acc, relPulses(uint24), CRC]`

- dir/speed/acc encoding same as 0xF6
- relPulses: relative pulse count (0~0xFFFFFF)

Example: `FD 01 40 02 00 FA 00` → CCW, speed=320, acc=2, 0xFA00=64000 pulses (20 revolutions at 16 subdivision)

### 0xFE — Absolute Pulse Motion (DLC=8)

`[FE, speed_hi, speed_lo, acc, absPulses(int24), CRC]`

- speed: uint16 (0-3000 RPM)
- absPulses: int24 (-8388607 ~ +8388607)
- **Must set zero point first** (0x92 or 0x91)

Example: `FE 02 58 02 00 0C 80` → speed=600, acc=2, to absolute pulse 0x0C80

### Stop 0xFD/0xFE (DLC=8)

`[FD/FE, 0, 0, acc, 0, 0, 0, CRC]`
- acc≠0: decelerate, acc=0: immediate stop

Response: 0=fail, 1=starting, 2=complete, 3=endstop stopped, 5=sync received

---

## Coordinate-Based Position Control

Coordinates = cumulative multi-turn encoder values (16384/turn). Read with 0x31.

### 0xF4 — Relative Coordinate Motion (DLC=8)

`[F4, speed_hi, speed_lo, acc, relAxis(int24), CRC]`

- speed: uint16 (0-3000)
- relAxis: int24 (-8388607 ~ +8388607)

Example: current=0x8000, relAxis=+0x4000 → target=0xC000
Example: current=0x8000, relAxis=-0x4000 → target=0x4000

### 0xF5 — Absolute Coordinate Motion (DLC=8)

`[F5, speed_hi, speed_lo, acc, absAxis(int24), CRC]`

- speed: uint16 (0-3000)
- absAxis: int24 (-8388607 ~ +8388607)
- **Must set zero point first** (0x92 or 0x91)
- **Supports real-time updates**: Can send new F5 while previous is running to change speed and target instantly.

Example: `F5 02 58 02 00 40 00` → speed=600, acc=2, to coordinate 0x4000
Example: `F5 02 58 02 FF C0 00` → speed=600, acc=2, to coordinate -0x4000

### Stop 0xF4/0xF5 (DLC=8)

`[F4/F5, 0, 0, acc, 0, 0, 0, CRC]`
- acc≠0: decelerate, acc=0: immediate stop

Response: 0=fail, 1=starting, 2=complete, 3=endstop stopped, 5=sync received

---

## 0xFF — Auto-Start on Power-On

DLC=3: `[FF, enable, CRC]`
- enable=0xC8: Enable auto-start
- enable=0xCA: Disable auto-start

Must first send a speed command (0xF6), then enable auto-start. Motor will run on every power-on with saved parameters.

Response: 0=fail, 1=start setting, 2=setup complete

> 🔀 **vs 42ES：** 0xFF 的啟用碼**不同**。42D 用 enable byte（DLC=3：`[FF, 0xC8/0xCA, CRC]`，先發 F6 再 enable）；42ES 改把 dir/speed/acc 直接內嵌進 frame（DLC=5：`[FF, dir|speed_hi, speed_lo, acc, CRC]`，`speed=0` 停用、`speed≠0` 啟用），且設定後**必須 `0x60` SAVE** 才在斷電後生效。byte 佈局與語意皆不同，見 [[servo42es/05-motion-commands|42ES]]。
