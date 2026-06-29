# Product Specs & Working Modes

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/02-specs-and-modes|42ES Product Specs & Working Modes]]

## Hardware Parameters

| Parameter | SERVO42D V1.0 | SERVO57D V1.2 |
|-----------|---------------|---------------|
| MCU | N32L403 (Cortex-M4) | N32L406 (Cortex-M4) |
| MOSFET | AP4008QD (40V, 20A) | AP30H80Q (30V, 70A) |
| Magnetic encoder | MT6816 (14-bit) | MT6816 (14-bit) |
| CAN transceiver | TJA1051T | TJA1051T |
| Operating voltage | 12V-24V | 12V-24V |
| Working current | 0-3000mA | 0-5200mA |
| Torque loop | 20 kHz | 20 kHz |
| Speed loop | 10 kHz | 10 kHz |
| Position loop | 10 kHz | 10 kHz |
| Maximum speed | 3000+ RPM | 3000+ RPM |
| Subdivisions | 1-256 arbitrary | 1-256 arbitrary |
| Pulse signal input | 3.3V-24V (common anode) | 3.3V-24V (common anode) |
| Max pulse frequency | 160 kHz | 160 kHz |
| CAN bit rate | 125K/250K/500K/1M | 125K/250K/500K/1M |
| CAN ID range | 1 broadcast + 2047 slave | 1 broadcast + 2047 slave |

> 🔀 **vs 42ES：** 功率/控制平台全面不同——[[servo42es/02-specs-and-modes|42ES]] 改用 **STM32F302** MCU（42D=N32L40x）、**MT6826** 編碼器（42D=MT6816 14-bit）、**20-48V / 60V·56A MOSFET**（42D=12-24V / 40V·20A）、max pulse **300 kHz**（42D=160 kHz），並新增 **RS485 + MODBUS-RTU** 介面。CAN 應用層位元組仍相容。

## Working Modes

| mode value | Mode Name | Interface | Max RPM | Work Current |
|------------|-----------|-----------|---------|-------------|
| 0x00 | CR_OPEN | Pulse, open-loop | 400 | Fixed (Ma) |
| 0x01 | CR_CLOSE | Pulse, closed-loop | 1500 | Fixed (Ma) |
| 0x02 | CR_vFOC | Pulse, FOC (default) | 3000 | Self-adapting (max=Ma) |
| 0x03 | SR_OPEN | Bus/CAN, open-loop | 400 | Fixed (Ma) |
| 0x04 | SR_CLOSE | Bus/CAN, closed-loop | 1500 | Fixed (Ma) |
| 0x05 | SR_vFOC | Bus/CAN, FOC | 3000 | Self-adapting (max=Ma) |

Bus control commands (0xF1-0xFF, 0xF4-0xF6, 0xFD-0xFE) only valid in SR_OPEN/SR_CLOSE/SR_vFOC.

> 🔀 **vs 42ES：** mode 值映射不同，**勿沿用本表的數字**。[[servo42es/02-specs-and-modes|42ES]] 改用 encoder-presence 分組（With/No encoder）+ 描述式命名，且本專案的 bus closed-loop FOC 在 42ES 是 **05H**（同號）、但 default 變為 `03H` Pulse+Direction Closed-Loop（42D default=02H CR_vFOC）；42ES 另增 `10H/11H/14H` No-encoder 群組。設值前對照 42ES 0x82 表。

## Speed Parameters

- Range: 0-3000. speed=0 → stop.
- Max depends on mode: OPEN=400, CLOSE=1500, FOC=3000 RPM.
- Speed calibrated for 16/32/64 subdivisions. Other subdivisions: `actual_RPM = speed × 16 / subdivision`.

## Acceleration Parameters

- Range: 0-255. acc=0 → no ramp (instant speed change).
- Ramp time per 1 RPM step: `(256 - acc) × 50 µs`.
- Higher acc = faster acceleration/deceleration.

## IO Ports

| Default port | Function | 57D | 28/35/42D |
|-------------|----------|-----|-----------|
| IN_1 | Home or left-limit | Yes | Yes |
| IN_2 | Right-limit | Yes | No |
| OUT_1 | Stall indication (0=protected, 1=unprotected) | Yes | No |
| OUT_2 | Pulse frequency division output | Yes | No |

After 0x9E limit remapping: IN_1/IN_2 invalid → En=left-limit, Dir=right-limit. COM must be high.

## Hardware Version Map

| hardVer | Board |
|---------|-------|
| 1 | S42D_485 |
| 2 | S42D_CAN |
| 3 | S57D_485 |
| 4 | S57D_CAN |
| 5 | S28D_RS485 |
| 6 | S28D_CAN |
| 7 | S35D_RS485 |
| 8 | S35D_CAN |

> 🔀 **vs 42ES — 設定持久化：** 42D 文檔集無顯式 SAVE 指令。[[servo42es/02-specs-and-modes|42ES]] 統一改為「先 write、後 `0x60` commit」模型——working mode / current / subdivision / speed / acc 等寫入後須執行一次 0x60 才存盤，否則下電丟失。移植到 42ES 時務必補 0x60。
