# Configuration Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/04-config-commands|42ES Configuration Commands]]

All config commands respond DLC=3: `[code, status, CRC]`. status: 0=fail, 1=success.

> 🔀 **vs 42ES：** 42D 的 4.3 設定寫入**直接持久化**（無獨立存檔指令）。[[servo42es/04-config-commands|42ES]] 反之：任何 4.3 寫入只進 RAM，必須再下一條 **0x60 SAVE** 才會在斷電後保留——這是兩線最大的行為差異。

## 0x80 — Calibrate Encoder

DLC=3: `[80, 00, CRC]`
Response status: 0=calibrating, 1=success, 2=fail.
Motor must be unloaded. Calibrate before installing into machine.

## 0x82 — Set Working Mode

DLC=3: `[82, mode, CRC]`

| mode | Name |
|------|------|
| 0x00 | CR_OPEN (Pulse open-loop) |
| 0x01 | CR_CLOSE (Pulse closed-loop) |
| 0x02 | CR_vFOC (Pulse FOC, **default**) |
| 0x03 | SR_OPEN (Bus open-loop) |
| 0x04 | SR_CLOSE (Bus closed-loop) |
| 0x05 | SR_vFOC (Bus FOC) |

> 🔀 **vs 42ES：** 模式編碼不同。42D 為扁平 `0x00`-`0x05`（CR/SR × OPEN/CLOSE/vFOC），預設 `0x02` CR_vFOC。[[servo42es/04-config-commands|42ES]] 把同一軸拆成 With-encoder `0x00`-`0x05` 與 No-encoder `0x10`/`0x11`/`0x14`，預設 `0x03`。mode byte **不可互換**。

## 0x83 — Set Working Current

**Save to flash** (DLC=4): `[83, current_hi, current_lo, CRC]`
- current: uint16 mA. 42D/28D/35D max=3000, 57D max=5200.
- Response status: 0=fail, 1=saved.

**No save** (DLC=5): `[83, current_hi, current_lo, 00, CRC]`
- Response status: 0=fail, 1=saved, 2=success without save.

Warning: Do not set maximum current directly — fluctuations may damage driver.

> 🔀 **vs 42ES：** 42D 0x83 有兩種形式——存檔型（`DLC=4`）與不存檔型（`DLC=5`、尾帶 `00`、status `2`=success-without-save）。[[servo42es/04-config-commands|42ES]] 只有單一 `DLC=4` 寫入型，且仍須 0x60 才持久；無 per-command no-save byte。

## 0x9B — Set Holding Current %

DLC=3: `[9B, ratio, CRC]`

| ratio | Percentage |
|-------|-----------|
| 0x00 | 10% |
| 0x01 | 20% |
| ... | ... |
| 0x08 | 90% |

Default: 50%. Only valid in OPEN/CLOSE modes (not vFOC).

## 0x84 — Set Subdivisions

DLC=3: `[84, micstep, CRC]`
- micstep: 1-256. Prefer powers of 2 (2, 4, 16, 32, 64).

> 🔀 **vs 42ES：** range 不同。42D 為 `1-256` 連續值（直接寫 `256`）。[[servo42es/04-config-commands|42ES]] 為固定離散集合，並把 `0` overload 成 256 細分。

## 0x85 — Set EN Pin Level

DLC=3: `[85, level, CRC]`
- 0x00=active low (default), 0x01=active high, 0x02=Hold (always enabled)

## 0x86 — Set Direction

DLC=3: `[86, dir, CRC]`
- 0x00=CW (default), 0x01=CCW
- Pulse interface only; bus interface direction is set by motion commands.

> 🔀 **vs 42ES：** value map 相同（`00`=CW / `01`=CCW），但 scope 不同。42D 0x86 僅作用於脈衝介面，匯流排方向由 motion 指令決定。[[servo42es/04-config-commands|42ES]] 的 0x86 **同時也會翻轉 bus-mode 方向**——移植 motion 碼時須留意。

## 0x87 — Set Auto Screen-Off

DLC=3: `[87, enable, CRC]`
- 0x00=disabled (default), 0x01=enabled (15s timeout, any button wakes)

> 🔀 **vs 42ES：** **同碼不同功能。** 42D 0x87 是 Set Auto Screen-Off。[[servo42es/04-config-commands|42ES]] 的 0x87 是 Set Pulse Delay（0/4/20/40 ms 四級枚舉，預設 20 ms）——切勿交叉移植 0x87。

## 0x89 — Set Subdivision Interpolation

DLC=3: `[89, enable, CRC]`
- 0x00=disabled, 0x01=enabled (default)
- Internal 256-subdivision interpolation, reduces vibration at low speed.

## 0x8A — Set CAN Bit Rate

DLC=3: `[8A, bitRate, CRC]`

| bitRate | Speed |
|---------|-------|
| 0x00 | 125K |
| 0x01 | 250K |
| 0x02 | 500K (**default**) |
| 0x03 | 1M |

## 0x8B — Set Slave CAN ID

DLC=4: `[8B, addr_hi, addr_lo, CRC]`
- addr: uint16, 0x01-0x7FF. 0x00=broadcast.

## 0x8C — Configure Slave Response

DLC=4: `[8C, XX, YY, CRC]`

| XX | YY | Behavior |
|----|-----|---------|
| 0 | 0 or 1 | No response at all |
| 1 | 0 | Immediate start/fail response only |
| 1 | 1 | Immediate start/fail + completion status (**default**) |

When no-response mode: query status via 0xF1 command.

Note: In no-response mode (XX=0), motor still executes commands but sends nothing back.
In XX=1/YY=0 mode, motor responds immediately with start(01)/fail(00) but does NOT send completion(02)/endstop(03).
In default mode (XX=1/YY=1), motor sends both immediate response AND completion response.

> 🔀 **vs 42ES：** **編碼不同。** 42D 用 2-byte `[8C, XX, YY]`（`DLC=4`）。[[servo42es/04-config-commands|42ES]] 用單一 `ackMode` byte `00`/`01`/`02`（`DLC=3`）。語意可對映（ES `00`≈XX=0；ES `01`≈XX=1/YY=0；ES `02`≈XX=1/YY=1 預設）但 **wire format 不可互換**。

## 0x8F — Set Key Lock

DLC=3: `[8F, enable, CRC]`
- 0x00=unlock, 0x01=lock

> 🔀 **vs 42ES：** **同碼不同功能。** 42D 0x8F 是 Set Key Lock（鎖實體 OLED 按鍵）。[[servo42es/04-config-commands|42ES]] 的 0x8F 是 lock-axis-on-bus-start（匯流排上電時的保持力矩）——切勿混淆或交叉移植 0x8F。

## 0x95 — Set Position Reach Threshold

DLC=5: `[95, enable, threshold_hi, threshold_lo, CRC]`
- enable: 0=disable, 1=enable (default)
- threshold: uint16, default=200, max=65535 (=360°)
- "Position completed" response only sent when actual position error < threshold.

Note: If motor has no magnet or magnet is far, 0xF1 may return incorrect values. Use 0x95 to disable encoder judgment, then use open-loop mode to check status.

> 🔀 **vs 42ES：** **不同碼且不同預設值。** 42D 用 `0x95`，預設 threshold **200**。[[servo42es/04-config-commands|42ES]] 用 `0x98`，預設 **800**。兩者皆上限 65535 = 360°、皆預設啟用。

## 0x0F — Set No-Verification Mode

DLC=3: `[0F, 01, CRC]`
- Disables CRC checking. Test only, not saved. Motor re-enters verification on power-on.

> 🔀 **vs 42ES：** 語意相同（`[0F, 01]`、僅測試用、永不持久）。在 [[servo42es/04-config-commands|42ES]] 線上這也是唯一一條 0x60 不會存的 4.3 指令——其餘 4.3 寫入都需 0x60，0x0F 是例外。

---

## Cross-references

- 42ES 對應章節 → [[servo42es/04-config-commands|42ES Configuration Commands]]
- 文檔總覽 → [[servo-can-hub|CAN 文檔總覽]]
