# Configuration Commands

All config commands respond DLC=3: `[code, status, CRC]`. status: 0=fail, 1=success.

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

## 0x83 — Set Working Current

**Save to flash** (DLC=4): `[83, current_hi, current_lo, CRC]`
- current: uint16 mA. 42D/28D/35D max=3000, 57D max=5200.
- Response status: 0=fail, 1=saved.

**No save** (DLC=5): `[83, current_hi, current_lo, 00, CRC]`
- Response status: 0=fail, 1=saved, 2=success without save.

Warning: Do not set maximum current directly — fluctuations may damage driver.

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

## 0x85 — Set EN Pin Level

DLC=3: `[85, level, CRC]`
- 0x00=active low (default), 0x01=active high, 0x02=Hold (always enabled)

## 0x86 — Set Direction

DLC=3: `[86, dir, CRC]`
- 0x00=CW (default), 0x01=CCW
- Pulse interface only; bus interface direction is set by motion commands.

## 0x87 — Set Auto Screen-Off

DLC=3: `[87, enable, CRC]`
- 0x00=disabled (default), 0x01=enabled (15s timeout, any button wakes)

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

## 0x8F — Set Key Lock

DLC=3: `[8F, enable, CRC]`
- 0x00=unlock, 0x01=lock

## 0x95 — Set Position Reach Threshold

DLC=5: `[95, enable, threshold_hi, threshold_lo, CRC]`
- enable: 0=disable, 1=enable (default)
- threshold: uint16, default=200, max=65535 (=360°)
- "Position completed" response only sent when actual position error < threshold.

Note: If motor has no magnet or magnet is far, 0xF1 may return incorrect values. Use 0x95 to disable encoder judgment, then use open-loop mode to check status.

## 0x0F — Set No-Verification Mode

DLC=3: `[0F, 01, CRC]`
- Disables CRC checking. Test only, not saved. Motor re-enters verification on power-on.
