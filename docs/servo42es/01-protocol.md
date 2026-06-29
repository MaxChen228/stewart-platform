# CAN Protocol Basics

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/01-protocol|CAN Protocol Basics]]

Covers the SERVO42ES **and** SERVO57ES line — identical firmware/protocol (V1.0.1). They differ only in current rating and I/O count (42ES: single `IN`; 57ES: `IN_1` + `IN_2` + `IN_COM`), not in CAN framing. See [[servo42es/10-io-ports|I/O Ports]] for the wiring split.

> 🔀 **vs 42D：** the 42ES doc set is explicitly a twin manual ("SERVO42ES/57ES"); 42D wiring/IO differs but the CAN frame layer is byte-identical.

## Frame Structure

Standard frame, max data field = 8 bytes (`Byte n, n≤8`).

**Downlink (host → driver, PC → SERVOxxES):**

| CAN ID | DLC | Byte1 (code) | Byte2..Byte(n-1) (data) | Byte n (CRC) |
|--------|-----|-------------|------------------------|--------------|
| ID | DLC(n) | code | data | CRC |

**Uplink (driver → host, PC ← SERVOxxES):** Same structure.

| CAN ID | DLC | Byte1 (code) | Byte2..Byte(n-1) (data) | Byte n (CRC) |
|--------|-----|-------------|------------------------|--------------|
| ID | DLC(n) | code | data | CRC |

`Byte1 = code` selects the command (e.g. `0x80` runs calibration, see [[servo42es/08-homing-zeroing|Calibrate]]). `data` is big-endian. The last data byte is always the CRC.

## CAN ID

- Range: 0x00 ~ 0x7FF (0 ~ 2047), per manual "00~2047"
- **0x00 = broadcast** — slaves do **not** respond to a broadcast ID
- Default: 0x01
- 0x01 ~ 0x10 settable in the OLED **CanID** display menu; >0x10 must be set by serial command [[servo42es/04-config-commands|0x8B set CAN ID]]
- CAN_ID 即為馬達地址。本專案 6 顆馬達使用 0x01 ~ 0x06。

> 🔀 **vs 42D：** identical range/default/broadcast/OLED-vs-0x8B split — no difference.

## CRC Calculation

CRC = CHECKSUM 8-bit = `(CAN_ID + all data bytes except CRC) & 0xFF`.

**Worked example (read-encoder command, straight from the PDF):** ID=0x01, `Byte1=0x30`

```
CRC = (0x01 + 0x30) & 0xFF = 0x31
```

So the full downlink frame is `ID=0x01, DLC=2, [30][31]`.

**Second worked example (multi-byte data):** ID=0x01, data=`83 0C 80`

```
CRC = (0x01 + 0x83 + 0x0C + 0x80) & 0xFF = 0x110 & 0xFF = 0x10
```

Data is **big-endian** (MSB first in the frame). The checksum sums the raw CAN_ID plus every data byte *including the code byte*, then masks to 8 bits.

> 🔀 **vs 42D：** CRC algorithm is **byte-identical** to SERVO42D — same `(CAN_ID + data) & 0xFF` formula, same worked result. A 42D CRC routine drops in unchanged. See [[servo42d/01-protocol|42D CRC]].

## Unit Conversions

| Quantity | Value | Notes |
|----------|-------|-------|
| 1 revolution (single-turn encoder) | **16384 counts = 0x4000** | PDF §4.2.1/§4.2.2: "single-turn value range is 0~0x4000" |
| Single-turn `value` field range | 0 ~ 0x3FFF (uint16) | wraps at 0x4000 → carry ±1 |
| 16-subdivision pulse / rev | 3200 pulses (0x0C80) | for pulse/position modes, see [[servo42es/05-motion-commands|Motion]] |
| Angle error unit | 0 ~ 51200 = 0° ~ 360° (≈142.22 / degree) | reported by [[servo42es/03-read-commands|read-error]] |
| Coordinate / cumulative value | cumulative multi-turn encoder, +0x4000 per CW turn | used as the coordinate by abs/rel motion |

**Confirmed against the PDF: 16384 counts/rev (0x4000).** Single-turn encoder range is stated twice (§4.2.1 and §4.2.2) as "0~0x4000". This matches SERVO42D exactly — no value difference.

> 🔀 **vs 42D：** 16384 counts/rev confirmed **identical** to the 42D twin (both 0x4000). No spec divergence on the encoder resolution.

## Encoder Value Encoding (read paths)

The 42ES manual splits encoder reads into two distinct frames with different number formats — document both precisely:

### Multi-turn encoder via `0x30` — carry + single-turn value (8-byte uplink)

Uplink: `[code=30][carry: int32_t (4B)][value: uint16_t (2B)][CRC]`, DLC=8.

- `carry`: the turn-count carry of the encoder (signed int32, big-endian).
- `value`: current single-turn encoder value, range **0 ~ 0x3FFF** (uint16).
- When `value` overflows past 0x3FFF, `carry += 1`; when `value` underflows below 0, `carry -= 1`.

Worked examples from the PDF:
- Current `carry|value` = `0x3FF0`. After one turn **CCW** (+0x4000): `carry|value` = `0x13FF0`.
- Current `carry|value` = `0x3FF0`. After one turn **CW** (−0x4000): `carry|value` = `0xFFFFFFFF3FF0`.

Note: the encoder value is updated **regardless of whether the motor is enabled**.

### Cumulative multi-turn value via `0x31` — single int48 (8-byte uplink)

Uplink: `[code=31][value: int48_t (6B)][CRC]`, DLC=8.

- `value`: cumulative multi-turn encoder value, data type **int48_t** (6 bytes, signed, big-endian).
- After one turn **CW**, `value += 0x4000`; after one turn **CCW**, `value -= 0x4000`.

Worked examples from the PDF:
- Current `value` = `0x3FF0`. After one turn CCW (+0x4000): `value` = `0x7FF0`.
- Current `value` = `0x3FF0`. After one turn CW (−0x4000): `value` = `0xFFFFFFFFFFF0` (int48 two's-complement).

**Use `0x31`'s value as the coordinate** when commanding relative or absolute moves by coordinate value (PDF §4.2.2 note). Full read frames are documented in [[servo42es/03-read-commands|Read Commands]].

> 🔀 **vs 42D：** the **two-frame split is the same shape** as 42D (`0x30` carry+value, `0x31` cumulative). Confirm field widths against [[servo42d/03-read-commands|42D read frames]]; on 42ES the cumulative `0x31` value is explicitly typed **int48_t** and `0x30` returns `int32 carry + uint16 value`.

## Real-Time Speed (`0x32`)

Uplink: `[code=32][rpm: int16_t (2B)][CRC]`, DLC=4. `rpm` unit = RPM, signed. CCW > 0, CW < 0. See [[servo42es/03-read-commands|Read Commands]].

## Absolute vs Relative Position

- **Absolute**: target measured from a fixed origin (zero point). Re-sending the same command to a position already reached produces no motion.
- **Relative**: target measured from the current position. The same command produces the same displacement each time.

Both reference the cumulative encoder coordinate (`0x31` value). Motion command framing is in [[servo42es/05-motion-commands|Motion Commands]].

> 🔀 **vs 42D：** semantics identical to [[servo42d/05-motion-commands|42D motion]] — abs vs rel both keyed to the cumulative encoder coordinate.

## Response / Status Convention

Write commands respond DLC=3: `[code, status, CRC]`.

| status | Meaning |
|--------|---------|
| 0 | fail |
| 1 | success |
| 2 | completed (motion) |
| 3 | endstop / limit stopped (motion) |
| 5 | sync-mode command received |

Some commands return **no data** (e.g. calibrate `0x80`, PDF §4.1: "No data was returned" — the motor restarts into calibration mode). Status codes 2/3/5 only appear on motion-class commands; see [[servo42es/06-bus-control|Bus Control / Sync]] for status 5.

> 🔀 **vs 42D：** status table identical to [[servo42d/01-protocol|42D status]] (0 fail / 1 success / 2 completed / 3 endstop / 5 sync).

## ⚠️ Persistence — the `0x60` SAVE requirement (42ES-only)

**Write-only / config parameters are NOT persisted to flash when you set them.** Per PDF Part 4 Note 3 and §4.5.1:

> "All write-only instructions are not saved immediately after being set; the 60H instruction must be executed to save them all at once."

So the correct flow for any config write is: send the write command(s) → then send **`0x60` (SAVE all)** once → power-cycle to confirm. Without `0x60`, the change is live in RAM but lost on reboot. This applies to anything documented in [[servo42es/04-config-commands|Config Commands]], [[servo42es/07-pid-tuning|PID Tuning]], [[servo42es/09-protection|Protection]], and [[servo42es/10-io-ports|I/O Ports]].

> 🔀 **vs 42D：** **major divergence.** The SERVO42D doc set has **no explicit save command** — its writes persist per-command. On 42ES you must batch-commit with `0x60` or lose every config write on power-down. This callout is surfaced again on every write/config command across the 42ES chapters. See the comparison in [[servo-can-hub|the hub]] and the 42D counterpart [[servo42d/04-config-commands|42D config]].

## Hardware Variants Recap

| Field | SERVO42ES V1.0 | SERVO57ES V1.0 |
|-------|----------------|----------------|
| CAN protocol / firmware | V1.0.1 (identical) | V1.0.1 (identical) |
| Current rating | lower | higher |
| Digital inputs | single `IN` | `IN_1` + `IN_2` + `IN_COM` |
| Supply (VIN) | 20–48V | 20–48V |

I/O detail and limit-switch wiring (TP808 / PM-T45 / PM-T45-P) live in [[servo42es/10-io-ports|I/O Ports]]; specs and run modes in [[servo42es/02-specs-and-modes|Specs & Modes]].
