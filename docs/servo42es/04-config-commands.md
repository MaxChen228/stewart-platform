# Configuration Commands (Write-only Parameters)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/04-config-commands|42D Configuration Commands]]

This chapter covers Part 4.3 of the manual — the **write-only parameter commands** (0x82..0x95, 0x0F). Frame shape, [[servo42es/01-protocol|CRC]] and big-endian rules are identical to [[servo42es/01-protocol|the protocol chapter]]. All config commands respond `DLC=3`: `[code, status, CRC]`, status `0=fail`, `1=success`.

> 🔀 **vs 42D：** 42ES requires an **explicit [[servo42es/12-errors-recovery|0x60 SAVE]]** after ANY 4.3 write — the value lives in RAM only until you save it. 42D's doc set has no such explicit save command (its config writes persist directly). This is the single biggest behavioural divergence; it is stressed on every command below.

> ⚠️ **SAVE REQUIREMENT (manual 4.3 note + 4.5.1)：** *"The settings are not saved immediately after writing the instructions; the 0x60 instruction must be executed to save them all at once."* Plan your config sequence as: write all params → fire **one** [[servo42es/12-errors-recovery|0x60]] → power-cycle to verify. Skipping 0x60 = silent loss on reboot.

> 🔀 **vs 42D：** There is **no PID-set command in 42ES's 4.3 block** (42D exposes 0x96 set-vFOC-PID / 0x97 set-position-PID). vFOC/position tuning on 42ES is handled differently — see [[servo42es/07-pid-tuning|PID tuning]]. Do not look for 0x96/0x97 here.

---

## 4.3.1 — 0x82 Set Working Mode

Downlink `DLC=3`: `[82, mode, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `82H` |
| Byte2 | mode | see table |
| Byte3 | CRC | checksum |

| mode | Encoder | Work mode |
|------|---------|-----------|
| `0x00` | With encoder | Pulse + Pulse Open-Loop FOC |
| `0x01` | With encoder | Pulse + Direction Open-Loop FOC |
| `0x02` | With encoder | Pulse + Pulse Closed-Loop FOC |
| `0x03` | With encoder | Pulse + Direction Closed-Loop FOC (**default**) |
| `0x04` | With encoder | Bus open-loop FOC |
| `0x05` | With encoder | Bus closed-loop FOC |
| `0x10` | No encoder | Pulse + Pulse Open-Loop FOC |
| `0x11` | No encoder | Pulse + Direction Open-Loop FOC |
| `0x14` | No encoder | Bus open-loop FOC |

- **With encoder** (`0x00`-`0x05`): shaft has magnets, drive board reads encoder.
- **No encoder** (`0x10`/`0x11`/`0x14`): shaft has no magnets, drive board can be mounted arbitrarily, encoder value cannot be read.
- Limits (manual note 2): pulse mode max input frequency **300 kHz**; bus mode max speed **3000 RPM**.

**CRC example** — set mode `0x03` (default, Pulse + Direction Closed-Loop FOC) on motor `0x01`:
`01 82 03 CRC`, CRC = `(01 + 82 + 03) & 0xFF` = `0x86` → frame `01 82 03 86`.

> 🔀 **vs 42D：** mode encoding differs. 42D uses a flat `0x00`-`0x05` map (CR_OPEN/CR_CLOSE/CR_vFOC/SR_OPEN/SR_CLOSE/SR_vFOC), default `0x02` CR_vFOC. 42ES splits the same axis into **With-encoder `0x00`-`0x05`** and **No-encoder `0x10`/`0x11`/`0x14`**, default **`0x03`** (Pulse+Direction Closed-Loop FOC). When porting firmware, the mode byte is NOT interchangeable.

> 🔀 **vs 42D：** Save needed — issue [[servo42es/12-errors-recovery|0x60]] after 0x82 or the mode reverts on reboot.

---

## 4.3.2 — 0x83 Set Working Current

Downlink `DLC=4`: `[83, current_hi, current_lo, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `83H` |
| Byte2-3 | current (mA) | uint16, big-endian |
| Byte4 | CRC | checksum |

- **SERVO42ES**: max operating current **3000 mA** (default **1600 mA**).
- **SERVO57ES**: max operating current **5200 mA** (default **3200 mA**).
- Response `DLC=3`: `[83, status, CRC]`, status `0=fail`, `1=success`.

**CRC example** — set 1600 mA (`0x0640`) on motor `0x01`:
`01 83 06 40 CRC`, CRC = `(01 + 83 + 06 + 40) & 0xFF` = `0xCA` → frame `01 83 06 40 CA`.

> 🔀 **vs 42D：** 42D's 0x83 has TWO forms — save form (`DLC=4`) and a no-save form (`DLC=5`, trailing `00`, status `2=success-without-save`). **42ES has only the single `DLC=4` write form**, and even that does not persist until [[servo42es/12-errors-recovery|0x60]]. There is no per-command no-save byte on 42ES.
> 🔀 **vs 42D：** rating/default values: 42D 42-series max 3000 mA, 57D max 5200 mA (defaults not stated as ES does). 42ES default **1600 mA**, 57ES default **3200 mA** — both explicitly documented here.

> ⚠️ As with 42D: do not jump straight to max current; supply fluctuation can damage the driver.

---

## 4.3.3 — 0x84 Set Subdivision

Downlink `DLC=3`: `[84, micstep, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `84H` |
| Byte2 | micstep | see range |
| Byte3 | CRC | checksum |

- Permitted MS values (decimal), **default 16**:
  `0, 2, 4, 8, 16, 32, 64, 128` and `5, 10, 20, 25, 40, 50, 100, 200`.
- **Note：`0` corresponds to 256 subdivisions.** （`0x00` 不是停用，而是最高 256 細分）

**CRC example** — set MS=32 (`0x20`) on motor `0x01`:
`01 84 20 CRC`, CRC = `(01 + 84 + 20) & 0xFF` = `0xA5` → frame `01 84 20 A5`.

> 🔀 **vs 42D：** range differs. 42D states `micstep: 1-256` ("prefer powers of 2"). 42ES enumerates a **fixed discrete set** (`0,2,4,8,16,32,64,128` + `5,10,20,25,40,50,100,200`) and **overloads `0` = 256** (42D would write `256` literally as the value, not `0`). Use the ES discrete table on ES hardware.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x84.

---

## 4.3.4 — 0x85 Set EN Pin Active Level

Downlink `DLC=3`: `[85, level, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `85H` |
| Byte2 | level | `0x00`/`0x01`/`0x02` |
| Byte3 | CRC | checksum |

| level | Meaning |
|-------|---------|
| `0x00` | Low-level enable (L) (**default**) |
| `0x01` | High-level enable (H) |
| `0x02` | Always enabled (Hold) |

- **Note 1：** after a successful setup, pulse signals are received only after a **100 ms delay**.
- **Note 2：** valid for **pulse control mode only**.

**CRC example** — set always-enabled (`0x02`) on motor `0x01`:
`01 85 02 CRC`, CRC = `(01 + 85 + 02) & 0xFF` = `0x88` → frame `01 85 02 88`.

> 🔀 **vs 42D：** value map is identical (`00`=active-low default / `01`=active-high / `02`=Hold). 42ES adds the explicit **100 ms post-setup pulse-acceptance delay** note not present in the 42D twin.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x85.

---

## 4.3.5 — 0x86 Set Motor Rotation Direction

Downlink `DLC=3`: `[86, dir, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `86H` |
| Byte2 | dir | `0x00`=CW / `0x01`=CCW |
| Byte3 | CRC | checksum |

- `dir = 0x00` clockwise (**default**); `dir = 0x01` counterclockwise.
- **Note：** this command **can also change the direction of bus-mode motor operation** （非僅脈衝介面）。

**CRC example** — set CCW (`0x01`) on motor `0x01`:
`01 86 01 CRC`, CRC = `(01 + 86 + 01) & 0xFF` = `0x88` → frame `01 86 01 88`.

> 🔀 **vs 42D：** value map identical (`00`=CW default / `01`=CCW). **Scope differs**: 42D states 0x86 is "Pulse interface only; bus direction is set by motion commands". 42ES explicitly says 0x86 **also flips bus-mode direction** — a real behavioural difference, mind it when reusing motion code.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x86.

---

## 4.3.6 — 0x87 Set Pulse Delay

Downlink `DLC=3`: `[87, delay, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `87H` |
| Byte2 | delay | see table |
| Byte3 | CRC | checksum |

| delay | Value |
|-------|-------|
| `0x00` | 0 ms |
| `0x01` | 4 ms |
| `0x02` | 20 ms (**default**) |
| `0x03` | 40 ms |

**CRC example** — set 20 ms default (`0x02`) on motor `0x01`:
`01 87 02 CRC`, CRC = `(01 + 87 + 02) & 0xFF` = `0x8A` → frame `01 87 02 8A`.

> 🔀 **vs 42D：** 42D's 0x87 is **Set Auto Screen-Off** (`00`=disabled / `01`=enabled, 15 s timeout). 42ES's 0x87 is **Set Pulse Delay** (a 4-level enum). **Same code, completely different function** — do not cross-port 0x87.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x87.

---

## 4.3.7 — 0x8A Set CAN Bit Rate

Downlink `DLC=3`: `[8A, bitRate, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `8AH` |
| Byte2 | bitRate | see table |
| Byte3 | CRC | checksum |

| bitRate | Speed |
|---------|-------|
| `0x00` | 125 K |
| `0x01` | 250 K |
| `0x02` | 500 K (**default**) |
| `0x03` | 1 M |

**CRC example** — set 500 K default (`0x02`) on motor `0x01`:
`01 8A 02 CRC`, CRC = `(01 + 8A + 02) & 0xFF` = `0x8D` → frame `01 8A 02 8D`.

> 🔀 **vs 42D：** value map identical (`00`=125K / `01`=250K / `02`=500K default / `03`=1M).
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x8A. After save + power-cycle the new bit rate takes effect; your bus master must match it.

---

## 4.3.8 — 0x8B Set Slave ID

Downlink `DLC=4`: `[8B, id_hi, id_lo, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `8BH` |
| Byte2-3 | ID | uint16, big-endian |
| Byte4 | CRC | checksum |

- Slave ID range **`0x00`-`0x7FF`**. Note: **`0x00` is the broadcast ID, `0x01` is the default ID**.
- OLED can set `0x01`-`0x10` directly; values `>0x10` must be set via this 0x8B command. See [[servo42es/01-protocol|CAN_ID rules]].

**CRC example** — set ID to `0x0011` (17) via current address `0x01`:
`01 8B 00 11 CRC`, CRC = `(01 + 8B + 00 + 11) & 0xFF` = `0x9D` → frame `01 8B 00 11 9D`.

> 🔀 **vs 42D：** range identical (`0x01`-`0x7FF`, `0x00`=broadcast). Equivalent command.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x8B, then the slave answers on the new ID after reboot.

---

## 4.3.9 — 0x8C Configure Slave Response Method

Downlink `DLC=3`: `[8C, ackMode, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `8CH` |
| Byte2 | ackMode | `0x00`/`0x01`/`0x02` |
| Byte3 | CRC | checksum |

| ackMode | Behaviour |
|---------|-----------|
| `0x00` | No response from the slave device |
| `0x01` | Passive acknowledge — slave does NOT actively initiate data transmission |
| `0x02` | Slave actively responds (**default**) |

**Worked example (manual, position-control mode 1):** host sends
`01 FD 02 80 02 00 00 FA 00 7C` and the slave behaves as:
- **`ackMode=00`** — no information returned.
- **`ackMode=01`** (passive) — slave immediately returns position-control start `01` or fail `00`.
- **`ackMode=02`** (active) — slave immediately returns start `01`/fail `00`, **and** after the motor finishes running (or stops on the limit switch) returns `02` (completed) or `03` (endstop-stopped).

- **Note：** after setting the slave to no-response, query motor running status via code **`F1H`** → see [[servo42es/03-read-commands|0xF1 read run-status]].

**CRC example** — set passive ack (`0x01`) on motor `0x01`:
`01 8C 01 CRC`, CRC = `(01 + 8C + 01) & 0xFF` = `0x8E` → frame `01 8C 01 8E`.

> 🔀 **vs 42D：** **encoding differs.** 42D uses a 2-byte `[8C, XX, YY]` scheme (`DLC=4`): `XX=0`→no response, `XX=1/YY=0`→immediate only, `XX=1/YY=1`→immediate+completion (default). 42ES uses a **single `ackMode` byte** (`DLC=3`): `00`/`01`/`02`. The semantics map (ES `00`≈42D `XX=0`; ES `01`≈42D `XX=1/YY=0`; ES `02`≈42D `XX=1/YY=1` default) but the **wire format is not interchangeable**.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x8C.

---

## 4.3.10 — 0x8F Lock Axis on Bus-Mode Start

Downlink `DLC=3`: `[8F, enable, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `8FH` |
| Byte2 | enable | `0x00`/`0x01` |
| Byte3 | CRC | checksum |

- `enable = 0x00` — shaft is **not** locked when starting in bus mode.
- `enable = 0x01` — shaft **is** locked when starting in bus mode (**default**).

**CRC example** — lock axis on start (`0x01`) on motor `0x01`:
`01 8F 01 CRC`, CRC = `(01 + 8F + 01) & 0xFF` = `0x91` → frame `01 8F 01 91`.

> 🔀 **vs 42D：** **same code, different function.** 42D's 0x8F is **Set Key Lock** (`00`=unlock / `01`=lock — locks the physical OLED buttons). 42ES's 0x8F is **lock-axis-on-bus-start** (holding torque at bus power-up). Do not confuse the two; do not cross-port 0x8F.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x8F.

---

## 4.3.11 — 0x8D Set Group ID

> 📎 Full treatment lives in the dedicated chapter — see [[servo42es/05-motion-commands|group-ID / broadcast motion]] and the hub. Documented here only as a 4.3 write member; **it too requires [[servo42es/12-errors-recovery|0x60]] to persist.**

Downlink `DLC=4`: `[8D, group_hi, group_lo, CRC]` → response `DLC=3` `[8D, status, CRC]`.

A group ID lets one frame drive several motors at once. Example layout (manual, 6 motors): Motors 1-3 share group `0x50`, Motors 4-6 share group `0x51`, every motor keeps broadcast ID `0`. Sending a motion frame addressed to a group/broadcast ID makes **all members move and none respond** (e.g. `50 FD 01 2C 64 00 0C 80 6A` rotates motors 1-3 one full circle).

> 🔀 **vs 42D：** cross-link only — group-ID semantics tracked in [[servo42d/05-motion-commands|42D motion/group chapter]]. Note: **packet/group-addressed commands get NO slave response** on both lines.

---

## 4.3.12 — 0x89 Set Heartbeat Protection Time

> 📎 Heartbeat behaviour (failsafe stop on host silence) is detailed in its own chapter — see [[servo42es/09-protection|heartbeat protection]]. Listed here as a 4.3 write member; **requires [[servo42es/12-errors-recovery|0x60]].**

Downlink `DLC=6`: `[89, t_b1, t_b2, t_b3, t_b4, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `89H` |
| Byte2-5 | times | uint32 ms, big-endian |
| Byte6 | CRC | checksum |

- `times`: protection time in **milliseconds**, **default `0`**. If the motor receives no host instruction within this window it stops urgently.
- **Note：** `times = 0` turns the heartbeat protection function **off**.

> 🔀 **vs 42D：** heartbeat is a 42ES/ES-line feature surfaced in this set; cross-reference [[servo42es/09-protection|heartbeat chapter]] for the full state machine and the [[servo42es/12-errors-recovery|0x60 save]] requirement.

---

## 4.3.13 — 0x98 Set Position-Reach Threshold

Downlink `DLC=5`: `[98, enable, val_hi, val_lo, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `98H` |
| Byte2 | enable | `0x00`/`0x01` |
| Byte3-4 | values | uint16, big-endian |
| Byte5 | CRC | checksum |

- `enable = 0x00` — disable position-reach threshold judgment.
- `enable = 0x01` — enable position-arrival threshold judgment (**default**).
- `values` — position threshold, **default 800**. In bus position-control mode, the *"position operation completed"* message is only returned when the actual position error of the motor is less than this threshold.
- **Note：** maximum value is **65535**, which equals **360 degrees**.

**CRC example** — enable with threshold 800 (`0x0320`) on motor `0x01`:
`01 98 01 03 20 CRC`, CRC = `(01 + 98 + 01 + 03 + 20) & 0xFF` = `0xBD` → frame `01 98 01 03 20 BD`.

> 🔀 **vs 42D：** **different code AND different default.** 42D's equivalent is **`0x95`** (`[95, enable, threshold_hi, threshold_lo, CRC]`, default threshold **200**). 42ES uses code **`0x98`** with default **800**. Both cap at 65535 = 360°, both default-enabled. State both: ES = `0x98`/default 800; 42D = `0x95`/default 200.
> 🔀 **vs 42D：** Save needed — [[servo42es/12-errors-recovery|0x60]] after 0x98.

---

## 4.3.14 — 0x0F Set No-Verification Mode

Downlink `DLC=3`: `[0F, 01, CRC]`

| Byte | Field | Value |
|------|-------|-------|
| Byte1 | code | `0FH` |
| Byte2 | data | `01` |
| Byte3 | CRC | checksum |

- After this, the motor executes a received message **without checking whether the [[servo42es/01-protocol|CRC checksum]] is correct**.
- **Note：** for testing purposes only, **cannot be saved** — the motor re-enters verification mode on every power-on.

**CRC example** — enter no-verify (`0x01`) on motor `0x01`:
`01 0F 01 CRC`, CRC = `(01 + 0F + 01) & 0xFF` = `0x11` → frame `01 0F 01 11`.

> 🔀 **vs 42D：** identical semantics (`[0F, 01]`, test-only, never persisted). This is the **one 4.3 command that 0x60 will NOT save** — by design it always resets to verify-on at power-on. Every *other* 4.3 write requires [[servo42es/12-errors-recovery|0x60]]; 0x0F is the exception.

---

## Cross-references

- Frame format · [[servo42es/01-protocol|CRC]] · big-endian → [[servo42es/01-protocol|01-protocol]]
- [[servo42es/03-read-commands|Read commands (incl. 0xF1 run-status)]]
- [[servo42es/05-motion-commands|Motion / group-ID commands]] · [[servo42es/12-errors-recovery|0x60 SAVE / restore]]
- [[servo42es/07-pid-tuning|PID tuning (no 0x96/0x97 here)]] · [[servo42es/09-protection|Heartbeat protection]]
- 42D twin → [[servo42d/04-config-commands|42D Configuration Commands]] · hub → [[servo-can-hub|CAN 文檔總覽]]
