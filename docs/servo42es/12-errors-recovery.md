# Error Messages & Recovery (SERVO42ES/57ES)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/12-errors-recovery|Error Messages & Recovery]]

Covers the SERVO42ES/57ES fault-decoding and recovery surface: the **LED status/error table**, the **0x60 SAVE-parameters** command, **0x3F restore-factory**, **0x41 reset-and-restart**, and a distilled precautions/FAQ section. Frame shape, addressing and CRC follow the shared rules in [[servo42es/01-protocol|01-protocol]]; this product line spans both **SERVO42ES V1.0** and **SERVO57ES V1.0** (same firmware/protocol; they differ only in current rating and I/O count — 42ES single `IN`, 57ES `IN_1`+`IN_2`+`IN_COM`).

---

## 1. LED Status / Error Decoding Table

The 42ES has **no OLED** in the fault path documented here — diagnostics are read off the **status indicator LED** as a *green-baseline + N-red-blink* code (manual §1.6). One green blink is always present; the number of red blinks selects the fault.

| LED pattern | Meaning | Action |
|-------------|---------|--------|
| **Green** stays on | Motor running | Normal (closed-loop active) |
| **Green** flash | Motor stopped | Normal idle / disabled |
| 1 Green + **1 Red** | Over Current | Reduce current / clear stall; check load |
| 1 Green + **2 Red** | Open-Phase | Check `A+ A- B+ B-` phase wiring & connector |
| 1 Green + **3 Red** | Supply Voltage High | Lower supply (within 20–48 V) |
| 1 Green + **4 Red** | Supply Voltage Low | Raise supply (within 20–48 V) |
| 1 Green + **5 Red** | Position Error | Tracking/位置超差 — see note below |
| 1 Green + **6 Red** | Encoder Error | Check magnet gap/alignment, recalibrate ([[servo42es/01-protocol|0x80]]) |

> **Tip (manual §1.6):** make sure the motor phase line is connected correctly, otherwise after receiving a pulse you will get a tracking-error alarm shown as **5 Red + 1 Green** (the *Position Error* pattern). 相線接錯是最常見的 5-red 誤報根因。

> 🔀 **vs 42D：** the 42D twin decodes faults from **OLED text strings** (`Magnet Loss!`, `Phase Line Error!`, `Wrong Protect!`, `Wrong2...`, `Low Voltage Error!`, …). The 42ES uses a **1 Green + N Red blink table** instead — there is no equivalent text screen. The fault *concepts* overlap but the mapping is not 1:1: e.g. 42D `Phase Line Error!` ≈ 42ES **2 Red Open-Phase**; 42D `Low Voltage Error!` ≈ 42ES **4 Red**; 42D `Wrong2...` (out-of-tolerance) ≈ 42ES **5 Red Position Error**; 42D `Encoder Error!` ≈ 42ES **6 Red**. The 42ES additionally separates **3 Red = Supply Voltage *High*** (over-voltage), which has no dedicated 42D OLED string.

### Fault-pattern notes

- **1 Red — Over Current:** stall or excessive load. Release the shaft, lower running/holding current. Related protection behaviour: [[servo42es/09-protection|09-protection]].
- **5 Red — Position Error:** the closed loop could not track the commanded position within tolerance (most often a mis-wired phase, see tip above; otherwise mechanical jam or under-current). The error-protection / out-of-tolerance window itself is configured in [[servo42es/09-protection|09-protection]].
- **6 Red — Encoder Error:** magnet missing/too close/too far/off-center. Restore the 0.5–3.0 mm gap and ±0.3 mm centering (§13), then recalibrate.

---

## 2. 0x60 — SAVE Parameters (42ES-ONLY)

> 🔀 **vs 42D：** **this command does not exist in the SERVO42D doc set.** On the 42D, configuration writes are treated as persisted by their own command path; the 42ES makes persistence **explicit** — *write-only parameters are NOT stored to flash until you send 0x60* (manual Part 4 note 3 + §4.5.1). This is the single most important behavioural difference between the two firmwares: every config write described in [[servo42es/04-config-commands|04-config]], [[servo42es/06-bus-control|06-bus-control]], [[servo42es/07-pid-tuning|07-pid]], [[servo42es/09-protection|09-protection]] and [[servo42es/10-io-ports|10-io-ports]] is **volatile until a 0x60 follows it.**

> ⚠️ **Workflow rule:** `write config command(s)` → **`0x60` SAVE** → (optionally) power-cycle. Skipping the SAVE means every changed parameter reverts on the next reboot.

`0x60` saves **all** currently-modified parameters to internal storage in one shot (§4.5.1). The downlink carries a **control word** of `0x01`.

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` (motor address) |
| 1 | code | `0x60` |
| 2 | control word | `0x01` |
| 3 | CRC | checksum |

DLC = 3 (data bytes `[code, control word, CRC]`).

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` |
| 1 | code | `0x60` |
| 2 | Save status | `0x00` fail / `0x01` success |
| 3 | CRC | checksum |

- `status = 0` → **Save failed**
- `status = 1` → **Saved successfully**

### Worked CRC — SAVE on address 0x01

Downlink `[ID=0x01][0x60][0x01][CRC]`
CRC = (0x01 + 0x60 + 0x01) & 0xFF = **0x62**
→ full frame: `01 60 01 62`

Success uplink `[ID=0x01][0x60][status=0x01][CRC]`
CRC = (0x01 + 0x60 + 0x01) & 0xFF = **0x62**
→ `01 60 01 62` (here byte2 is *status*, not control word — same numeric value, so same CRC).

CRC is the shared 8-bit checksum defined in [[servo42es/01-protocol|01-protocol]] — byte-identical to [[servo42d/01-protocol|SERVO42D]]. Data is big-endian.

---

## 3. 0x3F — Restore Factory Settings

Resets all parameters to factory defaults; the board **auto-restarts** afterward (manual §4.5.2 Note 1). No recalibration is implied by the doc, but a restore wipes any user calibration/PID/IO config, so re-verify before driving load.

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` |
| 1 | code | `0x3F` |
| 2 | CRC | checksum |

DLC = 2 (data bytes `[code, CRC]`).

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` |
| 1 | code | `0x3F` |
| 2 | Restored status | `0x00` fail / `0x01` success |
| 3 | CRC | checksum |

- `status = 0` → **Recovery failed**
- `status = 1` → **Recovery successful**

### Worked CRC — restore factory on address 0x01

Downlink `[ID=0x01][0x3F][CRC]`
CRC = (0x01 + 0x3F) & 0xFF = **0x40**
→ `01 3F 40`

Success uplink `[ID=0x01][0x3F][0x01][CRC]`
CRC = (0x01 + 0x3F + 0x01) & 0xFF = **0x41**
→ `01 3F 01 41`

> 🔀 **vs 42D：** code and semantics match the 42D `0x3F` (same `[3F, CRC]` downlink, same `[3F, status, CRC]` uplink, auto-restart on success). The difference is *downstream*: on 42ES a factory restore also discards anything you had committed via **0x60**, since 0x60 is the only persistence path. The 42D doc additionally lists a hardware fallback (hold **Next** at power-on until LED flashes); the 42ES has no physical Next/OLED button path documented here — recovery is via the LED-coded CAN command.

---

## 4. 0x41 — Reset and Restart the Motor

Soft-resets and restarts the motor. **Does NOT modify configuration parameters** (manual §4.5.3 Note) — use this to recover from a transient fault (e.g. a cleared over-current/position alarm) without touching saved settings.

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` |
| 1 | code | `0x41` |
| 2 | CRC | checksum |

DLC = 2 (data bytes `[code, CRC]`).

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 0 | CAN_ID | `0x01` |
| 1 | code | `0x41` |
| 2 | status | `0x00` fail / `0x01` success |
| 3 | CRC | checksum |

- `status = 0` → **Reset failed**
- `status = 1` → **Reset successful**

### Worked CRC — reset on address 0x01

Downlink `[ID=0x01][0x41][CRC]`
CRC = (0x01 + 0x41) & 0xFF = **0x42**
→ `01 41 42`

Success uplink `[ID=0x01][0x41][0x01][CRC]`
CRC = (0x01 + 0x41 + 0x01) & 0xFF = **0x43**
→ `01 41 01 43`

> 🔀 **vs 42D：** identical to the 42D `0x41` (reset/restart only, no config change, `[41, CRC]` ↔ `[41, status, CRC]`). No behavioural difference; persistence still lives in 0x60 on 42ES only.

---

## 5. Recovery Decision Table

| Situation | Command | Touches config? | Restarts? |
|-----------|---------|-----------------|-----------|
| Persist your edits to flash | **0x60** (`control=0x01`) | writes flash | no |
| Clear a transient fault, keep settings | **0x41** | no | yes |
| Wipe everything back to defaults | **0x3F** | resets all | yes (auto) |

> 🔀 **vs 42D：** the same 0x41/0x3F pair exists on 42D, but the **0x60 row is 42ES-only** — on 42D there is no "commit to flash" step to forget.

---

## 6. Precautions (manual §13.1)

1. **Supply voltage: 20 V – 48 V.**
   > 🔀 **vs 42D：** the 42D precautions list a **12 V – 24 V** input range. The 42ES/57ES range is **20 V – 48 V** — do not assume parity; over/under-voltage faults (LED 3 Red / 4 Red) key off this window.
2. **Do not hot-plug** power or signal cables while powered — it may damage the driver board.
3. The **`A+ A- B+ B-` phase lines must correspond**; mis-pairing causes an error (manifests as the **5 Red Position Error / tracking-error** alarm — see §1).
4. Prefer Makerbase matching motors. For a **custom motor in closed-loop**, all of the following must hold:
   1. Motor step distance = **1.8°**.
   2. Motor internal resistance **< 10 Ω**.
   3. A **radial magnet** can be mounted on the motor back (recommended **φ6.00 mm, height 2.5 mm**).
   4. Magnet ↔ encoder-chip gap **0.5–3.0 mm**, kept parallel (smaller gap = better, lower angle error); magnet center aligned to the sensing center within **±0.3 mm** or absolute accuracy degrades badly.
5. **After installing the magnet, run a calibration** to compensate installation deviation — refer to §4.1 (calibration `0x80`, see [[servo42es/01-protocol|01-protocol]]). 6 Red (Encoder Error) is the LED you will chase if this step is skipped.

> 🔀 **vs 42D：** magnet geometry (φ6.00 mm / 2.5 mm, 0.5–3.0 mm gap, ±0.3 mm centering), 1.8° step and <10 Ω resistance match the 42D twin exactly. Only the **supply window differs (20–48 V vs 12–24 V)**.

## 7. FAQ (manual §13.2)

Manual §13.2 ships a **Frequently Asked Questions table (Serial No. 1–12)** that is **blank/unpopulated** in the V1.0.1 PDF — no question/solution rows are filled. There is no FAQ content to transcribe. For real troubleshooting, work from the LED table (§1) and the recovery commands (§2–§5) above, then escalate via the contacts below.

## 8. Contact (manual Part 14)

- AliExpress store: <https://makerbase.aliexpress.com/>
- YouTube: <https://www.youtube.com/channel/UC2i5I1tcOXRJ2ZJiRxwpCUQ>
- GitHub: <https://github.com/makerbase-motor>

---

### See also

- [[servo42es/01-protocol|01-protocol]] — frame shape, CAN_ID addressing, CRC checksum, calibration `0x80`
- [[servo42es/04-config-commands|04-config-commands]] — writes that require a following **0x60** SAVE
- [[servo42es/07-pid-tuning|07-pid-tuning]] — PID writes (volatile until 0x60)
- [[servo42es/09-protection|09-protection]] — over-current / position-error / out-of-tolerance config behind the LED faults
- [[servo42es/10-io-ports|10-io-ports]] — 42ES single `IN` vs 57ES `IN_1`/`IN_2`/`IN_COM`
- [[servo42d/12-errors-recovery|SERVO42D · Error Messages & Recovery]] — the OLED-based twin of this chapter
