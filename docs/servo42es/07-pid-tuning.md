# PID Parameter Tuning (CANGAROO upper-computer)

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/07-pid-tuning|PID Parameter Tuning]]

**Warning: MKS motors have factory-tuned PID. The three-loop FOC is calibrated at the factory. Adjust with extreme caution to avoid motor damage.**

> 🔀 **vs 42D：** This is a **CRITICAL DIVERGENCE** chapter. SERVO42D set PID over the CAN bus via [[servo42d/07-pid-tuning|0x96 (vFOC) + 0x97 (CLOSE)]]. **SERVO42ES removes BOTH commands** — there is no set-PID entry anywhere in the 42ES write section (Part 4.3). PID/parameter tuning on 42ES is done **exclusively** through the **CANGAROO** upper-computer software over a **USB-CAN dongle** (Part 12). No CAN frame can write a PID gain.

---

## Three-Loop Control (CAN Bus Closed-Loop FOC Mode)

```
Position loop (10kHz): → target speed
Speed loop    (10kHz): → target current
Torque loop   (20kHz): → PWM → motor
```

The loop rates are **identical to 42D** (torque 20 kHz / speed 10 kHz / position 10 kHz). What changed is **who owns the gains**: on 42ES they are factory-tuned and only the CANGAROO host can touch them — they are **not exposed on the CAN command set** at all. This mode is selected on the OLED as `Mode → 05 CAN Bus Closed-Loop FOC Mode` (see [[servo42es/02-specs-and-modes|work modes]]).

> 🔀 **vs 42D：** Same loop frequencies, but 42D exposed two independent gain sets selectable per-loop over CAN (`vFOC` default Kp/Ki/Kd/Kv = 220/100/270/320, `CLOSE` = 200/80/250/300). **42ES exposes neither set over CAN.** The defaults table from the 42D twin does **not** apply to 42ES — those registers are simply unreachable via bus.

---

## Why there is no CAN-PID command on 42ES

The 42ES protocol write section ([[servo42es/04-config-commands|write/config commands]]) has **no opcode** for PID. Concretely:

| Capability | SERVO42D | SERVO42ES |
|------------|----------|-----------|
| Set vFOC Kp/Ki/Kd/Kv over CAN | `0x96` | **absent** |
| Set CLOSE-loop Kp/Ki/Kd/Kv over CAN | `0x97` | **absent** |
| Read PID over CAN | `0x00` read-config of `0x96`/`0x97` | **absent** |
| Tune PID | CAN frame (any node, on-bus) | CANGAROO host only (USB-CAN) |

Because the opcodes do not exist, **any host-side serial helper that wrapped them is inapplicable on 42ES**:

> 🔀 **vs 42D：** The Stewart firmware's serial commands `K` / `KS` / `KRESET` (apply / save-to-NVS / reset vFOC PID via `0x96`) and the `spid`/`pidn` PID telemetry **cannot function against a 42ES motor** — the underlying `0x96` frame would be rejected (write status `0`, see [[servo42es/01-protocol|protocol]]). On a 42ES-based build those commands must be removed or stubbed; a **USB-CAN dongle + CANGAROO is mandatory** for any gain change.

This also means the ESP32 bridge cannot self-tune or hot-reload PID at boot the way the 42D path could. The firmware implication is documented at the end of this chapter.

---

## CANGAROO workflow (Part 12)

The vendor's reference toolchain is the **"cangaroo"** host computer software driving an **"MKS CANable"** USB-to-CAN module. The exact sequence from Part 12 follows.

### 12.1 Motor parameter configuration (on the motor OLED)

Set these **on the motor itself** before connecting CANGAROO:

| Step | OLED menu | Value |
|------|-----------|-------|
| 1 | Select control mode | `Mode → 05 CAN Bus Closed-Loop FOC Mode` |
| 2 | Set the baud rate | `CanRate → 500K` (default value) |
| 3 | Set the slave ID | `CanID → 01` (default value) |

CanRate 500K and CanID 01 are the same defaults this project uses elsewhere ([[servo42es/01-protocol|CAN_ID / baud]]). CanID `01–10` are settable on the OLED; `>0x10` requires the `0x8B` command (see [[servo42es/04-config-commands|set CAN ID]]).

### 12.2 cangaroo parameter configuration (host side)

1. Double-click `cangaroo.exe` to run the host computer software.
2. In the Cangaroo window, select the menu `Measurement → Start Measurement` (shortcut **F5**).
3. In the pop-up **Measurement Setup** window, expand `Network 1 → Interfaces` and click **`candle0`**.
4. Use the default parameters **without making any changes**, and click **OK**. The interface details confirm the dongle:

   | Field | Value |
   |-------|-------|
   | Driver | CandleAPI |
   | Interface | candle0 |
   | Bitrate | 500000 |
   | Sample Point | 87.5% |
   | Options | (all unchecked — Listen-only / One-Shot / Loop-Back / Triple-Sampling / Auto-Restart off) |

5. Configuration complete. The Log pane shows `Starting measurement` then `Listening on interface: candle0`.
6. Select `File → Save Workspace` (Ctrl+S), choose the save path and name, and save the configuration. The Log shows `Saved workspace settings to …` and the title bar shows e.g. `cangaroo - SERVO57D.cangaroo`.

> 📝 **筆記：** Bitrate must read **500000** (= the motor's `CanRate 500K`) and Sample Point **87.5%**. The dongle enumerates as `candle0` (CandleAPI driver). Once a workspace is saved you reopen it via `File → Open Workspace` (Ctrl+O) instead of re-running setup. The window title in the vendor screenshots reads `SERVO57D` — the **same CANGAROO + candle0 flow serves both [[servo42es/02-specs-and-modes|42ES and 57ES]]** and is shared with the SERVO57D toolchain.

> 🔀 **vs 42D：** 42D required no host-computer software in its CAN PID path — you sent `0x96`/`0x97` frames directly from any bus master (the ESP32 itself). 42ES **adds a mandatory PC-based tool** (cangaroo) and a **dedicated USB-CAN adapter** (MKS CANable / candle0) into the tuning loop.

### 12.3 Example: reading the encoder value (verify the link)

The Part 12 worked example reads the encoder to confirm the dongle ↔ motor link is alive. It uses the **`0x31` read-encoder** command (documented in full at [[servo42es/03-read-commands|read commands]]).

**Send (Tx)** — DLC=2:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x31` |
| 2 | CRC | `0x32` |

**Return (Rx)** — DLC=8:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x31` |
| 2 | encoder[47:40] | `0x00` |
| 3 | encoder[39:32] | `0x00` |
| 4 | encoder[31:24] | `0x00` |
| 5 | encoder[23:16] | `0x00` |
| 6 | encoder[15:8] | `0x00` |
| 7 | encoder[7:0] | `0x03` |
| 8 | CRC | `0x35` |

In the CANGAROO trace (PDF §12.3) this appears as CAN ID `0x001`, Tx `DLC 2 / 31 32`, then Rx `DLC 8 / 31 00 00 00 00 00 03 35`. The 6-byte encoder field `00 00 00 00 00 03` decodes to **`3`**.

#### Worked CRC

CRC = CHECKSUM 8-bit = `(CAN_ID + all data bytes except CRC) & 0xFF`, **byte-identical to [[servo42d/01-protocol|SERVO42D]]**, big-endian.

- **Tx:** `(0x01 + 0x31) & 0xFF = 0x32` ✓ (matches the `32` byte sent)
- **Rx:** `(0x01 + 0x31 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00 + 0x03) & 0xFF = 0x35` ✓ (the `0x03` data byte is part of the sum)

> 📝 **筆記：** 手冊 §12.3 原例：`01 31 32`(Tx) / `01 31 00 00 00 00 00 03 35`(Rx)。前導 `01` 是 CAN_ID 不計入 data；Rx data field（DLC=8）= `31 00 00 00 00 00 03 35`（code + 6-byte encoder + CRC），encoder 值 = `3`。CRC 只算 CAN_ID + data、不含自身（[[servo42es/01-protocol|CRC rule]]）。

> 🔀 **vs 42D：** `0x31` 讀編碼器指令與 CRC 算法與 42D **完全一致**，僅讀取媒介不同（CANGAROO trace pane vs. 直接 bus master）。

---

## Persistence — SAVE is required (0x60)

Any parameter changed through CANGAROO that maps to a **write-only register is NOT persisted** to the motor until the **SAVE command `0x60`** is issued (Part 4 note 3 + section 4.5.1). Changes made live in CANGAROO are lost on power-cycle unless saved.

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x60` |
| 2 | CRC | — |

CRC for a broadcast-less save to motor `0x01`: `(0x01 + 0x60) & 0xFF = 0x61` → frame `01 60 61`.

> 🔀 **vs 42D：** SERVO42D's doc set has **no explicit save command** — its CAN-written params (including `0x96`/`0x97` PID) behaved as committed on write. On 42ES you must **deliberately `0x60`-save**; saving the CANGAROO *workspace* (`.cangaroo` file, §12.2 step 6) only saves the **host UI session**, NOT the motor's parameters. Two different "saves" — do not confuse them. See [[servo42es/04-config-commands|0x60 SAVE]].

---

## Firmware implication (this project)

For an ESP32 bridge migrated from 42D to 42ES:

1. **Drop the CAN-PID path entirely.** Remove `0x96`/`0x97` send code and the `K` / `KS` / `KRESET` serial handlers — they target opcodes the 42ES does not implement (write status would be `0`). PID telemetry (`spid`/`pidn`) has no source on 42ES.
2. **Tuning moves off-board.** Gains are set once, manually, via CANGAROO + MKS CANable, then `0x60`-saved into the motor's own NVS. The ESP32 never re-pushes PID at boot.
3. **The Stewart "pure-P working point" (`[1024,0,0,0]`) is not transferable.** That was a `0x96` vFOC override; on 42ES the closest equivalent must be dialed in inside CANGAROO and committed with `0x60`. Treat factory FOC as the baseline and change it only with the dongle attached.
4. **Bring-up check:** before any motion, run the §12.3 `0x31` read-encoder from CANGAROO to confirm CanID/baud/dongle wiring, exactly as the manual demonstrates.

> 🔀 **vs 42D：** The whole "firmware owns and re-applies PID at boot" model from 42D is gone. On 42ES the **motor owns its tuning** (set by CANGAROO, persisted by `0x60`); the ESP32 is reduced to a pure motion coordinator that never writes gains.

---

## See also

- [[servo42es/00-index|42ES index]] · [[servo-can-hub|CAN hub / MOC]]
- [[servo42es/01-protocol|01 Protocol & CRC]] · [[servo42es/02-specs-and-modes|02 Specs & modes (FOC mode 05)]]
- [[servo42es/03-read-commands|03 Read commands (0x31 encoder)]] · [[servo42es/04-config-commands|04 Config & 0x60 SAVE / 0x8B CAN ID]]
- 42D twin: [[servo42d/07-pid-tuning|07 PID Tuning (0x96/0x97 over CAN)]]
