# Multi-Motor Synchronous Motion

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/11-multi-motor-sync|Multi-Motor Synchronous Motion]]

Synchronous motion = starting several motors on one bus moving **simultaneously**. The 42ES manual (Part 11) names three methods. The first two share **one identical frame to multiple targets**; the third **buffers per-motor commands** and fires them all on a trigger.

| Method | Mechanism | Different cmd per motor? | Motor count |
|--------|-----------|-------------------------|-------------|
| 1 — Broadcast | Send to CAN ID `0x00` | No (all run same frame) | All on bus |
| 2 — Grouping | Send to a group CAN ID (`0x8D`) | No (group runs same frame) | One group |
| 3 — Sync flag | Buffer with `0x4A`, fire with `0x4B` | **Yes** | Unlimited |

All three use the standard frame `[CAN_ID][DLC][Byte1=code][data…][CRC]` with the 8-bit [[servo42es/01-protocol|CHECKSUM CRC]] = `(CAN_ID + all data bytes) & 0xFF`, big-endian data. CRC is **byte-identical to SERVO42D**.

---

## Method 1: Broadcast (CAN ID 0x00)

Send any motion command to CAN ID `0x00` → every motor on the bus executes that exact frame. **Slaves do NOT respond** (broadcast = silent). Note the CRC is still computed over `CAN_ID=0x00`, i.e. it equals just `(sum of data bytes) & 0xFF`.

### Worked example — all motors relative-move 3200 pulses

Manual 11.1 sends an [[servo42es/05-motion-commands|0xFD relative-position]] frame to `0x00`:

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | broadcast | `0x00` |
| DLC | length | `0x08` |
| Byte1 | code (0xFD rel-pos) | `0xFD` |
| Byte2 | dir + speed_hi | `0x01` |
| Byte3 | speed_lo | `0x2C` |
| Byte4 | acceleration | `0x02` |
| Byte5–7 | pulses (24-bit) | `0x00 0x0C 0x80` |
| Byte8 | CRC | `0xB8` |

`pulses = 0x000C80 = 3200`. CRC = `(00+FD+01+2C+02+00+0C+80) & 0xFF = 0xB8` ✅.

Frame: `00 FD 01 2C 02 00 0C 80 B8` → all motors move 3200 pulses, no acks.

> 🔀 **vs 42D：** 42D 的 broadcast 範例用 `00 FD 01 2C 64 00 0C 80 B8`（Byte3 speed_lo=`0x64`，CRC 同為 `0xB8`）；42ES 範例 speed_lo=`0x2C`。同樣是 broadcast 機制、同 silent 行為，僅範例速度參數不同。

---

## Method 2: Group CAN ID

Assign motors to a shared group address with `0x8D`, then send a single motion frame to that group address → all members of the group execute it. **Slaves do NOT respond** when addressed by a group/packet ID.

### 0x8D — Set Group ID (§4.3.11)

Downlink (PC → SERVOxxES):

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | motor slave addr | e.g. `0x01` |
| DLC | length | `0x04` |
| Byte1 | code | `0x8D` |
| Byte2–3 | Group ID (16-bit) | `id_hi id_lo` |
| Byte4 | CRC | checksum |

Uplink (PC ← SERVOxxES):

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | motor slave addr | `0x01` |
| DLC | length | `0x03` |
| Byte1 | code | `0x8D` |
| Byte2 | setting status | `0x00` fail / `0x01` success |
| Byte3 | CRC | checksum |

- Group ID range follows the [[servo42es/01-protocol|CAN_ID]] space (`0x00`–`0x7FF`); the §4.3.11 example uses `0x50`/`0x51`.
- The `0x8D` set is addressed to the **individual slave** (so it DOES ack), one slave at a time, mapping that slave into the group.

> ⚠️ **42ES SAVE caveat：** `0x8D` is a write/config command. Like all 42ES write-only params, the group ID is **not persisted until you issue the SAVE command [[servo42es/04-config-commands|0x60]]** (Part 4 note 3 + §4.5.1). Set group → `0x60` to make it survive power-cycle.

> 🔀 **vs 42D：** SERVO42D has **no explicit SAVE command** in its doc set — `0x8D` on 42D is assumed persisted on write. On 42ES you MUST follow `0x8D` with `0x60` or the grouping is lost on reboot.

Example group map for 6 motors (§4.3.11):

| Motor | Broadcast ID | Slave ID | Group ID |
|-------|-------------|----------|----------|
| 1 | 0 | 1 | 0x50 |
| 2 | 0 | 2 | 0x50 |
| 3 | 0 | 3 | 0x50 |
| 4 | 0 | 4 | 0x51 |
| 5 | 0 | 5 | 0x51 |
| 6 | 0 | 6 | 0x51 |

§4.3.11 then shows the same `0xFD` "rotate 1 full circle" (3200 pulses, speed_lo=`0x64`) sent to four addresses:

| Target | Frame | CRC math | Result |
|--------|-------|----------|--------|
| Motor 1 (slave `0x01`) | `01 FD 01 2C 64 00 0C 80 1B` | `(01+FD+01+2C+64+00+0C+80)&FF=1B` | Motor 1 only |
| All (broadcast `0x00`) | `00 FD 01 2C 64 00 0C 80 1A` | `…=1A` | Motors 1–6 |
| Group `0x50` | `50 FD 01 2C 64 00 0C 80 6A` | `…=6A` | Motors 1–3 |
| Group `0x51` | `51 FD 01 2C 64 00 0C 80 6B` | `…=6B` | Motors 4–6 |

### Worked example — group 0x50 relative-move 3200 pulses (11.2)

Part 11.2 sends the 11.1-style frame (speed_lo=`0x2C`) to group `0x50`:

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | group addr | `0x50` |
| DLC | length | `0x08` |
| Byte1 | code (0xFD) | `0xFD` |
| Byte2–3 | dir+speed | `0x01 0x2C` |
| Byte4 | acc | `0x02` |
| Byte5–7 | pulses=3200 | `0x00 0x0C 0x80` |
| Byte8 | CRC | `0x08` |

CRC = `(50+FD+01+2C+02+00+0C+80) & 0xFF = 0x08` ✅. Frame: `50 FD 01 2C 02 00 0C 80 08` → motors 1–3 run, no acks.

---

## Method 3: Synchronization Flag

With the sync flag enabled, a motor **does not execute a received motion command immediately** — it buffers it and waits for the broadcast sync-trigger before starting. This lets you preload **different** commands into different motors, then start them all on one trigger. Motor count is unlimited.

> Manual 11.3 lists the affected motion codes explicitly: **F4 / F5 / F6 / FD / FE** are deferred while the flag is set. See [[servo42es/05-motion-commands|motion commands]].

### 0x4A — Enable/Disable Sync Flag (§11.3.1)

Downlink (PC → SERVOxxES):

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | slave / `0x00` broadcast | e.g. `0x01` |
| DLC | length | `0x03` |
| Byte1 | code | `0x4A` |
| Byte2 | enable | `0x00` disable (default) / `0x01` enable |
| Byte3 | CRC | checksum |

Uplink (PC ← SERVOxxES):

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | slave | `0x01` |
| DLC | length | `0x03` |
| Byte1 | code | `0x4A` |
| Byte2 | status (uint8_t) | `0x00` fail / `0x01` success |
| Byte3 | CRC | checksum |

The flag may be set per-motor or broadcast to all (`CAN_ID=0x00`, then no ack).

> ⚠️ **42ES SAVE caveat：** `0x4A` is a write/config command → not persisted until [[servo42es/04-config-commands|0x60 SAVE]]. The 11.3.3 procedure shows the optional `0x60` step right after `0x4A`.

> 🔀 **vs 42D：** 42D documents `0x4A` identically (DLC=3, `00`=disable default / `01`=enable, individual-or-broadcast) but has **no SAVE step** — the flag is set transiently. On 42ES add `0x60` if you want the flag retained.

### 0x4B — Sync Trigger / Synchronous Execution (§11.3.2)

Must be sent on the **broadcast** ID so every motor receives it.

Downlink (PC → SERVOxxES):

| Byte | Field | Value |
|------|-------|-------|
| CAN_ID | **broadcast** | `0x00` |
| DLC | length | `0x02` |
| Byte1 | code | `0x4B` |
| Byte2 | CRC | checksum |

**Returned data: none.** All buffered motors begin moving at once.

> **Reliability (11.3.2):** on a heavily loaded bus some motors may miss the trigger due to interference. Consider sending the sync-execution command **repeatedly at ~1 ms intervals**.

> 🔀 **vs 42D：** identical on both lines — `0x4B`, broadcast-only (`CAN_ID=0x00`), DLC=2, no response, same ~1 ms repeat advice.

### Worked Procedure — 2 motors, different commands (§11.3.3)

Two motors, ID `0x01` and `0x02`. Steps 1–3 are skippable if already configured.

| # | Step | Frame | CRC math |
|---|------|-------|----------|
| 1 | Broadcast set mode = SR_vFOC ([[servo42es/04-config-commands|0x82]]) | `00 82 05 87` | `(00+82+05)&FF=87` |
| 2 | Broadcast enable sync (`0x4A 01`) | `00 4A 01 4B` | `(00+4A+01)&FF=4B` |
| 3 | Broadcast SAVE ([[servo42es/04-config-commands|0x60]]) | `00 60 01 62` | see note ⚠️ |
| 4 | Motor 1: [[servo42es/05-motion-commands|0xF6 speed]] fwd 640 RPM, acc 2 | `01 F6 02 80 02 7B` | `(01+F6+02+80+02)&FF=7B` |
| 5 | Motor 2: [[servo42es/05-motion-commands|0xFD rel-pos]] rev 100 RPM, acc 2, 320000 pulses | `02 FD 80 64 02 04 E2 00 CB` | `(02+FD+80+64+02+04+E2+00)&FF=CB` |
| 6 | Broadcast sync trigger (`0x4B`) | `00 4B 4B` | `(00+4B)&FF=4B` |

Decoding step 5: `0xFD` byte2 `0x80` = direction bit (reverse) + speed_hi; byte3 `0x64` = speed_lo → speed `0x064` = 100 RPM; byte4 `0x02` = acceleration; bytes5–7 `0x04 0xE2 0x00` = pulses `0x04E200 = 320000`. CRC = `0xCB` ✅.

After step 6 both motors start together: Motor 1 spins continuously at 640 RPM (speed mode), Motor 2 runs the finite 320000-pulse relative move in reverse. Steps 4–5, while the flag is set, are buffered — the manual's flowchart shows each motion frame acked then deferred until the `0x4B` arrives.

> 🔀 **vs 42D：** 42D's twin example uses **2 motors but with `status=5` (sync-received) acks called out** and a simpler 2-motor speed+position pair (`01 F6 02 80 02 7B` / `02 FD 80 64 02 04 E2 00 CB`). The 42ES frames for steps 4/5/6 are **byte-identical** to the 42D worked example; the difference is 42ES inserts the explicit **SAVE step 3 (`00 60 01 62`)** and explicitly skips/configures the bus mode in step 1 (`00 82 05 87`), neither of which the 42D twin carries. The motion-byte payloads match exactly.

> ⚠️ **Step 3 CRC discrepancy (PDF as-printed)：** the manual prints the SAVE frame as `00 60 01 62`, but the documented checksum rule gives `(0x00+0x60+0x01)&0xFF = 0x61`, not `0x62`. Treat `0x61` as the rule-correct CRC; `0x62` is a probable manual typo. (Flagged for the hub comparison matrix.)

---

## Quick Reference

| Code | Name | DLC | CAN_ID | Acks? | SAVE needed? |
|------|------|-----|--------|-------|--------------|
| 0x8D | Set group ID | 4 | slave addr | yes (status 0/1) | **yes (0x60)** |
| 0x4A | Enable/disable sync flag | 3 | slave or `0x00` | yes if addressed individually | **yes (0x60)** |
| 0x4B | Sync trigger | 2 | `0x00` only | **no** | n/a |
| — | Broadcast motion | per cmd | `0x00` | **no** | n/a |
| — | Group motion | per cmd | group addr | **no** | n/a |

Related: [[servo42es/01-protocol|frame & CRC]] · [[servo42es/04-config-commands|0x82 mode / 0x60 SAVE]] · [[servo42es/05-motion-commands|F4/F5/F6/FD/FE motion]] · [[servo42d/11-multi-motor-sync|42D twin]] · [[servo-can-hub|hub]].
