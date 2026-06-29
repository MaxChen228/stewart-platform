# IO Port Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/10-io-ports|IO Port Commands]]

Part 5 (IO Port Operation Instructions) covers reading the optocoupler input/output
states (`0x34`), writing the output ports (`0x36`), and mapping the PEND port to a
frequency-division output (`0x9F`). The companion En-pin active level (`0x85`) and
motor direction (`0x86`) live in Part 4 (page 30) and are summarised here because they
govern how the IN/EN input pins are interpreted.

All frames follow the shared shape — downlink `[CAN_ID][DLC][code][data…][CRC]`, uplink
same — and CRC = `(CAN_ID + all data bytes) & 0xFF`, byte-identical to SERVO42D. See
[[servo42es/01-protocol|frame format & CRC]] for the full rules; read commands index in
[[servo42es/03-read-commands|read commands]].

> 🔀 **vs 42D：** 42ES 的 IO 埠是 **IN/EN 輸入 + ALM/PEND 輸出**（光耦），而 42D 的 IO 埠在文檔中以 **IN_1/IN_2/OUT_1/OUT_2** 命名。讀/寫埠碼相同（`0x34`/`0x36`），但**脈衝分頻碼不同**：42ES 用 `0x9F`、42D 用 `0x99`（42D 的 `0x9F` 另作「Configure IN_1 mode」，42ES 無此指令）。位元語意亦不同 — 詳見各指令下方對照。

## Port pin map (page 9)

| PIN | I/O | Function | Electrical |
|-----|-----|----------|------------|
| STP± (CW±) | Input | Pulse signal (or CW pulse) | Optocoupler, falling-edge trigger; one step on high→low; +3.3–28 V; max 400 kHz, pulse ≥2.5 µs |
| DIR± (CCW±) | Input | Direction signal (or CCW pulse) | Single-pulse (Pulse+Dir) or double-pulse (CW+CCW), selected by command |
| EN± | Input | Enable signal | Optocoupler, **low-level effective** (default); when inactive the motor is released while clearing the alarm; +3.3–28 V |
| ALM± | Output | Alarm signal | Optocoupler turns **on** when an alarm occurs; max +35 V, max 50 mA |
| PEND± | Output | In-Position signal | Optocoupler turns **on** when the drive has finished a given pulse; max +35 V, max 50 mA |
| A±, B± | Output | Motor phase lines | Wrong phase sequence → motor alarm |
| VDC, GND | Input | Drive power supply | Operating voltage **+20–48 V** |

> 🔀 **vs 42D：** 42ES/57ES 標稱輸入電壓 **+20–48 V**；输出埠是 **ALM/PEND**（具體功能語意命名），而非 42D 的通用 OUT_1/OUT_2。

### 42ES 單 IN vs 57ES IN_1/IN_2/IN_COM

The product line shares one firmware/protocol but differs in I/O count:

- **SERVO42ES** 只有 **單一 IN** 輸入（無 IN_2）。在 `0x34` 回覆中 bit1 不對應實體 IN_2，而是 **En 埠狀態**（見下）。
- **SERVO57ES** 提供 **IN_1 + IN_2 + IN_COM**（共陰/共陽公共端），兩路獨立輸入皆映射到 `0x34` 的 bit0 / bit1。

> 🔀 **vs 42D：** 這條 42ES↔57ES 的單 IN / 雙 IN 差異是 42ES 產品線內部差異；42D 文檔一律以 IN_1/IN_2 雙輸入描述，無 IN_COM 公共端的拆分說明。

---

## 0x34 — Read IO Port Status

Read the current optocoupler port states.

**Downlink** DLC=2:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `34` |
| 2 | CRC | checksum |

**Uplink** DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `34` |
| 2 | status | port-status bitfield |
| 3 | CRC | checksum |

### Status bitfield (page 42)

| Bit | Port | Meaning |
|-----|------|---------|
| bit7–bit4 | Undefined | — |
| bit3 | ALM | 1 = no alarm, 0 = alarmed |
| bit2 | PEND | 1 = in place, 0 = not in place |
| bit1 | IN_2 | 57ES second input. **42ES：此位為 En 埠狀態（無實體 IN_2）** |
| bit0 | IN_1 | first input state |

> 🔀 **vs 42D：** 42D 的 `0x34` 回覆 bit0=IN_1, bit1=IN_2, **bit2=OUT_1, bit3=OUT_2**（通用輸出位），且文檔提到 `0x9E` 限位重映射後 bit0=En state / bit1=Dir state。42ES 的 bit2/bit3 改為 **PEND/ALM**（功能化命名），且 42ES 明載 **bit1 對應 En 埠**（因無 IN_2），無 `0x9E` 重映射說明。

### Example — read IO on motor 0x01

Downlink: `01 34 35`
CRC = `(0x01 + 0x34) & 0xFF = 0x35`. ✔

Uplink (no alarm, in place, IN_1 high): status = `0b00001101 = 0x0D` (bit3 ALM=1, bit2 PEND=1, bit0 IN_1=1)
CRC = `(0x01 + 0x34 + 0x0D) & 0xFF = 0x42` → frame `01 34 0D 42`. ✔

---

## 0x36 — Write IO Port Data

Drive the ALM / PEND output optocouplers under per-port write masks.

**Downlink** DLC=3:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `36` |
| 2 | data | port-data bitfield |
| 3 | CRC | checksum |

### Data bitfield (page 43)

| Bit | Field | Value |
|-----|-------|-------|
| bit7 | alm_mask | 0 = don't write ALM, 1 = write ALM value, 2 = ALM unchanged |
| bit6 | pend_mask | 0 = don't write PEND, 1 = write PEND value, 2 = PEND unchanged |
| bit5 | ALM | ALM port write value (0/1) |
| bit4 | PEND | PEND port write value (0/1) |
| bit3–bit0 | Reserved | 0 |

**Note (page 43):**
- Writing **1** to the ALM IO port **disconnects** the corresponding optocoupler.
- Writing **1** to the PEND IO port **closes** the corresponding optocoupler.
- The `alm_mask` / `pend_mask` value **2** leaves that port's value unchanged.

**Uplink** DLC=3: `[36, status, CRC]` — status 0 = write failed, 1 = write successful.

> 🔀 **vs 42D：** 42D 的 `0x36` 用 **OUT2_mask (bit7) / OUT1_mask (bit6)** + OUT_2 (bit5) / OUT_1·IN_1 (bit4)，且配合 `0x9F` 可把 IN_1 配成輸出再經 bit2 直寫。42ES **沒有 `0x9F`**，data 欄改為 **alm_mask / pend_mask + ALM / PEND**（功能化），mask 同樣支援 0/1/2 三態。寫 1 的物理效果也不同：42ES 寫 1→ALM 斷開光耦、PEND 閉合光耦。

> ⚠️ **42ES 持久化：** 寫類參數在發出 **保存指令 `0x60`** 前不寫入 NVM（Part 4 note 3 / §4.5.1）。`0x36` 雖是即時輸出驅動，但任何需保留的 IO 配置改動皆須 `0x60` 落盤 — 見 [[servo42es/12-errors-recovery|0x60 save]]。SERVO42D 文檔無此顯式保存指令。

### Example — write IO on motor 0x01

Drive PEND=1, leave ALM unchanged → data = `pend_mask=1 (bit6) | PEND=1 (bit4) | alm_mask=2 (bit7)`.
With bit7=alm_mask=2 we cannot encode value 2 in a single bit; using `alm_mask=0` (don't write ALM) and `pend_mask=1`, `PEND=1`: data = `0b01010000 = 0x50`.

Downlink: `01 36 50 87`
CRC = `(0x01 + 0x36 + 0x50) & 0xFF = 0x87`. ✔

Uplink success: `01 36 01 38`
CRC = `(0x01 + 0x36 + 0x01) & 0xFF = 0x38`. ✔

---

## 0x9F — IO Port Pulse Frequency Division Output

Map the **PEND** port to a pulse-divider output (e.g. one toggle per motor revolution).

**Downlink** DLC=7:

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `9F`（Byte1 = `9FH`，PDF V1.0.1 §5.3 downlink/uplink 表一致） |
| 2 | divLevel | 0 = low start level (default), 1 = high start level |
| 3–6 | divPeriod | uint32, big-endian, frequency-division period (default 0) |
| 7 | CRC | checksum |

- **divPeriod < 100** → no frequency-division output.
- **divPeriod ≥ 100** → the PEND port toggles **once every divPeriod pulse cycles**.
- Setting `divPeriod = 0` **disables** the feature.

Example (page 44): 16 microsteps, `divPeriod = 3200` → PEND flips **once per motor revolution**.

**Uplink** DLC=3: `[code, status, CRC]` — status 0 = set failed, 1 = set successful.

> 🔀 **vs 42D：** **指令碼不同。** 脈衝分頻在 42D 是 `0x99`（且標 **「57D Only」**、綁通用 OUT_2）；在 42ES/57ES 改為 **`0x9F`**（PDF V1.0.1 §5.3，無型號限制、直接映射到 **PEND** 埠）。⚠️ 注意 42D 的 `0x9F` 另作「Configure IN_1 mode」，42ES 無此指令 → 同一碼 `0x9F` 在兩產品線語意完全不同，移植勿混。divLevel / divPeriod 語意（<100 不輸出、≥100 每 divPeriod 個脈衝翻轉一次、3200@16細分=每圈一翻、0=停用）兩者一致。

> ⚠️ **42ES 持久化：** divLevel/divPeriod 屬寫類配置，須 `0x60` 保存才跨重啟保留（見上）。

### Example — 3200/rev divider on motor 0x01

divLevel=0, divPeriod=3200 = `0x00000C80`.
Downlink: `01 9F 00 00 00 0C 80 2C`
CRC = `(0x01 + 0x9F + 0x00 + 0x00 + 0x00 + 0x0C + 0x80) & 0xFF` = `0x12C & 0xFF` = **`0x2C`**. ✔

---

## En-pin active level & direction (Part 4, page 30 — context)

These two write commands decide how the **EN** input pin (and bus direction) is
interpreted; they pair with the IN/EN input states reported by `0x34`.

### 0x85 — Set En-pin active level

DLC=3: `[85, level, CRC]`

| level | Meaning |
|-------|---------|
| 00 | low-level enable (L) — **default** |
| 01 | high-level enable (H) |
| 02 | always enabled (Hold) |

Uplink `[85, status, CRC]`; status 0 = fail, 1 = success.
*Note 1: after successful setup, pulse signals are accepted after a 100 ms delay.
Note 2: only valid for pulse control mode.*

### 0x86 — Set motor rotation direction

DLC=3: `[86, dir, CRC]`

| dir | Meaning |
|-----|---------|
| 00 | clockwise (default) |
| 01 | counter-clockwise |

Uplink `[86, status, CRC]`. *Note: this command also changes the direction of
bus-controlled motor operation.*

> ⚠️ **42ES 持久化：** `0x85`/`0x86` 為寫類參數，須 `0x60` 保存（見 [[servo42es/12-errors-recovery|0x60 save]]）。

> 🔀 **vs 42D：** 42D twin 的 IO 章以 `0x9F` 配置 IN_1 為輸入/輸出口；42ES 無 `0x9F`，改以 `0x85`（En 有效電平，新增 **02=Hold 常使能**）與 `0x86`（旋向）治理輸入/方向語意。

---

## See also

- [[servo42es/01-protocol|協議 / CRC / 幀格式]]
- [[servo42es/03-read-commands|讀取類指令（含 0x34）]]
- [[servo42es/04-config-commands|配置指令]] · [[servo42es/12-errors-recovery|0x60 保存]]
- [[servo42d/10-io-ports|42D IO Port 對應章]]
- [[servo-can-hub|CAN 文檔總覽 / 對照矩陣]]
