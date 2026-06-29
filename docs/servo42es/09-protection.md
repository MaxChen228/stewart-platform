# Protection Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42D 對應 [[servo42d/09-protection|Protection Commands]]

42ES 把「堵轉/位置超差保護」與「心跳保護」分成兩條獨立路徑。所有寫入指令遵循 [[servo42es/01-protocol|標準幀格式]]，CRC = `(CAN_ID + 所有 data bytes) & 0xFF`（[[servo42es/01-protocol|CRC 算法]]，big-endian，與 SERVO42D byte-identical）。

> 🔀 **vs 42D：** 42D 用兩個碼分工 — `0x88` 只開關過流保護（DLC=3）、`0x9D` 才設位置超差（DLC=7, time + errors 兩欄）。42ES 把「堵轉保護開關 + 超差門檻」**合併進單一 `0x88`（DLC=5）**，手冊 V1.0.1 §4.4.1 不存在 `0x9D`。下文以 PDF 實際內容為準。

> ⚠️ **42ES SAVE 警告：** 所有保護參數（`0x88`/`0x89`/`0x98`）為 write-only，**寫入後不落 NVS，直到送 [[servo42es/06-bus-control|SAVE 0x60]]**（Part 4 note 3 + §4.5.1）。重開機即回預設。SERVO42D 文檔無此顯式 SAVE 指令。

---

## Two Protection Mechanisms

| 機制 | 碼 | 觸發條件 | 觸發後 |
|------|----|---------|--------|
| Stall / over-tolerance protection | `0x88` | 響應時間內因過載等原因未到達指定位置、位置誤差超 `value` | 馬達解鎖、軸釋放（avoid damage） |
| Heartbeat protection | `0x89` | 設定時間內未收到任何上位機指令 | 馬達緊急停止（urgent stop），防通訊中斷失控 |

兩者獨立開關。OLED 觸發顯示由韌體決定（與 [[servo42es/12-errors-recovery|錯誤恢復]] 章相關）。

### Stall Release Methods（堵轉解除三法）

1. 送 `0x3D` 指令（command 3DH）
2. 控制馬達軸鬆動（physically loosen / turn the shaft，即操控 "EN signal" 使軸鬆脫）

> 🔀 **vs 42D：** 42D 另列「按驅動板 Enter 按鈕」為解除法之一；42ES 手冊 §4.4.3 只明列「送 3DH」與「鬆動軸」兩法。

---

## 0x88 — Set Stall Protection & Over-Tolerance（§4.4.1）

> If the motor fails to reach the designated position within the response time due to overload or any other reason, the stall protection will be triggered, the motor will unlock and the shaft will be released to avoid damage.

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 (Byte1) | code | `0x88` |
| 2 (Byte2) | Stall protection enable | `0x00`=disable / `0x01`=enable（**default `0x01`**） |
| 3–4 (Bytes 3–4) | Excess tolerance value (uint16) | `0x0000`–`0x7FFF`，**default `0x0064`** |
| 5 (Byte5) | Checksum | CRC |

DLC=5。

- **enable**：`0x00` 關閉堵轉保護；`0x01` 開啟（出廠預設）。
- **value**：超差門檻，範圍 `0–0x7FFF`，預設 `0x64`。誤差超過 value 即觸發堵轉保護、軸釋放。
  - `value = 0x64` → 角度 180°
  - `value = 0xC8` → 角度 360°
  - 以此類推（線性，1 unit ≈ 1.8°）。

> 🔀 **vs 42D：** 42D 的 `0x88` 僅 `[88, enable, CRC]`（DLC=3，default disable）；超差另走 `0x9D`（DLC=7），其 errors 以 `28000=360°`、`14000=180°` 計、外加獨立 time 視窗（1 unit≈15ms）。42ES `0x88` 無 time 視窗欄、超差刻度改為 `0xC8=360°`/`0x64=180°`（單位完全不同，**勿沿用 42D 的 28000 換算**）。42ES default enable=`0x01`（42D default disable）。

#### Worked example — 開啟保護、超差門檻 360°

Downlink `[01, 05, 88, 01, 00, C8, CRC]`（CAN_ID=`0x01`, value=`0x00C8`=360°）。

CRC = `(0x01 + 0x88 + 0x01 + 0x00 + 0xC8) & 0xFF` = `(0x01+0x88+0x01+0x00+0xC8)` = `0x152 & 0xFF` = **`0x52`**。

完整幀：`01 05 88 01 00 C8 52`。

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x88` |
| 2 | Setting status (uint8) | `0`=fail / `1`=success |
| 3 | Checksum | CRC |

DLC=3。寫入指令統一回 `[code, status, CRC]`（見 [[servo42es/04-config-commands|寫入回覆格式]]）。

回覆範例 `[01, 03, 88, 01, CRC]`：CRC = `(0x01+0x88+0x01) & 0xFF` = `0x8A`。完整幀 `01 03 88 01 8A`。

---

## 0x3E — Read Motor Stall Status（§4.4.2）

> When a motor stalls, a stall flag will be set. If the stall protection option is enabled, the drive board will automatically shut down the driver upon a stall.

讀指令，不寫參數、無 SAVE 需求。屬 [[servo42es/03-read-commands|讀取類指令]]。

### Downlink

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x3E` |
| 2 | Checksum | CRC |

DLC=2。

#### Worked example

Downlink `[01, 02, 3E, CRC]`：CRC = `(0x01 + 0x3E) & 0xFF` = `0x3F`。完整幀 `01 02 3E 3F`。

### Uplink

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x3E` |
| 2 | stalled state (status) | `0x00`=motor not stalled / `0x01`=motor is stalled |
| 3 | Checksum | CRC |

DLC=3。

Note：堵轉後可用 `0x3D`（command 3DH）釋放堵轉狀態。

> 🔀 **vs 42D：** byte layout 與語意一致（`0=not stalled`/`1=stalled`）。

---

## 0x3D — Release Motor Stall（§4.4.3）

> When the motor stalls, sending this command can release the current stall state. **If a stall occurs again after the stall is cleared, the stall protection will still be triggered.**

### Downlink

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x3D` |
| 2 | Checksum | CRC |

DLC=2。

#### Worked example

Downlink `[01, 02, 3D, CRC]`：CRC = `(0x01 + 0x3D) & 0xFF` = `0x3E`。完整幀 `01 02 3D 3E`。

### Uplink

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x3D` |
| 2 | Released status (status) | `0x00`=removal failed / `0x01`=removal successful |
| 3 | Checksum | CRC |

DLC=3。

Note：亦可藉控制 "EN signal"（即控制馬達軸鬆動）來釋放堵轉狀態。

> 🔀 **vs 42D：** byte layout 與 status 語意一致（`0=release failed`/`1=release success`）。

---

## Recovery Flow（堵轉恢復流程）

```
觸發保護 (馬達解鎖、軸釋放)
   │
   ├─► 讀 0x3E 確認 stalled state == 0x01
   │
   ├─► 排除物理原因 (過載/卡死/超差路徑)
   │
   ├─► 送 0x3D 釋放  →  讀回 status==0x01 才算解除
   │        (或控制軸鬆動 / EN signal)
   │
   └─► 若堵轉條件仍在 → 釋放後保護會 re-trigger
```

恢復後若要重新進入正常 [[servo42es/06-bus-control|bus 位置控制]]，比照 [[servo42es/12-errors-recovery|錯誤恢復]] 章重啟運動序列。

---

## 0x89 — Set Heartbeat Protection Time（§4.3.12）

> Heartbeat protection refers to the system that, if the motor does not receive any instructions from the host computer within the set protection time, it will control the motor to stop urgently, preventing abnormal accidents caused by communication interruption.

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 (Byte1) | code | `0x89`（手冊 §4.3.12 表列 "89H"） |
| 2–5 (Bytes 2–5) | Protection time / times (uint32) | ms，**default `0`** |
| 6 (Byte6) | Checksum | CRC |

DLC=6。

- **times**：保護時間，單位 ms。`times = 0` → 心跳保護關閉（default）。
- 逾時內未收到任何指令 → 馬達緊急停止。

> 🔀 **vs 42D：** **指令碼不同。** 42D 心跳保護碼為 `0x98`；42ES 改為 **`0x89`**（PDF V1.0.1 §4.3.12 downlink/uplink 表 Byte1 一致印 `89H`）。byte layout 一致（`[code, time(uint32), CRC]`，DLC=6、`time=0` 停用、單位 ms），僅碼值在兩產品線間遷移 — 移植 42D 韌體時務必改碼。

#### Worked example — 心跳逾時 1000ms

以 code `0x89`、time=`0x000003E8`（1000ms）：Downlink `[01, 06, 89, 00, 00, 03, E8, CRC]`。

CRC = `(0x01 + 0x89 + 0x00 + 0x00 + 0x03 + 0xE8) & 0xFF` = `0x175 & 0xFF` = **`0x75`**。

完整幀：`01 06 89 00 00 03 E8 75`。

> 寫入後須送 [[servo42es/06-bus-control|SAVE 0x60]] 才落 NVS，否則重開機回 `0`（停用）。

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 | code | `0x89` |
| 2 | Setting status | `0`=set failed / `1`=set successed |
| 3 | Checksum | CRC |

DLC=3。

---

## 0x98 — Set Position Reach Threshold（§4.3.13）

> In bus position control mode, the "position operation completed" message will only be returned when the actual position of the motor is less than the threshold.

此門檻決定 [[servo42es/05-motion-commands|位置運動指令]] 何時回報 `status=2`（completed）；屬 [[servo42es/06-bus-control|bus 位置控制]] 配套設定。

### Downlink (PC → SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 (Byte1) | code | `0x98`（手冊 §4.3.13 downlink/uplink 表 Byte1 一致印 "98H"） |
| 2 (Byte2) | Enable | `0x00`=disable / `0x01`=enable（**default `0x01`**） |
| 3–4 (Bytes 3–4) | values (uint16) | position threshold，**default `800`**；最大 `65535`=360° |
| 5 (Byte5) | Checksum | CRC |

DLC=5。

- **enable**：`0x00` 停用到位門檻判斷；`0x01` 啟用到位門檻判斷（default）。
- **values**：位置門檻，預設 `800`（`0x0320`）。max `65535`=360°。

> 🔀 **vs 42D：** **指令碼與預設值皆不同。** 兩產品線都有「位置到位門檻」設定：42D 用 `0x95`（預設門檻 `200`）；42ES 改用 **`0x98`**（預設 **`800`**，PDF V1.0.1 §4.3.13）。注意 42ES 的 `0x98` 在 42D 是「heartbeat」碼 → 移植時兩碼皆遷移（見上節 heartbeat callout）。

#### Worked example — 啟用、門檻 800

以 code `0x98`、values=`0x0320`（800）：Downlink `[01, 05, 98, 01, 03, 20, CRC]`。

CRC = `(0x01 + 0x98 + 0x01 + 0x03 + 0x20) & 0xFF` = `0x0BD & 0xFF` = **`0xBD`**。

完整幀：`01 05 98 01 03 20 BD`。

> 寫入後須送 [[servo42es/06-bus-control|SAVE 0x60]] 才落 NVS。

### Uplink (PC ← SERVOxxES)

| Byte | Field | Value |
|------|-------|-------|
| 1 | Code | `0x98` |
| 2 | Setting status (uint8) | `0`=set failed / `1`=set successed |
| 3 | Checksum | CRC |

DLC=3。

---

## Cross-references

- 幀格式 / CRC：[[servo42es/01-protocol|01-protocol]]
- 寫入回覆格式 / config：[[servo42es/04-config-commands|04-config-commands]]
- 讀取類指令：[[servo42es/03-read-commands|03-read-commands]]
- 位置運動 / completed 回報：[[servo42es/05-motion-commands|05-motion-commands]]
- bus 位置控制 + SAVE 0x60：[[servo42es/06-bus-control|06-bus-control]]
- 錯誤恢復：[[servo42es/12-errors-recovery|12-errors-recovery]]
- 42D 對應章：[[servo42d/09-protection|servo42d/09-protection]]
- 總覽：[[servo-can-hub|servo-can-hub]]

> 適用 **SERVO42ES V1.0 與 SERVO57ES V1.0**（同韌體/協議；差於電流額定與 I/O 數，見 [[servo42es/10-io-ports|10-io-ports]]）。
