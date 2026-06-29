# Protection Commands

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/09-protection|Protection Commands]]

## Two Protection Modes (Independent)

1. **Overcurrent protection (0x88)**: Triggered when motor overcurrent detected. OLED shows "Wrong..."
2. **Position out-of-tolerance (0x9D)**: Triggered when position error exceeds threshold for set duration. OLED shows "Wrong2..."

Both can be enabled/disabled independently. When triggered: motor unlocks, shaft released.

## Stall Release Methods

1. Press Enter button on driver board
2. Send 0x3D command
3. Physically loosen/turn motor shaft

> 🔀 **vs 42ES：** [[servo42es/09-protection|42ES]] 手冊只列「送 3DH」與「鬆動軸」兩法，未列「按驅動板 Enter 按鈕」。

## 0x88 — Set Overcurrent Protection

DLC=3: `[88, enable, CRC]`
- 0x00=disable (**default**), 0x01=enable

> 🔀 **vs 42ES：** [[servo42es/09-protection|42ES]] 的 `0x88` 為合併型堵轉/超差保護（DLC=5），且保護參數為 write-only — 寫入後須送 [[servo42es/06-bus-control|SAVE 0x60]] 才落 NVS，否則重開機回預設（42D 此處無顯式 SAVE 指令）。

## 0x9D — Set Position Out-of-Tolerance Protection

> 🔀 **vs 42ES：** 42D 用兩個獨立碼分工 — `0x88` 只開關過流保護、`0x9D` 才設位置超差（DLC=7, time + errors 兩欄）。[[servo42es/09-protection|42ES]] 把堵轉/超差門檻合併進單一 `0x88`（DLC=5），手冊無 `0x9D`。

DLC=7: `[9D, enable, time_hi, time_lo, errors_hi, errors_lo, CRC]`

- enable: 0=disable (**default**), 1=enable
- time: uint16. 1 unit ≈ 15ms. Error statistics time window.
- errors: uint16. Error count threshold. 28000=360°.

Example: `[9D, 01, 00, 14, 36, B0, CRC]`
→ Enable, time=0x0014 (20×15ms=300ms), errors=0x36B0 (14000=180°)
→ If >180° position error within 300ms → protection activates.

## Stall Detection & Release

當保護觸發時：馬達解鎖、軸釋放。流程：讀 0x3E 確認堵轉 → 排除原因 → 用 0x3D 釋放。

## 0x3E — Read Stall Status

DLC=2: `[3E, CRC]`
Response DLC=3: `[3E, status, CRC]`
- 0=not stalled, 1=stalled

## 0x3D — Release Stall

DLC=2: `[3D, CRC]`
Response DLC=3: `[3D, status, CRC]`
- 0=release failed, 1=release success

Note: If stall condition persists after release, protection will re-trigger.

## 0x98 — Set Heartbeat Protection

DLC=6: `[98, time(uint32), CRC]`

- time: ms. 0=disabled (**default**).
- If motor receives no command within timeout, it stops automatically.
- Prevents runaway if communication is lost.
