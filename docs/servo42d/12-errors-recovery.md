# Error Messages & Recovery

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/12-errors-recovery|Error Messages & Recovery (LED)]]

## OLED Error Messages

> 🔀 **vs 42ES：** 42D 從 **OLED 文字字串**（`Magnet Loss!`/`Phase Line Error!`/`Wrong Protect!`/`Wrong2...` 等）解錯。42ES 無 OLED，改以 **1 綠 + N 紅閃爍碼** 解錯（見 [[servo42es/12-errors-recovery|42ES]]）；概念重疊但非 1:1（如 `Phase Line Error!` ≈ 42ES 2 紅、`Low Voltage Error!` ≈ 4 紅、`Wrong2...` ≈ 5 紅、`Encoder Error!` ≈ 6 紅）。

| Message | Meaning | Solution |
|---------|---------|----------|
| Not Cal | Not calibrated | Calibrate the motor (0x80) |
| Reverse Lookup Error! | Calibration failed | Check magnet and motor shaft |
| Magnet Loss! | No magnet detected | Install the magnet |
| Magnet Strong! | Magnet too close | Increase distance (0.5-3.0mm) |
| Magnet Weak! | Magnet too far | Decrease distance |
| Encoder Error! | Encoder issue | Check magnet and motor shaft |
| Offset Current Error! | Reference voltage error | - |
| Phase Line Error! | Wrong wiring or low power | Check motor wiring; ensure 24V/1A or 12V/2A power |
| Wrong Protect! | Overcurrent stall protection | Release stall (0x3D / Enter button / loosen shaft) |
| Wrong2... | Position out-of-tolerance | Release stall (0x3D / Enter button / loosen shaft) |
| Low Voltage Error! | Supply voltage too low | Check power supply |
| Coming Back to Origin.. | Homing in progress | Wait for completion |
| Reboot Again | Motor needs restart | Power cycle or send 0x41 |
| Press Next Key To Fixed | Key lock activated | Press and hold Next until reboot |

## 0x3F — Restore Factory Settings

DLC=2: `[3F, CRC]`
Response DLC=3: `[3F, status, CRC]` (0=fail, 1=success)

- Auto restarts after restore. No recalibration needed.
- Alternative: Press and hold "Next" button at power-on until LED flashes.

> 🔀 **vs 42ES：** 指令碼與時序相同（`[3F, CRC]` ↔ `[3F, status, CRC]`、成功後自動重啟）。差異在持久化層：42ES 上 factory restore 還會抹掉所有經 **0x60 SAVE** 提交的參數（0x60 是 42ES 唯一持久化路徑，42D 無此指令）。此處的 **Next 鈕硬體還原** 為 42D 專屬，42ES 無對應實體按鍵路徑。詳見 [[servo42es/12-errors-recovery|42ES]]。

## 0x41 — Reset and Restart

DLC=2: `[41, CRC]`
Response DLC=3: `[41, status, CRC]` (0=fail, 1=success)

- Only resets/restarts the motor. Does NOT modify configuration parameters.

## Precautions

> 🔀 **vs 42ES：** 供電窗口不同 — 42D 為 **12V–24V**，42ES/57ES 為 **20V–48V**（STM32 + MT6826 平台），勿假設一致。磁鐵幾何（φ6.00mm/2.5mm、0.5–3.0mm gap、±0.3mm 對心）、1.8° step distance、<10Ω 內阻則兩者相同。見 [[servo42es/12-errors-recovery|42ES]]。

1. Power input: 12V-24V
2. Do not hot-plug power or signal cables
3. Calibrate without load, before installing into machine
4. Recalibrate after first install or motor wiring change
5. Motor step distance must be 1.8°
6. Motor internal resistance < 10 ohms
7. Magnet-to-encoder gap: 0.5-3.0mm, center alignment within ±0.3mm
8. Recommended radial magnet: φ6.00mm, height 2.5mm
