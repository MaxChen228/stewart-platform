# Error Messages & Recovery

## OLED Error Messages

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

## 0x41 — Reset and Restart

DLC=2: `[41, CRC]`
Response DLC=3: `[41, status, CRC]` (0=fail, 1=success)

- Only resets/restarts the motor. Does NOT modify configuration parameters.

## Precautions

1. Power input: 12V-24V
2. Do not hot-plug power or signal cables
3. Calibrate without load, before installing into machine
4. Recalibrate after first install or motor wiring change
5. Motor step distance must be 1.8°
6. Motor internal resistance < 10 ohms
7. Magnet-to-encoder gap: 0.5-3.0mm, center alignment within ±0.3mm
8. Recommended radial magnet: φ6.00mm, height 2.5mm
