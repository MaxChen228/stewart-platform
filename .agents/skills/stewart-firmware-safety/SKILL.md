---
name: stewart-firmware-safety
description: Use for Stewart Platform firmware changes that touch motor motion, enable/disable, calibration, WiFi/TCP control, failsafe, IK/FK, geometry, or upload/build verification.
---

Use this safety checklist before and after firmware changes.

1. Inspect the changed path for hardware impact:
   - Motor enable/disable or emergency stop.
   - `Z`/`Z0`-`Z5`, `0x92`, NVS calibration, or `sessionZeroRaw`.
   - `P`, `PF`, `FOLLOW`, `VF`, `V`, `K`, `FS`, `HB`, WiFi/TCP command dispatch.
   - IK/FK geometry or `angleToCoord` conversion.
2. Preserve invariants from `AGENTS.md`:
   - ESP32 coordinates trajectories; motors run `0xF5` position mode with internal PID.
   - Main operating path is HOLD with vFOC `[1024,0,0,0]` unless explicitly changed.
   - TCP must not allow calibration commands.
   - Failsafe default is HOLD-current unless the user intentionally changes it.
3. If geometry changes, update all truth sources together: `src/kinematics.h`, `web/index.html`, and `sysid/kin.js`.
4. Build verification:
   - Run `pio run` for firmware-only changes when hardware upload is not required.
   - Use `npm run upload` only when the user intends to flash the ESP32.
5. For live commands or upload, confirm a human is physically near the rig when motors may move or calibration may be written.
6. In the final response, separate "verified by build/static checks" from "requires hardware verification".
