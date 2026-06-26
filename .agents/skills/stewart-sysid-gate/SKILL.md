---
name: stewart-sysid-gate
description: Use for Stewart Platform system-identification, Gate A, disturbance tests, telemetry recording, analysis scripts, and research-log updates.
---

Use this workflow for system-identification tasks.

1. Read `docs/sysid-research-log.md` first. Treat it as the current research ledger.
2. For autonomous or repeatable experiments, read `docs/research-closed-loop.md` and use the shared SoT path:
   `/api/platform-config` for mutable platform config, `sysid/kin.js` for geometry/FK, and
   `sysid/disturb_modes.js` for disturbance definitions.
3. If the task involves live data, read `curl localhost:3000/api/latest` before inferring state.
4. Use existing scripts in `sysid/` where possible. Do not invent a new experiment harness until checking for a nearby script.
   Prefer `npm run research:trial` for turnkey preflight → record → motion → safe-land → summary/plots.
5. For recording, use the documented flow:
   - `npm start` for the server.
   - `curl "localhost:3000/api/rec/start?name=NAME"` to start.
   - Run the motion/test.
   - `curl localhost:3000/api/rec/stop` to stop.
6. Hardware safety gates for live experiments:
   - A human must be at the hardware.
   - Platform should be centered or intentionally positioned.
   - Power cutoff must be reachable.
   - `ok:6` should be confirmed when encoder data is required.
7. Gate A specifically answers whether a faster clean polling loop stabilizes the platform. Do not skip ahead to TWAI/1M baud work until Gate A evidence says polling is insufficient.
8. When updating `docs/sysid-research-log.md`, append a dated entry. Do not rewrite old conclusions except to add a clearly dated correction later.
