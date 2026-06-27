---
name: stewart-sysid-gate
description: Use for Stewart Platform system-identification, Workspace sessions, standard motion blocks, Gate A, telemetry recording, run analysis, and research-log updates.
---

Use this workflow for system-identification and agent-operated Workspace tasks.

1. Read `docs/sysid-research-log.md` first. Treat it as the current research ledger.
2. For autonomous or repeatable experiments, read `docs/research-closed-loop.md` and use the shared SoT path:
   `/api/platform-config` for mutable platform config, `sysid/kin.js` for geometry/FK,
   `web/shared/motion-library.js` for standard Workspace motions,
   `web/shared/workspace-core.js` for block normalization/duration/closed-loop audit, and
   `sysid/disturb_modes.js` for disturbance definitions.
3. If the task involves live data, read `curl localhost:3000/api/latest` before inferring state.
4. Use existing scripts in `sysid/` where possible. Do not invent a new experiment harness until checking for a nearby script.
   Prefer `npm run workspace:session` for Workspace sessions and `npm run research:trial` for legacy turnkey trials.
5. For manual recording outside Workspace, use the documented flow:
   - `npm start` for the server.
   - `curl "localhost:3000/api/rec/start?name=NAME"` to start.
   - Run the motion/test.
   - `curl localhost:3000/api/rec/stop` to stop.
   Do not use global recorder endpoints while a Workspace session is active; session-owned runs use `/api/session/rec/start` and `/api/session/rec/stop` through the Workspace runner.
6. Hardware safety gates for live experiments and live Workspace sessions:
   - A human must be at the hardware.
   - Platform should be centered or intentionally positioned.
   - Power cutoff must be reachable.
   - `ok:6` should be confirmed when encoder data is required.
   - `/api/transport` must report `state:"connected"`.
   - `/api/rec/status` must not already be recording unless the active session owns it.
   - telemetry age must be fresh enough for the planned motion.
7. Gate A specifically answers whether a faster clean polling loop stabilizes the platform. Do not skip ahead to TWAI/1M baud work until Gate A evidence says polling is insufficient.
8. When updating `docs/sysid-research-log.md`, append a dated entry. Do not rewrite old conclusions except to add a clearly dated correction later.

## Workspace Session Contract

Workspace is the default path for agent-composed repeatable motion experiments.

- The fixed outer sequence is `TAKE OFF -> HOME -> FOLLOW -> blocks -> HOME -> LAND -> RELEASE -> analyze`.
- Raw Workspace recording covers the full lifecycle: TAKE OFF, HOME, FOLLOW, blocks, close loop, LAND, and RELEASE. Closed-loop scoring/analysis still treats the HOME-to-HOME block interval as the motion window inside that full recording.
- A Workspace with zero blocks is valid as a smoke test: `TAKE OFF -> HOME record window -> HOME -> LAND`.
- Session execution must use `npm run workspace:session` for dry-run validation.
- Live execution must use `npm run workspace:session -- --live --i-am-at-rig --name NAME` and only after the user confirms they are physically at the rig.
- The Workspace runner owns the session token and recorder. Do not manually stream `PF` loops from an agent unless the user explicitly asks to bypass the runner for debugging.
- LANDING includes the final motor release (`D`) before declaring the session complete. Analysis happens after release so the platform is not held while summary/plot scripts run.
- During an active session, monitor `/api/session/status`, `/api/session/events`, `/api/latest`, and `/api/transport`. Treat command drops, stale telemetry, transport down, or `ok != 6` as stop-and-diagnose conditions.
- The UI and agent must share the same operation model: blocks come from `web/shared/motion-library.js`; block normalization and closed-loop audit come from `web/shared/workspace-core.js`; config comes from `/api/platform-config`.
- The experiment unit is a saved Workspace program under `uiState.workspace.programs`. A program contains a named closed-loop block sequence and can have many recorded runs. Treat ad-hoc `workspace.blocks` as a scratch draft until it is saved as a program.
- Runs are part of the Workspace feedback loop, not a separate source of truth. After execution, compare `/api/runs` and `/api/runs/:id` results against the active program before proposing the next program variant. Prefer explicit run metadata `run.program.id/hash`; use label/name matching only for older recordings.

## Standard Motion Library

Agents may add new standard motions, but they must preserve the motion-library contract.

1. Edit `web/shared/motion-library.js`; add one `MOTIONS` entry with:
   - `id`: unique ascii kebab-case.
   - `label`: short user-facing name.
   - `params`: bounded numeric controls with `{ k, l, min, max, def, step, u }`.
   - `per(params)`: finite positive period in seconds.
   - `fn(t, params)`: returns a relative pose `[x, y, z, roll, pitch, yaw]` using mm and degrees.
   - optional `category`: user-facing grouping for the Workspace motion library.
   - optional `envelope:false`: only for motions whose `fn()` already returns HOME exactly at `t=0` and `t=per(params)`.
2. Keep defaults conservative and IK-valid around configured HOME. The envelope in `motionPoseAt()` brings the block back to HOME at cycle boundaries, so `fn()` should describe the relative motion shape rather than doing its own session takeoff/landing.
   For step-hold-return primitives, use `envelope:false` and make the primitive itself rise, hold, and return to HOME within one period.
3. Do not duplicate motion math in UI, CLI, or server files. If a new helper is needed, put it in `web/shared/motion-library.js` or `web/shared/workspace-core.js`.
4. Run `npm run workspace:validate` after adding or changing motions. This validates schema, parameter bounds, default samples, IK reachability, HOME endpoints, and the saved Workspace blocks.
5. Only after validation passes may the agent add the new motion to `uiState.workspace.blocks` via `/api/platform-config` or the Workspace UI.

## Post-Run Analysis

After any live Workspace or research run:

- Locate the recording through the existing runs/research APIs or `sysid/data`.
- Associate the run with the active Workspace program through recorder/session metadata `program.id`, `program.name`, and `program.hash`. For old recordings without metadata, fall back to label/name matching.
- Summarize duration, sample count, command drops, transport health, `ok` coverage, and whether the motion window returned to HOME.
- Use existing analysis scripts before inventing new ones.
- If the run changes the research conclusion, append a dated entry to `docs/sysid-research-log.md`.
