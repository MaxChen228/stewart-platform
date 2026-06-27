# Research Closed Loop

Date: 2026-06-27

Goal: make UI operation and Codex/CLI experiments use one shared system instead
of parallel control paths.

## Source of Truth Layers

| Layer | SoT | Consumers |
|---|---|---|
| Geometry / IK / FK | `sysid/kin.js` plus synced firmware/frontend geometry | UI, sysid tools, firmware |
| Disturbance modes / battery sequence | `sysid/disturb_modes.js` | UI dashboard, `disturb.js`, `disturb_battery.js` |
| Mutable platform config | `GET/POST /api/platform-config` backed by `sysid/config/platform.json` | UI, `safe_land.js`, `coupling_probe.js`, `research_trial.js` |
| Live state | `GET /api/latest` | UI, Codex preflight, summaries |
| Experiment data | JSONL recorder under `sysid/data/` | summaries, plots, research log |

`/api/home` remains as a compatibility shim. New code should use
`/api/platform-config`.

## Platform Config

Mutable config includes:

- `homeRelative`: UI Home pose relative to neutral.
- `landingRelative`: support-fixture landing pose relative to neutral.
- `followLimits`: default `VF` limits.
- `trialDefaults`: default profile and timing for automated trials.
- `uiState`: persisted UI dashboard state:
  pose target, pose move duration, selected standard motion, per-motion
  parameters, loop count, disturbance U/W parameters, battery backstop, and
  recorder name.
- `safety`: non-negotiable preflight expectations.

UI is the human-friendly editor/operator. CLI tools are experiment executors.
Neither should keep private defaults beyond `sysid/platform_sot.js` fallback
values for offline dry-runs.

When the UI dashboard changes a pose, motion, VF, disturbance, or recorder
field, it should update `uiState` through `/api/platform-config`. This lets a
human tune several motions in the UI and then ask Codex to execute or analyze
the exact same saved parameters.

## Standard Trial Flow

For a motion experiment:

1. Read `/api/latest`, `/api/transport`, `/api/rec/status`, and
   `/api/platform-config`.
2. Block live motion if telemetry is missing, `ok != 6`, or recorder is already
   running.
3. Require explicit human-at-rig confirmation for live motion.
4. Create a manifest before motion.
5. Start recorder.
6. Execute the motion using the shared config.
7. Safe-land if configured.
8. Stop recorder even on errors.
9. Generate telemetry summary and `motor6` / `plane6` plots.
10. Append a dated research-log entry only after the data is inspected.

The current turnkey runner is:

```bash
npm run research:trial
npm run research:trial -- --live --i-am-at-rig --name heave60 --heave 60 --ms 5000
```

Dry-run is the default. Live motion needs both `--live` and `--i-am-at-rig`.

## Output Bundle

A completed runner should produce:

- `*.jsonl`: raw recording.
- `*.manifest.json`: software/config/live-state snapshot.
- `*.summary.json`: machine-readable health summary.
- `*.evaluation.json`: full-lifecycle raw feature report.
- `sysid/data/plots/*_motor6.svg`: six motor curves.
- `sysid/data/plots/*_plane6.svg`: six FK pose curves.
- `*.bundle.json`: paths to the above.

## Design Rule

If a control concept appears in both UI and CLI, extract or route it through a
shared source first. Do not implement a second copy just because it is faster in
the moment.

## Workspace Execution

Workspace live execution is owned by the server-side runner exposed through
`POST /api/workspace/run`, `GET /api/workspace/status`, and
`POST /api/workspace/abort`.

The browser Workspace page is the editor/preview/observer. The CLI
`npm run workspace:session` is the dry-run validator and live request client.
Neither should stream its own live `PF` loop during normal operation; the only
normal live `PF` loop is inside `sysid/workspace_executor.js`, under a session
token owned by `server.js`.
