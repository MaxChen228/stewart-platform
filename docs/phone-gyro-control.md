# Phone IMU Control

Date: 2026-06-28

Goal: use a phone browser as a live attitude controller for the Stewart platform
without changing firmware motion semantics.

## Control Semantics

The phone page is a manual real-time operator:

- `P` remains the waypoint / settle command.
- `PF` remains the immediate real-time follow command.
- The phone page only streams `PF` after the firmware has acknowledged
  `FOLLOW 1`.
- It does not run `E`, `Z`, or any calibration command.
- `HOLD HERE` sends `H`; `RELEASE` sends `D` after first stopping follow.

Runtime smoothing is split across two layers:

- Phone page: permission, calibration, axis mapping, deadband, low-pass filter,
  IK precheck, and 30 Hz PF coalescing.
- Firmware: `FOLLOW` pose-rate limiting and `VF` remain the hard safety gate.

## Start

Normal desktop dashboard:

```bash
npm start
```

Phone IMU mode, which also opens an HTTPS listener for browser sensor access:

```bash
npm run start:phone
```

Open the phone page from a phone on the same LAN:

```text
https://<computer-lan-ip>:3443/phone.html
```

The first launch creates a local self-signed cert under `sysid/config/`. Those
cert files are ignored by git. The phone may ask you to accept/trust the cert;
without HTTPS, modern mobile browsers can block `DeviceOrientationEvent`.

## Operating Sequence

1. Put the rig in a safe physical state and open `/phone.html`.
2. Confirm `WS`, `RIG`, and `HOLD` status on the page.
3. Press `HOLD HERE` if the rig is not holding.
4. Press `感測器` and grant orientation permission.
5. Hold the phone in the neutral hand pose and press `校準`.
6. Press `FOLLOW`.
7. Tilt the phone. Roll and pitch are enabled by default; yaw is optional.
8. Press `STOP FOLLOW` before putting the phone away.

`HOME` behaves differently by mode:

- While following: it moves the follow base to the saved Home pose and
  recalibrates the current phone pose as zero offset.
- While not following: it sends a normal `P home 1500ms`.

## Mapping

The page maps phone orientation relative to the calibration pose:

- Phone side tilt -> platform roll.
- Phone forward/back tilt -> platform pitch.
- Phone heading delta -> platform yaw when yaw is enabled.

Controls:

- `平台角`: maximum platform roll/pitch offset in degrees.
- `手機滿刻`: phone tilt angle that maps to the platform maximum.
- `Yaw 上限`: maximum yaw offset in degrees.
- `平滑`: phone-side low-pass time constant in seconds.
- `死區`: small phone motion ignored before scaling.
- Axis toggles: include/exclude roll, pitch, yaw.
- `+/-` buttons: invert individual axis signs.

If an IMU target is outside IK workspace, the page marks the dot red and stops
streaming that invalid target; the firmware continues holding the last valid
follow target.

## Notes

This is intentionally not a Workspace/session runner. If a Workspace session is
active, the server session lock blocks live phone motion commands. This keeps
manual phone control and automated experiments from fighting for `PF`.
