---
name: stewart-servo42d-can
description: Use for Stewart Platform SERVO42D CAN work: command lookup, CRC/frame details, MCP2515 buffer risks, encoder reads, F5/F6 motion commands, PID and zeroing semantics.
---

Follow this workflow for SERVO42D CAN tasks in this repository.

1. Start with `docs/servo42d/00-index.md` and route by command code or task.
2. For frame format, CRC, coordinate units, and absolute vs relative semantics, read `docs/servo42d/01-protocol.md`.
3. For motion commands, read `docs/servo42d/05-motion-commands.md` before changing any `0xF4`, `0xF5`, `0xF6`, `0xFD`, or `0xFE` code.
4. For enable/disable/status/emergency-stop behavior, read `docs/servo42d/06-bus-control.md`.
5. For PID changes, read `docs/servo42d/07-pid-tuning.md` and preserve the current known-stable vFOC pure-P operating point unless the user explicitly asks to retune.
6. For zeroing or calibration, distinguish `0x80`, `0x91`, and `0x92`; do not treat all "zero" operations as equivalent.
7. Remember project-specific constraints:
   - CAN bitrate is 500 Kbps.
   - MCP2515 has only two RX buffers; drain/flush behavior matters.
   - CRC is `(CAN_ID + all data bytes) & 0xFF`.
   - Encoder failures should preserve the previous angle, not fall back to neutral.
8. Before suggesting hardware execution, state whether the action moves motors, writes NVS, changes CAN IDs/bitrate, or can require recovery.
