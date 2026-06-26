# CAN Upgrade Plan: MCP2515 to ESP32 TWAI

Date: 2026-06-27

## Goal

Reach reliable 200 Hz control first, then evaluate 250-300 Hz as a stretch goal.

The target architecture is:

```text
ESP32 TWAI controller -> 3.3V CAN transceiver -> CANH/CANL -> SERVO42D x6
```

This replaces:

```text
ESP32 SPI -> MCP2515 -> TJA1050 -> CANH/CANL -> SERVO42D x6
```

## Current Baseline

Current firmware/hardware:

- MCP2515 module with 8 MHz crystal.
- CAN bitrate: 500 Kbps.
- Firmware initialization uses `CAN_500KBPS, MCP_8MHZ`.
- SERVO42D documentation supports 125K/250K/500K/1M via command `0x8A`.
- Main stable motor PID point remains vFOC `[1024,0,0,0]`.

Recent HOLD observations:

- `C 1 0`, `L 10`: RX overflow almost every monitor sample.
- `C 0 0`, `L 10`: overflow improves but still occurs.
- `C 0 0`, `L 20`: best observed MCP2515 HOLD state so far.

Interpretation:

- Core0 RX drain is useful but not sufficient.
- F5 command rate and command responses are material contributors.
- MCP2515 2-buffer RX design is a major bottleneck.
- 1M bitrate helps bus bandwidth, while TWAI helps host-side RX latency/queueing. They are complementary.

## Ordered Work

### Phase 0 - While Waiting for Parts

1. Keep the current known-better runtime profile for HOLD tests:
   - `C 0 0`
   - `L 20`
   - `AR 10` or slower when MCP2515 overflow matters
2. Use `sysid/can_budget.js` for all proposed rates before live tests.
3. Use `sysid/can_benchmark.js` for repeatable CAN/HOLD measurements instead of eyeballing `/api/latest`.
4. Keep appending real experiment conclusions to `docs/sysid-research-log.md`.

### Phase 1 - TWAI at 500 Kbps

Purpose: remove MCP2515 RX buffer/SPI bottleneck without changing motor bitrate.

Hardware:

- ESP32 GPIO TX -> transceiver TXD
- ESP32 GPIO RX -> transceiver RXD
- 3.3V -> transceiver VCC
- GND -> transceiver GND
- CANH/CANL -> existing bus
- Keep proper 120 ohm bus termination at the two physical ends only.

Firmware:

- Add a TWAI backend that preserves the existing upper-level CAN API:
  - `sendMsgBuf`
  - `checkReceive`
  - `readMsgBuf`
  - equivalent CAN health counters/state
- A draft compatibility layer now exists at `src/can_twai_compat.h`.
  It is intentionally not wired into the default firmware path yet.
- Draft default GPIOs:
  - ESP32 GPIO21 -> transceiver TXD/CTX
  - ESP32 GPIO22 -> transceiver RXD/CRX
- Draft queue sizing:
  - RX queue: 64 frames
  - TX queue: 32 frames
- Expose telemetry:
  - `can.backend`: `mcp2515` or `twai`
  - `can.bitrate`
  - `can.state`
  - `can.rxDrop`
  - `can.txFail`
  - `can.rxQueueMax`
  - `can.loadPct`

Validation:

- Boot with motors disabled.
- Confirm `ok:6` with polling or auto-return.
- Run HOLD at `C 0 0`, `L 20`, `AR 10`.
- Then try `L 10`.
- Success criteria:
  - No bus-off.
  - Encoder delivery stable.
  - No persistent RX queue drop.
  - Control loop rate close to requested rate.

### Phase 2 - TWAI at 1 Mbps

Purpose: increase bus bandwidth for 200-300 Hz work.

SERVO42D bitrate command:

```text
0x8A bitRate CRC
bitRate 0x03 = 1M
```

Safety:

- Changing CAN bitrate can temporarily strand motors if controller and motor bitrate diverge.
- Only do this with a human beside the rig.
- Motors should be disabled or mechanically safe.
- Prepare a recovery command path that can scan 500K and 1M.

Migration concept:

1. Start at 500K and verify all 6 motors respond.
2. Disable motion.
3. Send `0x8A 0x03` to each motor.
4. Switch controller to 1M.
5. Verify all 6 motors respond.
6. If any fail, scan known bitrates and restore a common bitrate.

Validation ladder:

1. 1M, `C 0 0`, `L 20`, `AR 10`.
2. 1M, `C 0 0`, `L 10`, `AR 10`.
3. 1M, `C 0 0`, `L 5`, `AR 5`.
4. Only then attempt 250-300 Hz cases.

## Traffic Budget Rules

Use:

```bash
node sysid/can_budget.js --control 200 --encoder 200 --ack 0 --bitrate 1000000
node sysid/can_budget.js --sweep
```

Rules of thumb:

- Under 45% estimated bus load: comfortable.
- 45-65%: usable.
- 65-80%: tight.
- Over 80%: redline.

For 200-300 Hz:

- Main running mode should use `C 0 0`.
- Avoid ACK on every F5.
- Encoder auto-return does not have to equal control frequency unless data proves it is needed.
- Consider sending F5 only when targets change meaningfully.

## Benchmark Command Examples

Dry run:

```bash
node sysid/can_benchmark.js --loops 20,10,5 --ar 10,5 --resp "0 0"
```

Live monitor-only benchmark:

```bash
node sysid/can_benchmark.js --live --loops 20,10,5 --ar 10,5 --resp "0 0" --seconds 20
```

Live benchmark that enables/disables each condition:

```bash
node sysid/can_benchmark.js --live --enable --loops 20,10,5 --ar 10,5 --resp "0 0" --seconds 20
```

Only run live commands when a human is beside the rig and power cutoff is reachable.

## Open Decisions

- TWAI GPIO pair to standardize in firmware.
- Whether final hardware should use non-isolated transceiver or isolated CAN.
- Whether 300 Hz requires lower encoder frequency, target-change F5 throttling, or a different command scheduling strategy.
- Whether to keep MCP2515 backend as a fallback after TWAI is stable.
