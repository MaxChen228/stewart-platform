# Research Dashboard

Generated: 2026-06-27T05:27:17.170Z

## Read This First

- Current best heave finding: use a clean inner loop and move static height compensation upward.
- Best tested candidate: **pure-P + zBias 4.5**; final height error -0.07mm, cross-axis peak 0.78mm.
- Practical interpretation: joint Ki makes height accurate, but it injects much larger MIMO coupling. A simple Z bias proves the compensation belongs above the joint loop; it is not the final controller.
- Professional target: replace fixed bias with a calibrated static compensation model, then let pose-space LQI remove the remaining slow error.
- Still unresolved: MCP2515/500K continues to show CAN error flags and RX drops during motion; treat high-rate control results with caution.

## Best Known Operating Direction

| Layer | Recommendation | Evidence |
|---|---|---|
| Inner motor PID | Prefer pure-P / low Ki for clean dynamics | pure-P cross peak 0.84mm vs Ki=20 2.12mm |
| Height compensation | Use zBias ~4.5mm only as a local calibration point | final error -0.07mm at this one operating point |
| Professional compensation | Fit a static pose/load compensation map, not one fixed constant | generalizes across height, tilt, payload, and workspace position |
| Next controller | pose-space feedforward + LQI residual, then LQR/MPC | avoids six independent joint integrators fighting each other |
| Data plane | Replace/upgrade MCP2515 path before trusting 200-300Hz results | CAN ef/rxDrop remains visible in all motion runs |

## Heave Evidence Table

| Case | PID | zBias | actual final Z | final err vs 143 | cross peak | cross/Z | CAN ef/rxDrop | Data | Plots |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| baseline mixed | [1024,20,0,0] | 0.0mm | - | - | 4.78mm | 21.7% | 85/231 | [jsonl](../sysid/data/heave10_ms5000_baseline_heave-step_h38_ms5000_2026-06-27T04-50-29.jsonl) | [plane](../sysid/data/plots/heave10_ms5000_baseline_heave-step_h38_ms5000_2026-06-27T04-50-29_plane6.svg) / [motor](../sysid/data/plots/heave10_ms5000_baseline_heave-step_h38_ms5000_2026-06-27T04-50-29_motor6.svg) |
| Ki=20 | [1024,20,0,0] | 0.0mm | 143.08mm | 0.08mm | 2.12mm | 20.2% | 85/153 | [jsonl](../sysid/data/clean_heave10_ms5000_heave-step_h38_ms5000_2026-06-27T05-12-32.jsonl) | [plane](../sysid/data/plots/clean_heave10_ms5000_heave-step_h38_ms5000_2026-06-27T05-12-32_plane6.svg) / [motor](../sysid/data/plots/clean_heave10_ms5000_heave-step_h38_ms5000_2026-06-27T05-12-32_motor6.svg) |
| pure-P | [1024,0,0,0] | 0.0mm | 138.85mm | -4.15mm | 0.84mm | 8.6% | 85/146 | [jsonl](../sysid/data/clean_heave10_ms5000_pureP_heave-step_h38_ms5000_2026-06-27T05-13-43.jsonl) | [plane](../sysid/data/plots/clean_heave10_ms5000_pureP_heave-step_h38_ms5000_2026-06-27T05-13-43_plane6.svg) / [motor](../sysid/data/plots/clean_heave10_ms5000_pureP_heave-step_h38_ms5000_2026-06-27T05-13-43_motor6.svg) |
| pure-P + zBias 4.0 | [1024,0,0,0] | 4.0mm | 142.49mm | -0.51mm | 0.84mm | 8.7% | 85/155 | [jsonl](../sysid/data/clean_heave10_ms5000_pureP_zbias4_heave-step_h38_ms5000_2026-06-27T05-18-57.jsonl) | [plane](../sysid/data/plots/clean_heave10_ms5000_pureP_zbias4_heave-step_h38_ms5000_2026-06-27T05-18-57_plane6.svg) / [motor](../sysid/data/plots/clean_heave10_ms5000_pureP_zbias4_heave-step_h38_ms5000_2026-06-27T05-18-57_motor6.svg) |
| pure-P + zBias 4.5 | [1024,0,0,0] | 4.5mm | 142.93mm | -0.07mm | 0.78mm | 8.0% | 85/154 | [jsonl](../sysid/data/clean_heave10_ms5000_pureP_zbias4p5_heave-step_h38_ms5000_2026-06-27T05-19-39.jsonl) | [plane](../sysid/data/plots/clean_heave10_ms5000_pureP_zbias4p5_heave-step_h38_ms5000_2026-06-27T05-19-39_plane6.svg) / [motor](../sysid/data/plots/clean_heave10_ms5000_pureP_zbias4p5_heave-step_h38_ms5000_2026-06-27T05-19-39_motor6.svg) |

## What To Do Next

1. Treat `zBias=4.5mm` as an anchor point, not a final setting.
2. Repeat clean sweeps for heave +5, +10, +20mm and a few tilted poses to fit `bias = f(pose, load)`.
3. Keep the fitted compensation as feedforward; do not hide it inside joint Ki.
4. Add pose-space LQI only for the residual slow error after feedforward.
5. Keep recorder main-only for sysid; landing/release data belongs in safety logs, not model fitting.

## Data Inventory

- Heave runs indexed: 5
- FreeRTOS/CAN diagnostic files: 9
- Battery/PID disturbance files: 10
- Machine-readable index: [research-index.json](../sysid/data/research-index.json)

## How To Refresh

```bash
npm run research:index
```
