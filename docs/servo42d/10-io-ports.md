# IO Port Commands

## 0x34 — Read IO Port Status

DLC=2: `[34, CRC]`
Response DLC=3: `[34, status, CRC]`

| Bit | Port |
|-----|------|
| bit0 | IN_1 |
| bit1 | IN_2 |
| bit2 | OUT_1 |
| bit3 | OUT_2 |

After 0x9E limit remapping: bit0=En state, bit1=Dir state.

## 0x9F — Configure IN_1 Mode (28D/35D/42D Only)

DLC=3: `[9F, type, CRC]`
- type=0: IN_1 as input port (default)
- type=1: IN_1 as output port

When configured as output, bit2 of 0x36 writes directly to IN_1 without OUT1_mask restriction.

## 0x36 — Write IO Port Data

DLC=3: `[36, data, CRC]`

| Bit | Function |
|-----|----------|
| bit7 | OUT2_mask: 0=don't write, 1=write OUT_2 value, 2=unchanged |
| bit6 | OUT1_mask: 0=don't write, 1=write OUT_1 value, 2=unchanged |
| bit5 | OUT_2 write value (0/1) |
| bit4 | OUT_1/IN_1 write value (0/1) |
| bit1-0 | Reserved (0) |

Response: 0=write failed, 1=write successful

## 0x99 — Pulse Frequency Division Output (57D Only)

DLC=7: `[99, divLevel, divPeriod(uint32), CRC]`

- divLevel: 0=low start level (default), 1=high start level
- divPeriod: Frequency division period (default 0)
  - <100: no output
  - ≥100: PEND port (OUT_2) toggles once every divPeriod pulse cycle
  - Set to 0 to disable

Example: 16 subdivisions, divPeriod=3200 → OUT_2 flips once per revolution.
