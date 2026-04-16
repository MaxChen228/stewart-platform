# Multi-Motor Synchronous Motion

Three methods for controlling multiple motors simultaneously.

## Method 1: Broadcast (CAN ID 0x00)

Send any motion command to CAN ID 0x00 → all motors on bus execute the same command. **No response from slaves.**

Example: `00 FD 01 2C 64 00 0C 80 B8` → all motors run FD command.

## Method 2: Group CAN ID

### 0x8D — Set Group CAN ID

DLC=4: `[8D, id_hi, id_lo, CRC]`
- id: 0x01-0x7FF

Send motion command to group ID → all motors in that group execute. **No response.**

Example setup for 6 motors:

| Motor | Slave CAN ID | Group CAN ID |
|-------|-------------|-------------|
| 1-3 | 1, 2, 3 | 0x50 |
| 4-6 | 4, 5, 6 | 0x51 |

Send to 0x50 → motors 1-3 execute. Send to 0x51 → motors 4-6 execute.

## Method 3: Synchronization Flag

Motors buffer commands and wait for sync trigger before executing. Allows different commands to different motors, all starting simultaneously.

### 0x4A — Enable/Disable Sync Flag

DLC=3: `[4A, enable, CRC]`
- 0x00=disable (default), 0x01=enable

Can be sent individually or via broadcast (ID=0x00).

### 0x4B — Sync Trigger (Broadcast)

Must be sent to CAN ID 0x00:
DLC=2: `[4B, CRC]`
No response.

**Important**: Consider sending sync command multiple times at ~1ms intervals for reliability, as some motors may miss it due to bus interference.

### Sync Procedure Example (2 motors)

1. Broadcast set mode: `00 [82 05 87]` (SR_vFOC)
2. Broadcast enable sync: `00 [4A 01 4B]`
3. Motor 1 speed command: `01 [F6 02 80 02 7B]` (→ buffered, responds status=5)
4. Motor 2 position command: `02 [FD 80 64 02 04 E2 00 CB]` (→ buffered, responds status=5)
5. Broadcast sync trigger: `00 [4B 4B]`
6. Both motors start simultaneously.
