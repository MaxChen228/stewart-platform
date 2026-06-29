# Multi-Motor Synchronous Motion

> 🗺️ [[servo-can-hub|CAN 文檔總覽]] · 🔀 42ES 對應 [[servo42es/11-multi-motor-sync|Multi-Motor Sync]]

Three methods for controlling multiple motors simultaneously.

## Method 1: Broadcast (CAN ID 0x00)

Send any motion command to CAN ID 0x00 → all motors on bus execute the same command. **No response from slaves.**

Example: `00 FD 01 2C 64 00 0C 80 B8` → all motors run FD command.

## Method 2: Group CAN ID

### 0x8D — Set Group CAN ID

DLC=4: `[8D, id_hi, id_lo, CRC]`
- id: 0x01-0x7FF

Send motion command to group ID → all motors in that group execute. **No response.**

> 🔀 **vs 42ES：** 42D 的 `0x8D` 寫入即視為持久化（本 doc set 無顯式 SAVE 指令）；42ES 為 write-only 參數，設群組後須再下 [[servo42es/11-multi-motor-sync|0x60 SAVE]] 才會 power-cycle 保留。

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

> 🔀 **vs 42ES：** 同樣 DLC=3、`00` disable 預設 / `01` enable、可單發或廣播；差別是 42ES 此旗標 write-only，須加 [[servo42es/11-multi-motor-sync|0x60 SAVE]] 才保留，且 42ES 程序在 `0x4A` 後多一步顯式 SAVE。

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

> 🔀 **vs 42ES：** 上述 motion 幀（`01 F6 02 80 02 7B` / `02 FD 80 64 02 04 E2 00 CB`）與 sync 幀（`00 4A 01 4B` / `00 4B 4B`）在 [[servo42es/11-multi-motor-sync|42ES]] 程序中位元組完全相同；42ES 多兩步：起手廣播設模式（`00 82 05 87`）與 `0x4A` 後的顯式 SAVE（`00 60 01 62`），42D 此例不帶。
