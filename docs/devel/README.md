# Duet3Expansion Developer Documentation

This directory contains developer documentation for **Duet3Expansion** — the firmware that runs on Duet 3 expansion and tool boards. It is a deliberately slimmer firmware than the main-board [RepRapFirmware](https://github.com/Duet3D/RepRapFirmware): it has no G-code parser, no SD card, no network stack, and no print state machine. Its sole job is to act as a CAN-FD slave that does whatever the main board tells it to do — drive motors, read sensors, run heaters, blink LEDs — synchronously and accurately.

For build / setup instructions, see [../DEVELOPER.md](../DEVELOPER.md).

## How to read these docs

Start with [ARCHITECTURE.md](ARCHITECTURE.md). From there, jump into the subsystem most relevant to your work.

| Document | What it covers |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Top-level architecture, boot, FreeRTOS tasks, comparison to RRF, single-driver vs multi-driver builds. |
| [CAN_PROTOCOL.md](CAN_PROTOCOL.md) | The CAN-FD protocol from the slave's perspective: announce, time sync, motion reception, generic commands, replies. |
| [COMMAND_PROCESSING.md](COMMAND_PROCESSING.md) | The dispatch table in `CommandProcessor` — every CAN message type that the firmware handles. |
| [MOTION.md](MOTION.md) | Local motion: receiving a shaped move, scheduling against the master's step clock, `MoveSegment` / `DriveMovement` / step ISR. |
| [INPUT_MONITORS.md](INPUT_MONITORS.md) | Endstops, Z-probes, filament monitors, GP-in pins exposed back to the master via remote handles. |
| [CLOSED_LOOP.md](CLOSED_LOOP.md) | Closed-loop control on tool boards with magnetic encoders. |
| [BUILD_VARIANTS.md](BUILD_VARIANTS.md) | Per-board build matrix, processor flags, single-driver vs multi-driver paths. |

## Companion repositories

Duet3Expansion does not run alone. The other two repositories complete the system:

- **[RepRapFirmware](https://github.com/Duet3D/RepRapFirmware)** — runs on the main board. Acts as the CAN bus master. Parses G-code, plans motion, and forwards work to expansion boards as needed.
- **[DuetSoftwareFramework](https://github.com/Duet3D/DuetSoftwareFramework)** (DSF) — runs on a Linux SBC paired with the main board. Has no direct relationship with expansion boards; everything routes through the main board.

For the cross-repo picture (how a G-code from a browser ends up driving a stepper on a tool board), see the integration documentation under `DuetSoftwareFramework/docs/architecture/`.
