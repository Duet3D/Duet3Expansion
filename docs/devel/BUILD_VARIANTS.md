# Build Variants

How Duet3Expansion is sliced across boards. Setup and toolchain prerequisites are in [../DEVELOPER.md](../DEVELOPER.md); this file documents *what* gets built.

## 1. Make targets

The top-level [`Makefile`](../../Makefile) has one target per supported board.

| Target | Board | Processor | Drivers |
|---|---|---|---|
| `EXP3HC` | EXP3HC expansion | SAME5x | 3 |
| `TOOL1LC` | TOOL1LC tool board | SAMC21 | 1 |
| `TOOL1RR` | TOOL1RR tool board | RP2040 | 1 |
| `EXP1XD` | EXP1XD external-driver expansion | SAMC21 | 0 (passes step/dir to external driver) |
| `M23CL` | M23CL closed-loop driver | SAME5x | 1 (closed-loop) |
| `1HCL` | 1HCL single closed-loop tool | SAME5x | 1 (closed-loop) |
| `SZP` | Scanning Z-probe | SAMC21 | 0 |

`make all` builds every target.

## 2. Submodules

Three external submodules pulled in via `make init-submodules`:

| Submodule | Provides |
|---|---|
| `CoreN2G` | HAL wrappers for SAME5x / SAMC21 / RP2040 — peripherals, NVIC, USB, I²C, SPI. |
| `RRFLibraries` | `String`, `Bitmap`, `RTOSIface`, `NamedEnum`, etc. (subset shared with RRF.) |
| `CANlib` | The on-the-wire CAN-FD types — must match the version in RepRapFirmware. |

Note that this firmware does **not** pull in FreeRTOS as its own submodule — it shares a vendored copy via CoreN2G. The kernel version and the RTOSIface version must be aligned with whatever RRF is shipping for binary compatibility of CANlib structs (which is the only thing that actually crosses the bus).

## 3. Conditional compilation

The single biggest header is `src/Config/BoardDef.h` and per-board `Boards/*.h` files which set the feature flags.

| Flag | Meaning |
|---|---|
| `SAME5x` / `SAMC21` / `RP2040` | Processor selection — chosen by the make target. |
| `SUPPORT_DRIVERS` | Has any stepper drivers at all (false for MFM, EXP1XD, SZP). |
| `SINGLE_DRIVER` | Has exactly one driver (enables closed-loop path on supported boards). |
| `SUPPORT_TMC2660` / `SUPPORT_TMC22xx` / `SUPPORT_TMC51xx` | Which smart-driver IC is fitted. |
| `HAS_CPU_TEMP_SENSOR`, `HAS_VOLTAGE_MONITOR`, `HAS_VREF_MONITOR`, `HAS_12V_MONITOR` | ADC monitoring channels available. |
| `SUPPORT_THERMISTORS` | Board exposes thermistor inputs. |
| `SUPPORT_LIS3DH` | Onboard accelerometer. |
| `SUPPORT_LDC1612` | Onboard inductive scanning sensor. |
| `SUPPORT_AS5601` | Onboard magnetic encoder for filament monitor. |
| `SUPPORT_CLOSED_LOOP` | Build the ClosedLoop module. Implies `SINGLE_DRIVER`. |
| `SUPPORT_ADS131M02` | Onboard load-cell ADC (loadcell-based probes). |
| `USE_SERIAL_DEBUG` | Send `debugPrintf` over a UART; otherwise it goes over CAN. |

## 4. Output and deployment

Each `make` target produces a binary at the top level of the matching directory. The main board updates each expansion / tool board over CAN with `M997 B<addr>` — the user does not flash these by hand.

```
EXP3HC/Duet3Firmware_EXP3HC.bin
TOOL1LC/Duet3Firmware_TOOL1LC.bin
…
```

The release zip pulled from RepRapFirmware's release page contains all of these bundled together (see the RRF release notes). The main-board firmware contains the right loader behaviour to receive a chunked binary over CAN and pass it to the bootloader on the target board.

Bootloader update — separate path, used rarely (e.g. when the bootloader itself needs new functionality). Triggered by writing the magic value `UpdateBootloaderMagicValue` into NVM and rebooting; see [`AppMain`](../../src/Platform/Tasks.cpp) for the SAME5x / SAMC21 path and the RP2040 watchdog-scratch path.

## 5. Compatibility contracts

| Contract | Constant / source | Must match |
|---|---|---|
| CAN-FD message struct layouts | CANlib commit | RepRapFirmware build |
| Step clock rate (750 kHz on Duet 3) | `StepClockRate` in shared `Duet3Common.h` | RepRapFirmware build |
| `CanMessageType` enum values | CANlib | RepRapFirmware build |

Bumping any of these in CANlib without rebuilding both firmwares causes silent misinterpretation of frames — every message has a `messageType` field but the payload structures differ.

## 6. Where this connects to the rest of the system

- See [RepRapFirmware/docs/devel/BUILD_VARIANTS.md](../../../RepRapFirmware/docs/devel/BUILD_VARIANTS.md) for the matching matrix on the master side.
- See the integration overview in [DuetSoftwareFramework/docs/architecture/COMPATIBILITY.md](../../../DuetSoftwareFramework/docs/architecture/COMPATIBILITY.md) for the cross-repo version table.
