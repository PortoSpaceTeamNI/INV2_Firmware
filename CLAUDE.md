# INV2_Firmware

Firmware for Porto Space Team's **INV2** hybrid (N2O/paraffin) sounding rocket and its
ground/filling station. The repo is a **PlatformIO monorepo** — one root `platformio.ini`,
one environment per board, all sharing a single `lib/` directory. **Every board is a
Raspberry Pi Pico (RP2040)** built with the Arduino framework via the
`maxgerhardt/platform-raspberrypi` platform — *not* an ESP32. (Some doc comments and the
`_old_code/` tree still mention ESP32; that is the previous-generation hardware and is stale.)

## Repository layout

```
INV2_Firmware/
├── platformio.ini          ; one env per board + [env:native] for host tests
├── lib/                    ; shared libraries — auto-discovered by PlatformIO LDF
│   ├── rocket_core/        ; canonical protocol types, parser, IDs, commands, states
│   ├── Crc/                ; CRC-16 (currently disabled on the wire, stubs in place)
│   ├── LoRa/               ; LoRa driver (CORTEX)
│   ├── AD5593R/            ; ADC/DAC IC (HYDRA)
│   ├── MAX31856/           ; thermocouple IC (HYDRA)
│   ├── SPI/                ; Adafruit_SPIDevice helper (HYDRA, Navigator)
│   ├── HX711/              ; load-cell ADC (LIFT)
│   ├── ArduinoEigen/       ; Eigen for Arduino (Navigator)
│   ├── ArxTypeTraits/      ; type-trait helpers for Eigen (Navigator)
│   ├── FastTrig/           ; fast trig approximations (Navigator)
│   ├── LPS/                ; LPS pressure sensor driver (Navigator, PocketSat)
│   └── HDC302x/            ; humidity sensor driver (PocketSat)
├── src/                    ; board sources — one subdirectory per board
│   ├── cortex/             ; headers + sources merged; build with [env:cortex]
│   ├── hydra/
│   ├── lift/
│   ├── navigator/
│   └── pocketsat/
├── test/
│   └── test_protocol/      ; Unity host tests for lib/rocket_core
└── _old_code/              ; retired ESP32/OBC-era firmware — reference only
```

Board-specific headers (`Pinout.h`, `IO_Map.h`, `HardwareCfg.h`, …) live inside
`src/<board>/` alongside the `.cpp` sources. Each env adds `-Isrc/<board>` to `build_flags`
so `#include "Pinout.h"` resolves to the right board's header without path prefix.

## Boards / environments

| Env(s) | Board role | Bus role | Key libs |
|--------|-----------|----------|----------|
| `cortex` | Flight computer — FreeRTOS task-based, master state machine | RS485 **master**, USB UART / LoRa operator interface | rocket_core, LoRa |
| `hydra_uf` / `hydra_lf` / `hydra_fs` | Hydraulic/actuation node — valves, thermocouples, pressures (**UF** upper feed, **LF** lower feed, **FS** fill station) | RS485 **slave** | rocket_core, AD5593R, MAX31856, SPI |
| `lift_tank` / `lift_bottle` / `lift_thrust` | Load-cell node — HX711 load cells + SD logging | RS485 **slave** | rocket_core, HX711, SdFat |
| `navigator` | Flight navigation — IMU/mag/baro fusion, quaternion Kalman filter | (ID reserved, not yet on bus) | ArduinoEigen, LSM6DS, LIS2MDL, BMP5xx, LPS |
| `pocketsat` | Standalone atmospheric payload | none | BME280, BMP5xx, LPS, HDC302x |

## Build & flash

```bash
pio run -e cortex                    # build CORTEX
pio run -e cortex -t upload          # build + flash CORTEX
pio run -e hydra_uf -t upload        # build + flash HYDRA upper-feed node
pio run -e lift_tank -t upload       # build + flash LIFT tank node
pio device monitor -e cortex         # serial monitor (115200 baud everywhere)
pio run                              # build ALL board instances
pio test -e native                   # host-side protocol unit tests (9/9)
```

Flash via Picoprobe (CMSIS-DAP): `upload_protocol = cmsis-dap` is set in all board envs.

**Board identity for multi-instance boards** is injected at compile time via the env's
`build_flags` — no source edits needed. Each HYDRA/LIFT env passes `-DDEFAULT_ID=...` /
`-DMY_ID=...`; the header falls back to a safe default if the flag is absent.

## Shared library — `lib/rocket_core/`

The single source of truth for everything that crosses the RS485/LoRa/UART boundary. All
bus boards (cortex, hydra, lift) pull it in automatically via the root `lib/` path — no
`lib_extra_dirs` or `lib_deps` entry needed.

| Header | Contents |
|--------|----------|
| `rc_commands.h` | `command_t` — canonical wire `cmd` byte enum (all boards) |
| `rc_manual.h`   | `manual_command_t` — sub-commands for `CMD_MANUAL_EXEC` payload[0] |
| `rc_ids.h`      | Board ID `#define`s (`MISSION_CONTROL_ID=0` … `BROADCAST_ID=0xFF`) |
| `rc_states.h`   | `state_t` — shared flight state enum |
| `rc_packet.h`   | `packet_t`, `SYNC_BYTE`, `MAX_PAYLOAD_SIZE`, `HEADER_SIZE`, `MAX_FRAME_SIZE` |
| `rc_parser.h/c` | Pure C `rc_parse_byte()` (byte-at-a-time FSM) and `rc_serialize()` |

## Communication protocol

All boards speak one framed byte protocol (RS485 between boards, USB UART to the operator,
LoRa for ground↔rocket telemetry):

```
0x55 | sender_id | target_id | cmd | payload_size | payload[payload_size] | crc_hi | crc_lo
```

Parsed by `rc_parse_byte` (state machine: `RC_SYNC → RC_SENDER_ID → … → RC_END`).
Serialised by `rc_serialize`. **There is one copy of both in `lib/rocket_core/`.**

- **CRC is disabled** (`#define CRC_ENABLED false`). Bytes are sent but not validated.
- **Command set** (`rc_commands.h`): `CMD_NONE=0, CMD_STATUS=1, CMD_ABORT=2, CMD_STOP=3,
  CMD_ARM=4, CMD_FIRE=5, CMD_FILL_EXEC=6, CMD_MANUAL_EXEC=7, CMD_ACK=8, CMD_NACK=9`.
  CORTEX→HYDRA traffic uses `CMD_STATUS` for polling and `CMD_MANUAL_EXEC` (with a
  `manual_command_t` in `payload[0]`) for valve control.
- **Board IDs** (`rc_ids.h`): `MISSION_CONTROL_ID=0, CORTEX_ID=1, HYDRA_UF_ID=2,
  HYDRA_LF_ID=3, HYDRA_FS_ID=4, NAVIGATOR_ID=5, LIFT_TANK_ID=6, LIFT_BOTTLE_ID=7,
  LIFT_THRUST_ID=8, BROADCAST_ID=0xFF`. Aliases `GROUND_ID`/`OBC_ID` kept for compatibility.

## CORTEX flight computer

CORTEX is a **FreeRTOS task-based** architecture. Tasks communicate via queues; sensor data
is shared through a mutex-protected `RocketData` struct.

| Task | File | Responsibility |
|------|------|----------------|
| `MissionControl` | `MissionControl.cpp` | Reads operator commands (USB UART / LoRa), enqueues to StateMachine |
| `StateMachine`   | `StateMachine.cpp`   | Owns the flight state machine; drives filling and flight sequences |
| `DataPolling`    | `DataPolling.cpp`    | Round-robin polls HYDRA/LIFT slaves, updates `RocketData` |
| `RS485`          | `RS485.cpp`          | Low-level RS485 send/receive, direction-pin control |

State progression: `IDLE → FILL_N2 → PRE_PRESSURIZE → FILL_OX → POST_PRESSURIZE → ARMED →
IGNITION → BOOST → COAST → DROGUE_DESCENT → MAIN_DESCENT → TOUCHDOWN → RECOVERY → ABORT`.

**Valve routing** (`ValveRouting.cpp`): maps logical `valve_t` → (HYDRA node ID, physical
valve index). Non-obvious mapping — this is the one place to change it.

## Gotchas / things to watch

- **Command numbering is unified** via `rc_commands.h`. CORTEX's numbering is canonical. If
  you add or reorder commands, update the pinned-value tests in `test/test_protocol/test_main.cpp`.
- **HYDRA's `CMD_MANUAL_VALVE_MS` handler blocks** with `delay(ms)`. The `ms` field is a
  `uint16_t` at `payload[3..4]` (little-endian), giving up to 65535 ms on the wire.
- CRC stubs (`check_crc()`) return `true` — harmless only because `CRC_ENABLED false`.
- `src/hydra/VeryImportantFile.h` plays a MIDI tune on the buzzer. Not load-bearing.
- `navigator` and `pocketsat` are not yet wired into the RS485 command bus.
- Include order in LIFT: `rc_ids.h` must precede `HardwareCfg.h` because `HardwareCfg.h`
  uses `LIFT_BOTTLE_ID` (from `rc_ids.h`) to define `MY_ID`.
- Multi-byte wire fields are little-endian; use `memcpy` for unaligned reads (e.g. `uint16_t
  ms` in `CMD_MANUAL_VALVE_MS`).

## Conventions

- C++ / Arduino framework, RP2040. 115200 baud serial throughout.
- Sensor/peripheral drivers are grouped under `src/<board>/Sensors/` or `src/<board>/Peripherals/`
  with matching headers in the same directory. Pin assignments live in `Pinout.h` /
  `IO_Map.h` / `HardwareCfg.h` per board.
- Prefer fixed-width integers and `__attribute__((packed))` for anything that goes on the
  wire or to storage — the data models rely on exact layout.
