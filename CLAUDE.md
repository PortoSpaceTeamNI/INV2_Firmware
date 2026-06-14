# INV2_Firmware

Firmware for Porto Space Team's **INV2** hybrid (N2O/paraffin) sounding rocket and its
ground/filling station. The repo is a collection of independent
[PlatformIO](https://platformio.org/) projects, one per physical board. **Every board is a
Raspberry Pi Pico (RP2040)** built with the Arduino framework via the
`maxgerhardt/platform-raspberrypi` platform — *not* an ESP32. (Some doc comments and the
`_old_code/` tree still mention ESP32; that is the previous-generation hardware and is stale.)

## Boards / sub-projects

Each top-level directory is a standalone PlatformIO project with its own `platformio.ini`,
`src/`, `include/`, and `lib/`. Build/flash each independently.

| Dir          | Role | Bus role | Key hardware / libs |
|--------------|------|----------|---------------------|
| `OBC/`       | On-Board Computer — central flight + ground controller, runs the master state machine | RS485 **master**, takes operator commands over USB UART (LoRa planned) | LoRa, LittleFS, SerialFlash, Crc |
| `HYDRA/`     | Hydraulic/actuation node — drives valves, reads thermocouples + pressures. Three instances: **UF** (upper feed), **LF** (lower feed), **FS** (fill station) | RS485 **slave** | AD5593R (ADC/DAC), MAX31856 (thermocouple), valves |
| `LIFT/`      | Load-cell node — HX711 load cells + SD logging. Three instances: **TANK**, **BOTTLE**, **THRUST** | RS485 **slave** | SdFat, FreeRTOS (dual-core) |
| `Navigator/` | Flight navigation computer — IMU/mag/baro fusion via a quaternion Kalman filter | (RS485 ID reserved, not yet wired into the bus) | ArduinoEigen, LSM6DS, LIS2MDL, BMP5xx, LPS |
| `PocketSat/` | Standalone atmospheric sensor payload | none | BME280, BMP5xx, LPS, HDC302x |
| `_old_code/` | Previous-generation (ESP32/Arduino) firmware + Python ground tools. **Reference only — do not modify or treat as current.** | | |

## Build & flash

PlatformIO, per project directory. From the repo root:

```bash
pio run -d OBC                 # build
pio run -d OBC -t upload       # build + flash
pio device monitor -d OBC      # serial monitor (115200 baud everywhere)
```

Most boards flash via a Picoprobe (CMSIS-DAP) — see `upload_protocol = cmsis-dap` in the
relevant `platformio.ini`. OBC/Navigator/PocketSat have that line commented out in places;
uncomment for picoprobe or use BOOTSEL/UF2.

## Communication protocol

All boards speak one framed byte protocol (RS485 between boards, USB UART to the operator,
LoRa for ground↔rocket telemetry). The frame is parsed by a byte-at-a-time state machine
(`SYNC → SENDER_ID → TARGET_ID → CMD → PAYLOAD_SIZE → PAYLOAD → CRC1 → CRC2`):

```
0x55 | sender_id | target_id | cmd | payload_size | payload[payload_size] | crc_hi | crc_lo
```

- The **command set** (`command_t`), the `CMD_MANUAL_*` sub-commands (`manual_command_t`),
  board IDs, and the canonical `state_t` now live **once** in the shared library
  `lib/rocket_core/` (`rc_commands.h`, `rc_manual.h`, `rc_ids.h`, `rc_states.h`). Boards pull
  it in via `lib_extra_dirs = ../lib`. Change a command/ID there, not per-board. Host-side
  unit tests for this core run with `pio test -e native` (root `platformio.ini`).
- `packet_t`, the byte parser, and the frame `#define`s are **still duplicated** in each
  project's `Comms.h` (migration to `rocket_core`'s `rc_packet.h` / `rc_parser.*` is pending —
  see `Documentation/monorepo-shared-lib-refactor.md`). Until then, a wire-format change must
  be applied in *every* `*/include/Comms.h` (OBC, HYDRA, LIFT) and kept byte-compatible.
- **CRC is currently disabled** everywhere (`#define CRC_ENABLED false`; OBC's `check_crc()`
  is a stub that returns `true`). CRC bytes are still transmitted but not validated.
- Board IDs are global and consistent (now in `rc_ids.h`): `GROUND=0, OBC=1, HYDRA_UF=2,
  HYDRA_LF=3, HYDRA_FS=4, NAVIGATOR=5, LIFT_TANK=6, LIFT_BOTTLE=7, LIFT_THRUST=8,
  BROADCAST=0xFF`.
- **Per-board identity is set at compile time, manually**: HYDRA sets `DEFAULT_ID` in
  `HYDRA/include/Comms.h`; LIFT sets `MY_ID` in `LIFT/include/HardwareCfg.h`. You must edit
  and reflash the correct value for each physical unit.

## OBC state machine

The heart of the system. `OBC/src/main.cpp` runs a single super-loop that, each pass: runs
the current state's periodic **work** functions, polls the next RS485 slave, parses any slave
reply, evaluates the current state's **events**, then reads/executes an operator command.

The machine is **table-driven** (`OBC/src/StateMachine.cpp`):
- `state_t` (in `OBC/include/DataModels.h`): `IDLE, FILLING, SAFE_IDLE, FILLING_N2,
  PRE_PRESSURE, FILLING_N2O, POST_PRESSURE, READY, ARMED, IGNITION, LAUNCH, FLIGHT, RECOVERY,
  ABORT`. `state_count` must stay last; `S_NONE = -1`.
- `expected_state[state][cmd]` is the transition lookup table — a command only changes state
  if its entry differs from the current state. `ABORT` and `STOP` are reachable from almost
  everywhere; the `CMD_FILL_*` family drives the filling sub-states.
- Each state has `work[]` (periodic sampling/control via `sm_work_t`) and `events[]`
  (condition→reaction→next-state via `sm_event_t`). Most are currently empty stubs; filled
  ones live in `StMWork.cpp` / `StMEvent.cpp` (e.g. safety vent in `SAFE_IDLE`, launch
  override timeout in `IGNITION`).
- Command-driven transitions take precedence over event-driven ones.

Operator commands (`command_t`, from `lib/rocket_core/rc_commands.h`) and their handlers live in `OBC/src/StMComms.cpp`
(`run_command` → `handle_*_cmd`). `CMD_STATUS` serializes the whole `system_data_t`
(pressures, thermocouples, actuators bitmap, loadcells) back to the requester.

OBC↔slave glue: `OBC/src/HYDRA.cpp` and `OBC/src/LIFT.cpp` round-robin poll each slave
(`fetch_next_*`), map slave readings into `system_data`, and translate logical valves
(`valve_t`) to physical HYDRA valves in `OBC/src/Valves.cpp`. **Note the logical→physical
valve mapping is non-obvious** (e.g. the abort valve is HYDRA_LF controlled-1, vent is
HYDRA_UF controlled-1) — see `Valves.cpp` and `update_data_from_hydra()`.

## Gotchas / things to watch

- **Command numbering is now unified** via `lib/rocket_core/rc_commands.h` (one `command_t`
  for all boards). This replaced the old divergence where OBC's `cmd_type_t` and the slaves'
  `command_t` gave the same name different values, *and* the OBC↔slave sub-protocol used a
  third set (`HCMD_*`/`LCMD_*`) — OBC even polled with `HCMD_STATUS = 0` while slaves checked
  `CMD_STATUS = 1`, so status polling never matched. OBC↔slave traffic now uses `CMD_STATUS`
  for polling and `CMD_MANUAL_EXEC` (+ a `manual_command_t` byte in `payload[0]`) for valves.
- HYDRA's blocking `CMD_MANUAL_VALVE_MS` handler uses `delay(ms)` in the main loop (there's a
  TODO to make it non-blocking). It also reads the duration as a single byte, so OBC's
  `set_hydra_valve_ms` is currently capped at 255 ms on the wire.
- Several `crc(...)` calls in `StMComms.cpp` pass `&packet_rep` where `packet_rep` is already
  a pointer — harmless only because CRC is disabled.
- `HYDRA/include/VeryImportantFile.h` is exactly what it sounds like: it plays a MIDI tune on
  the buzzer. Not load-bearing.
- `Navigator` and `PocketSat` are independent sensor/compute boards and are not yet
  participants in the RS485 command bus.

## Conventions

- C++ / Arduino framework, RP2040. 115200 baud serial throughout.
- Sensor/peripheral drivers are grouped under `src/Sensors/` or `src/Peripherals/` with
  matching headers under `include/`. Pin assignments live in `Pinout.h` / `IO_Map.h` /
  `HardwareCfg.h` per board.
- Prefer fixed-width integers and `__attribute__((packed))` for anything that goes on the wire
  or to storage — the data models rely on exact layout.

> Note: Claude Code auto-loads `CLAUDE.md` (uppercase). This file is `Claude.md`; on a
> case-sensitive filesystem rename it to `CLAUDE.md` if you want it picked up automatically.
