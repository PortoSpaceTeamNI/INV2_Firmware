# Refactor: Monorepo with a Shared Core Library

**Status:** Phases 0–5 complete. Full Option-3 monorepo layout achieved: single
`platformio.ini`, all sources under `src/<board>/`, all libraries under `lib/`. Remaining work
is optional quality-of-life (see §9 Still TODO).
**Scope:** Repository layout + shared code extraction. Command-protocol unification *does*
change behaviour on the wire (intentionally — it fixes a divergence bug); see §9.
**Author:** drafted with Claude

---

## 1. Why

Today each board is a fully independent PlatformIO project (`OBC/`, `HYDRA/`, `LIFT/`,
`Navigator/`, `PocketSat/`). There is **no mechanism to share code**, so the parts that
*must* be identical across boards are instead copy-pasted and have started to drift:

| Duplicated thing | Where | Problem |
|------------------|-------|---------|
| `packet_t`, frame parser, `write_packet`/`read_packet` skeleton | `OBC/include/Comms.h`, `HYDRA/include/Comms.h`, `LIFT/include/Comms.h` | Three copies of the wire format; any change must be hand-applied 3× and kept byte-compatible. |
| Board ID `#define`s (`OBC_ID`, `HYDRA_UF_ID`, …) | same three files | Already inconsistent — LIFT defines `GROUND_ID 0`, the others don't. |
| **Command enum** (`cmd_type_t` on OBC vs `command_t` on slaves) | `OBC/include/DataModels.h` vs `HYDRA`/`LIFT` `DataModels.h` | **Live bug:** the enums have diverged, so the same command name has a different integer value on OBC vs a slave (e.g. `CMD_ARM` = 5 on OBC, 4 on a slave). Only `CMD_STATUS`/`CMD_MANUAL_EXEC` happen to line up. |
| `Crc` library | vendored in `HYDRA/lib/`, `Navigator/lib/` (and `_old_code/`) | Multiple copies of the same CRC implementation. |
| `Adafruit_SPIDevice` (SPI helper) | `HYDRA/lib/SPI/`, `Navigator/lib/SPI/` | Same. |
| `LPS` driver | `Navigator/lib/LPS/`, `PocketSat/lib/LPS/` | Same. |

There is also **no place to unit-test** the protocol/state-machine logic off-target. The
enum-divergence bug above is exactly the kind of defect a host-side (`native`) test would
have caught immediately.

**Goal:** one canonical home for cross-board code, consumed by every board, with a single
place to run host tests.

## 2. Approach

Adopt a **monorepo with a single root `platformio.ini`** (the "Option 3" direction), with two
adaptations that the generic recipe leaves out:

- The centrepiece is a shared library, **`lib/rocket_core/`**, holding the one true copy of the
  protocol, IDs, command + state enums, and shared data models. This is where most of the
  value is and it is what fixes the divergence bug.
- Because a single PlatformIO project has exactly **one** `include/` directory, **board-specific
  headers move next to their board's sources** (under `src/<board>/`) and are exposed with a
  per-environment include flag — they cannot all share a single global `include/`.

Each board becomes a build *environment* (`[env:obc]`, `[env:hydra]`, …) rather than a
separate project directory. External dependencies stay **per-environment** via `lib_deps`, so
Navigator's Eigen is never compiled into HYDRA.

### Why not the literal Option-3 example

The commonly-shown snippet uses `build_src_filter = +<../src-ignition/>`, reaching out of
`src/` with `../`. `build_src_filter` paths are relative to `src_dir`; keeping board folders
*inside* `src/` and using exclude-then-include is cleaner and avoids surprising path
behaviour. See the corrected `platformio.ini` below.

### Why not git submodules / separate repos

Submodules add per-clone friction for a small student team and make atomic cross-board changes
(e.g. "bump the protocol version") painful. Separate repos only pay off when different
subteams deploy boards independently — not the case here (one rocket, one flash session). A
monorepo keeps a protocol change and all its call-site updates in a single commit/PR.

## 3. Target layout

```
INV2_Firmware/
├── platformio.ini                 ; one file, one env per board + a native test env
├── lib/
│   ├── rocket_core/               ; SHARED, auto-discovered by PlatformIO LDF
│   │   ├── protocol/              ; packet_t, framing, board IDs
│   │   ├── commands.h             ; ONE canonical command enum (fixes divergence)
│   │   ├── states.h               ; state_t, shared across OBC + future participants
│   │   └── data_models/           ; on-the-wire structs (packed)
│   ├── Crc/                       ; de-duplicated, single copy
│   └── SPIHelper/                 ; de-duplicated Adafruit_SPIDevice, single copy
├── src/
│   ├── obc/                       ; OBC sources + obc-only headers (Pinout/HardwareCfg/...)
│   ├── hydra/
│   ├── lift/
│   ├── navigator/
│   └── pocketsat/
├── test/
│   └── test_protocol/             ; Unity tests for rocket_core (run on native)
└── Documentation/
    └── monorepo-shared-lib-refactor.md   ; this file
```

Board-specific headers (`Pinout.h`, `IO_Map.h`, `HardwareCfg.h`, board pin maps) live inside
`src/<board>/` and are reached with `-Isrc/<board>` per environment. Anything that crosses the
RS485/LoRa/UART boundary, or that two boards must agree on, belongs in `lib/rocket_core/`.

## 4. `platformio.ini` (corrected form)

```ini
[common]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico
framework = arduino
monitor_speed = 115200

[env:obc]
extends = common
build_src_filter = -<*> +<obc/>
build_flags = -Isrc/obc
lib_deps =
    LoRa
    ; LittleFS / SerialFlash as currently used

[env:hydra]
extends = common
build_src_filter = -<*> +<hydra/>
build_flags = -Isrc/hydra
; HYDRA still selects its identity at build time:
;   -DBOARD_ID=HYDRA_UF_ID   (replaces the manual edit of Comms.h DEFAULT_ID)

[env:lift]
extends = common
build_src_filter = -<*> +<lift/>
build_flags = -Isrc/lift
lib_deps =
    https://github.com/greiman/SdFat
build_unflags =
; LIFT identity:  -DBOARD_ID=LIFT_TANK_ID  (replaces MY_ID in HardwareCfg.h)

[env:navigator]
extends = common
build_src_filter = -<*> +<navigator/>
build_flags = -Isrc/navigator
lib_deps =
    adafruit/Adafruit BMP5xx Library
    adafruit/Adafruit LSM6DS
    adafruit/Adafruit LIS2MDL
    adafruit/Adafruit Unified Sensor
    ; ArduinoEigen is vendored in lib/

[env:pocketsat]
extends = common
build_src_filter = -<*> +<pocketsat/>
build_flags = -Isrc/pocketsat
lib_deps =
    adafruit/Adafruit BMP5xx Library
    BME280
    adafruit/Adafruit BusIO

; Host-side unit tests for the shared protocol/state code. No board, no Arduino.
[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>          ; tests pull in lib/rocket_core directly
```

Key points:
- `build_src_filter = -<*> +<board/>` compiles **only** that board's sources; the shared
  `lib/rocket_core` is pulled in automatically by the Library Dependency Finder wherever it is
  `#include`d.
- Per-board identity moves from a hand-edited `#define` in a committed header to a
  `-DBOARD_ID=...` build flag, so flashing the three HYDRAs / three LIFTs is a choice of
  environment/flag, not a source edit you must remember to revert.

## 5. Making `rocket_core` host-testable

For `pio test -e native` to work, the shared headers must be **free of `Arduino.h` and other
target-only includes**. Two concrete changes are needed during extraction:

- `OBC/include/DataModels.h` currently `#include <Arduino.h>` — drop it; the data models only
  need `<stdint.h>`.
- The frame parser uses `clock()`/`millis()` for timeouts. Split the **pure** parsing/
  serialization (testable) from the **I/O + timing** (board-specific). The byte state machine
  in `parse_input()` is already pure and moves cleanly into `rocket_core`; `read_packet()`'s
  transport/timeout stays per-board.

First test to write (it would have caught the current bug):

```c
// test_protocol/test_commands.c
TEST_ASSERT_EQUAL(CMD_ARM, expected_arm_value);   // same value everywhere now
TEST_ASSERT_EQUAL(roundtrip(pkt).cmd, pkt.cmd);   // serialize → parse is identity
```

## 6. Migration plan (incremental, one board at a time)

The repo keeps building throughout — do not move all five boards at once.

1. **Scaffold** `lib/rocket_core/` + root `platformio.ini` with just `[env:native]` and the
   shared protocol moved out of OBC. Add the first Unity tests. Nothing else changes yet.
2. **Fix the canonical enum**: define one `command_t` in `rocket_core/commands.h`. Decide the
   final ordering; update every handler/`switch` that referenced the old per-board values.
   Lock it with a test.
3. **Migrate OBC** to `src/obc/` + `[env:obc]`, deleting `OBC/include/Comms.h`'s duplicate in
   favour of the shared header. Verify build + a hardware smoke test.
4. **Migrate HYDRA, then LIFT** the same way; replace `DEFAULT_ID`/`MY_ID` edits with
   `-DBOARD_ID`. These two share the most with OBC, so they validate the shared lib hardest.
5. **Migrate Navigator and PocketSat** (lowest coupling — they don't ride the command bus yet).
   Fold their duplicated `Crc`, `SPIHelper`, `LPS` into the shared `lib/`.
6. **Delete** the old per-board project directories once their env builds and flashes.
7. Point CI at the root: `pio run` (all envs) + `pio test -e native`.

Each step is its own PR/commit and leaves the tree releasable.

## 7. Risks & mitigations

| Risk | Mitigation |
|------|------------|
| Single global `include/` collides across boards | Board headers live in `src/<board>/`, exposed via per-env `-Isrc/<board>`; only truly shared headers go in `lib/`. |
| Re-ordering the command enum changes wire values and breaks an un-migrated board / the ground GUI | Do the enum fix (step 2) as one explicit, reviewed change; document the new values; update the ground tooling in the same PR. |
| `native` build fails because shared code drags in `Arduino.h` | Keep `rocket_core` target-agnostic; split pure logic from I/O (section 5). |
| Differing `lib_deps` accidentally compiled into the wrong board | `lib_deps` is per-env; `build_src_filter` excludes other boards' sources. Verify each env builds in isolation. |
| Big-bang refactor stalls / blocks the team | Strictly incremental (section 6); repo stays green after every step. |

## 8. Explicitly out of scope

- No change to state-machine behaviour, valve mappings, or control logic.
- No change to the on-the-wire frame format (`0x55 | sender | target | cmd | size | payload |
  crc`). The bytes stay identical; only the *source of truth* for them is consolidated.
- Enabling CRC (still `CRC_ENABLED false`) is a separate follow-up, though the shared lib makes
  it a one-place change when the team is ready.

## 9. Progress log

### Phase 0 — shared core + command-enum unification (DONE, builds verified)

Rather than restructure all five board directories at once, the first landed step shares the
new core **without moving any board sources**, so each board keeps building from its own
directory and the change is reviewable in isolation:

- **`lib/rocket_core/`** created at the repo root, target-agnostic (no `Arduino.h`):
  `rc_packet.h` (frame), `rc_parser.{h,c}` (pure byte parser + serializer, lifted from the
  per-board `parse_input`), `rc_ids.h`, `rc_commands.h`, `rc_manual.h`, `rc_states.h`.
  The parser also gained a **payload-size clamp** the per-board copies lack (a corrupt length
  byte could previously overflow `payload[]`).
- **Root `platformio.ini`** with `[env:native]` (Unity, host). Each board's `platformio.ini`
  gained `lib_extra_dirs = ../lib` so PlatformIO discovers `rocket_core`. (This is the
  pragmatic incremental alternative to the single-root-`platformio.ini` layout in §3/§4; the
  full directory move can still happen later, but is no longer a prerequisite for sharing
  code.)
- **Command protocol unified onto the canonical `command_t`** (`rc_commands.h`):
  - Deleted OBC's `cmd_type_t`, HYDRA's & LIFT's local `command_t`, and the OBC-side
    sub-protocol enums `hydra_cmd_t` (`HCMD_*`) and `lift_cmd_t` (`LCMD_*`).
  - OBC↔slave polling now sends `CMD_STATUS`; valve actions send `CMD_MANUAL_EXEC` with a
    `manual_command_t` sub-command in `payload[0]` (`rc_manual.h`), matching HYDRA's receiver.
  - **Bug fixed:** OBC previously polled with `HCMD_STATUS = 0` while HYDRA/LIFT checked
    `CMD_STATUS = 1`, so status polling could never match. Both ends now share the value.
  - OBC's `expected_state` transition table grew a `CMD_NACK` column (no-op, like `CMD_ACK`)
    and is sized by `CMD_COUNT`.
- **Tests:** `test/test_protocol/` pins the canonical command / manual / ID / state values and
  covers serialize↔parse roundtrip, resync, and the payload clamp. `pio test -e native` →
  9/9 green. `pio run -d {HYDRA,LIFT,OBC}` all build with `rocket_core` in the dependency graph.

### Phase 1 — CORTEX adopted as bus master; command numbering re-canonicalised (DONE, builds verified)

`CORTEX/CORTEX_V1/` (a FreeRTOS, task-based rewrite of the OBC: MissionControl /
StateMachine / DataPolling / RS485 tasks, mutex-protected `RocketData`, valve-routing table,
richer flight state machine) is now **the** flight computer and bus master. The
previous-generation `OBC/` super-loop is superseded.

CORTEX had re-introduced the very divergence Phase 0 removed — it shipped its own `command_t`,
`manual_command_t`, `packet_t`, and board-ID `#define`s, none of them from `rocket_core`. Most
damaging: CORTEX numbered `CMD_MANUAL_EXEC = 7` while the `rocket_core`-based HYDRA/LIFT had it
at `10`, so CORTEX valve-actuation frames were decoded by HYDRA as `CMD_LAUNCH_OVERRIDE` —
manual valve control across CORTEX↔HYDRA was silently broken. (Polling worked only because
`CMD_STATUS = 1` happened to line up.)

Resolution (decision: **CORTEX's numbering is canonical**, since it is the master):

- **`rc_commands.h` re-numbered** to CORTEX's leaner set: `NONE,STATUS,ABORT,STOP,ARM,FIRE,
  FILL_EXEC,MANUAL_EXEC,ACK,NACK,CMD_COUNT`. The old OBC-only `READY`/`LAUNCH_OVERRIDE`/
  `FILL_RESUME` commands are gone — CORTEX drives those phases through its event-based state
  machine, not dedicated wire commands. HYDRA/LIFT only reference `CMD_STATUS`/`CMD_ACK`/
  `CMD_MANUAL_EXEC` by name, so they pick up the new values automatically and stay
  wire-compatible with the master.
- **`rc_ids.h`** gained the canonical CORTEX-era names `MISSION_CONTROL_ID` (0) and `CORTEX_ID`
  (1), with `GROUND_ID`/`OBC_ID` kept as backwards-compatible aliases (same values).
- **CORTEX now consumes `rocket_core`**: its `Commands.h` includes `rc_commands.h` +
  `rc_manual.h` (keeping only the CORTEX/GUI-local `fill_command_t` and the error `#define`s);
  its `Communications.h` includes `rc_ids.h` instead of local ID `#define`s; `platformio.ini`
  gained `lib_extra_dirs = ../../lib`.
- **Tests:** `test_command_values_pinned` updated to the new numbering. `pio test -e native` →
  9/9 green. `pio run -d {CORTEX,HYDRA,LIFT}` all build with `rocket_core` in the graph.
- **Repo structure:** the previous-generation `OBC/` was retired to `_old_code/OBC/` (reference
  only; it does not build against the new enum), and `CORTEX/CORTEX_V1/` was flattened to
  `CORTEX/`. CORTEX's `lib_extra_dirs` is therefore `../lib`.

### Phase 2 — CORTEX transport migrated onto the shared parser (DONE, build verified)

CORTEX's `Communications.{h,cpp}` no longer defines its own frame. It now pulls `packet_t`,
`SYNC_BYTE`, `MAX_PAYLOAD_SIZE`, `HEADER_SIZE`, `MAX_FRAME_SIZE` from `rc_packet.h` and uses
`rc_parse_byte` / `rc_serialize` from `rc_parser.h`:

- Deleted CORTEX's local `packet_t` (was `MAX_PAYLOAD_SIZE = 200`, now the shared 150), its
  `cmd_parse_state_t`, and its copy of `parse_input`. `MAX_PACKET_SIZE` is now an alias of
  `MAX_FRAME_SIZE`.
- The frame's `begin` timestamp (timeout reference) was **removed from the struct** and moved
  into the transport: `read_packet` keeps a per-interface `begin` array and `read_stream()`
  stamps it when the SYNC byte is consumed (`RC_SYNC` → non-`RC_SYNC` transition). This is the
  §5 "split pure parsing from I/O + timing" split, applied.
- Overflow behaviour is now the shared one: `rc_parse_byte` *clamps* an oversized length byte to
  `MAX_PAYLOAD_SIZE` (CORTEX's old copy reset to SYNC). `interface_t`, `write_to_rs485`,
  `create_packet`, the timeout logic and the addressing/CRC checks are unchanged.

### Phase 3 — HYDRA & LIFT transports migrated onto the shared parser (DONE, builds verified)

Both slaves now use the same approach as CORTEX (Phase 2). Their `include/Comms.h` dropped the
local `packet_t`, `cmd_parse_state_t`, `SYNC_BYTE`/`MAX_PAYLOAD_SIZE`/`HEADER_SIZE` and the
copy-pasted board-ID `#define`s, pulling them from `rc_ids.h` / `rc_packet.h` / `rc_parser.h`
instead. Their `src/Comms.cpp` replaced the duplicated `parse_input` with `rc_parse_byte` and
the hand-rolled serializer in `write_packet` with `rc_serialize` (then `rs485_send`).

- Per-board identity is unchanged in spirit: HYDRA still sets `DEFAULT_ID` in `Comms.h`, LIFT
  still sets `MY_ID` in `HardwareCfg.h` — both now resolve against `rc_ids.h`.
- The frame's `begin` timestamp left the struct; each `read_packet` keeps a single local `begin`
  (these boards have one serial port — Serial2 on HYDRA, Serial1 on LIFT) stamped on the
  `RC_SYNC`→non-`RC_SYNC` transition, using `millis()` as before.
- Two small behaviour changes, both improvements and consistent with CORTEX: `read_packet` now
  drains the serial buffer in a `while` loop (was one byte per call), and an oversized length
  byte is *clamped* by `rc_parse_byte` rather than handled ad hoc.
- Side fix: including `rc_ids.h` before `HardwareCfg.h` in LIFT removed a latent include-order
  fragility (`HardwareCfg.h` references `LIFT_BOTTLE_ID` to define `MY_ID`).

With this, **every board on the live command bus (CORTEX master + HYDRA/LIFT slaves) shares one
`packet_t` and one parser/serializer.** `pio test -e native` → 9/9; `pio run -d {CORTEX,HYDRA,LIFT}`
all SUCCESS.

### Phase 4 — Crc library consolidated; VALVE_MS widened; CLAUDE.md refreshed (DONE)

- **`lib/Crc/`** created at the repo root from the four identical per-board copies.
  `CORTEX/lib/Crc/`, `HYDRA/lib/Crc/`, `LIFT/lib/Crc/`, `Navigator/lib/Crc/` removed.
  HYDRA and LIFT pick it up via the existing `lib_extra_dirs = ../lib`; Navigator's local copy
  was vestigial (unused) and was simply deleted.
- **`HYDRA`: `CMD_MANUAL_VALVE_MS` now decodes a `uint16_t` (little-endian `payload[3..4]`)**
  instead of a single `uint8_t`. `payload_size` check raised to `>= 5`. The `rc_manual.h`
  comment updated to reflect the 5-byte layout `[VALVE_MS, valve, state, ms_lo, ms_hi]`.
  Durations up to 65535 ms are now encodable on the wire.
- **`CLAUDE.md` fully refreshed**: OBC replaced by CORTEX throughout, board table updated,
  `rocket_core` shared-lib section added, "OBC state machine" section replaced by "CORTEX
  flight computer" section, gotchas updated (removed OBC-specific notes, added CORTEX/LIFT
  include-order note). Build examples now show `pio run -d CORTEX`.

### Phase 5 — Full Option-3 monorepo layout (DONE, all envs verified)

The per-board project directories (`CORTEX/`, `HYDRA/`, `LIFT/`, `Navigator/`, `PocketSat/`)
are gone. Everything now lives under the repo root:

- **`src/<board>/`** holds each board's `.cpp` sources and board-specific headers (previously
  split between `<BOARD>/src/` and `<BOARD>/include/`), merged into a flat tree. Each env
  gains `-Isrc/<board>` in `build_flags` so bare `#include "Pinout.h"` etc. still resolve.
- **`lib/`** holds all shared and vendored libraries (`rocket_core`, `Crc`, `LoRa`,
  `AD5593R`, `MAX31856`, `SPI`, `HX711`, `ArduinoEigen`, `ArxTypeTraits`, `FastTrig`, `LPS`,
  `HDC302x`). PlatformIO's Library Dependency Finder compiles only what each env includes —
  Navigator's Eigen never appears in a HYDRA build. Previously-duplicated `SPI` (three copies)
  and `LPS` (two copies) are now a single copy each.
- **Root `platformio.ini`** expanded to six environments: `cortex`, `hydra`, `lift`,
  `navigator`, `pocketsat`, `native`. Per-board `platformio.ini` files deleted. `pio run`
  (no arguments) builds all five boards. `pio run -e cortex` builds a single one.
- **Per-board `platformio.ini` files deleted**: `CORTEX/platformio.ini`, `HYDRA/platformio.ini`,
  `LIFT/platformio.ini`, `Navigator/platformio.ini`, `PocketSat/platformio.ini`.
- **Pre-existing Navigator bug fixed**: `Navigator/src/Comms.cpp` referenced `READ_OBC_PIN` /
  `WRITE_OBC_PIN` which were never defined in `Pinout.h`. Corrected to `OBC_RX_PIN` /
  `OBC_TX_PIN` (the names that exist in `Pinout.h`). Navigator was not building before this
  migration; now it does.

Build verification: `pio run` → 5/5 SUCCESS; `pio test -e native` → 9/9 PASSED.

### Still TODO

- Optionally promote CORTEX's `fill_command_t` (and `RocketState`) into `rocket_core` if the
  ground GUI or other boards need to share them.
- HYDRA's `CMD_MANUAL_VALVE_MS` handler still blocks the slave loop with `delay(ms)` — make
  it non-blocking (timer-based state) when time allows.
- Enable CRC end-to-end: implement `check_crc()` on all boards and flip `CRC_ENABLED true`.
  The shared `Crc` library and stubs are already in place; the only work is wiring them up
  and adding a protocol test.
