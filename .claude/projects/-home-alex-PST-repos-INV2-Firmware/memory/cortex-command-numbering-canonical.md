---
name: cortex-command-numbering-canonical
description: Canonical wire command_t numbering follows CORTEX's leaner set, in lib/rocket_core
metadata:
  type: project
---

The canonical `command_t` (lib/rocket_core/rc_commands.h) follows CORTEX's numbering because
CORTEX is the bus master ([[cortex-is-new-bus-master]]):

`CMD_NONE=0, STATUS=1, ABORT=2, STOP=3, ARM=4, FIRE=5, FILL_EXEC=6, MANUAL_EXEC=7, ACK=8,
NACK=9, CMD_COUNT`.

The old OBC-only `READY`/`LAUNCH_OVERRIDE`/`FILL_RESUME` commands were removed (CORTEX handles
those phases via its event-based state machine). This re-numbering fixed a live wire bug where
CORTEX sent `CMD_MANUAL_EXEC=7` but rocket_core/HYDRA had it at 10, so CORTEX valve commands
were misread by HYDRA. HYDRA/LIFT reference commands by name via rc_commands.h, so they stay
compatible. CORTEX now consumes rocket_core (rc_commands.h, rc_manual.h, rc_ids.h) instead of
its own copies. `OBC/` does NOT build against this numbering and is superseded.

Decided/implemented 2026-06-14. Details + remaining steps in
Documentation/monorepo-shared-lib-refactor.md §9 (Phase 1).
