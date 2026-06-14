---
name: cortex-is-new-bus-master
description: CORTEX (not OBC) is the flight computer / RS485 bus master going forward
metadata:
  type: project
---

`CORTEX/` is the new flight computer and RS485 bus master, superseding the old `OBC/`
super-loop. CORTEX is a FreeRTOS task-based rewrite (MissionControl / StateMachine /
DataPolling / RS485 tasks, mutex-protected `RocketData`, a valve-routing table, and a richer
event-driven flight state machine with BOOST/COAST/DESCENT states). It talks to a separate
Java "Mission Control" ground GUI over UART.

Confirmed by the user on 2026-06-14. The old OBC was retired to `_old_code/OBC/` (reference
only; does not build against the new command enum — see [[cortex-command-numbering-canonical]]),
and `CORTEX/CORTEX_V1/` was flattened to `CORTEX/`. CLAUDE.md still describes OBC as the master
and is stale on this point.
