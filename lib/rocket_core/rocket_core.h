/**
 * @file rocket_core.h
 * @brief Umbrella include for the shared INV2 core (protocol, IDs, enums).
 *
 * The single source of truth for everything that must be identical across boards.
 * Include this from a board's transport layer instead of a local Comms.h copy.
 * See Documentation/monorepo-shared-lib-refactor.md.
 */
#ifndef ROCKET_CORE_H
#define ROCKET_CORE_H

#include "rc_ids.h"
#include "rc_commands.h"
#include "rc_manual.h"
#include "rc_states.h"
#include "rc_packet.h"
#include "rc_parser.h"
#include "rc_packet_utils.h"
// rc_state_machine.h is NOT included here: its RocketState enum values (IDLE,
// ABORT, ARMED, IGNITION, LAUNCH…) conflict with rc_states.h's state_t names.
// Include rc_state_machine.h directly in files that need it (StateMachine.h,
// test_state_machine).
#include "rc_valve_routing.h"

#endif // ROCKET_CORE_H
