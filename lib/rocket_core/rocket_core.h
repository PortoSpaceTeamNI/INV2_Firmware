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

#endif // ROCKET_CORE_H
