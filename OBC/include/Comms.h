/**
 * @file Comms.h
 * @author AM
 * @brief
 *      Shared file bettewn all the systems,
 *      This way we only have one definition for each command working everywhere
 * @version 0.1
 * @date 2024-02-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _COMMS_H_
#define _COMMS_H_

#include <inttypes.h>
#include <time.h>

#include "HardwareCfg.h"

#define SYNC_BYTE 0x55

#define MAX_PAYLOAD_SIZE 150
#define RS485_TIMEOUT_TIME_MS 50 // try to get limit bounds
// #define RS485_TIMEOUT_TIME_MS 5000

/*
    Read command errors
 */
#define CMD_READ_OK 0
#define CMD_READ_TIMEOUT 1
#define CMD_READ_BAD_CRC 2
#define CMD_READ_DEFAULT_ERROR 3
#define CMD_READ_NO_CMD 4

/*
    Run command errors
 */
#define CMD_RUN_OK 0
#define CMD_RUN_ARM_ERROR 1
#define CMD_RUN_DEFAULT_ERROR 2
#define CMD_RUN_NO_ACTION 3
#define CMD_RUN_OUT_OF_BOUND 4
#define CMD_RUN_STATE_ERROR 5

/*
    Arm Stages
*/
#define ARM_TRIGGER_1 1
#define ARM_TRIGGER_2 2
#define ARM_TRIGGER_3 3

typedef enum
{
    ASK_STATE = 0,
    ASK_ACTUATOR_STATES,
    ASK_R_PRESSURES,
    ASK_R_TEMPERATURES,
    ASK_FS_PRESSURES,
    ASK_FS_TEMPERATURES,
    ASK_NAV_SENSORS,
    ASK_NAV_KALMAN,
    ask_groups_size
} ask_groups_t;


/*
    Sensors bit mask
*/
#define BIT(x) (1 << (x))

#define STATE_BIT BIT(ASK_STATE)
#define ACTUATOR_STATES_BIT BIT(ASK_ACTUATOR_STATES)
#define R_PRESSURES_BIT BIT(ASK_R_PRESSURES)
#define R_TEMPERATURES_BIT BIT(ASK_R_TEMPERATURES)
#define FS_PRESSURES_BIT BIT(ASK_FS_PRESSURES)
#define FS_TEMPERATURES_BIT BIT(ASK_FS_TEMPERATURES)
#define NAV_SENSORS_BIT BIT(ASK_NAV_SENSORS)
#define NAV_KALMAN_BIT BIT(ASK_NAV_KALMAN)


//----------------------------------------

typedef struct __attribute__((__packed__))
// typedef struct
{
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t payload_size;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;

    uint8_t data_recv; // helper pointer to fill data[]
    unsigned long begin;
} packet_t;

typedef enum
{
    SYNC = 0,
    SENDER_ID,
    TARGET_ID,
    CMD,
    PAYLOAD_SIZE,
    PAYLOAD,
    CRC1, // first byte of crc
    CRC2, // second byte of crc
    END,
} cmd_parse_state_t;

#define HEADER_SIZE 5

typedef enum
{
    LORA_INTERFACE,
    RS485_INTERFACE,
    UART_INTERFACE,
    interface_t_size
} interface_t;

#define DEFAULT_CMD_INTERFACE UART_INTERFACE

#define CRC_ENABLED false

#define OBC_ID 1
#define HYDRA_UF_ID 2
#define HYDRA_LF_ID 3
#define HYDRA_FS_ID 4
#define NAVIGATOR_ID 5
#define LIFT_TANK_ID 6
#define LIFT_BOTTLE_ID 7
#define LIFT_THRUST_ID 8
#define BROADCAST_ID 0xFF

#define DEFAULT_ID OBC_ID

void write_packet(packet_t *cmd, interface_t interface);
packet_t *read_packet(int *error, interface_t interface);

#endif