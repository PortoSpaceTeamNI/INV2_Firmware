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

#define MAX_COMMAND_BUFFER 100
#define RS485_TIMEOUT_TIME_MS 50 //try to get limit bounds 
//#define RS485_TIMEOUT_TIME_MS 5000

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

/*
    Arm Stages
*/
#define ARN_TRIGGER_1 1
#define ARN_TRIGGER_2 2
#define ARN_TRIGGER_3 3

/*
    Commands
*/
typedef enum
{
//shared commands
    CMD_STATUS,
    CMD_ABORT,
    CMD_EXEC_PROG,
    CMD_STOP_PROG,
    CMD_FUELING,

//FLIGHT computer commands
    CMD_READY,
    CMD_ARM,

    CMD_LED_ON,
    CMD_LED_OFF,

    CMD_IMU_CALIBRATE,

//FILLING station commands
    CMD_RESUME_PROG,

//State machine commands
    CMD_ADD_WORK,
    CMD_REMOVE_WORK,

//used to get the number of commands 
    cmd_size,

//ACKs
    CMD_STATUS_ACK,
    CMD_FUELING_ACK,
    CMD_READY_ACK,
    CMD_ARM_ACK,
    CMD_ABORT_ACK,
    CMD_LED_ON_ACK,
    CMD_LED_OFF_ACK,
    CMD_IMU_CALIBRATE_ACK,
    CMD_EXEC_PROG_ACK,
    CMD_STOP_PROG_ACK,
    CMD_RESUME_PROG_ACK,
    CMD_ADD_WORK_ACK,
    CMD_REMOVE_WORK_ACK,
} cmd_type_t;

typedef enum
{
    logging_work,
    pressure_safety_work,
    work_enum_size,
} cmd_work_enum;

//----------------------------------------

typedef struct __attribute__((__packed__))
{
    cmd_type_t cmd;
    //int id;
    uint8_t size;
    uint8_t data[MAX_COMMAND_BUFFER];
    uint16_t crc;
} command_t;

typedef enum 
{
    SYNC,
    CMD,
    SIZE,
    DATA,
    CRC1, //first byte of crc
    CRC2,  //second byte of crc
    END,
} COMMAND_STATE;


void write_command(command_t* cmd);
command_t* read_command(int* error);

#endif