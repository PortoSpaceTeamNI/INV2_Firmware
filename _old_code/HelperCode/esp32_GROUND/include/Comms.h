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

#define MAX_COMMAND_BUFFER 150
//#define RS485_TIMEOUT_TIME_MS 500 //try to get limit bounds 
#define RS485_TIMEOUT_TIME_MS 250 //try to get limit bounds 
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
#define CMD_RUN_STATE_ERROR 5

/*
    Arm Stages
*/
#define ARN_TRIGGER_1 1
#define ARN_TRIGGER_2 2
#define ARN_TRIGGER_3 3


/*
    Commands
*/
typedef enum __attribute__ ((__packed__))
{
//shared commands
    CMD_STATUS,
    CMD_LOG,
    CMD_ABORT,
    CMD_EXEC_PROG,
    CMD_STOP_PROG,
    CMD_FUELING,
    CMD_MANUAL,
    CMD_MANUAL_EXEC,
    CMD_READY,
    CMD_ARM,

//FLIGHT computer commands
    CMD_ALLOW_LAUNCH,

//FILLING station commands
    CMD_RESUME_PROG,
    CMD_FIRE_PYRO,

//used to get the number of commands 
    cmd_size,

//ACKs
    CMD_STATUS_ACK,
    CMD_LOG_ACK,
    CMD_ABORT_ACK,
    CMD_EXEC_PROG_ACK,
    CMD_STOP_PROG_ACK,
    CMD_FUELING_ACK,
    CMD_MANUAL_ACK,
    CMD_MANUAL_EXEC_ACK,
    CMD_READY_ACK,
    CMD_ARM_ACK,
    CMD_ALLOW_LAUNCH_ACK,
    CMD_RESUME_PROG_ACK,
    CMD_FIRE_PYRO_ACK,
} cmd_type_t;

typedef enum
{
    logging_work,
    pressure_safety_work,
    work_enum_size,
} cmd_work_enum;

//----------------------------------------

typedef struct __attribute__((__packed__))
//typedef struct 
{
    uint8_t cmd;
    uint8_t id;
    uint8_t size;
    uint8_t data[MAX_COMMAND_BUFFER];
    uint16_t crc;

    uint8_t data_recv; //helper pointer to fill data[]
    clock_t begin;
} command_t;

typedef enum 
{
    SYNC = 0,
    CMD,
    ID,
    SIZE,
    DATA,
    CRC1, //first byte of crc
    CRC2,  //second byte of crc
    END,
} COMMAND_STATE;

typedef enum
{
    LoRa_INTERFACE,
    RS485_INTERFACE,
    Uart_INTERFACE,
    interface_t_size
} interface_t;

#define DEFAULT_CMD_INTERFACE LoRa_INTERFACE
//#define DEFAULT_CMD_INTERFACE Uart_INTERFACE
#define DEFAULT_LOG_INFERFACE RS485_INTERFACE
#define DEFAULT_SYNC_INTERFACE Uart_INTERFACE

#define GROUND_ID 0x0
#define ROCKET_ID 0X1
#define FILL_STATION_ID 0X2
#define IGNITION_ID 0x3
#define BROADCAST_ID 0xFF

#define DEFAULT_ID GROUND_ID 

void write_command(command_t* cmd, interface_t interface);
command_t* read_command(int* error, interface_t interface);
command_t* read_command_sync(int* error, interface_t interface);

#endif