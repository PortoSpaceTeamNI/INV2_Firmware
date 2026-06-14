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

#define MAX_COMMAND_BUFFER 150
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
#define CMD_RUN_STATE_ERROR 5

/*
    Arm Stages
*/
#define ARN_TRIGGER_1 1
#define ARN_TRIGGER_2 2
#define ARN_TRIGGER_3 3

/*
    Sensors bit mask
*/

#define ROCKET_STATE_BIT 1 << 0
#define TANK_PRESSURE_BIT 1 << 1
#define TANK_TEMPERATURE_BIT 1 << 2
#define GPS_BIT 1 << 3
#define BAROMETER_BIT 1 << 4
#define IMU_BIT 1 << 5
#define KALMAN_BIT 1 << 6
#define CHUTE_EMATCH_BIT 1 << 7

#define FILL_FLAGS_BIT  1 << 8
#define FILL_PRESSURE_BIT 1 << 9
#define FILL_TEMPERATURE_BIT 1 << 10
#define FILL_LOAD_CELL_BIT 1 << 11

#define IGNITION_FLAGS_BIT 1 << 12
#define IGNITION_CHAMBER_TEMPERATURE_BIT 1 << 13
#define IGNITION_EMATCH_BIT 1 << 14

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
    SAFETY_PRESSURE_PROG,
    PURGE_PRESSURE_PROG,
    PURGE_LIQUID_PROG,
    FILL_He_PROG,
    FILL_N2O_PROG,
    PURGE_LINE_PROG,
} fill_program_t;

typedef enum
{
    VPU_valve,
    Engine_valve,
    He_valve,
    N2O_valve,
    Line_valve,
} valve_t;

typedef enum
{
    CMD_MANUAL_FLASH_LOG_START,
    CMD_MANUAL_FLASH_LOG_STOP,
    CMD_MANUAL_FLASH_IDS,
    CMD_MANUAL_FLASH_DUMP,

    CMD_MANUAL_VALVE_STATE,
    CMD_MANUAL_VALVE_MS,

    CMD_MANUAL_IMU_CALIBRATE,
    CMD_MANUAL_BAROMETER_CALIBRATE,
    CMD_MANUAL_KALMAN_CALIBRATE,

    CMD_MANUAL_LOADCELL_CALIBRATE,
    CMD_MANUAL_LOADCELL_TARE,

    CMD_MANUAL_TANK_TARE,

    manual_cmd_size,

    CMD_MANUAL_FLASH_LOG_START_ACK,
    CMD_MANUAL_FLASH_LOG_STOP_ACK,
    CMD_MANUAL_FLASH_IDS_ACK,
    CMD_MANUAL_FLASH_DUMP_ACK,
    CMD_MANUAL_VALVE_STATE_ACK,
    CMD_MANUAL_VALVE_MS_ACK,
    CMD_MANUAL_IMU_CALIBRATE_ACK,
    CMD_MANUAL_BAROMETER_CALIBRATE_ACK,
    CMD_MANUAL_KALMAN_CALIBRATE_ACK,
    CMD_MANUAL_LOADCELL_CALIBRATE_ACK,
    CMD_MANUAL_LOADCELL_TARE_ACK,
    CMD_MANUAL_TANK_TARE_ACK,
} manual_command_t;

//----------------------------------------

//typedef struct __attribute__((__packed__))
typedef struct 
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

//#define DEFAULT_CMD_INTERFACE RS485_INTERFACE
#define DEFAULT_CMD_INTERFACE LoRa_INTERFACE 
//#define DEFAULT_CMD_INTERFACE Uart_INTERFACE

#define DEFAULT_LOG_INFERFACE RS485_INTERFACE
//#define DEFAULT_LOG_INFERFACE LoRa_INTERFACE

#define DEFAULT_SYNC_INTERFACE Uart_INTERFACE

#define CRC_ENABLED false 

#define GROUND_ID 0
#define ROCKET_ID 1
#define FILL_STATION_ID 2
#define IGNITION_ID 3
#define BROADCAST_ID 0xFF

#define DEFAULT_ID IGNITION_ID


void write_command(command_t* cmd, interface_t interface);
command_t* read_command(int* error, interface_t interface);

#endif