#ifndef DATA_MODELS_H
#define DATA_MODELS_H

#include <inttypes.h>
#include <stdbool.h>

/*
typedef enum
{
    CMD_BMP_SETUP = 0,   
    CMD_BMP_READ,        
    CMD_BMP_ERROR,       
} baro_cmd_t;*/

/*typedef enum {
    CMD_STATUS = 0,
    CMD_VALVE_SET,
    CMD_VALVE_MS,
    CMD_ACK,
} cmd_t;*/


typedef struct {
    int16_t temperature1;
    int16_t pressure1;
    bool cam_enable;
} data_t;

#endif // DATA_MODELS_H
