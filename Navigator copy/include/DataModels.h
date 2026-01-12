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
    float temperature_bmp;
    float pressure_bmp;
    float temperature_lps;
    float pressure_lps;
    float humidity_hdc;
    float temperature_hdc;
    float temperature_bme;
    float humidity_bme;
    float pressure_bme;
    bool cam_enable;
} data_t;


#endif // DATA_MODELS_H
