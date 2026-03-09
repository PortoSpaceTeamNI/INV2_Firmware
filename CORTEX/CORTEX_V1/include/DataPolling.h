#ifndef DATAPOLLING_H
#define DATAPOLLING_H

#include "StateMachine.h"
#include <inttypes.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define VALVE_OPEN 1
#define VALVE_CLOSED 0

// TODO: Double check all data types and units

// IMPORTANT: All 16-bit values are the unit * 100 (e.g. 20.00 bar = 2000)

struct __attribute__((__packed__)) CortexData {
    RocketState rocket_state;
    uint8_t sd_log_status;
    uint32_t time_state_start_ms;
};

struct __attribute__((__packed__)) NavigatorData {
    uint16_t altitude;
    uint16_t velocity;
    uint16_t acceleration;
    uint32_t gps_latitude;
    uint32_t gps_longitude;
};

struct __attribute__((__packed__)) HydraUFData {
    uint8_t valve_vent;
    uint8_t valve_pressurizing;
    uint16_t tank_top_pressure;
    uint16_t probe_temperature_1;
    uint16_t probe_temperature_2;
    uint16_t probe_temperature_3; 
};

struct __attribute__((__packed__)) HydraLFData {
    uint8_t valve_abort;
    uint8_t valve_main;
    uint16_t tank_bottom_pressure;
    uint16_t chamber_pressure;
    uint16_t probe_temperature_1;
    uint16_t probe_temperature_2;
    uint16_t chamber_temperature;
};

struct __attribute__((__packed__)) HydraFSData {
    uint8_t valve_n2_fill;
    uint8_t valve_n2_purge;
    uint8_t valve_ox_fill;
    uint8_t valve_ox_purge;
    uint8_t valve_n2o_quick_dc;
    uint8_t valve_n2_quick_dc;
    uint16_t n2_pressure;
    uint16_t ox_pressure;
    uint16_t n2_temperature;
    uint16_t ox_temperature;
};

struct __attribute__((__packed__)) LiftTankData {
    int16_t tank_weight; // unit * 100
};

struct __attribute__((__packed__)) LiftBottleData {
    int16_t bottle_weight; // unit * 100
};

struct __attribute__((__packed__)) LiftThrustData {
    int16_t thrust_1; // unit * 100
    int16_t thrust_2; // unit * 100
    int16_t thrust_3; // unit * 100
};

struct __attribute__((__packed__)) RocketData {
    CortexData cortexData;
    NavigatorData navigatorData;
    HydraUFData hydraUFData;
    HydraLFData hydraLFData;
    HydraFSData hydraFSData;
    LiftTankData liftTankData;
    LiftBottleData liftBottleData;
    LiftThrustData liftThrustData;
};

// Mutex for protecting rocketData
extern SemaphoreHandle_t rocketDataMutex;

void vDataPollingTask(void *pvParameters);

#endif // DATAPOLLING_H