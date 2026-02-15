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

struct CortexData {
    RocketState rocket_state;
    uint8_t sd_log_status;
    uint32_t time_since_state_start_ms;
};

struct NavigatorData {
    uint16_t altitude;
    uint16_t velocity;
    uint16_t acceleration;
    uint32_t gps_latitude;
    uint32_t gps_longitude;
};

struct HydraUFData {
    uint8_t valve_vent;
    uint8_t valve_pressurizing;
    uint16_t tank_top_pressure;
    uint16_t probe_temperature_1;
    uint16_t probe_temperature_2;
    uint16_t probe_temperature_3; 
};

struct HydraLFData {
    uint8_t valve_abort;
    uint8_t valve_main;
    uint16_t tank_bottom_pressure;
    uint16_t chamber_pressure;
    uint16_t probe_temperature_1;
    uint16_t probe_temperature_2;
};

struct HydraFSData {
    uint8_t valve_n2_fill;
    uint8_t valve_n2_purge;
    uint8_t valve_ox_fill;
    uint8_t valve_ox_purge;
    uint16_t n2_pressure;
    uint16_t ox_pressure;
    uint16_t n2_temperature;
    uint16_t ox_temperature;
};

struct LiftTankData {
    float tank_weight;
};

struct LiftBottleData {
    float bottle_weight;
};

struct LiftThrustData {
    float thrust_1;
    float thrust_2;
    float thrust_3;
};

struct RocketData {
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