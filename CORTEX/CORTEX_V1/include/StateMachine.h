#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <inttypes.h>
#include <FreeRTOS.h>
#include <semphr.h>

// Forward declaration
struct RocketData;

typedef enum {
    IDLE = 0,
    ABORT,
    FILL_N2,
    PRE_PRESSURIZE,
    FILL_OX,
    POST_PRESSURIZE,
    ARMED,
    IGNITION,
    LAUNCH,
    BOOST,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    TOUCHDOWN,
    _STATE_COUNT,
} RocketState; 

enum RocketEvent {
    EV_NONE = 0,
    EV_DATA_UPDATE,
    EV_STOP_CMD,
    EV_ABORT_CMD,
    EV_FILL_N2_CMD,
    EV_PRE_PRESSURIZE_CMD,
    EV_FILL_OX_CMD,
    EV_POST_PRESSURIZE_CMD,
    EV_ARM_CMD,
    EV_FIRE_CMD,
    EV_LAUNCH,
    EV_TAKEOFF,
    EV_BOOST_END,
    EV_APOGEE,
    EV_MAIN_ALTITUDE,
    EV_TOUCHDOWN,
    _EVENT_COUNT,
};

struct StateMachineConfigs {
    // TODO: Configure thresholds
    // Values in Unit * 100 (e.g. 20.00 bar = 2000) to avoid using floats
    // Filling thresholds
    uint16_t n2_fill_pressure = 20000; // max
    uint16_t pre_pressurizing_pressure = 2300; // max
    uint16_t ox_fill_pressure = 2500; // max
    uint16_t ox_fill_temperature = -2000; // min
    uint16_t post_pressurizing_pressure = 5000; // max

    // Arming thresholds
    uint16_t arming_timeout_ms = 30000; // max

    // Ignition thresholds
    uint16_t ignition_timeout_ms = 3000; // max
    uint16_t ignition_temperature = 400; // min

    // Launch thresholds
    uint16_t launch_acceleration = 20; // min
    uint16_t launch_velocity = 10; // min

    // Boost thresholds
    uint16_t boost_acceleration = 10; // min

    // Apogee thresholds
    uint16_t apogee_velocity = 0; // max

    // Main descent thresholds
    uint16_t main_deployment_altitude = 450; // max

    // Touchdown thresholds
    uint16_t touchdown_velocity = 5; // max
    uint16_t touchdown_acceleration = 5; // max
    uint16_t touchdown_altitude = 0; // max

};

// Mutex for protecting stateMachineConfigs
extern SemaphoreHandle_t stateMachineConfigsMutex;

RocketState updateState(RocketState currentState, RocketEvent event);

int performStateEntry(RocketState state, StateMachineConfigs *configs, RocketData *data);
int performStateRun(RocketState currentState, StateMachineConfigs *configs, RocketData *data);
int performStateExit(RocketState state, StateMachineConfigs *configs, RocketData *data);

void vStateMachineTask(void *pvParameters);

#endif // STATEMACHINE_H
