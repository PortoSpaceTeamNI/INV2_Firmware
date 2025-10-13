#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "Comms.h"
#include "DataModels.h"

#define MAX_WORK_SIZE 15
#define MAX_EVENT_SIZE 10

#define LAUNCH_OVERRIDE_TIMEOUT 12000 // milliseconds

#define ARM_TIMER_TRIGGER 60

#define MOTOR_BURN_TIMER_TRIGGER 82 //0.1sec 

#define MIN_TANK_PRESSURE 30.0

#define CHAMBER_TEMP_THREASHOLD 6000 // 0.1c units

#define MAIN_OPENING_ALTITUDE 450

#define MAX_DELTA_ALTITUDE 1 

#define MAX_RESTART_ALLOWED 6

#define DEPRESSUR_TIMEOUT 10
#define DEPRESSUR_GLOBAL_TIMEOUT 35

typedef struct 
{
    bool (*condition)(void);
    void (*reaction)(void);
    state_t next_state;
} sm_event_t;

typedef struct
{
    void (*channel)(void);
    unsigned long sample; //millis of delay between samples
    unsigned long delay; //millis phase for reading sensors
    unsigned long begin; //millis of last sensor reading
} sm_work_t;

typedef struct
{
    sm_work_t work[MAX_WORK_SIZE];
    sm_event_t events[MAX_EVENT_SIZE];
    state_t *next_states;

    //used as the time base when dealing with sensor sampling rate and delays
    unsigned long entry_time; 
} sm_state_t;

extern sm_state_t state; 
extern state_t expected_state[state_count][cmd_size]; //save transition state for communication
extern sm_state_t state_machine[state_count]; 


state_t event_handler();
bool work_handler();
#define WORK_HANDLER() work_handler()
#define EVENT_HANDLER() event_handler()


#endif