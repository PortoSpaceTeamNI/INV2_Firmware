#ifndef __LOG_H_
#define __LOG_H_

#include <stdbool.h>

//#include <SerialFlash.h>
#include <SD.h>

#include "Comms.h"

#define LOG_NAME_PATTERN "/%u_log.bin"

typedef enum
{
    SENSOR_READING,
    MSG_RECEIVED,
    MSG_SENT,
    SYSTEM_ERROR,
    STATE_CHANGE,
    EVENT_REACTION,
} log_event_t;

extern bool log_running;
extern File file;

extern uint8_t current_id;

void start_log();
void stop_log();

void log(void* data, uint16_t size, log_event_t event);

void dump_log(uint8_t log); 
void get_log_ids(uint8_t* files, uint8_t *file_index);

uint8_t get_last_id();

#endif