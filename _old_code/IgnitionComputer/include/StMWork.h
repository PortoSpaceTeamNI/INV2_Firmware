#ifndef STM_WORK_H_
#define STM_WORK_H_

#include "HardwareCfg.h"

/*
 * Define here all functions that can run as work in a state
 */

void V_He_close(void);
void V_N2O_close(void);
void V_Line_close(void);

void V_He_open(void);
void V_N2O_open(void);
void V_Line_open(void);

void read_temperature_He(void);
void read_temperature_N2O(void);
void read_temperature_Line(void);

void module_pressure_reader_slow();
void module_pressure_reader_fast();

void read_pressure_He(void);
void read_pressure_N2O(void);
void read_pressure_Line(void);

void read_N20_weight(void);

void ematch_high(void);
void ematch_low(void);
void read_ematch(void);
void read_active_ematch(void);

void read_chamber_temp(void);

void logger(void);
void flash_log_sensors(void);
void echo_reply(void);
void send_alow_launch(void);

//---------TIMERS---------------
void reset_timers(void);
void timer_tick(uint16_t* timer);
void arm_timer_tick(void);
void fire_timer_tick(void);
void launch_timer_tick(void);
void timeout_timer_tick(void);
#endif