#ifndef _STM_VAR_H_
#define _STM_VAR_H_

#include <inttypes.h>

#include "HardwareCfg.h"

#include <I2Cdev.h>
#include <MPU6050.h>
#include <ADS1115_WE.h>
#include <Max6675.h>
#include <HX711.h>

//----------- IMU vars ------------
extern MPU6050 accelgyro;

//lattest readding s from the imu sensor
extern int16_t imu_ax;
extern int16_t imu_ay;
extern int16_t imu_az;

extern int16_t imu_gx;
extern int16_t imu_gy;
extern int16_t imu_gz;

//---------------Valve state------------
//extern uint8_t v1;
//extern uint8_t v3;
//extern uint8_t v2;

//----------- Line Modules ------------
extern Control_Module He_Module;
extern Control_Module N2O_Module;
extern Control_Module Line_Module;

//-----------Pressure AMP--------------
extern ADS1115_WE ADS;

//-----------LOADCELL--------------
extern HX711 scale1;

extern int64_t weight1;

//---------------Tank sensor state--------------
extern int16_t tank_pressure;
extern int16_t tank_liquid;
extern int16_t tank_liquid_kg;


//---------------Tank stm vars----------------
extern int16_t FP1, FP2;
extern int16_t FL1;

//---------------EMATCH state----------------
extern uint16_t ematch_v_reading;

//-----------------Combustion chamber vars-----------
extern int16_t chamber_temp;

//---------------Timers----------------
extern uint16_t arm_reset_timer;
extern uint16_t fire_reset_timer; 
extern uint16_t launch_reset_timer; 
extern uint16_t timeout_timer; 
#endif