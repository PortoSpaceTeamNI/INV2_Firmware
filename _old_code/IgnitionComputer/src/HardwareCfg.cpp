#include <Arduino.h>

#include "HardwareCfg.h"

#include <Max6675.h>
#include <ADS1115_WE.h>
#include <HX711.h>
#include <MPU6050.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

ADS1115_WE ADS(PRESSURE_AMP1_ADDR);

HX711 scale1;

Control_Module He_Module = 
{
    .valve_pin = V1_PIN,

    .ADC_pressure_id = 0,
    .pressure_serial = S5A2D,
    //.pressure_serial = JX8IQ,

    .thermocouple = MAX6675(TEMP_AMP1_SS_PIN, &SPI)

};

Control_Module N2O_Module = 
{
    .valve_pin = V2_PIN,

    .ADC_pressure_id = 1,
    .pressure_serial = JYJ31,

    .thermocouple = MAX6675(TEMP_AMP2_SS_PIN, &SPI)
};

Control_Module Line_Module = 
{
    .valve_pin = V3_PIN,

    .ADC_pressure_id = 2,
    .pressure_serial = JX8IQ,

    .thermocouple = MAX6675(TEMP_AMP3_SS_PIN, &SPI)
};


double scale_m = -4.6e-5;
double scale_b = 2.5;