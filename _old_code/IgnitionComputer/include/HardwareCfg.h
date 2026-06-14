#ifndef HARDWARE_CFG_H_
#define HARDWARE_CFG_H_

#include <inttypes.h>

#include <Max6675.h>

//---------------PINS-----------------

#define LED_PIN 2
#define TRIGGER 34 

#define V1_PIN 13 
#define V2_PIN 4
#define V3_PIN 27

#define Flash_SS_PIN 25

#define LORA_SS_PIN 12

#define TEMP_AMP1_SS_PIN 26 
#define TEMP_AMP2_SS_PIN 32
#define TEMP_AMP3_SS_PIN 33

#define LORA_RESET_PIN 14
#define LORA_DIO0_PIN 16

#define LOADCELL_OUT_PIN 15 
#define LOADCELL_SCK_PIN 5

#define EMATCH_IGNITE_PIN 2
#define EMATCH_V_PIN 34

//---------------BAUD-----------------
#define SERIAL_BAUD 115200
#define SERIAL2_BAUD 115200

//---------------ADDR-----------------
#define PRESSURE_AMP1_ADDR 0x48

//---------------CALIB Values-----------------
typedef struct { float m; float b; } pressure_calib;

#define S5A2D {20.06008, -0.22376}
#define JYJ31 {20.04804, -0.21367}
#define K2LI9 {20.06813, -0.22385}
#define JX8IQ {20.04801, -0.20689}
#define K0P2A {20.04805, -0.19363}
#define K1WIJ {20.07217, -0.26072}

#define P1_Serial_No S5A2D
#define P2_Serial_No JYJ31
#define P3_Serial_No K2LI9 

//--------------sensors and actuators ---------------

typedef struct 
{
    uint8_t valve_pin;
    
    uint8_t ADC_pressure_id;
    pressure_calib pressure_serial;
    
    MAX6675 thermocouple;

    int16_t pressure;    
    int16_t temperature;
    bool valve_state;

} Control_Module;


//--------------Loadcell Params
extern double scale_m, scale_b;

#endif