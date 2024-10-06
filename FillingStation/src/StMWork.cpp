#include <Arduino.h>

#include <time.h>

#include "HardwareCfg.h"
#include "StMWork.h"
#include "StateMachine.h"
#include "GlobalVars.h"
#include "Comms.h"
#include "FlashLog.h"

#include <I2Cdev.h>
#include <MPU6050.h>
#include <Max6675.h>
#include <ADS1X15.h>
#include <Crc.h>

int16_t imu_ax;
int16_t imu_ay;
int16_t imu_az;

int16_t imu_gx;
int16_t imu_gy;
int16_t imu_gz;

int64_t weight1;

int16_t tank_pressure = 0;
int16_t tank_liquid = 0;
int16_t tank_liquid_kg = 0;

uint16_t ematch_v_reading = 0;

int16_t chamber_temp = 0;

uint16_t arm_reset_timer;
uint16_t fire_reset_timer; 
uint16_t launch_reset_timer; 

void V_He_close(void) { 
    digitalWrite(He_Module.valve_pin, 0); 
    He_Module.valve_state = 0;
}
void V_N2O_close(void) { 
    digitalWrite(N2O_Module.valve_pin, 0); 
    N2O_Module.valve_state = 0;
}
void V_Line_close(void) { 
    digitalWrite(Line_Module.valve_pin, 0); 
    Line_Module.valve_state = 0;
}

void V_He_open(void) {
    digitalWrite(He_Module.valve_pin, 1); 
    He_Module.valve_state = 1;
}
void V_N2O_open(void) {
    digitalWrite(N2O_Module.valve_pin, 1); 
    N2O_Module.valve_state = 1;
}
void V_Line_open(void) { 
    digitalWrite(Line_Module.valve_pin, 1); 
    Line_Module.valve_state = 1;
}

void logger(void)
{

}

void echo_reply(void)
{
    int error;
    command_t* cmd = read_command(&error, DEFAULT_LOG_INFERFACE);
    if( cmd != NULL &&
        error == CMD_READ_OK &&
        cmd->cmd == CMD_STATUS) 
    {
        Serial.println("send response");
        /* Update rocket tank values */
        tank_pressure =  (cmd->data[0] << 8) + (cmd->data[1]);
    }
}

void flash_log_sensors(void)
{
     log(NULL, 0, SENSOR_READING);
}

void read_temperature_He(void)
{
    He_Module.thermocouple.read();

    float temp = He_Module.thermocouple.getTemperature();
    He_Module.temperature = (int16_t)(temp * 10.0); 
}

void read_temperature_N2O(void)
{
    N2O_Module.thermocouple.read();

    float temp = N2O_Module.thermocouple.getTemperature();
    N2O_Module.temperature = (int16_t)(temp * 10.0); 
}

void read_temperature_Line(void)
{
    Line_Module.thermocouple.read();

    float temp = Line_Module.thermocouple.getTemperature();
    Line_Module.temperature = (int16_t)(temp * 10.0); 
}

static float calibrated_pressure(float value, pressure_calib paramm)
{
    return paramm.b + (value * paramm.m);
}

#define TIMEOUT 3
void ads_handler_stm(unsigned long sample_timer)
{
    static uint8_t state = 0;
    static uint8_t counter = 0;

    static mux arr[] = {ADS1115_COMP_0_GND, ADS1115_COMP_1_GND, ADS1115_COMP_2_GND};
    static mux read_channel;

    static unsigned long time; 
    static unsigned long time0;

    switch (state)
    {
        case 0: //start new cycle of readings
        {
            //take time before the readings start
            time0 = millis();
        }
        //no break, it is intentional
        case 1: //set channel for reading
        {
            //Wire.setClock(400000);

            read_channel = arr[counter];
            ADS.setCompareChannels(arr[counter++]); //comment line/change parameter to change range
            counter = counter % 3;

            ADS.startSingleMeasurement();

            //Wire.setClock(100000);

            time = millis();

            state = 2;
        }
        break;

        case 2: //wait for reading
        {
            if(millis() - time >= TIMEOUT) state = 3;
        }
        break;

        case 3: //get reading
        {
            float voltage = 0.0;
            //Wire.setClock(400000);
            voltage = ADS.getResult_V(); // alternative: getResult_mV for Millivolt
            //Wire.setClock(100000);

            switch(read_channel)
            {
                case ADS1115_COMP_0_GND: 
                {
                    float pressure_calib = calibrated_pressure(voltage,
                                            He_Module.pressure_serial);

                    //Serial2.printf("Pressure %f\n", pressure_calib);
                    //Serial2.flush();
                    He_Module.pressure = (int16_t)(pressure_calib * 100);
                }
                break;
                case ADS1115_COMP_1_GND: 
                {
                    float pressure_calib = calibrated_pressure(voltage,
                                            N2O_Module.pressure_serial);
                
                    N2O_Module.pressure = (int16_t)(pressure_calib * 100);
                }
                break;
                case ADS1115_COMP_2_GND: 
                {
                
                    float pressure_calib = calibrated_pressure(voltage,
                                            Line_Module.pressure_serial);
                
                    Line_Module.pressure = (int16_t)(pressure_calib * 100);
                }
                break;
                default:
                break;
            }

            if(counter == 0) state = 4; 
            else state = 1;
        }
        break;

        case 4: //wait for next cycle of readings
        {
            if(millis() - time0 >= sample_timer) state = 0;
        }

        default:
            break;
    }
}

void module_pressure_reader_slow() { ads_handler_stm(1000); }
void module_pressure_reader_fast() { ads_handler_stm(50); }

void ematch_high(void)
{
    digitalWrite(EMATCH_IGNITE_PIN, HIGH);
}

void ematch_low(void)
{
    digitalWrite(EMATCH_IGNITE_PIN, LOW);
}

void read_ematch(void)
{
    //should already be low 
    digitalWrite(EMATCH_IGNITE_PIN, LOW);
    
    ematch_v_reading = analogRead(EMATCH_V_PIN);
}

void read_active_ematch(void)
{
    digitalWrite(EMATCH_IGNITE_PIN, LOW);
    
    //delay to allow the transitos to switch
    delay(10);

    ematch_v_reading = analogRead(EMATCH_V_PIN);
    
    digitalWrite(EMATCH_IGNITE_PIN, HIGH);
}

void read_N20_weight(void)
{
    static float lpf_val = 0.0f;
    const float betha = 0.250f;
    if (scale1.is_ready()) {
        long reading = scale1.read();
        float val = (reading * scale_m) + scale_b;
        lpf_val = lpf_val - (betha * (lpf_val - val));
        //Serial.printf("reading 1: %f\n", val);
        weight1 = (int16_t)(lpf_val * 10);
        //printf("HX711 1 %d\n", weight1);
    } else {
        //Serial.println("HX711 1 not found.");
    }
}

//read chamber temperature to dectect ignicion after the ematch
void read_chamber_temp(void)
{
    read_temperature_He();
    chamber_temp = He_Module.temperature;
    return;
}

void send_alow_launch(void)
{
    command_t cmd;
    cmd.cmd = CMD_ALLOW_LAUNCH;
    cmd.id = ROCKET_ID;
    cmd.size = 0;
    cmd.crc = 0x7878;

    write_command(&cmd, LoRa_INTERFACE);

    return;
}

//---------TIMERS---------------
void reset_timers(void)
{
    arm_reset_timer = 0;
    fire_reset_timer = 0;
    launch_reset_timer = 0;
}

void timer_tick(uint16_t* timer) { (*timer)++;}

void arm_timer_tick(void) { timer_tick(&arm_reset_timer); }
void fire_timer_tick(void) { timer_tick(&fire_reset_timer); }
void launch_timer_tick(void) { timer_tick(&launch_reset_timer); }