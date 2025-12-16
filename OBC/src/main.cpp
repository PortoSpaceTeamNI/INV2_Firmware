/**
 * @file main.cpp
 * @author Andre M. (portospaceteam.pt)
 * @brief Entry point for esp32 after boot
 *      Main message parsing and command executing
 *
 * @version 0.1
 * @date 2024-01-31
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <Arduino.h>

#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include "HardwareCfg.h"
#include "GlobalVars.h"

#include "Comms.h"
#include "Buzzer.h"
#include "StateMachine.h"
#include "StMComms.h"
#include "StMWork.h"
#include "HYDRA.h"
#include "LIFT.h"

#include "Control_Work.h"
#include <LoRa.h>
#include <Crc.h>
#include <SerialFlash.h>
#include <LittleFS.h>

#define FORMAT_LITTLEFS_IF_FAILED true

bool fast_reboot = 0;

bool Launch = false;

system_data_t system_data;
hydra_t hydras[hydra_id_count];
lift_t lifts[lift_id_count];
uint32_t last_slave_poll_time = 0;
uint8_t current_slave = 0;
filling_params_t filling_params;
flight_params_t flight_params;

bool slave_request_pending = false;

void sd_setup(void)
{
    // TODO: Setup SD card
}

void lora_setup(void)
{
    // TODO: Setup LoRa module
}

void slaves_setup(void)
{
    init_hydra(&hydras[HYDRA_UF]);
    hydras[HYDRA_UF].id = HYDRA_UF;

    init_hydra(&hydras[HYDRA_LF]);
    hydras[HYDRA_LF].id = HYDRA_LF;

    init_hydra(&hydras[HYDRA_FS]);
    hydras[HYDRA_FS].id = HYDRA_FS;

    init_lift(&lifts[LIFT_TANK]);
    lifts[LIFT_TANK].id = LIFT_TANK;

    init_lift(&lifts[LIFT_BOTTLE]);
    lifts[LIFT_BOTTLE].id = LIFT_BOTTLE;

    init_lift(&lifts[LIFT_THRUST]);
    lifts[LIFT_THRUST].id = LIFT_THRUST;
}

void sys_data_setup(void)
{
    system_data.state = IDLE;
    system_data.pressures = {0};
    system_data.thermocouples = {0};
    system_data.actuators = {0};
    system_data.loadcells = {0};
}

void setup()
{
    Serial.begin(USB_BAUD_RATE);    // USBC serial
    
    // RS485
    pinMode(ENABLE_RS_PIN, OUTPUT); // RS485 transceiver enable pin
    digitalWrite(ENABLE_RS_PIN, LOW);   // start in receive mode
    Serial2.setTX(WRITE_RS_PIN);
    Serial2.setRX(READ_RS_PIN);
    Serial2.begin(RS485_BAUD_RATE);

    sys_data_setup();
    slaves_setup();
    setup_buzzer();
    play_buzzer_success();
    printf("Setup done\n");
}

void loop()
{
    while (true)
    {
        static bool init_flag = false;
        if (!init_flag)
        {
            state_machine[system_data.state].entry_time = millis();
            for (int i = 0; i < MAX_WORK_SIZE; i++)
                state_machine[system_data.state].work[i].begin =
                    state_machine[system_data.state].work[i].delay;
            init_flag = true;
        }

        state_t command_state = system_data.state,
               event_state = system_data.state;

        /*
            Execute the state function
        */
        bool work_performed = WORK_HANDLER();

        int error;
        if (millis() - last_slave_poll_time > RS_SLAVE_POLL_INTERVAL) {
            fetch_next_hydra(hydras, &system_data);
            last_slave_poll_time = millis();
        }

        packet_t *slave_packet = read_packet(&error, RS485_INTERFACE);
        if (slave_packet != NULL && error == CMD_READ_OK)
        {
            //tone(BUZZER_PIN, 2000, 50);
            switch(slave_packet->sender_id) {
                case HYDRA_UF_ID:
                case HYDRA_LF_ID:
                case HYDRA_FS_ID:
                    {
                        int result = parse_hydra_response(hydras, slave_packet, &system_data);
                        if (result != 0) {
                            // log cmd execution error
                        }
                    }
                    break;
                case LIFT_TANK_ID:
                case LIFT_BOTTLE_ID:
                case LIFT_THRUST_ID:
                    {
                        int result = parse_lift_response(lifts, slave_packet, &system_data);
                        if (result != 0) {
                            // log cmd execution error
                        }
                    }
                    break;
                default:
                    // Unknown slave ID
                    break;
            }
            slave_request_pending = false;
            //Serial.println("Time taken for slave poll: " + String(millis() - last_slave_poll_time) + " ms");
            fetch_next_hydra(hydras, &system_data);
            last_slave_poll_time = millis();

        }
        else if (error != CMD_READ_OK &&
                 error != CMD_READ_NO_CMD)
        {
            // log cmd read error
            
        }
        /*
        Event handling
        */
        // if (work_performed) event_state = EVENT_HANDLER();
        if(slave_packet != NULL && slave_packet->payload_size != 1) event_state = EVENT_HANDLER();
        if (event_state == S_NONE)
            event_state = system_data.state;

        /*
        Comms
        */

        // check if we have new data
        // if we get a valid message, execute the command associated to it
        packet_t *packet = read_packet(&error, DEFAULT_CMD_INTERFACE);
        if (packet != NULL && error == CMD_READ_OK)
        {
            int error = run_command(packet, system_data.state, DEFAULT_CMD_INTERFACE);

            // make transition to new state on the state machine
            if (error == CMD_RUN_OK &&
                state_machine[system_data.state].next_states[packet->cmd] != system_data.state)
            {
                // we have new state, use lookup table
                command_state = expected_state[system_data.state][packet->cmd];
                // Serial2.printf("change state to %d\n", state_machine[state].next_states[cmd->cmd]);
            }
            else if (error != CMD_RUN_OK)
            {
                // log cmd execution error
                // Serial2.printf("EXECUTING MESSAGE ERROR %d\n", error);
            }
        }
        else if (error != CMD_READ_OK &&
                 error != CMD_READ_NO_CMD)
        {
            // log cmd read error
            // Serial.printf("READING MESSAGE ERROR %d\n", error);
        }


        /*
         * Do state transition
         */
        if (command_state != system_data.state)
        {
            // command change of state as priority over
            // internal events changes of state
            system_data.state = command_state;
            state_machine[system_data.state].entry_time = millis();

            // reset sensor timer
            for (int i = 0; i < MAX_WORK_SIZE; i++)
                state_machine[system_data.state].work[i].begin =
                    state_machine[system_data.state].work[i].delay;

        }

        // command state transitions must take precedence to event state transitions
        else if (event_state != system_data.state)
        {
            // only if next_states haven't changed the state we can
            // acept a new state from internal events
            system_data.state = event_state;
            state_machine[system_data.state].entry_time = millis();

            // reset sensor timer
            for (int i = 0; i < MAX_WORK_SIZE; i++)
                state_machine[system_data.state].work[i].begin =
                    state_machine[system_data.state].work[i].delay;

        }
    }

}
