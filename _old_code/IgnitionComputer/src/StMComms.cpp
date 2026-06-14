#include <Arduino.h>

#include <Crc.h>

#include "GlobalVars.h"
#include "Comms.h"

#include "StMComms.h"
#include "FlashLog.h"

int run_command(command_t* cmd, rocket_state_t state, interface_t interface)
{
    command_t command_rep;
    command_rep.id = GROUND_ID;
    switch(cmd->cmd)
    {
        case CMD_STATUS:
        {
            uint16_t index = 0;
            /*
                * Prepare ACK response
                * Send response
                */
            command_rep.cmd = CMD_STATUS_ACK;

            command_rep.data[index++] = cmd->data[0];
            command_rep.data[index++] = cmd->data[1];

            uint16_t log_bits = (cmd->data[0] << 8) + cmd->data[1];

            if((log_bits & IGNITION_FLAGS_BIT))
            {
                command_rep.data[index++] = state;
            }

            if((log_bits & IGNITION_CHAMBER_TEMPERATURE_BIT))
            {
                command_rep.data[index++] = ((chamber_temp >> 8) & 0xff);
                command_rep.data[index++] = ((chamber_temp) & 0xff);
            }

            if((log_bits & IGNITION_EMATCH_BIT))
            {
                command_rep.data[index++] = (ematch_v_reading >> 8) & 0xff;
                command_rep.data[index++] = (ematch_v_reading) & 0xff;;
            }
            
            command_rep.size = index;
            command_rep.crc = crc((unsigned char *)&command_rep, command_rep.size + 3);
            write_command(&command_rep, interface);

            return CMD_RUN_OK;
        }
        break; 

        case CMD_ARM:
        {
            //stage 1
            if(cmd->data[0] != ARN_TRIGGER_1)
            {
                //error
                return CMD_RUN_ARM_ERROR;
            }

            command_rep.cmd = CMD_ARM_ACK;
            command_rep.size = 1;

            command_rep.data[0] = ARN_TRIGGER_1;
            command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
            write_command(&command_rep, interface);

            //stage 2
            int error = 0;
            command_t* arm_cmd;
            while((arm_cmd = read_command(&error, DEFAULT_CMD_INTERFACE)) == NULL && error == CMD_READ_NO_CMD) {}

            if(error != CMD_READ_OK || arm_cmd->cmd != CMD_ARM || arm_cmd->data[0] != ARN_TRIGGER_2)
            {
                //error
                return CMD_RUN_ARM_ERROR;
            }

            command_rep.data[0] = ARN_TRIGGER_2;
            command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
            write_command(&command_rep, interface);

            //stage 3
            while((arm_cmd = read_command(&error, DEFAULT_CMD_INTERFACE)) == NULL && error == CMD_READ_NO_CMD) {}

            if(error != OK || arm_cmd->cmd != CMD_ARM || arm_cmd->data[0] != ARN_TRIGGER_3)
            {
                //error
                return CMD_RUN_ARM_ERROR;
            }

            command_rep.data[0] = ARN_TRIGGER_3;
            command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
            write_command(&command_rep, interface);

            return CMD_RUN_OK;
        }
        break;

        case CMD_EXEC_PROG:
        {
            if(cmd->size != 7) //1 byte for prog 2 bytes per var (p1,p2,p3,l1,l2)
            {
                //Serial.printf("out of bounds exc prog");
                return CMD_RUN_OUT_OF_BOUND;
            }

            //Serial.printf("exec prog");
            //ensure that the rocket is in fueling mode before running a fueling program
            if(state == FUELING)
            {
                rocket_state_t next_state = -1;
                switch(cmd->data[0])
                {
                    case FILL_He_PROG:
                    {
                        //Serial.printf("exec FILL_He_PROG");
                        next_state = FILL_He;
                        FP1 = (cmd->data[1] << 8) + cmd->data[2];
                        FP2 = (cmd->data[3] << 8) + cmd->data[4];
                        FL1 = (cmd->data[5] << 8) + cmd->data[6];
                    }
                    break;
                    case FILL_N2O_PROG:
                    {
                        //Serial.printf("exec FILL_N2O_PROG");
                        next_state = FILL_N2O;
                        FP1 = (cmd->data[1] << 8) + cmd->data[2];
                        FP2 = (cmd->data[3] << 8) + cmd->data[4];
                        FL1 = (cmd->data[5] << 8) + cmd->data[6];
                    }
                    break;
                    case PURGE_LINE_PROG:
                    {
                        //Serial.printf("exec PURGE_LINE_PROG");
                        next_state = PURGE_LINE;
                        FP1 = (cmd->data[1] << 8) + cmd->data[2];
                        FP2 = (cmd->data[3] << 8) + cmd->data[4];
                        FL1 = (cmd->data[5] << 8) + cmd->data[6];
                    }
                    break;
                    default:
                    {
                        return CMD_RUN_OUT_OF_BOUND;
                    }
                }

                comm_transition[FUELING][CMD_EXEC_PROG] = next_state;
                
                command_rep.cmd = CMD_EXEC_PROG_ACK;
                command_rep.size = 0;
                command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
                write_command(&command_rep, interface);

                return CMD_RUN_OK;
            }

            return CMD_RUN_STATE_ERROR;
        }
        break;

        case CMD_MANUAL_EXEC:
        {
            if(state != MANUAL)
                return CMD_RUN_STATE_ERROR;
            
            command_rep.cmd = CMD_MANUAL_EXEC_ACK;
            command_rep.size = 1;
            command_rep.data[0] = cmd->data[0] + manual_cmd_size + 1;

            switch(cmd->data[0])
            {
                case CMD_MANUAL_FLASH_LOG_START:
                {
                    start_log();
                }
                break; 

                case CMD_MANUAL_FLASH_LOG_STOP:
                {
                    stop_log();
                }
                break; 

                case CMD_MANUAL_FLASH_IDS:
                {
                    uint8_t files[256];
                    uint8_t index;
                    get_log_ids(files, &index);

                    //fill data buff with all file indexs
                    for(int i = 0; i < index; i++)
                        command_rep.data[i + 1] = files[i];

                    command_rep.size += index;
                    
                }
                break; 

                case CMD_MANUAL_FLASH_DUMP:
                {
                    uint8_t id = cmd->data[1];
                    dump_log(id);
                }
                break; 

                case CMD_MANUAL_VALVE_STATE:
                {
                    int valve = cmd->data[1];
                    int valve_state = cmd->data[2];

                    switch(valve)
                    {
                        case He_valve:
                        {
                            digitalWrite(He_Module.valve_pin, valve_state);
                            He_Module.valve_state = valve_state;
                        }
                        break;
                        case N2O_valve:
                        {
                            digitalWrite(N2O_Module.valve_pin, valve_state);
                            N2O_Module.valve_state = valve_state;
                        }
                        break;
                        case Line_valve:
                        {
                            digitalWrite(Line_Module.valve_pin, valve_state);
                            Line_Module.valve_state = valve_state;
                        }
                        break;
                        default:
                        {
                            //bad valve
                            return CMD_RUN_OUT_OF_BOUND;
                        }
                    };
                }
                break; 
                
                case CMD_MANUAL_VALVE_MS:
                {
                    int valve = cmd->data[1];
                    int valve_time = cmd->data[2];

                    switch(valve)
                    {
                        case He_valve:
                        {
                            if(!He_Module.valve_state)
                            {
                                digitalWrite(He_Module.valve_pin, 1);
                                delay(valve_time);
                                digitalWrite(He_Module.valve_pin, 0);
                            }
                        }
                        break;
                        case N2O_valve:
                        {
                            if(!N2O_Module.valve_state)
                            {
                                digitalWrite(N2O_Module.valve_pin, 1);
                                delay(valve_time);
                                digitalWrite(N2O_Module.valve_pin, 0);
                            }
                        }
                        break;
                        case Line_valve:
                        {
                            if(!Line_Module.valve_state)
                            {
                                digitalWrite(Line_Module.valve_pin, 1);
                                delay(valve_time);
                                digitalWrite(Line_Module.valve_pin, 0);
                            }
                        }
                        break;
                        default:
                        {
                            //bad valve
                            return CMD_RUN_OUT_OF_BOUND;
                        }
                    };
                }
                break; 

                case CMD_MANUAL_LOADCELL_TARE:
                {
                    double read = scale1.read_average(10);
                    float read_scaled = (read * scale_m) + scale_b;

                    scale_b -= read_scaled;
                }
                break;

                default:
                {
                    return CMD_RUN_OUT_OF_BOUND;
                }
            };

            //send manual command ack
            command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
            write_command(&command_rep, interface);
            
            return CMD_RUN_OK;

        }
        break;

        default:
            // if the command has no action it still needs to return ok to change state
            if(cmd->cmd < cmd_size)
            {
                if(cmd->id == BROADCAST_ID)
                {
                    //if we recieve a broadcast message, we don't send an ack
                    return CMD_RUN_OK;
                }
                command_rep.cmd = (cmd_type_t)(cmd->cmd + cmd_size + 1);
                command_rep.size = 0;

                command_rep.crc = crc((unsigned char*)&command_rep, command_rep.size + 3);
                write_command(&command_rep, interface);

                //printf("good command\n");
                return CMD_RUN_OK;
            }
            else //cmd code out of bounds, return error
                return CMD_RUN_OUT_OF_BOUND;
        break;
    };

    return CMD_RUN_OUT_OF_BOUND;

    
    //Serial.printf("cmd: %x state: %d return state %d table: %d\n", cmd->cmd, state, return_state,
    //(rocket_state_t)comm_transition[state][cmd->cmd]);
}