#include <Arduino.h>

#include <Crc.h>

#include "GlobalVars.h"
#include "Comms.h"

#include "StMComms.h"
#include "StMWork.h"
#include "HYDRA.h"
#include "Valves.h"

extern system_data_t system_data;
extern filling_params_t filling_params;

extern hydra_t hydras[];

int handle_status_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    uint16_t index = 0;
    // union used to get bit representation of float
    union ufloat
    {
        float f;
        uint32_t i;
    };
    union ufloat f;

    /*
     * Prepare ACK response
     * Send response
     */
    packet_rep->sender_id = DEFAULT_ID;
    packet_rep->target_id = packet->sender_id;
    packet_rep->cmd = CMD_ACK;
    packet_rep->payload[index++] = CMD_STATUS;

    packet_rep->payload[index++] = system_data.state;
    packet_rep->payload[index++] = (system_data.actuators.raw >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.actuators.raw) & 0xff;

    packet_rep->payload[index++] = (system_data.pressures.n2o_tank_pressure >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.pressures.n2o_tank_pressure) & 0xff;

    packet_rep->payload[index++] = (system_data.pressures.chamber_pressure >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.pressures.chamber_pressure) & 0xff;

    packet_rep->payload[index++] = (system_data.pressures.n2o_line_pressure >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.pressures.n2o_line_pressure) & 0xff;

    packet_rep->payload[index++] = (system_data.pressures.n2_line_pressure >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.pressures.n2_line_pressure) & 0xff;

    packet_rep->payload[index++] = (system_data.pressures.quick_dc_pressure >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.pressures.quick_dc_pressure) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t1 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t1) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t2 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t2) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t3 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_uf_t3) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_lf_t1 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_lf_t1) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_lf_t2 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_tank_lf_t2) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.chamber_thermo >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.chamber_thermo) & 0xff;

    packet_rep->payload[index++] = (system_data.thermocouples.n2o_line_thermo >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.thermocouples.n2o_line_thermo) & 0xff;

    packet_rep->payload[index++] = (system_data.loadcells.n2o_bottle_weight >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.loadcells.n2o_bottle_weight) & 0xff;

    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell1 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell1) & 0xff;

    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell2 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell2) & 0xff;

    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell3 >> 8) & 0xff;
    packet_rep->payload[index++] = (system_data.loadcells.thrust_loadcell3) & 0xff;

    // EUROC: Add nav sensors
    // EUROC: Add kalman payload

    packet_rep->payload_size = index;
    packet_rep->crc = crc((unsigned char *)&packet_rep, packet_rep->payload_size + 3);
    write_packet(packet_rep, interface);
    return CMD_RUN_OK;
}

int handle_arm_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    // TODO: check arming logic
    if (system_data.state != READY)
        return CMD_RUN_STATE_ERROR;

    packet_rep->cmd = CMD_ACK;
    packet_rep->payload_size = 1;

    packet_rep->payload[0] = CMD_ARM;

    packet_rep->crc = crc((unsigned char *)packet_rep, packet_rep->payload_size + 3);
    write_packet(packet_rep, interface);

    return CMD_RUN_OK;
}

int handle_fire_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    if (system_data.state != ARMED)
        return CMD_RUN_STATE_ERROR;
    packet_rep->cmd = CMD_ACK;
    packet_rep->payload_size = 0;
    packet_rep->crc = crc((unsigned char *)packet_rep, packet_rep->payload_size + 3);
    write_packet(packet_rep, interface);
    
    close_all_valves();
    delay(LAUNCH_OVERRIDE_TIMEOUT);
    valve_set(VALVE_MAIN, VALVE_OPEN);

    return CMD_RUN_OK;
}

int handle_fill_exec_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    if (packet->payload_size > 5 * 2 + 1) // 1 byte for prog 2 bytes per parameter
    {
        return CMD_RUN_OUT_OF_BOUND;
    }

    // ensure that the rocket is in filling mode before running a filling program
    if (system_data.state == FILLING || system_data.state == IDLE)
    {
        int index = 1;
        switch (packet->payload[0])
        {
        case CMD_SAFE_IDLE:
            index = 1;
            filling_params.target_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.trigger_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            expected_state[FILLING][CMD_FILL_EXEC] = SAFE_IDLE;
            break;
        case CMD_FILLING_N2:
            index = 1;
            filling_params.target_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            expected_state[FILLING][CMD_FILL_EXEC] = FILLING_N2;
            break;
        case CMD_PRE_PRESSURE:
            index = 1;
            filling_params.target_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.trigger_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            expected_state[FILLING][CMD_FILL_EXEC] = PRE_PRESSURE;
            break;
        case CMD_FILLING_N2O:
            index = 1;
            filling_params.target_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.trigger_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.target_temperature = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.trigger_temperature = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.target_weight = (packet->payload[index++] << 8) + packet->payload[index++];
            expected_state[FILLING][CMD_FILL_EXEC] = FILLING_N2O;
            break;
        case POST_PRESSURE:
            index = 1;
            filling_params.target_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            filling_params.trigger_pressure = (packet->payload[index++] << 8) + packet->payload[index++];
            expected_state[FILLING][CMD_FILL_EXEC] = POST_PRESSURE;
            break;
        default:
            return CMD_RUN_OUT_OF_BOUND;
        }

        packet_rep->cmd = CMD_ACK;
        packet_rep->payload_size = 1;
        packet_rep->payload[0] = CMD_FILL_EXEC;
        packet_rep->crc = crc((unsigned char *)&packet_rep, packet_rep->payload_size + 3);
        write_packet(packet_rep, interface);

        return CMD_RUN_OK;
    }
    else
    {
        return CMD_RUN_STATE_ERROR;
    }
    return CMD_RUN_OK;
}

int handle_manual_valve_cmd(packet_t *packet)
{
    int valve = packet->payload[1];
    int valve_state = packet->payload[2];
    if (valve_set((valve_t)valve, valve_state) != 0)
    {
        return CMD_RUN_OUT_OF_BOUND;
    }
    else
    {
        return CMD_RUN_OK;
    }
}

int handle_manual_valve_ms_cmd(packet_t *packet)
{
    int valve = packet->payload[1];
    int valve_time = packet->payload[2];
    if (valve_set((valve_t)valve, VALVE_OPEN) != 0)
    {
        return CMD_RUN_DEFAULT_ERROR;
    }
    else
    {
        delay(valve_time);
        if (valve_set((valve_t)valve, VALVE_CLOSE) != 0)
        {
            return CMD_RUN_DEFAULT_ERROR;
        }
        return CMD_RUN_OK;
    }
}

int handle_manual_exec_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    /* For now this check will be done by the GUI
        if (state != MANUAL)
            return CMD_RUN_STATE_ERROR;
        */
    packet_rep->cmd = CMD_ACK;
    packet_rep->payload_size = 1;
    packet_rep->payload[0] = CMD_MANUAL_EXEC;

    switch (packet->payload[0])
    {
    case CMD_MANUAL_SD_LOG_START:
        break;
    case CMD_MANUAL_SD_LOG_STOP:
        break;
    case CMD_MANUAL_VALVE_STATE:
        handle_manual_valve_cmd(packet);
        break;
    case CMD_MANUAL_VALVE_MS:
        handle_manual_valve_ms_cmd(packet);
        break;
    default:
        return CMD_RUN_OUT_OF_BOUND;
        break;
    }
    // send manual command ack
    packet_rep->crc = crc((unsigned char *)&packet_rep, packet_rep->payload_size + 3);
    write_packet(packet_rep, interface);

    return CMD_RUN_OK;
}

int handle_abort_cmd(packet_t *packet, interface_t interface, packet_t *packet_rep)
{
    // Close all valves except vent
    packet_rep->cmd = CMD_ACK;
    packet_rep->payload_size = 1;
    packet_rep->payload[0] = CMD_ABORT;
    packet_rep->crc = crc((unsigned char *)packet_rep, packet_rep->payload_size + 3);
    write_packet(packet_rep, interface);

    valve_set(VALVE_ABORT, VALVE_OPEN);

    return CMD_RUN_OK;
}

int run_command(packet_t *packet, state_t state, interface_t interface)
{
    // Serial.printf("run command %d\n", cmd->cmd);
    packet_t packet_rep;
    packet_rep.sender_id = DEFAULT_ID;
    packet_rep.target_id = packet->sender_id;
    // Serial.printf("State: %d\n", state);
    switch (packet->cmd)
    {
    case CMD_STATUS:
        // Serial.println("Status cmd");
        return handle_status_cmd(packet, interface, &packet_rep);
        break;
    case CMD_ARM:
        return handle_arm_cmd(packet, interface, &packet_rep);
        break;
    case CMD_FIRE:
        return handle_fire_cmd(packet, interface, &packet_rep);
        break;
    case CMD_FILL_EXEC:
        return handle_fill_exec_cmd(packet, interface, &packet_rep);
        break;
    case CMD_MANUAL_EXEC:
        return handle_manual_exec_cmd(packet, interface, &packet_rep);
        break;
    case CMD_ABORT:
        return handle_abort_cmd(packet, interface, &packet_rep);
        break;
    default:
        // if the command has no action it still needs to return ok to change state
        if (packet->cmd < cmd_size)
        {
            if (packet->target_id == BROADCAST_ID)
            {
                // if we recieve a broadcast message, we don't send an ack
                return CMD_RUN_OK;
            }

            packet_rep.cmd = CMD_ACK;
            packet_rep.payload_size = 1;
            packet_rep.payload[0] = packet->cmd;
            packet_rep.crc = crc((unsigned char *)&packet_rep, packet_rep.payload_size + 3);

            write_packet(&packet_rep, interface);

            // printf("Command %d state %d resulting transition state %d\n",
            // cmd->cmd, state, state_machine[state].next_states[cmd->cmd]);
            return CMD_RUN_OK;
        }
        else // cmd code out of bounds, return error
            return CMD_RUN_OUT_OF_BOUND;
        break;
    };

    return CMD_RUN_OUT_OF_BOUND;
    // Serial.printf("cmd: %x state: %d return state %d table: %d\n", cmd->cmd, state, return_state,
    //(rocket_state_t)expected_state[state][cmd->cmd]);
}