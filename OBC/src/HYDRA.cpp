#include "HYDRA.h"
#include <string.h> // for memset
#include <Arduino.h>

hydra_id_t current_hydra = HYDRA_UF;

void init_hydra(hydra_t *hydra)
{
    if (hydra)
    {
        memset(&hydra->data, 0, sizeof(hydra->data));
        hydra->data.cam_enable = false;
    }
}

int update_data_from_hydra(hydra_t *hydra, system_data_t *system_data)
{
    if (hydra && system_data)
    {
        switch (hydra->id)
        {
        case HYDRA_UF:
            //system_data->actuators.v_pressurizing = hydra->data.valve_states.v_controlled_1;
            system_data->actuators.v_vent = hydra->data.valve_states.v_controlled_1;
            system_data->thermocouples.n2o_tank_uf_t1 = hydra->data.thermo1;
            system_data->thermocouples.n2o_tank_uf_t2 = hydra->data.thermo2;
            system_data->thermocouples.n2o_tank_uf_t3 = hydra->data.thermo3;
            break;
        case HYDRA_LF:
            system_data->actuators.v_abort = hydra->data.valve_states.v_controlled_1;
            system_data->actuators.v_main = hydra->data.valve_states.v_controlled_2;
            system_data->actuators.v_pressurizing = hydra->data.valve_states.v_controlled_3;
            system_data->thermocouples.n2o_tank_lf_t1 = hydra->data.thermo2;
            system_data->thermocouples.n2o_tank_lf_t2 = hydra->data.thermo1;
            system_data->thermocouples.chamber_thermo = hydra->data.thermo3;
            system_data->pressures.n2o_tank_pressure = hydra->data.pressure2;
            system_data->pressures.chamber_pressure = hydra->data.pressure3 * 2.5f;
            break;
        case HYDRA_FS:
            //system_data->actuators.v_n2o_fill = hydra->data.valve_states.v_controlled_2;
            system_data->actuators.v_n2o_fill = hydra->data.valve_states.v_servo;
            system_data->actuators.v_n2o_purge = hydra->data.valve_states.v_controlled_3;
            system_data->actuators.v_n2_fill = hydra->data.valve_states.v_steel_ball_2;
            system_data->actuators.v_n2_purge = hydra->data.valve_states.v_steel_ball_1;
            system_data->actuators.v_n2o_quick_dc = hydra->data.valve_states.v_quick_dc_1;
            system_data->actuators.v_n2_quick_dc = hydra->data.valve_states.v_quick_dc_2;
            system_data->pressures.n2o_line_pressure = hydra->data.pressure1 * 2.5f;
            system_data->pressures.n2_line_pressure = hydra->data.pressure3 * 2.5f;
            system_data->pressures.quick_dc_pressure = hydra->data.pressure2;
            system_data->thermocouples.n2o_line_thermo = hydra->data.thermo2;
            break;
        default:
            return -1; // Invalid hydra ID
        }
        return 0;
    }
    return -1;
}

int set_hydra_valve(hydra_t *hydra, hydra_valve_t valve, bool state)
{
    if (hydra && valve < hydra_valve_count)
    {
        // Set the valve state
        uint8_t payload[2];
        payload[0] = (uint8_t)(valve);
        payload[1] = (uint8_t)(state);
        send_hydra_command(hydra, HCMD_VALVE_SET, payload, 2);
        return 0;
    }
    return -1;
}

int set_hydra_valve_ms(hydra_t *hydra, hydra_valve_t valve, uint16_t ms)
{
    if (hydra && valve < hydra_valve_count)
    {
        if (ms <= 0 || ms > 60000) // Limit to 60 seconds
            return -1;
        uint8_t payload[3];
        payload[0] = (uint8_t)valve;
        payload[1] = (ms >> 8) & 0xFF;
        payload[2] = ms & 0xFF;
        send_hydra_command(hydra, HCMD_VALVE_MS, payload, 3);
        return 0;
    }
    return -1;
}

int send_hydra_command(hydra_t *hydra, hydra_cmd_t cmd, uint8_t payload[], uint8_t payload_size)
{
    if (hydra)
    {
        packet_t packet;
        packet.sender_id = DEFAULT_ID;
        packet.target_id = (uint8_t)hydra->id + HYDRA_UF_ID; // Map enum to ID
        packet.cmd = (uint8_t)cmd;
        packet.payload_size = payload_size;
        if (payload && payload_size > 0)
        {
            memcpy(packet.payload, payload, payload_size);
        }
        packet.crc = 0; // TODO: Calculate CRC if needed
        write_packet(&packet, RS485_INTERFACE);
        // Send command to the hardware
        return 0;
    }
    return -1;
}

int parse_hydra_response(hydra_t hydras[], packet_t *packet, system_data_t *system_data)
{
    if (packet)
    {
        hydra_t *hydra = NULL;
        switch (packet->sender_id)
        {
        case HYDRA_UF_ID:
            hydra = &hydras[HYDRA_UF];
            break;
        case HYDRA_LF_ID:
            hydra = &hydras[HYDRA_LF];
            break;
        case HYDRA_FS_ID:
            hydra = &hydras[HYDRA_FS];
            break;
        default:
            return -1; // Unknown hydra ID
        }

        int index = 0;
        if (packet->cmd == HCMD_ACK)
        {
            switch (packet->payload[index++])
            {
            case HCMD_STATUS:
                if (packet->payload_size < sizeof(hydra_data_t) + 1) // Minimum size check
                    return -1;
                memcpy(&hydra->data, &packet->payload[index], sizeof(hydra_data_t));
                update_data_from_hydra(hydra, system_data);
                return 0;
            case HCMD_VALVE_SET:
            case HCMD_VALVE_MS:
                return 0; // Acknowledged
                break;
            default:
                return -1; // Unknown command in ACK
            }
        }
    }
    return -1;
}

int fetch_next_hydra(hydra_t hydras[], system_data_t *system_data)
{
    if (hydras && system_data)
    {
        int result = send_hydra_command(&hydras[current_hydra], HCMD_STATUS, NULL, 0);
        if (result == 0)
        {
            // Move to the next hydra in the sequence
            current_hydra = (hydra_id_t)((current_hydra + 1) % hydra_id_count);
            return 0;
        }
    }
    return -1;
}