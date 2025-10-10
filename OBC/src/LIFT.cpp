#include "LIFT.h"
#include "Comms.h"
#include "Arduino.h"
#include <string.h> // for memset

lift_id_t current_lift = LIFT_TANK;

void init_lift(lift_t *lift)
{
    if (lift)
    {
        memset(&lift->data, 0, sizeof(lift->data));
    }
}

int update_data_from_lift(lift_t *lift, system_data_t *system_data)
{
    if (lift && system_data)
    {
        switch (lift->id)
        {
            case LIFT_TANK:
                system_data->loadcells.main_tank_weight = lift->data.main_tank_weight;
                break;
            case LIFT_BOTTLE:
                system_data->loadcells.n2o_bottle_weight = lift->data.n2o_bottle_weight;
                break;
            case LIFT_THRUST:
                system_data->loadcells.thrust_loadcell1 = lift->data.thrust_loadcell1;
                system_data->loadcells.thrust_loadcell2 = lift->data.thrust_loadcell2;
                system_data->loadcells.thrust_loadcell3 = lift->data.thrust_loadcell3;
                break;
            default:
                break;
        }
    }
    return 0;
}

int send_lift_command(lift_t *lift, lift_cmd_t cmd, uint8_t payload[], uint8_t payload_size)
{
    if (lift)
    {
        packet_t packet;
        packet.sender_id = DEFAULT_ID;
        packet.target_id = (uint8_t)lift->id + LIFT_TANK_ID; // Map enum to ID
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


int fetch_next_lift(lift_t lifts[], system_data_t *system_data)
{
    if (lifts && system_data)
    {
        int result = send_lift_command(&lifts[current_lift], LCMD_STATUS, NULL, 0);
        
        if (result == 0)
        {
            // Move to the next lift in the sequence
            current_lift = (lift_id_t)((current_lift + 1) % lift_id_count);
            return 0;
        }
    }
    return -1;
}

int parse_lift_response(lift_t lifts[], packet_t *packet, system_data_t *system_data)
{
    if (packet)
    {
        lift_t *lift = NULL;
        switch (packet->sender_id)
        {
            case LIFT_TANK_ID:
                lift = &lifts[LIFT_TANK];
                break;
            case LIFT_BOTTLE_ID:
                lift = &lifts[LIFT_BOTTLE];
                break;
            case LIFT_THRUST_ID:
                lift = &lifts[LIFT_THRUST];
                break;
            default:
                return -1; // Unknown lift ID
        }

        int index = 0;
        if (packet->cmd == LCMD_ACK)
        {
            switch (packet->payload[index++])
            {
                case LCMD_STATUS:
                    if (packet->payload_size < sizeof(loadcells_t) + 1) // Minimum size check
                        return -1;
                    memcpy(&lift->data, &packet->payload[index], sizeof(loadcells_t));
                    update_data_from_lift(lift, system_data);
                    return 0;
                default:
                    return -1; // Unknown command in ACK
            }
        }
    }
    return -1;
}