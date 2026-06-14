#include <Arduino.h>
#include <Crc.h>
#include <Wire.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "Comms.h"
#include "DataModels.h"
#include "HardwareCfg.h"
#include "Peripherals/IO_Map.h"
#include "Sensors.h"

#include "Peripherals/Buzzer.h"
#include "Peripherals/IO_Map.h"
#include "Peripherals/Pressures.h"
#include "Peripherals/RS485.h"
#include "Peripherals/Thermo.h"
#include "Peripherals/Valves.h"

bool setup_error = false;
data_t my_data = {0};

static bool valve_timer_active = false;
static uint32_t valve_timer_start_ms = 0;
static uint16_t valve_timer_duration_ms = 0;
static uint8_t valve_timer_valve = 0;
static uint8_t valve_timer_return_state = 0;

void run_command(packet_t *packet)
{
    if (packet->cmd == CMD_STATUS)
    {
        // send status packet
        packet_t status_packet;
        status_packet.sender_id = DEFAULT_ID;
        status_packet.target_id = packet->sender_id;
        status_packet.cmd = CMD_ACK;
        status_packet.payload_size = sizeof(data_t) + 1; // +1 for cmd ack
        status_packet.payload[0] = CMD_STATUS;
        memcpy(&status_packet.payload[1], &my_data, sizeof(data_t));
        status_packet.crc = 0;
        if (CRC_ENABLED)
            status_packet.crc = crc((uint8_t *)&status_packet,
                                    HEADER_SIZE + status_packet.payload_size);
        write_packet(&status_packet);
    }
    else if (packet->cmd == CMD_MANUAL_EXEC)
    {
        if (packet->payload_size >= 3 && packet->payload[0] == CMD_MANUAL_VALVE_STATE)
        {
            // set valve state
            valve_t valve = (valve_t)(packet->payload[1]);
            uint8_t state = packet->payload[2];
            tone(BUZZER_PWM_PIN, 1000, 50); // Play a tone for 50ms at 1kHz on manual valve command
            valve_set(&my_data, valve, state);

            // send ack packet
            packet_t ack_packet;
            ack_packet.sender_id = DEFAULT_ID;
            ack_packet.target_id = packet->sender_id;
            ack_packet.cmd = CMD_ACK;
            ack_packet.payload_size = 1; // cmd ack
            ack_packet.payload[0] = CMD_MANUAL_EXEC;
            ack_packet.crc = 0;
            if (CRC_ENABLED)
                ack_packet.crc = crc((uint8_t *)&ack_packet,
                                     HEADER_SIZE + ack_packet.payload_size);
            write_packet(&ack_packet);
        } else if (packet->payload_size >= 5 && packet->payload[0] == CMD_MANUAL_VALVE_MS) {
            // set valve state for a certain amount of milliseconds (non-blocking)
            valve_t valve = (valve_t)(packet->payload[1]);
            uint8_t state = packet->payload[2];
            uint16_t ms;
            memcpy(&ms, &packet->payload[3], sizeof(ms)); // little-endian uint16

            valve_set(&my_data, valve, state);
            valve_timer_active = true;
            valve_timer_start_ms = millis();
            valve_timer_duration_ms = ms;
            valve_timer_valve = (uint8_t)valve;
            valve_timer_return_state = !state;

            // send ack immediately; the timer fires asynchronously in loop()
            packet_t ack_packet;
            ack_packet.sender_id = DEFAULT_ID;
            ack_packet.target_id = packet->sender_id;
            ack_packet.cmd = CMD_ACK;
            ack_packet.payload_size = 1; // cmd ack
            ack_packet.payload[0] = CMD_MANUAL_EXEC;
            ack_packet.crc = 0;
            if (CRC_ENABLED)
                ack_packet.crc = crc((uint8_t *)&ack_packet,
                                     HEADER_SIZE + ack_packet.payload_size);
            write_packet(&ack_packet);
        }
    }
}

void setup()
{
    memset(&my_data, 0, sizeof(data_t));
    my_data.cam_enable = false;

    setup_buzzer();
    Serial.begin(USB_BAUD_RATE); // USBC serial
    rs485_init();                // RS-485 serial

    // Initialize I2C with custom pins from IO_Map.h
    pinMode(AD5593R_RST_PIN, OUTPUT);
    digitalWrite(AD5593R_RST_PIN, HIGH); // Keep AD5593R out of reset

    Wire1.setSDA(I2C_SDA_PIN); // Set SDA to pin 14
    Wire1.setSCL(I2C_SCL_PIN); // Set SCL to pin 15
    Wire1.begin();             // Initialize I2C with custom pins
    Wire1.setClock(400000);    // Set I2C clock to 400kHz

    setup_error |= pressures_setup();
    setup_error |= thermo_setup();
    setup_error |= valves_setup(&my_data);

    // setup status leds and buzzer
    pinMode(RED_STATUS_PIN, OUTPUT);
    pinMode(GREEN_STATUS_PIN, OUTPUT);

    if (setup_error != 0)
    {
        Serial.println("Setup error detected!");
        digitalWrite(RED_STATUS_PIN, LOW);
        play_buzzer_error();
    }
    else
    {
        Serial.println("Setup complete!");
        digitalWrite(GREEN_STATUS_PIN, LOW);
        play_buzzer_success();
    }
}

void loop()
{
    if (valve_timer_active && (millis() - valve_timer_start_ms >= valve_timer_duration_ms))
    {
        valve_set(&my_data, valve_timer_valve, valve_timer_return_state);
        valve_timer_active = false;
    }

    int error;
    packet_t *packet = read_packet(&error);
    if (packet != NULL && error == CMD_READ_OK)
    {
        run_command(packet);
    }
    read_sensors(&my_data);
}
