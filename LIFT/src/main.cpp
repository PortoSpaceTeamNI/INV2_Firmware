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
#include "Logging.h"
#include "Peripherals/IO_Map.h"
#include "Peripherals/RS485.h"

bool setup_error = false;
data_t my_data = {0};

void run_command(packet_t *packet)
{
    if (packet->cmd == CMD_STATUS && packet->target_id == DEFAULT_ID)
    {
        // send status packet
        packet_t status_packet;
        status_packet.sender_id = DEFAULT_ID;
        status_packet.target_id = packet->sender_id;
        status_packet.cmd = CMD_ACK;
        status_packet.payload_size = sizeof(data_t) + 1; // +1 for cmd ack
        status_packet.payload[0] = CMD_STATUS;
        memcpy(status_packet.payload + 1, &my_data, sizeof(data_t));
        status_packet.crc = 0;
        if (CRC_ENABLED)
            status_packet.crc = crc((uint8_t *)&status_packet,
                                    HEADER_SIZE + status_packet.payload_size);
        write_packet(&status_packet);
    }
}

void setup()
{
    memset(&my_data, 0, sizeof(data_t));
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(USB_BAUD_RATE); // USBC serial
    Serial.println("Setting up...");
    //sd_init(SD_CS_PIN); // SD card
    //rs485_init(); // RS-485 serial
    setup_error |= loadcells_setup(); // change to loadcell setup
    if (setup_error)
    {
        Serial.println("Setup error!");
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    } 
    Serial.println("Setup good");
    // List all files on SD card
}

void loop()
{
    int error;
    
    // check if we have new data
    // if we get a valid message, execute the command associated to it
    digitalWrite(LED_BUILTIN, HIGH);

    /*
    packet_t *packet = read_packet(&error);
    if (packet != NULL && error == CMD_READ_OK)
    {
        run_command(packet);
        
    }
    */

    read_sensors(&my_data);
    Serial.print("Loadcells: ");
    Serial.print(my_data.loadcells.loadcell1);
    if(MY_ID == LIFT_THRUST_ID)
    {
        Serial.print(", ");
        Serial.print(my_data.loadcells.loadcell2);
        Serial.print(", ");
        Serial.print(my_data.loadcells.loadcell3);
    }
    Serial.println();
    
    /*
    // Write loadcell values to CSV
    char csv_line[256];
    snprintf(csv_line, sizeof(csv_line), "%lu,%d,%d,%d\n", 
             millis(), 
             my_data.loadcells.loadcell1, 
             my_data.loadcells.loadcell2, 
             my_data.loadcells.loadcell3);
    sd_log_raw(csv_line);
    */
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
}
