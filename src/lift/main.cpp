#include <Arduino.h>
#include <Crc.h>
#include <Wire.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

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
SemaphoreHandle_t data_mutex = NULL;

//#define DEBUG_LOADCELLS
//#define DEBUG_COMMS

void run_command(packet_t *packet)
{
    if (packet->cmd == CMD_STATUS && packet->target_id == MY_ID)
    {
        // send status packet
        packet_t status_packet;
        status_packet.sender_id = MY_ID;
        status_packet.target_id = packet->sender_id;
        status_packet.cmd = CMD_ACK;
        status_packet.payload_size = sizeof(data_t) + 1; // +1 for cmd ack
        status_packet.payload[0] = CMD_STATUS;

        if (data_mutex != NULL)
        {
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                memcpy(status_packet.payload + 1, &my_data, sizeof(data_t));
                xSemaphoreGive(data_mutex);
            }
            else
            {
                memset(status_packet.payload + 1, 0, sizeof(data_t));
            }
        }
        else
        {
            memcpy(status_packet.payload + 1, &my_data, sizeof(data_t));
        }

        status_packet.crc = 0;
        if (CRC_ENABLED)
            status_packet.crc = crc((uint8_t *)&status_packet,
                                    HEADER_SIZE + status_packet.payload_size);
        Serial.println("Sending status packet...");
        for (int i = 0; i < status_packet.payload_size + HEADER_SIZE + 2; i++)
        {
            Serial.print(((uint8_t *)&status_packet)[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        write_packet(&status_packet);
    }
}

static void sensor_task(void *pvParameters)
{
    (void)pvParameters;

#ifndef CALIBRATE_LOADCELLS
    data_t snapshot = {0};

    while (true)
    {
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            read_sensors(&my_data);
            memcpy(&snapshot, &my_data, sizeof(data_t));
            xSemaphoreGive(data_mutex);
            sd_log_sample(&snapshot);

#ifdef DEBUG_LOADCELLS
            Serial.print("LC1: ");
            Serial.print(snapshot.loadcells.loadcell1);
            Serial.print(" g");
#if MY_ID == LIFT_THRUST_ID // Only LIFT THRUST has all 3 loadcells connected
            Serial.print("  LC2: ");
            Serial.print(snapshot.loadcells.loadcell2);
            Serial.print(" g");
            Serial.print("  LC3: ");
            Serial.print(snapshot.loadcells.loadcell3);
            Serial.print(" g");
#endif
            Serial.println();
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
#else
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
#endif
}

static void comms_task(void *pvParameters)
{
    (void)pvParameters;

    while (true)
    {
        int error = CMD_READ_NO_CMD;
        packet_t *packet = read_packet(&error);
        if (packet != NULL && error == CMD_READ_OK)
        {
#ifdef DEBUG_COMMS
            //Serial.println("Packet received!");
#endif
            run_command(packet);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void setup()
{
    memset(&my_data, 0, sizeof(data_t));
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(USB_BAUD_RATE); // USBC serial
    Serial.println("Setting up...");
    if (!sd_init(SD_CS_PIN)) {
        Serial.println("SD logging unavailable; continuing without SD card.");
    }
    Serial.println("Initializing peripherals...");
    rs485_init(); // RS-485 serial
    setup_error |= loadcells_setup();
    #ifdef CALIBRATE_LOADCELLS
        calibrate_loadcells();
    #endif
    if (setup_error)
    {
        Serial.println("Setup error!");
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
        }
    }
    Serial.println("Setup good");

    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL)
    {
        Serial.println("Failed to create data mutex");
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
        }
    }

    BaseType_t sensor_created = xTaskCreate(sensor_task,
                                            "sensor_task",
                                            1024,
                                            NULL,
                                            tskIDLE_PRIORITY + 2,
                                            NULL);
    BaseType_t comms_created = xTaskCreate(comms_task,
                                           "comms_task",
                                           1024,
                                           NULL,
                                           tskIDLE_PRIORITY + 1,
                                           NULL);

    if (sensor_created != pdPASS || comms_created != pdPASS)
    {
        Serial.println("Failed to create tasks");
        while (1)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
}

void loop()
{
    // FreeRTOS tasks are running continuously; keep loop idle.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void setup1() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}