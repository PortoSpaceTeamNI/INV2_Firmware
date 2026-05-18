#include "Display.h"
#include "Communications.h"
#include "Commands.h"

// FOR DEBUG, CHANGE LATER
struct NavigatorDataPkt {
    uint16_t altitude;
    uint16_t velocity;
    int16_t acceleration;
    uint32_t gps_latitude;
    uint32_t gps_longitude;
} nd;

void DisplayData(SensorDataResult *sensorData)
{
    Serial.printf("BMP: %.2f ºC, %.2f hPa, %.2f m\n",
                  sensorData->bmpData.Temperature,
                  sensorData->bmpData.Pressure,
                  sensorData->bmpData.Altitude);
    Serial.printf("LPS: %.2f ºC, %.2f hPa, %.2f m\n",
                  sensorData->lpsData.Temperature,
                  sensorData->lpsData.Pressure,
                  sensorData->lpsData.Altitude);
    Serial.printf("LSM: %.2f ºC, %.2fx %.2fy %.2fz m/s^2, %.2f %.2f %.2f rad/s\n",
                  sensorData->lsmData.Temperature,
                  sensorData->lsmData.AccelX, sensorData->lsmData.AccelY, sensorData->lsmData.AccelZ,
                  sensorData->lsmData.GyroX, sensorData->lsmData.GyroY, sensorData->lsmData.GyroZ);
    Serial.println(" ");
    /*
    Serial.printf("LIS: %.2f uT, %.2f uT, %.2f uT\n",
        sensorData->lisData.MagX, sensorData->lisData.MagY, sensorData->lisData.MagZ);
        */
}

int SendData(SensorDataResult *sensorData)
{
    // Change later
    nd.altitude = (uint16_t)(sensorData->bmpData.Altitude * 100.0f);
    nd.velocity = 0; 
    nd.acceleration = (int16_t)(sensorData->lsmData.AccelZ * 100.0f); 
    nd.gps_latitude = 0;
    nd.gps_longitude = 0;

    Serial.println("Sending Data:");
    Serial.printf("Altitude: %.2fm | Acceleration: %.2fm^2s\n",
              nd.altitude / 100.0f, nd.acceleration / 100.0f);
    Serial.println();

    packet_t packet;
    const size_t struct_size = sizeof(nd);
    if (struct_size + 1 > MAX_PAYLOAD_SIZE) {
        Serial.println("SendData: payload too large.");
    }

    uint8_t payload[struct_size + 1];
    payload[0] = CMD_STATUS;
    memcpy(&payload[1], &nd, struct_size);
    const uint8_t size = (uint8_t)(struct_size + 1);

    if (create_packet(&packet, NAVIGATOR_ID, CORTEX_ID, CMD_ACK, payload, size) != 0) {
        Serial.println("SendData: create_packet failed.");
        return -1;
    }

    if (write_packet(&packet, UART_INTERFACE) != 0) {
        Serial.println("SendData: write_packet failed.");
        return -1;
    }

    return 0;
}