#include "Display.h"
#include "Communications.h"
#include "Commands.h"

void DisplayData(SensorDataResult *sensorData)
{
    Serial.printf("BMP: %.2f ºC, %.2f hPa, %.2f m\n",
                  sensorData->bmpData.Temperature,
                  sensorData->bmpData.Pressure,
                  sensorData->bmpData.Altitude);
    Serial.printf("LSM: T=%.2f C, A=%.2f %.2f %.2f m/s^2, G=%.2f %.2f %.2f rad/s\n",
                  sensorData->lsmData.Temperature,
                  sensorData->lsmData.AccelX, sensorData->lsmData.AccelY, sensorData->lsmData.AccelZ,
                  sensorData->lsmData.GyroX, sensorData->lsmData.GyroY, sensorData->lsmData.GyroZ);
    Serial.printf("LPS: %.2f ºC, %.2f hPa, %.2f m\n",
                  sensorData->lpsData.Temperature,
                  sensorData->lpsData.Pressure,
                  sensorData->lpsData.Altitude);
    /*
    Serial.printf("LIS: %.2f uT, %.2f uT, %.2f uT\n",
        sensorData->lisData.MagX, sensorData->lisData.MagY, sensorData->lisData.MagZ);
        */
}

void SendData(SensorDataResult *sensorData)
{
    packet_t packet;
    uint8_t payload[sizeof(SensorDataResult) + 1] = { 0 };

    memcpy(&payload[1], sensorData, sizeof(sensorData));

    int size = 0;
    create_packet(&packet, NAVIGATOR_ID, CORTEX_ID, CMD_ACK, payload, sizeof(payload));

    write_packet(&packet, UART_INTERFACE);
}