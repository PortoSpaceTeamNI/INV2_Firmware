#include "Display.h"

void DisplayData(SensorDataResult *sensorData){
    Serial.printf("BMP: %.2f m\n", 
        //sensorData->bmpData.Temperature, 
        //sensorData->bmpData.Pressure, 
        sensorData->bmpData.Altitude);
    Serial.printf("LSM: A=%.2f %.2f %.2f m/s^2, G=%.2f %.2f %.2f rad/s\n",
        //sensorData->lsmData.Temperature,
        sensorData->lsmData.AccelZ, sensorData->lsmData.AccelY, sensorData->lsmData.AccelX,
        sensorData->lsmData.GyroZ, sensorData->lsmData.GyroY, sensorData->lsmData.GyroX);
    Serial.printf("LPS: %.2f m\n", 
        //sensorData->lpsData.Temperature, 
        //sensorData->lpsData.Pressure, 
        sensorData->lpsData.Altitude);
}