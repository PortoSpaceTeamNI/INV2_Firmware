#include "Display.h"

void DisplayData(SensorDataResult *sensorData){
    Serial.println("SENSOR BAY 1");
    Serial.printf("BME: %.2f ºC, %.2f hPa, %.2f %%\n",
        sensorData->bmeData.Temperature,
        sensorData->bmeData.Pressure,
        sensorData->bmeData.Humidity);
    Serial.printf("BMI323: T=%.2f C, A=%.2f %.2f %.2f m/s^2, G=%.2f %.2f %.2f rad/s\n",
        sensorData->bmiData.Temperature,
        sensorData->bmiData.AccelX, sensorData->bmiData.AccelY, sensorData->bmiData.AccelZ,
        sensorData->bmiData.GyroX, sensorData->bmiData.GyroY, sensorData->bmiData.GyroZ);
    Serial.println("SENSOR BAY 2");
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
}