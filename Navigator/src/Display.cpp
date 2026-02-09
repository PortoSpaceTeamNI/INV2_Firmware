#include "Display.h"

void DisplayData(SensorDataResult *sensorData){
    // Serial.printf("HDC: %.2f ºC, %.2f %%\n", sensorData.hdcData.Temperature, sensorData.hdcData.Humidity);
    Serial.printf("BMP: %.2f ºC, %.2f hPa, %.2f m\n", 
        sensorData->bmpData.Temperature, 
        sensorData->bmpData.Pressure, 
        sensorData->bmpData.Altitude);
    // Serial.printf("LPS: %.2f ºC, %.2f hPa, %.2f m\n", sensorData.lpsData.Temperature, sensorData.lpsData.Pressure, sensorData.lpsData.Altitude);
    // Serial.printf("BME: %.2f ºC, %.2f hPa, %.2f %%\n", sensorData.bmeData.Temperature, sensorData.bmeData.Pressure, sensorData.bmeData.Humidity);
}