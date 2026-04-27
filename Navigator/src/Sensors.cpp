#include "Sensors.h"

SensorDataResult sensorData;
bool baro1_ready = false, baro2_ready = false, imu_ready = false, mag_ready = false;

int InitializeSensors() {
    int ret = 0;

    if (InitializeBMP581() != 0) {
        Serial.println("Failed to initialize BMP581.");
        ret = -1;
    } else Serial.println("BMP581 sensor initialized.");

    if (InitializeLSM6DSO() != 0) {
        Serial.println("Failed to initialize LSM6DSO.");
        ret = -1;
    } else Serial.println("LSM6DSO sensor initialized.");
    
    if (InitializeLPS22DF() != 0) {
        Serial.println("Failed to initialize LPS22DF.");
        ret = -1;
    } else Serial.println("LPS22DF sensor initialized.");

    return ret;
}

int ConfigureSensors() {
    int ret = 0;

    if (ConfigureBMP581() != 0) {
        Serial.println("Failed to configure BMP581.");
        ret = -1;
    } else Serial.println("BMP581 sensor configured.");

    if (ConfigureLSM6DSO() != 0) {
        Serial.println("Failed to configure LSM6DSO.");
        ret = -1;
    } else Serial.println("LSM6DSO sensor configured.");

    if (ConfigureLPS22DF() != 0) {
        Serial.println("Failed to configure LPS22DF.");
        ret = -1;
    } else Serial.println("LPS22DF sensor configured.");

    // Resto dos Sensores

    return ret;
}

int ReadSensors() {
    // Interrupt-driven reads only - no polling needed
    // Data is read in ISR when ready pins trigger
    // This function can be removed or kept for compatibility
    return 0;
}