#include "Sensors.h"

SensorDataResult sensorData;

int InitializeSensors() {
    if (InitializeBMP581() != 0) {
        Serial.println("Failed to initialize BMP581.");
        return -1;
    } else Serial.println("BMP581 sensor initialized.");

    if (InitializeLSM6DSO() != 0) {
        Serial.println("Failed to initialize LSM6DSO.");
        return -1;
    } else Serial.println("LSM6DSO sensor initialized.");
    // Resto dos sensores

    return 0;
}

int ConfigureSensors() {
    if (ConfigureBMP581() != 0) {
        Serial.println("Failed to configure BMP581.");
        return -1;
    } else Serial.println("BMP581 sensor configured.");

    if (ConfigureLSM6DSO() != 0) {
        Serial.println("Failed to configure LSM6DSO.");
        return -1;
    } else Serial.println("LSM6DSO sensor configured.");

    // Resto dos Sensores

    return 0;
}

int ReadSensors() {
    if (IsBMP581Ready()) {
        if (ReadBMP581(sensorData.bmpData) != 0) {
            Serial.println("Could not read from BMP581.");
            return -1;
        }
    }

    if (IsLSM6DSOReady()) {
        if (ReadLSM6DSO(sensorData.lsmData) != 0) {
            Serial.println("Could not read from LSM6DSO.");
            return -1;
        }
    }

    // Resto dos sensores

    return 0;
}