#include "Sensors.h"

SensorDataResult sensorData;

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

    if (IsLPS22DFReady()) {
        if (ReadLPS22DF(sensorData.lpsData) != 0) {
            Serial.println("Could not read from LPS22DF.");
            return -1;
        }
    }

    // Resto dos sensores

    return 0;
}