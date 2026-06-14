#include "Sensors.h"

SensorDataResult sensorData;
bool baro1_ready = false, baro2_ready = false, imu_ready = false, mag_ready = false;

static bool bmpInitialized = false;
static bool lsmInitialized = false;
static bool lpsInitialized = false;

int InitializeSensors() {
    int ret = 0;

    if (InitializeBMP581() != 0) {
        Serial.println("Failed to initialize BMP581.");
        ret = -1;
    } else { bmpInitialized = true; Serial.println("BMP581 sensor initialized."); }

    if (InitializeLSM6DSO() != 0) {
        Serial.println("Failed to initialize LSM6DSO.");
        ret = -1;
    } else { lsmInitialized = true; Serial.println("LSM6DSO sensor initialized."); }

    if (InitializeLPS22DF() != 0) {
        Serial.println("Failed to initialize LPS22DF.");
        ret = -1;
    } else { lpsInitialized = true; Serial.println("LPS22DF sensor initialized."); }

    /*
    if (InitializeLIS2MDL() != 0) {
        Serial.println("Failed to initialize LIS2MDL.");
        ret = -1;
    } else Serial.println("LIS2MDL sensor initialized.");
    */

    return ret;
}

int ConfigureSensors() {
    int ret = 0;

    if (bmpInitialized) {
        if (ConfigureBMP581() != 0) {
            Serial.println("Failed to configure BMP581.");
            ret = -1;
        } else Serial.println("BMP581 sensor configured.");
    } else Serial.println("Skipping BMP581 configure (init failed).");

    if (lsmInitialized) {
        if (ConfigureLSM6DSO() != 0) {
            Serial.println("Failed to configure LSM6DSO.");
            ret = -1;
        } else Serial.println("LSM6DSO sensor configured.");
    } else Serial.println("Skipping LSM6DSO configure (init failed).");

    if (lpsInitialized) {
        if (ConfigureLPS22DF() != 0) {
            Serial.println("Failed to configure LPS22DF.");
            ret = -1;
        } else Serial.println("LPS22DF sensor configured.");
    } else Serial.println("Skipping LPS22DF configure (init failed).");

    /*
    if (lisInitialized) {
        if (ConfigureLIS2MDL() != 0) {
            Serial.println("Failed to configure LIS2MDL.");
            ret = -1;
        } else Serial.println("LIS2MDL sensor configured.");
    } else Serial.println("Skipping LIS2MDL configure (init failed).");
    */

    return ret;
}

int ReadSensors() {
    if (IsBMP581Ready()) {
        baro1_ready = true;
        if (ReadBMP581(sensorData.bmpData) != 0) {
            Serial.println("Could not read from BMP581.");
            return -1;
        }
    }

    if (IsLSM6DSOReady()) { //IMU
        imu_ready = true;
        if (ReadLSM6DSO(sensorData.lsmData) != 0) {
            Serial.println("Could not read from LSM6DSO.");
            return -1;
        }
    }

    if (IsLPS22DFReady()) {
        baro2_ready = true;
        if (ReadLPS22DF(sensorData.lpsData) != 0) {
            Serial.println("Could not read from LPS22DF.");
            return -1;
        }
    }

    /*
    if (IsLIS2MDLReady()) {
        if (ReadLIS2MDL(sensorData.lisData) != 0) {
            Serial.println("Could not read from LIS2MDL.");
            return -1;
        }
    }
    */
    return 0;
}