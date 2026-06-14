#include "Sensors/lis.h"
#include "Pinout.h"

#define MAG_READ_INTERVAL_MS 10

unsigned long lis_last_read_time = 0;

Adafruit_LIS2MDL lis = Adafruit_LIS2MDL(12345); // Unique sensor ID for identification in sensor events

int InitializeLIS2MDL() {
    lis.enableAutoRange(true); // Enable auto-gain
    int ret = lis.begin_SPI(MAG_CS_PIN, &SPI1);
    return ret ? 0 : -1;
}

int ConfigureLIS2MDL() {
    lis.printSensorDetails();
    return 0;
}

bool IsLIS2MDLReady() {
    return millis() - lis_last_read_time >= MAG_READ_INTERVAL_MS;
}

int ReadLIS2MDL(LIS2MDLDataResult& result) {
    sensors_event_t event;
    lis.getEvent(&event);

    result.MagX = event.magnetic.x;
    result.MagY = event.magnetic.y;
    result.MagZ = event.magnetic.z;

    lis_last_read_time = millis();
    return 0;
}