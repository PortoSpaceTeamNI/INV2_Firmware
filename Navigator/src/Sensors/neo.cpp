#include "Sensors.h"
#include "Sensors/neo.h"
#include "Pinout.h"

#include <TinyGPS++.h>

TinyGPSPlus gps;

int InitializeNEOMQ8() {
    Serial2.setRX(GPS_TX_PIN);
    Serial2.setTX(GPS_RX_PIN);
    Serial2.begin(GPS_BAUD, SERIAL_8N1);
    return 0;
}

int ConfigureNEOMQ8() {
    // No configs to be made
    return 0;
}

bool IsNEOMQ8Ready() {
    // Also no great thing to be made, module is p much self sufficient
    return Serial2.available();
}

int ReadNEOMQ8(NEOMQ8DataResult& result) {
    gps.encode(Serial2.read());

    result.latitude = gps.location.lat();
    result.longitude = gps.location.lng();

    result.speedMps = gps.speed.mps();
    result.speedKmh = gps.speed.kmph();
    result.altitude = gps.altitude.meters();

    result.satellites = gps.satellites.value();
    result.hdop = gps.hdop.value() / 100.0;

    result.timestamp = String(gps.time.hour()) + ":" + 
                       String(gps.time.minute()) + ":" + 
                       String(gps.time.second());
    
    return 0;
}
