#ifndef NEOMQ8_H
#define NEOMQ8_H

#define GPS_BAUD 9600

struct NEOMQ8DataResult {
    float latitude;
    float longitude;

    float speedMps;
    float speedKmh;
    float altitude;

    int satellites;
    float hdop;

    String timestamp; 
};

int InitializeNEOMQ8();
int ConfigureNEOMQ8();
int ReadNEOMQ8(NEOMQ8DataResult& result);
bool IsNEOMQ8Ready();

#endif