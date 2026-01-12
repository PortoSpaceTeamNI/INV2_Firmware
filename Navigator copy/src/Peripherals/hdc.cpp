#include "Peripherals\hdc.h"
#include "Peripherals\IO_Map.h"
#include "DataModels.h"
#include <Wire.h>

Adafruit_HDC302x hdc_dev;

int hdc_setup(void) {
    pinMode(ADC_HUM_RDY, INPUT);

    if (!hdc_dev.begin(HDC_I2C_ADDR, &Wire)) {
        Serial.println("Failed to find HDC302x sensor!");
        return -1;
    }

    Serial.println("HDC302x initialized!");
    return 0;
}

int read_hdc(data_t *data) {
    double temp, rh;  // local doubles required by Adafruit function

    if (!hdc_dev.readTemperatureHumidityOnDemand(temp, rh,  TRIGGERMODE_LP0)) {
        Serial.println("Error reading HDC3020.");
        return -1;
    }

    data->temperature_hdc = static_cast<float>(temp);
    data->humidity_hdc = static_cast<float>(rh);

    Serial.print("HDC3020 -> Humidity: ");
    Serial.print(data->humidity_hdc, 2);
    Serial.print(" %, Temperature: ");
    Serial.println(data->temperature_hdc, 2);

    return 0;
}
