#include "Sensors.h"
#include "HardwareCfg.h"

clock_t last_adc_poll = 0;
clock_t last_thermo_poll = 0;

void read_sensors(data_t *data) {
    // For each sensor, check if ready and read value
    if((millis() - last_adc_poll) > ADC_POLL_INTERVAL) {
        read_adc_channels(data);
        last_adc_poll = millis();
    }

    if((millis() - last_thermo_poll) > THERMO_POLL_INTERVAL) {
        read_thermocouples(data);
        last_thermo_poll = millis();
    }
}
