#include "Peripherals/lps.h"
#include "Peripherals/IO_Map.h"
#include "DataModels.h"
#include <Wire.h>
#include "lps22df_reg.h"  

stmdev_ctx_t lps22df_dev;


int32_t lps22df_i2c_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    uint8_t dev_addr = *(uint8_t*)handle;
    Wire1.beginTransmission(dev_addr);
    Wire1.write(reg);
    Wire1.write(bufp, len);
    return (Wire1.endTransmission() == 0) ? 0 : -1;
}


int32_t lps22df_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    uint8_t dev_addr = *(uint8_t*)handle;
    Wire1.beginTransmission(dev_addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return -1;
    Wire1.requestFrom(dev_addr, (uint8_t)len);
    for (uint16_t i = 0; i < len; i++) bufp[i] = Wire1.read();
    return 0;
}

void lps22df_delay_ms(uint32_t ms) {
    delay(ms);
}


int lps_setup(void) {
    pinMode(BAR2_RDY, INPUT);

    static uint8_t dev_addr = LPS_I2C_ADDR;

    lps22df_dev.write_reg = lps22df_i2c_write;
    lps22df_dev.read_reg  = lps22df_i2c_read;
    lps22df_dev.mdelay    = lps22df_delay_ms;
    lps22df_dev.handle    = &dev_addr;

    lps22df_id_t id;
    if (lps22df_id_get(&lps22df_dev, &id) != 0 || id.whoami != LPS22DF_ID) {
        Serial.println("Failed to find LPS22DF sensor!");
        return -1;
    }

    lps22df_md_t mode = {LPS22DF_1Hz, LPS22DF_4_AVG, LPS22DF_LPF_DISABLE};
    if (lps22df_mode_set(&lps22df_dev, &mode) != 0) {
        Serial.println("Failed to set LPS22DF mode");
        return -1;
    }

    Serial.println("LPS22DF initialized!");
    return 0;
}

// Read data
int read_baro3(data_t *data) {
    if (digitalRead(BAR1_RDY) == LOW) {
        lps22df_data_t sensor_data;
        if (lps22df_data_get(&lps22df_dev, &sensor_data) != 0) {
            Serial.println("Error reading data from LPS22DF.");
            return -1;
        }

        data->pressure_lps = sensor_data.pressure.hpa;  // Pressure in hPa
        data->temperature_lps = sensor_data.heat.deg_c; // Temperature in °C

        Serial.print("LPS22DF -> Pressure: ");
        Serial.print(data->pressure_lps);
        Serial.print(" hPa, Temperature: ");
        Serial.println(data->temperature_lps);

        return 0;
    }
    return -1;  // No data ready
}
