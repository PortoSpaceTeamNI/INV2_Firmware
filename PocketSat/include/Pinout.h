#ifndef PINOUT_H
#define PINOUT_H

#define I2C0_SCL_PIN 25
#define I2C0_SDA_PIN 24
#define I2C1_SCL_PIN 3
#define I2C1_SDA_PIN 2

// BMP581 - Barometric Pressure & Temperature Sensor
#define BMP581_SCL_PIN IDC0_SCL_PIN
#define BMP581_SDA_PIN IDC0_SDA_PIN
#define BMP581_RDY_PIN 21

// ST LPS22DF - Barometric Pressure & Temperature Sensor
#define LPS22DF_SCL_PIN I2C1_SCL_PIN
#define LPS22DF_SDA_PIN I2C1_SDA_PIN
#define LPS22DF_RDY_PIN 0

// BME280 - Barometric Pressure, Temperature & Humidity Sensor
#define BME280_MOSI_PIN 11
#define BME280_MISO_PIN 8
#define BME280_SCK_PIN 10
#define BME280_CS_PIN 9

// Texas Instruments HDC3020 - Temperature & Humidity Sensor
#define HDC3020_SCL_PIN I2C0_SCL_PIN
#define HDC3020_SDA_PIN I2C0_SDA_PIN
#define HDC3020_RDY_PIN 26

// LEDs
#define LED_GREEN_PIN 14
#define LED_RED_PIN 15

// SD Card
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 16
#define SD_SCK_PIN 18
#define SD_CS_PIN 17

#endif // PINOUT_H