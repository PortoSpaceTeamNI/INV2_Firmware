#ifndef PINOUT_H
#define PINOUT_H

#define BUZZER 3

// BMP581 - Barometric Pressure & Temperature Sensor
#define BMP581_SCL_PIN I2C_SCL_PIN0
#define BMP581_SDA_PIN I2C_SDA_PIN0
#define BMP581_RDY_PIN 23

// LSM6DSO: 6.4KHz, I2C Fast Mode 400kHz, ADDR 0x6A
#define LSM6SO_SCL_PIN I2C_SCL_PIN1
#define LSM6SO_SDA_PIN I2C_SDA_PIN1
#define INT1_ST_IMU 12

//* I2C
#define I2C_SDA_PIN0 20
#define I2C_SCL_PIN0 21
#define I2C_SDA_PIN1 14
#define I2C_SCL_PIN1 15

#define BAR2_RDY 23
#define INT1_ST_IMU 12
#define BAR1_RDY 22

//* SPI
#define SPI_MISO_PIN 8
#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN 10

#define HUM_CS_PIN 9
#define MAG_CS_PIN 13
#define IMU1_CS_PIN 25

//
#define RECOVERY_1_PIN 0
#define RECOVERY_2_PIN 1
#define RECOVERY_3_PIN 2
#define ADC_ELYTRA_PIN 29

//* STATUS
#define RED_STATUS_PIN 27
#define GREEN_STATUS_PIN 26

//* GPS
#define WRITE_1_PIN 4
#define READ_1_PIN 5
#define WRITE_OBC_PIN 16
#define READ_OBC_PIN 17
#define CTS_OBC_PIN 18
#define RTS_OBC_PIN 19

#endif // PINOUT_H