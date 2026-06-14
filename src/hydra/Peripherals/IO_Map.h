#ifndef IO_MAP_H
#define IO_MAP_H

//* Actuation Unit
#define SOL_VALVE_1_PIN 18
#define SOL_VALVE_2_PIN 19
#define SOL_VALVE_3_PIN 20
#define QDC_VALVE_1_PIN 21
#define QDC_VALVE_2_PIN 22
#define ST_VALVE_1_PIN 23
#define ST_VALVE_2_PIN 24
#define CAM_EN_PIN 25
#define PWM_SIG_PIN 3

//* Sensor Unit
#define THERMO1_DRDY_PIN 7
#define THERMO2_DRDY_PIN 12
#define THERMO3_DRDY_PIN 17
#define AD5593R_RST_PIN 16

#define FAULT_STATES_PIN 27
#define TEMP_SENSE_PIN 26

//* I2C
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15

//* SPI
#define SPI_MISO_PIN 8
#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN 10

#define THERMO1_CS_PIN 9
#define THERMO2_CS_PIN 13
#define THERMO3_CS_PIN 29

//* STATUS
#define RED_STATUS_PIN 0
#define GREEN_STATUS_PIN 1
#define BUZZER_PWM_PIN 2

//* RS-485
#define WRITE_RS_PIN 4
#define READ_RS_PIN 5
#define ENABLE_RS_PIN 6

#endif  // IO_MAP_H