#ifndef PINOUT_H
#define PINOUT_H

// Recovery pins
#define RECOVERY1_PIN 0
#define RECOVERY2_PIN 1
#define RECOVERY3_PIN 2

#define BUZZER_PIN 3

// GPS pins
#define GPS_RX_PIN 4 // UART1 TX
#define GPS_TX_PIN 5 // UART1 RX

// SPI pins
#define SPI1_MOSI_PIN 11
#define SPI1_SCK_PIN 10
#define SPI1_MISO_PIN 8
#define HUM_CS_PIN 9
#define MAG_CS_PIN 13
#define IMU1_CS_PIN 25

// Interrupt pins
#define GPS_INT_PIN 6
#define BOSH_IMU_INT_PIN 7
#define ST_IMU_INT_PIN 12

// I2C pins
#define SDA0_PIN 20
#define SCL0_PIN 21
#define SDA1_PIN 14
#define SCL1_PIN 15

// DRDY pins
#define BAR1_RDY_PIN 22
#define BAR2_RDY_PIN 23

// OBC pins
#define OBC_RX_PIN 16 // UART0 TX
#define OBC_TX_PIN 17 // UART0 RX
#define OBC_RTS_PIN 18 // UART0 CTS
#define OBC_CTS_PIN 19 // UART0 RTS

// LED pins
#define RED_STATUS_LED_PIN 27
#define GREEN_STATUS_LED_PIN 26

// Elytra pins
#define ADC_ELYTRA_PIN 29

#endif // PINOUT_H