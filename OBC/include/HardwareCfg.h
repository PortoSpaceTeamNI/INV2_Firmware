#ifndef HARDWARE_CFG_H_
#define HARDWARE_CFG_H_

const int USB_BAUD_RATE = 115200;
const int RS485_BAUD_RATE = 57600;

#define ENABLE_RS_PIN 23 // GPIO pin to enable RS485 transceiver
#define WRITE_RS_PIN 24
#define READ_RS_PIN 25
#define BUZZER_PIN 29

#define PIN_PDN 4
#define PIN_RX_EN 15
#define PIN_TX_EN 20
#define PIN_MODE 14

// SX1280 (2.4GHz LoRa) pin mapping for Raspberry Pi Pico
// SPI1 pins are explicitly set in code: SCK=10, MOSI(TX)=11, MISO(RX)=12
#define SX1280_CS_PIN    13
#define SX1280_BUSY_PIN  12
#define SX1280_RESET_PIN -1  // Not connected
#define SX1280_DIO3_PIN 6     // DIO3 available on the module

// SPI pin mapping used by Radio module
#define SPI_SCK_PIN  10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 8

// Optional convenience alias for onboard LED (RPI Pico)
#ifndef LED_PIN
#define LED_PIN 25
#endif

#endif // HARDWARE_CFG_H_