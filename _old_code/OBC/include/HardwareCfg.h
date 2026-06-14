#ifndef HARDWARE_CFG_H_
#define HARDWARE_CFG_H_

const int USB_BAUD_RATE = 115200;
const int RS485_BAUD_RATE = 115200;

#define ENABLE_RS_PIN 23 // GPIO pin to enable RS485 transceiver
#define WRITE_RS_PIN 24
#define READ_RS_PIN 25
#define BUZZER_PIN 29

#endif // HARDWARE_CFG_H_