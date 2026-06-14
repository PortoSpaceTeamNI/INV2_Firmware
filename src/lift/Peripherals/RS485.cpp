#include "Peripherals/RS485.h"
#include "Peripherals/IO_Map.h"
#include "HardwareCfg.h"

void rs485_init() {
    Serial1.setTX(WRITE_RS_PIN);
    Serial1.setRX(READ_RS_PIN);
    Serial1.begin(RS485_BAUD_RATE); // RS485
    pinMode(ENABLE_RS_PIN, OUTPUT);
    digitalWrite(ENABLE_RS_PIN, LOW); // Start in receive mode
}

void rs485_send(uint8_t *data, size_t length) {
    digitalWrite(ENABLE_RS_PIN, HIGH); // Enable trasmit mode
    Serial1.write(data, length);
    Serial1.flush();
    digitalWrite(ENABLE_RS_PIN, LOW); // Disable transmit mode, back to receive
}