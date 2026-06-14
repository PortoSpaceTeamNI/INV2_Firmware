#include "Peripherals/RS485.h"
#include "Peripherals/IO_Map.h"
#include "HardwareCfg.h"

void rs485_init() {
    pinMode(ENABLE_RS_PIN, OUTPUT);
    digitalWrite(ENABLE_RS_PIN, LOW);   // start in receive mode
    Serial2.setTX(WRITE_RS_PIN);
    Serial2.setRX(READ_RS_PIN);
    Serial2.begin(RS485_BAUD_RATE); // RS485
}

void rs485_send(uint8_t *data, size_t length) {
    digitalWrite(ENABLE_RS_PIN, HIGH);  // switch to transmit mode
    Serial2.write(data, length);
    Serial2.flush();
    digitalWrite(ENABLE_RS_PIN, LOW);   // switch back to receive mode
}