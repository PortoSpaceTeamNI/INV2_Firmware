#include "FEM.h"

void fem_set_rx() {
    digitalWrite(PIN_RX_EN, HIGH);
    digitalWrite(PIN_TX_EN, LOW);
    digitalWrite(PIN_MODE, LOW); // optional rx gain
    delayMicroseconds(15);
}

void fem_set_tx() {
    digitalWrite(PIN_RX_EN, LOW);
    digitalWrite(PIN_TX_EN, HIGH);
    digitalWrite(PIN_MODE, HIGH); // high tx power
    delayMicroseconds(15);
}

void fem_setup() {
    pinMode(PIN_PDN, OUTPUT); // Power Down pin is negated
    pinMode(PIN_RX_EN, OUTPUT);
    pinMode(PIN_TX_EN, OUTPUT);
    pinMode(PIN_MODE, OUTPUT);

    digitalWrite(PIN_PDN, HIGH);      // Power down disabled

    fem_set_rx();                     // Start in RX mode
}