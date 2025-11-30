#include "Radio.h"
#include <SPI.h>
#include "sx1280OverSpi.h"

// SX1280 driver instance with CS, BUSY, RESET pins from HardwareCfg
static sx1280OverSpi sx1280(
	SX1280_CS_PIN,
	SX1280_BUSY_PIN,
	SX1280_RESET_PIN
);

// Buffers for TX/RX payloads
static uint8_t txBuf[255] = {0};
static uint8_t rxBuf[255] = {0};

void radio_setup() {
	pinMode(LED_PIN, OUTPUT);

	// If DIO3 is needed, configure here (currently unused in demo)
	pinMode(SX1280_DIO3_PIN, INPUT);

	// Configure SPI pins (Arduino-Pico core supports setSCK/setTX/setRX)
	SPI.setSCK(SPI_SCK_PIN);
	SPI.setTX(SPI_MOSI_PIN);
	SPI.setRX(SPI_MISO_PIN);
	SPI.begin();

	// Initialize radio GPIOs
	sx1280.begin();
}

// Demonstrates a receive attempt followed by a transmit of "hi"
void radio_task() {
	// Blink the onboard LED briefly
	digitalWrite(LED_PIN, HIGH);
	delay(RADIO_LED_BLINK_MS);
	digitalWrite(LED_PIN, LOW);

	// Initialize working buffers
	for (uint16_t i = 0; i < sizeof(txBuf); ++i) txBuf[i] = 0xFF;
	txBuf[254] = 0x00; // ensure null-termination somewhere
	for (uint16_t i = 0; i < sizeof(rxBuf); ++i) rxBuf[i] = 0xFF;

	// Configure radio common params (LoRa, 2.4GHz). Mirrors example values
	sx1280.sx1280Setup(
		RADIO_STANDBY_MODE,
		RADIO_PACKET_TYPE,
		RADIO_RFFREQ_23_16,
		RADIO_RFFREQ_15_8,
		RADIO_RFFREQ_7_0,
		RADIO_SPREADING_FACTOR,
		RADIO_BANDWIDTH,
		RADIO_CODING_RATE,
		RADIO_PREAMBLE_LENGTH,
		RADIO_HEADER_TYPE,
		RADIO_CRC,
		RADIO_CHIRP_INVERT,
		txBuf /* outbound placeholder */
	);

	// Try to receive something briefly (continuous RX with limited polling inside)
	sx1280.sx1280Rx(
		RADIO_RX_IRQ_15_8,
		RADIO_RX_IRQ_7_0,
		RADIO_RX_PERIOD_BASE,
		RADIO_RX_COUNT_15_8,
		RADIO_RX_COUNT_7_0,
		rxBuf
	);

	// Inspect inbound data
	if (rxBuf[0] == 'h' && rxBuf[1] == 'i') { // default message
		for (uint8_t i = 0; i <= 2; i++) {
			Serial.print(F("Inbound Message: 0x"));
			Serial.println(rxBuf[i], HEX);
		}
	} else if (rxBuf[0] != 0 && (rxBuf[3] != 'h' && rxBuf[4] != 'i')) {
		for (uint8_t i = 0; rxBuf[i] != 0x00 && i < sizeof(rxBuf); i++) {
			Serial.print(F("Inbound Message: 0x"));
			Serial.println(rxBuf[i], HEX);
		}
	} else if (rxBuf[0] == 0) {
		Serial.println(F("No Inbound Message"));
	}

	// Prepare and send "hi" back
	for (uint16_t i = 0; i < sizeof(txBuf); ++i) txBuf[i] = 0x00;
	{
		const char* msg = RADIO_DEFAULT_TX_MESSAGE;
		uint16_t idx = 0;
		while (msg[idx] != '\0' && idx < sizeof(txBuf) - 1) {
			txBuf[idx] = static_cast<uint8_t>(msg[idx]);
			++idx;
		}
		txBuf[idx] = 0x00; // null terminator
	}

	// Re-apply setup with the new outbound message buffer (payload length derived internally)
	sx1280.sx1280Setup(
		RADIO_STANDBY_MODE,
		RADIO_PACKET_TYPE,
		RADIO_RFFREQ_23_16,
		RADIO_RFFREQ_15_8,
		RADIO_RFFREQ_7_0,
		RADIO_SPREADING_FACTOR,
		RADIO_BANDWIDTH,
		RADIO_CODING_RATE,
		RADIO_PREAMBLE_LENGTH,
		RADIO_HEADER_TYPE,
		RADIO_CRC,
		RADIO_CHIRP_INVERT,
		txBuf
	);

	// Transmit settings mirror the example
	sx1280.sx1280Tx(
		RADIO_TX_POWER,
		RADIO_TX_RAMP_TIME,
		txBuf,
		RADIO_TX_IRQ_15_8,
		RADIO_TX_IRQ_7_0,
		RADIO_TX_PERIOD_BASE,
		RADIO_TX_COUNT_15_8,
		RADIO_TX_COUNT_7_0
	);
}