#include "Peripherals/Pressures.h"
#include "Peripherals/Buzzer.h"

AD5593R ad5593r = AD5593R(0x10, &Wire1);  // I2C address 0x10

int dac_adc_setup(void) {
    // Initialize I2C communication with AD5593R
    if (!ad5593r.begin()) {
        Serial.println("Failed to initialize AD5593R!");
        return -1;  // Exit if initialization fails
    }

    ad5593r.reset();  // Reset the device to ensure clean state
    delay(10);        // Small delay after reset


    ad5593r.setExternalReference(false, 2.5);  // Configure voltage reference (false = internal 2.5V reference)
    ad5593r.setMode("AAAATDDD");               // Set I/O configuration: 0-3 as ADC, 4 as THREESTATE (unused), 5-7 as DAC
    ad5593r.powerDownDac(4);                   // Power down I/O4 to save power (since it's unused)

    // ADC Configs:
    ad5593r.setADCRange2x(true);             // ADC input range: 0V to 5V (2x Vref)
    ad5593r.enableADCBuffer(true);           // Enable ADC buffer for better accuracy (optional but recommended)
    ad5593r.enableADCBufferPreCharge(true);  // Enable ADC buffer pre-charge for faster settling (optional)

    // DAC Configs:
    ad5593r.setDACRange2x(true);
    ad5593r.setLDACmode(AD5593R_LDAC_DIRECT);  // Set LDAC mode (direct write to DAC outputs)

    // With 2x range (5V) and 12-bit resolution: output = (Vout/5.0) * 4095
    uint16_t dac_output = (2.5 / 5) * 4095;  // 2.5 output voltage

    ad5593r.writeDAC(5, dac_output);  // Set DAC channel 5
    ad5593r.writeDAC(6, dac_output);  // Set DAC channel 6
    ad5593r.writeDAC(7, dac_output);  // Set DAC channel 7

    Serial.println("AD5593R initialized and configured!");
    Serial.print("Device address: 0x");
    Serial.println(ad5593r.getAddress(), HEX);
    Serial.println("DAC channels 5-7 set to 2.5V output");
    return 0;  // Successful initialization
}

float adc_read_to_bar(int16_t reading) {
    const float ADC_MAX = 4095.0f;       // 12-bit full scale
    const float VREF    = 5.0f;         // ADC reference voltage
    const float S_MIN_V = 0.5f;         // sensor output at 0 bar
    const float S_SPAN  = 4.0f;         // 4.5 - 0.5 V span
    const float P_MAX   = 100.0f;       // max pressure (bar)

    // Convert ADC count to voltage
    float voltage = ((float)reading / ADC_MAX) * VREF;

    // Compute sensor voltage above minimum and clamp to [0, S_SPAN]
    float v_above_min = voltage - S_MIN_V;
    if (v_above_min < 0.0f) v_above_min = 0.0f;
    if (v_above_min > S_SPAN) v_above_min = S_SPAN;

    // Convert to pressure
    float pressure = (v_above_min / S_SPAN) * P_MAX;
    return pressure; // in bar
}

// Read ADC channels 0 to 3 and store values in adc_values array
void read_adc_channels(data_t *data) {
    uint16_t adc_values[3];
    adc_values[0] = ad5593r.readADC(1);
    adc_values[1] = ad5593r.readADC(2);
    adc_values[2] = ad5593r.readADC(3);
    
    data->pressure1 = (int16_t)(adc_read_to_bar(adc_values[0]) * 100);
    data->pressure2 = (int16_t)(adc_read_to_bar(adc_values[1]) * 100);
    data->pressure3 = (int16_t)(adc_read_to_bar(adc_values[2]) * 100);
    /*
    data->pressure1 = adc_values[0];
    data->pressure2 = adc_values[1];
    data->pressure3 = adc_values[2];
    */
}


int pressures_setup(void) {
    return dac_adc_setup();
}