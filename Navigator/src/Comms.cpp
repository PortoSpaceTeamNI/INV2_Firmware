#include "Comms.h"

#include <hardware/gpio.h>
#include <hardware/uart.h>

static uart_inst_t* sensorUart = uart1;

void InitializeSensorUART() {
    uart_init(sensorUart, 115200);
    gpio_set_function(READ_OBC_PIN, GPIO_FUNC_UART);
    gpio_set_function(WRITE_OBC_PIN, GPIO_FUNC_UART);
    gpio_pull_up(READ_OBC_PIN);
}

void SendSensorDataOverUART(const SensorDataResult& data) {
    char packet[160];
    snprintf(packet, sizeof(packet),
             "BMP,%.2f,%.2f,%.2f;LSM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f;LPS,%.2f,%.2f,%.2f\n",
             data.bmpData.Temperature,
             data.bmpData.Pressure,
             data.bmpData.Altitude,
             data.lsmData.AccelX,
             data.lsmData.AccelY,
             data.lsmData.AccelZ,
             data.lsmData.GyroX,
             data.lsmData.GyroY,
             data.lsmData.GyroZ,
             data.lpsData.Temperature,
             data.lpsData.Pressure,
             data.lpsData.Altitude);
    uart_puts(sensorUart, packet);
}