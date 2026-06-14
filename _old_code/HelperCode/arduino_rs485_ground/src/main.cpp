#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Wire.h"

#include "Comms.h"
#include "HardwareCfg.h"

#include <LoRa.h>

SoftwareSerial Serial2(6,7); //RX, TX

typedef struct {
  COMMAND_STATE state;
  command_t* command;
  clock_t end;
} Interface;

Interface interfaces[interface_t_size];

void LoRa_Setup(void)
{
  LoRa.setPins(LORA_SS_PIN, LORA_RESET_PIN, LORA_DIO0_PIN);
  LoRa.setSignalBandwidth(300E3);
  Serial.println("Lora starting");
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD); //USBC serial
  Serial2.begin(SERIAL2_BAUD); //RS485 interface
  
  Wire.begin();

  LoRa_Setup();
}

void loop() {
  int error;

  for(uint8_t i = 0; i < interface_t_size; i++)
  {
    interfaces[i].command = read_command( &error, (interface_t) i );
    if( interfaces[i].command != NULL &&
      error == CMD_READ_OK )
    {
      switch(i)
      {
          case LoRa_INTERFACE:
          {
            //got ack response from lora
            write_command(interfaces[i].command, Uart_INTERFACE);
          }
          break;
          case RS485_INTERFACE:
          {
            //got log packet from rs485
            write_command(interfaces[i].command, Uart_INTERFACE);
          }
          break;
          case Uart_INTERFACE:
          {
            //got message to send to the rocket or fill station
            if(interfaces[i].command->id == ROCKET_ID ||
               interfaces[i].command->id == FILL_STATION_ID ||
               interfaces[i].command->id == BROADCAST_ID)
            {
              Serial.println("sending command");
              write_command(interfaces[i].command, LoRa_INTERFACE);
              //write_command(interfaces[i].command, RS485_INTERFACE);
            }
            else if(interfaces[i].command->id ==  GROUND_ID)
            {
              continue;
            }
          }
          break;
      };
        
    }

  }
}