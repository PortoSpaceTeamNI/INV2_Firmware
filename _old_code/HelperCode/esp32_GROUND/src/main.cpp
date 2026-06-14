#include <Arduino.h>
// #include <SoftwareSerial.h>
#include "Wire.h"

#include "Comms.h"
#include "HardwareCfg.h"

#include <LoRa.h>
#include <Crc.h>

typedef enum
{
  LAUNCH,
  ABORT,
  NO_ACTION
} button_action_t;

// SoftwareSerial Serial2(6,7); //RX, TX

typedef struct
{
  COMMAND_STATE state;
  command_t *command;
  clock_t end;
} Interface;

Interface interfaces[interface_t_size];

static bool check_crc(command_t *command)
{
  uint16_t crc1, crc2;

  crc1 = command->crc;
  crc2 = crc((unsigned char *)command, command->size + 3); //+3 bytes cmd, id, size

  return (crc1 == crc2);
}

void LoRa_Setup(void)
{
  LoRa.setPins(LORA_SS_PIN, LORA_RESET_PIN, LORA_DIO0_PIN);
  LoRa.setSignalBandwidth(300E3);
  LoRa.setCodingRate4(8);
  LoRa.setSpreadingFactor(12);
  LoRa.setGain(6);

  Serial.println("Lora starting");
  if (!LoRa.begin(868E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);   // USBC serial
  Serial2.begin(SERIAL2_BAUD); // RS485 interface

  //pinMode(2, OUTPUT);

  Wire.begin();

  LoRa_Setup();

  pinMode(ABORT_BUTTON_PIN, INPUT);
  pinMode(ARM_KEY_PIN, INPUT);
  pinMode(LAUNCH_BUTTON_PIN, INPUT);
}

int arm(int id)
{
  command_t cmd;
  command_t uart_cmd;

  cmd.cmd = CMD_ARM;
  cmd.id = id;
  cmd.size = 1;

  uart_cmd.cmd = CMD_ARM_ACK;
  uart_cmd.id = 0;
  uart_cmd.size = 1;
  uart_cmd.crc = 0x7171;

  Serial.printf("start stage1 %d\n", id);
  Serial.println();

  // stage 1
  cmd.data[0] = ARN_TRIGGER_1;
  cmd.crc = crc((unsigned char *)&cmd, cmd.size + 3);
  write_command(&cmd, LoRa_INTERFACE);

  // wait for response
  int error = 0;
  command_t *cmd_rep = read_command_sync(&error, LoRa_INTERFACE);

  // while((cmd_rep = read_command(&error, LoRa_INTERFACE)) == NULL &&
  // error == CMD_READ_NO_CMD) {}

  // Serial.printf("stage 1 last err %d %x %d %d\n", error, cmd_rep, cmd_rep->cmd, cmd_rep->data[0]);
  // Serial.println();

  if (error != CMD_READ_OK ||
      cmd_rep->cmd != CMD_ARM_ACK ||
      cmd_rep->data[0] != ARN_TRIGGER_1 ||
      !check_crc(cmd_rep))
  {
    // error
    uart_cmd.data[0] = 1;
    write_command(&uart_cmd, Uart_INTERFACE);
    return CMD_RUN_ARM_ERROR;
  }

  Serial.printf("start stage2 \n");
  Serial.println();

  // stage 2
  cmd.data[0] = ARN_TRIGGER_2;
  cmd.crc = crc((unsigned char *)&cmd, cmd.size + 3);
  write_command(&cmd, LoRa_INTERFACE);

  error = 0;
  cmd_rep = read_command_sync(&error, LoRa_INTERFACE);

  // while((cmd_rep = read_command(&error, LoRa_INTERFACE)) == NULL &&
  // error == CMD_READ_NO_CMD) {}

  Serial.printf("stage 2 last err %d\n", error);
  Serial.println();

  if (error != CMD_READ_OK ||
      cmd_rep->cmd != CMD_ARM_ACK ||
      cmd_rep->data[0] != ARN_TRIGGER_2 ||
      !check_crc(cmd_rep))
  {
    // error
    uart_cmd.data[0] = 2;
    write_command(&uart_cmd, Uart_INTERFACE);
    return CMD_RUN_ARM_ERROR;
  }

  Serial.printf("start stage3 \n");
  Serial.println();

  // stage 3
  cmd.data[0] = ARN_TRIGGER_3;
  cmd.crc = crc((unsigned char *)&cmd, cmd.size + 3);
  write_command(&cmd, LoRa_INTERFACE);

  error = 0;
  cmd_rep = read_command_sync(&error, LoRa_INTERFACE);

  // while((cmd_rep = read_command(&error, LoRa_INTERFACE)) == NULL &&
  // error == CMD_READ_NO_CMD) {}

  // Serial2.printf("stage 3 last err %d\n", error);
  // Serial2.println();

  if (error != CMD_READ_OK ||
      cmd_rep->cmd != CMD_ARM_ACK ||
      cmd_rep->data[0] != ARN_TRIGGER_3 ||
      !check_crc(cmd_rep))
  {
    // error
    uart_cmd.data[0] = 3;
    write_command(&uart_cmd, Uart_INTERFACE);
    return CMD_RUN_ARM_ERROR;
  }

  uart_cmd.data[0] = 4;
  write_command(&uart_cmd, Uart_INTERFACE);
  return CMD_RUN_OK;
}

button_action_t interfaceButtons()
{
  if (!digitalRead(ABORT_BUTTON_PIN))
  {
      return ABORT;
  }
    
  uint8_t KEY_ARMED = digitalRead(ARM_KEY_PIN);
  if (KEY_ARMED && digitalRead(LAUNCH_BUTTON_PIN))
  {
    return LAUNCH;
  }

  return NO_ACTION;
}

button_action_t buttonInputAnterior = NO_ACTION;
int launchCounter = 0;

command_t command;

void loop()
{
  static unsigned long last_cmd = millis();
  int error;

  button_action_t buttonInput = interfaceButtons();

  if(millis() - last_cmd > 250)
  {
    //Serial.write("read new command\n");
    switch (buttonInput)
    {
      case ABORT:
      {
        command.cmd = CMD_ABORT;
        break;
      }
      case LAUNCH:
      {
        command.cmd = CMD_FIRE_PYRO;
        break;
      }
    }

    if(buttonInput != NO_ACTION)
    {
      command.id = 0xFF;
      command.size = 0;
      command.crc = crc((unsigned char*)&command, command.size + 3);

      last_cmd = millis();

      //Serial.write("button %d\n", buttonInput);

      write_command(&command, LoRa_INTERFACE);
    }
  }

  for (uint8_t i = 0; i < interface_t_size; i++)
  {
    interfaces[i].command = read_command(&error, (interface_t)i);
    if (interfaces[i].command != NULL &&
        error == CMD_READ_OK)
    {
      switch (i)
      {
      case LoRa_INTERFACE:
      {
        // got ack response from lora
        if (check_crc(interfaces[i].command))
          write_command(interfaces[i].command, Uart_INTERFACE);
      }
      break;
      case RS485_INTERFACE:
      {
        // got log packet from rs485
        if (check_crc(interfaces[i].command))
          write_command(interfaces[i].command, Uart_INTERFACE);
      }
      break;
      case Uart_INTERFACE:
      {
        // got message to send to the rocket or fill station
        switch (interfaces[i].command->cmd)
        {
          case CMD_ARM:
          {
            if (interfaces[i].command->id == BROADCAST_ID)
            {
              int error = arm(IGNITION_ID);
              if (error == CMD_RUN_OK)
              {
                delay(50);
                error = arm(ROCKET_ID);
              }
            }
            else
            {
              int error = arm(interfaces[i].command->id);
            }
          }
          break;

          default:
          {
            interfaces[i].command->crc = crc((unsigned char *)interfaces[i].command,
                                            interfaces[i].command->size + 3);
            Serial2.printf("sending command %d %d\n\r", interfaces[i].command->id, interfaces[i].command->cmd);
            write_command(interfaces[i].command, LoRa_INTERFACE);
          }
        };
        //last_cmd = millis();
      }
      break;
      }
    }
  }
  //delay(10);
}

/*
void loop()
{
  static int sentAnterior = millis();
  int error;

  button_action_t buttonInput = interfaceButtons();

  if (buttonInput != NO_ACTION && buttonInputAnterior == NO_ACTION || (launchCounter > 0 && launchCounter < 3 && millis()-sentAnterior > 200))
  {
    buttonInputAnterior = buttonInput;
    command_t *command = new command_t();

    switch (buttonInput)
    {
    case ABORT:
      command->cmd = CMD_ABORT;
      break;
    case LAUNCH:
      command->cmd = CMD_FIRE_PYRO;
      break;
    case NO_ACTION:
      command->cmd = CMD_FIRE_PYRO;
    }

    command->id = 0xFF;
    command->size = 0;
    command->crc = crc((unsigned char*)command, command->size + 3);

    write_command(command, LoRa_INTERFACE);

    ++launchCounter;
    sentAnterior = millis();
  }

  if(launchCounter == 3){
    launchCounter = 0;
  }

  for (uint8_t i = 0; i < interface_t_size; i++)
  {
    interfaces[i].command = read_command(&error, (interface_t)i);
    if (interfaces[i].command != NULL &&
        error == CMD_READ_OK)
    {
      switch (i)
      {
      case LoRa_INTERFACE:
      {
        // got ack response from lora
        if (check_crc(interfaces[i].command))
          write_command(interfaces[i].command, Uart_INTERFACE);
      }
      break;
      case RS485_INTERFACE:
      {
        // got log packet from rs485
        if (check_crc(interfaces[i].command))
          write_command(interfaces[i].command, Uart_INTERFACE);
        else
        {
          static int led_state = 0;
          digitalWrite(2, led_state);
          led_state ^= 1;
        }
      }
      break;
      case Uart_INTERFACE:
      {
        // got message to send to the rocket or fill station
        switch (interfaces[i].command->cmd)
        {
        case CMD_ARM:
        {
          if (interfaces[i].command->id == BROADCAST_ID)
          {
            int error = arm(IGNITION_ID);
            if (error == CMD_RUN_OK)
            {
              delay(50);
              error = arm(ROCKET_ID);
            }
          }
          else
          {
            int error = arm(interfaces[i].command->id);
          }
        }
        break;

        default:
        {
          interfaces[i].command->crc = crc((unsigned char *)interfaces[i].command,
                                           interfaces[i].command->size + 3);
          // Serial2.printf("sending command %d\n", interfaces[i].command->crc);
          write_command(interfaces[i].command, LoRa_INTERFACE);
        }
        };
        // write_command(interfaces[i].command, RS485_INTERFACE);
      }
      break;
      };
    }
  }
}
*/