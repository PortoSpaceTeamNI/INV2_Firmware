#include <Arduino.h>
//#include <SoftwareSerial.h>

#include <inttypes.h>
#include <time.h>
#include <string.h>

#include "Comms.h"

#include <LoRa.h>
#include <Crc.h>

bool check_crc(command_t* command);

void write_command(command_t* cmd, interface_t interface)
{

    int size = 0;
    uint8_t buff[MAX_COMMAND_BUFFER + 5] = {0};
    
    buff[size++] = 0x55;
    buff[size++] = cmd->cmd;
    buff[size++] = cmd->id;
    buff[size++] = cmd->size;
    for(int i = 0; i < cmd->size; i++)
        buff[size++] = cmd->data[i];
    buff[size++] = ((cmd->crc >> 8) & 0xff);
    buff[size++] = ((cmd->crc) & 0xff);

    //Serial.print("Sent to interface");
    //Serial.println(interface);
    //for(int i = 0; i < size; i++) 
    //{
        //Serial.print(buff[i]);
        //Serial.print(" ");
    //}
    //Serial.println("");

    switch(interface)
    {
        case LoRa_INTERFACE:
        {
            LoRa.beginPacket();
            int sz = LoRa.write(buff, size);
            LoRa.endPacket(true);
            //Serial.print("Lora sent ");
            //Serial.print(sz);
            //Serial.println(" packets\n");
        }
        break;
        case RS485_INTERFACE:
            Serial2.write(buff, size);
        break;
        case Uart_INTERFACE:
            Serial.write(buff, size);
        break;
        default:
        break;
    }
}

static COMMAND_STATE parse_input(uint8_t read_byte, command_t* command, COMMAND_STATE cmd_state)
{
    uint8_t state = cmd_state;
    //Serial.print("data ");
    //Serial.println(read_byte);
    switch(state)
    {
        case SYNC:
        {
            if(read_byte == 0x55)
            {
                //start timeout timer
                state = CMD;
                command->data_recv = 0;
                memset(command, 0, sizeof(command_t));
                command->begin = millis();
            }
        }
        break;

        case CMD:
            command->cmd = (cmd_type_t)read_byte;
            state = ID;
        break;

        case ID:
            command->id = (cmd_type_t)read_byte;
            state = SIZE;
        break;

        case SIZE:                
            command->size = read_byte;
            if(command->size == 0)
                state = CRC1;
            else state = DATA;
        break;

        case DATA:
            command->data[command->data_recv++] = read_byte;
            if(command->data_recv == command->size)
                state = CRC1;
        break;

        case CRC1:
            command->crc = read_byte << 8;
            state = CRC2;
        break;

        case CRC2:
            command->crc += read_byte;
            state = END;
        break;

        default:
            state = SYNC;
    };

    return (COMMAND_STATE)state;
}

command_t* read_command_sync(int* error, interface_t interface)
{
    command_t* cmd = NULL;
    clock_t begin = millis();
    int count = 0;
    
    do {
        cmd = read_command(error, interface);
        //Serial.printf("hello %d %d %d\n", count++, *error, millis() - begin);
        //Serial.flush();
    } 
    while ( *error == CMD_READ_NO_CMD && 
            (millis() - begin) < RS485_TIMEOUT_TIME_MS);
    
    /* 
    //ugly version
    while(
        ( (cmd = read_command(error, interface)) == NULL &&  
          *error == CMD_READ_NO_CMD ) ||
        (millis() - begin) < RS485_TIMEOUT_TIME_MS )
    {
        Serial.printf("hello %d\n", count++);
        Serial.flush();
    }
    */    

    return cmd;
}

command_t* read_command(int* error, interface_t interface)
{
    static command_t command_arr[interface_t_size];
    static COMMAND_STATE state_arr[interface_t_size] = {SYNC};
    //static clock_t end_arr[interface_t_size] = {0};
    
    //Serial.print("Interface ");
    //Serial.println(interface);
    
    uint8_t index = (uint8_t)interface;
    command_t *command = &command_arr[index];
    COMMAND_STATE *state = &state_arr[index];
    //clock_t *end = &end_arr[index];

    size_t size;
    uint8_t read_byte;

    switch(interface)
    {
        case LoRa_INTERFACE:
        {

            //Work arourd for lora to not spam the spi bus
            static unsigned long begin_c = 0, end_c = 0;
            end_c = millis();
            if(end_c - begin_c < 5) break;
            begin_c = end_c;
            
            int packetSize = LoRa.parsePacket();
            //if(packetSize > 0) Serial.printf("packet recived %d\n", packetSize);
            
            while(packetSize != 0 && LoRa.available() && *state != END)
            {
                read_byte = LoRa.read();
                *state = parse_input(read_byte, command, *state);
            }
        }
        break;

        case RS485_INTERFACE:
        {
            while(Serial2.available() && *state != END)
            {
                read_byte = Serial2.read();
                *state = parse_input(read_byte, command, *state);
            }
        }
        break;
        
        case Uart_INTERFACE:
        {
            //if(Serial.available() > 0)
            //{
                //Serial.println("interface lora");
            //}
            while(Serial.available() && *state != END)
            {
                read_byte = Serial.read();
                *state = parse_input(read_byte, command, *state);
            }
        }
        break;

        default:
        break;
    };

    clock_t end = millis();
    int msec = (end - command->begin);

    //if timeout reset state
    if(*state != SYNC && msec > RS485_TIMEOUT_TIME_MS) //timeout
    {
        //Serial.printf("TIMEOUT\n"); //debug
        *state = SYNC;
        
        *error = CMD_READ_TIMEOUT;

        return NULL;
    }
    //if bad cr reset state
    else if(*state == END)
    {
        //Serial.println("got message");
        *state = SYNC;

        *error = CMD_READ_OK;

        //log_command(command);
        return command;
    }
    else //default
    {
        *error = CMD_READ_NO_CMD;
        return NULL;
    }
}

bool check_crc(command_t* command)
{
    uint16_t crc1, crc2;

    crc1 = command->crc;
    crc2 = crc((unsigned char*)command, command->size + 3); //+3 bytes cmd, id, size

    return (crc1 == crc2);
}