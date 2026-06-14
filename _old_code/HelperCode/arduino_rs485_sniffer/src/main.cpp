#include <Arduino.h>
#include <SoftwareSerial.h>

#include <time.h>

#define MAX_COMMAND_BUFFER 150
#define RS485_TIMEOUT_TIME_MS 500 //try to get limit bounds 
/*
    Commands
*/
typedef enum
{
//shared commands
    CMD_STATUS,
    CMD_ABORT,
    CMD_EXEC_PROG,
    CMD_STOP_PROG,
    CMD_FUELING,

//FLIGHT computer commands
    CMD_READY,
    CMD_ARM,

    CMD_LED_ON,
    CMD_LED_OFF,

    CMD_IMU_CALIBRATE,

//FILLING station commands
    CMD_RESUME_PROG,

//used to get the number of commands 
    cmd_size,

//ACKs
    CMD_STATUS_ACK,
    CMD_FUELING_ACK,
    CMD_READY_ACK,
    CMD_ARM_ACK,
    CMD_ABORT_ACK,
    CMD_LED_ON_ACK,
    CMD_LED_OFF_ACK,
    CMD_IMU_CALIBRATE_ACK,
    CMD_EXEC_PROG_ACK,
    CMD_STOP_PROG_ACK,
    CMD_RESUME_PROG_ACK,
} cmd_type_t;

typedef enum 
{
    SYNC = 0,
    CMD,
    SIZE,
    DATA,
    CRC1, //first byte of crc
    CRC2,  //second byte of crc
    END,
} COMMAND_STATE;

typedef struct 
{
    cmd_type_t cmd;
    //int id;
    uint8_t size;
    uint8_t data[MAX_COMMAND_BUFFER];
    uint16_t crc;

    uint8_t data_recv; //helper pointer to fill data[]
    clock_t begin;
} command_t;

SoftwareSerial Serial2(6,7); //RX, TX

COMMAND_STATE state;
command_t command;
clock_t end;

void write_command(command_t* cmd)
{
    int size = 0;
    uint8_t buff[MAX_COMMAND_BUFFER + 5] = {0};
    
    buff[size++] = 0x55;
    buff[size++] = cmd->cmd;
    buff[size++] = cmd->size;
    for(int i = 0; i < cmd->size; i++)
        buff[size++] = cmd->data[i];
    buff[size++] = ((cmd->crc >> 8) & 0xff);
    buff[size++] = ((cmd->crc) & 0xff);

    Serial.write(buff, size);
}

void setup() {
  Serial.begin(115200);
  //Serial.begin(57600);
  Serial2.begin(115200);
  //Serial2.begin(38400);

  state = SYNC;
}

void loop() {
    //while(Serial.available()) Serial2.write(Serial.read());
    //while(Serial2.available() && state != END) 
    //{
        //uint8_t read_byte = Serial2.read();
    
        //switch(state)
        //{
            //case SYNC:
                //if(read_byte == 0x55)
                //{
                    ////start timeout timer
                    //state = CMD;
                    //command.data_recv = 0;
                    //memset(&command, 0, sizeof(command_t));
                    //command.begin = clock();
                //}
            //break;

            //case CMD:
                //command.cmd = (cmd_type_t)read_byte;
                //state = SIZE;
            //break;

            //case SIZE:                
                //command.size = read_byte;
                //if(command.size == 0)
                    //state = CRC1;
                //else state = DATA;
            //break;

            //case DATA:
                //command.data[command.data_recv++] = read_byte;
                //if(command.data_recv == command.size)
                    //state = CRC1;
            //break;

            //case CRC1:
                //command.crc = read_byte << 8;
                //state = CRC2;
            //break;

            //case CRC2:
                //command.crc += read_byte;
                //state = END;
            //break;

            //default:
                //state = SYNC;
        //};

    //}

    //end = clock();
    //int msec = (end - command.begin) * 1000 / CLOCKS_PER_SEC;

    ////if timeout reset state
    //if(state != SYNC && msec > RS485_TIMEOUT_TIME_MS) //timeout
    //{
        //Serial.println("reset"); //debug
        //state = SYNC;


    //}
    ////if bad cr reset state
    //else if(state == END /* && check_crc(&command) */)
    //{
        //state = SYNC;

    //}
    //else if(state == END)
    //{
        //state = SYNC;
    //}
    //else //default
    //{
    //}

  while(Serial2.available()) Serial.write(Serial2.read());
}                                                                                                                       