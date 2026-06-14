#include "FlashLog.h"

#include "HardwareCfg.h"
#include "GlobalVars.h"


bool log_running = false;
File file;

uint8_t current_id;
uint32_t data_used;

void start_log()
{ 
    log_running = true; 
    
    /*
     * TODO
     *   before opening verify that the file does not exist, 
     *   256 diferent reboots before we are out of files 
    */
    char filename[100] = "";
    int count = sprintf(filename, LOG_NAME_PATTERN, current_id);
    current_id++;
    Serial.print("new file \n");
    Serial.print(filename);
    Serial.print("\n");

    file = SD.open(filename, FILE_WRITE, true);

    if(!file)
    {
        log_running = 0;
    }
}
void stop_log() 
{ 
    log_running = false; 
    file.close();
}

uint8_t get_last_id()
{
    File root = SD.open("/");
    uint8_t last_log_id = 0;
    while(1)
    {
        File entry = root.openNextFile();

        if(!entry)
            break; //no more files

        if(!entry.isDirectory()) 
        {
            uint8_t temp;
            sscanf(entry.name(), "%u", &temp);
            Serial.print(temp);
            last_log_id = max(last_log_id, temp);
        }
        Serial.print(" Name in sd: ");
        Serial.print(entry.name());
        Serial.print(" ");
        Serial.print(entry.size());
        Serial.print(" \n");

        entry.close();
    }

    root.close();

    return last_log_id;
}

void log(void* data, uint16_t size, log_event_t event)
{
    if(log_running == false) return;

    static uint8_t buff[256];
    uint16_t index = 0;

    uint32_t time = millis();

    buff[index++] = event;

    buff[index++] = (time >> 24) & 0xff;
    buff[index++] = (time >> 16) & 0xff;
    buff[index++] = (time >> 8)  & 0xff;
    buff[index++] = (time)       & 0xff;

    switch (event)
    {
    case SENSOR_READING:
    {
        buff[index++] = (tank_pressure >> 8) & 0xff;
        buff[index++] = (tank_pressure) & 0xff;
        buff[index++] = (tank_liquid >> 8) & 0xff;
        buff[index++] = (tank_liquid) & 0xff;

        buff[index++] = (He_Module.temperature >> 8) & 0xff;
        buff[index++] = (He_Module.temperature) & 0xff;
        buff[index++] = (N2O_Module.temperature >> 8) & 0xff;
        buff[index++] = (N2O_Module.temperature) & 0xff;
        buff[index++] = (Line_Module.temperature >> 8) & 0xff;
        buff[index++] = (Line_Module.temperature) & 0xff;

        buff[index++] = (He_Module.pressure >> 8) & 0xff;
        buff[index++] = (He_Module.pressure) & 0xff;
        buff[index++] = (N2O_Module.pressure >> 8) & 0xff;
        buff[index++] = (N2O_Module.pressure) & 0xff;
        buff[index++] = (Line_Module.pressure >> 8) & 0xff;
        buff[index++] = (Line_Module.pressure) & 0xff;

        buff[index++] = (ematch_v_reading >> 8) & 0xff;
        buff[index++] = (ematch_v_reading) & 0xff;
        
        buff[index++] = (weight1 >> 8) & 0xff;
        buff[index++] = (weight1) & 0xff;

        buff[index++] = (uint8_t)((log_running << 7) |
                                        (He_Module.valve_state << 6) | 
                                        (N2O_Module.valve_state << 5) |
                                        (Line_Module.valve_state << 4) );
    }
    break;
    
    case MSG_RECEIVED:
    {
        command_t* cmd = (command_t*) data;
        buff[index++] = cmd->cmd; 
        buff[index++] = cmd->id; 
        buff[index++] = cmd->size; 

        for(int i = 0; i < cmd->size; i++)
            buff[index++] = cmd->data[i];
    }
    break;
    
    case MSG_SENT:
    break;
    
    case SYSTEM_ERROR:
    break;
    
    case STATE_CHANGE:
        buff[index++] = *(uint8_t*)(data);
    break;
    
    case EVENT_REACTION:
    break;

    default:
    break;
    }

    file.write(buff, index);
}

void dump_log(uint8_t id)
{
    if(log_running)
    {
        Serial.write(0);
        Serial.write(0);
        Serial.write(0);
        Serial.write(0);
        printf("error, cannot dump flash while logging\n");
        return;
    }

    char filename[100];
    sprintf(filename, LOG_NAME_PATTERN, id);

    File flashDump = SD.open(filename);
    if(!flashDump)
    {
        Serial.write(0);
        Serial.write(0);
        Serial.write(0);
        Serial.write(0);
        printf("error openening log file\n");
        return;
    }

    delay(50);
    //first send the size of the file
    Serial.write((flashDump.size() >> 24) & 0xff);
    Serial.write((flashDump.size() >> 16) & 0xff);
    Serial.write((flashDump.size() >> 8) & 0xff);
    Serial.write((flashDump.size() & 0xff));
    

    //Serial.print(flashDump.size());

    while(flashDump.available())
    {
        Serial.write(flashDump.read());
    }

    flashDump.close();
}

void get_log_ids(uint8_t* files, uint8_t* files_index)
{
    *files_index = 0;

    File root = SD.open("/");
    while(1)
    {
        File entry = root.openNextFile();
        if(!entry)
            break; //no more files

        if(!entry.isDirectory()) 
        {
            uint8_t temp;
            sscanf(entry.name(), "%u", &temp);
            *(files + (*files_index)) = temp;
            Serial.printf("%u\n", *(files + (*files_index)));
            (*files_index)++;
        }

        //Serial.print(" Name in sd: ");
        //Serial.print(entry.name());
        //Serial.print(" ");
        //Serial.print(*files_index);
        //Serial.print(" size: ");
        //Serial.print(entry.size());
        //Serial.print(" \n");

        entry.close();
    }

    root.close();

}