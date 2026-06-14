import PySimpleGUI as sg
import time
import serial
import sys

#mac
#ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.015) #set read timeout of 1s
#ser = serial.Serial('/dev/cu.usbserial-3', 115200, timeout=0.015) #set read timeout of 1s
#windows
#ser = serial.Serial('COM3', 115200, timeout=0.01) #set read timeout of 1s

ser_timeout = 0.01
if(len(sys.argv) > 2):
    ser = serial.Serial(sys.argv[2], 115200, timeout=ser_timeout) #set read timeout of 1s
else:
    ser = serial.Serial('COM3', 115200, timeout=ser_timeout) #set read timeout of 1s

#log_delay = 0.05
log_delay = 0.5
#message_timeout = 0.25
message_timeout = 1

missed_packets = 0
log_speed = 0


cmd_id = 1

command_map = {
    "STATUS" : 0,
    "LOG" : 1,
    "ABORT" : 2,
    "EXEC_PROG" : 3, 
    "STOP_PROG" : 4,
    "FUELING" : 5,
    "MANUAL" : 6,
    "MANUAL_EXEC" : 7,
    "READY" : 8,
    "ARM" : 9,

#FLIGHT computer commands
    "ALLOW_LAUNCH" : 10,

#FILLING station commands
    "RESUME_PROG" : 11,
    "FIRE_PYRO" : 12,

#used to get the number of commands 
    "cmd_size" : 13,

#ACKs
    "STATUS_ACK" : 14,
    "LOG_ACK" : 15,
    "ABORT_ACK" : 16, 
    "EXEC_PROG_ACK" : 17,
    "STOP_PROG_ACK" : 18,
    "FUELING_ACK" : 19,
    "MANUAL_ACK" : 20,
    "MANUAL_EXEC_ACK" : 21,
    "READY_ACK" : 22,
    "ARM_ACK" : 23,
    "ALLOW_LAUNCH_ACK" : 24,
    "RESUME_PROG_ACK" : 25,
    "FIRE_PYRO_ACK" : 26, 
}

state_map_fill = {
    "IDLE" : 0,
    "FUELING" : 1,
    "MANUAL" : 2,
    "FILL_He" : 3,
    "FILL_N2O" : 4,
    "PURGE_LINE" : 5,
    "STOP" : 6,
    "ABORT": 7,
    "READY" : 8,
    "ARMED" : 9,
    "FIRE" : 10,
    "LAUNCH": 11,
}

state_map_rocket = {
    "IDLE" : 0,
    "FUELING" : 1,
    "MANUAL" : 2,
    "SAFETY_PRESSURE" : 3,
    "PURGE_PRESSURE" : 4,
    "PURGE_LIQUID" : 5,
    "SAFETY_PRESSURE_ACTIVE" : 6,
    "READY" : 7,
    "ARMED" : 8,
    "LAUNCH": 9,
    "ABORT": 10,
    "IMU_CALIB": 11
}

state_map_to_string_fill = {
    0 : "IDLE",
    1 : "FUELING",
    2 : "MANUAL",
    3 : "FILL_He",
    4 : "FILL_N2O",
    5 : "PURGE_LINE",
    6 : "SAFETY",
    7 : "ABORT",
    8 : "READY",
    9 : "ARMED",
    10 : "FIRE",
    11 : "LAUNCH",
}

state_map_to_string_rocket = {
    0 : "IDLE",
    1 : "FUELING",
    2 : "MANUAL",
    3 : "SAFETY_PRESSURE",
    4 : "PURGE_PRESSURE",
    5 : "PURGE_LIQUID",
    6 : "SAFETY_PRESSURE_ACTIVE",
    7 : "READY",
    8 : "ARMED",
    9 : "LAUNCH",
    10 : "ABORT",
    11 : "IMU_CALIB"
}

manual_command_map = {
    "MANUAL_FLASH_LOG_START" : 0,
    "MANUAL_FLASH_LOG_STOP" : 1,
    "MANUAL_FLASH_IDS" : 2,
    "MANUAL_FLASH_DUMP" : 3,

    "MANUAL_VALVE_STATE" : 4,
    "MANUAL_VALVE_MS" : 5,
    
    "MANUAL_IMU_CALIBRATE" : 6,
    
    "MANUAL_LOADCELL_CALIBRATE" : 7,
    "MANUAL_LOADCELL_TARE" : 8,
    
    "manual_cmd_size" : 9,

    "MANUAL_FLASH_LOG_START_ACK" : 10,
    "MANUAL_FLASH_LOG_STOP_ACK" : 11,
    "MANUAL_FLASH_IDS_ACK" : 12,
    "MANUAL_FLASH_DUMP_ACK" : 13,
    "MANUAL_VALVE_STATE_ACK" : 14,
    "MANUAL_VALVE_MS_ACK" : 15,
    "MANUAL_IMU_CALIBRATE_ACK" : 16,
    "MANUAL_LOADCELL_CALIBRATE_ACK" : 17,
    "MANUAL_LOADCELL_TARE_ACK" : 18
}


sync_state = 1
cmd_state = 2
id_state = 3
size_state = 4
data_state = 5
crc_1_state = 6
crc_2_state = 7
end_state = 8

comm_state = sync_state
data_total = 0
data_recv = 0
begin = time.perf_counter()
end = time.perf_counter()
buff = bytearray()

sg.theme('DarkAmber')    # Keep things interesting for your users

layout_log_test = [[sg.Text("State: None", key = '_STATUS_OUT_', size = (20, 5), auto_size_text=True, font=('Arial Bold', 16))],
          [sg.Button('Ready', key = '_READY_', size = (10,5)),
           sg.Button('Stop', key = '_STOP_', size = (10, 5)),
           sg.Button('Manual Mode', key = '_MANUAL_', size = (10, 5))],
          [sg.Button('Start log', key = '_START_LOG_', size = (10, 5)),
           sg.Button('Stop log', key = '_STOP_LOG_', size = (10, 5)),
           sg.Button('Log IDs', key = '_LOG_IDS_', size = (10, 5)),
           sg.Button('Log dump', key = '_LOG_DUMP_', size = (10, 5)),
           sg.Input(key = '_LOG_ID_', size = (5,5), font=('Arial Bold', 16)) ],
          [sg.Button('Status', key = "_STATUS_", size = (10,5)),
           sg.Button('Toggle Status', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]      

window_name = ""
layout = layout_log_test
window_name = "Flash logger "
if(len(sys.argv) > 1):
    if(sys.argv[1] == 'r'):
        state_map = state_map_rocket
        state_map_to_string = state_map_to_string_rocket
        window_name += "Rocket"
        cmd_id = 1
    elif(sys.argv[1] == 'f'):
        state_map = state_map_fill
        state_map_to_string = state_map_to_string_fill
        window_name += "Fill Station"
        cmd_id = 2

window = sg.Window(window_name, layout)      

log_status = 0
def toggle_status():
    global log_status
    if log_status == 0: log_status = 1
    elif log_status == 1: log_status = 0

def dump(id):

    cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 2, manual_command_map['MANUAL_FLASH_DUMP'], id, 0x20, 0x21])
    ser.reset_input_buffer()
    ser.write(cmd)

    #size1 = int.from_bytes(ser.read(1), 'little')
    #size2 = int.from_bytes(ser.read(1), 'little')
    ser._timeout = 2
    
    buff = []
    print("buff", buff)
    for i in range(4):
        ch = ser.read(1)
        ch = int.from_bytes(ch, 'little')
        buff.append(ch)

    print("buff", buff[0], buff[1], buff[2], buff[3])
    if(len(buff) < 4):
        print("error not enouch time")
        return

    size = int.from_bytes(buff[0:4], byteorder='big', signed=False)
    print("log size", size)

    with open("{}.bin".format(id), 'wb+') as f:
        res = ''
        buff = []
        for i in range(0, size + 4095, 4096):
            #5 seconds to retreive dump data
            res = ser.read(4096)
            buff.append(res)
            #f.write(res)
            print("Progress:",i, round((i / size) * 100, 2), "%")

        for i in buff:
            f.write(i)

        #print("Log dump: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
        #print(res)
        print("Log size recieved", len(buff))

    ser._timeout = ser_timeout
    read_cmd()

def print_status_rocket():
    global missed_packets
    global log_speed
    #if(len(buff) < 16):
        #print("bad status")
        #return

    state = int.from_bytes(buff[4:5], byteorder='big', signed=True)
    if state < 0 or state >= len(state_map_to_string): 
        print("bad state decoding", state)
        return
    
    s1 = "State: " + state_map_to_string[state] + "\n"

    s = s1
    window['_STATUS_OUT_'].update(s)

def print_status_fill():
    global missed_packets
    global log_speed
    #if(len(buff) < 16):
        #print("bad status")
        #return

    state = int.from_bytes(buff[4:5], byteorder='big', signed=True)
    if state < 0 or state >= len(state_map_to_string): 
        print("bad state decoding", state)
        return
    
    s1 = "State: " + state_map_to_string[state] + "\n"
    s = s1
    window['_STATUS_OUT_'].update(s)

def print_status():
    id = int.from_bytes(buff[2:3], byteorder='big', signed=False)
    print("id: ", id)
    if sys.argv[1] == 'r': return print_status_rocket()
    elif sys.argv[1] == 'f': return print_status_fill()
    else: return

def read_cmd():
    global comm_state, sync_state, cmd_state, size_state, data_state, crc_1_state, crc_2_state, end_state
    global buff
    global data_recv, data_total
    global begin, end
    global missed_packets

    buff = bytearray()
    data_recv = 0
    begin = time.perf_counter()
    
    while True: 
        while ser.in_waiting > 0:
            ch = ser.read(1)
            ch = int.from_bytes(ch, 'little')

            if comm_state == sync_state:
                if ch == 0x55:
                    #begin = time.perf_counter()
                    comm_state = cmd_state
                    buff.append(ch)
                else:
                    print(chr(ch), end="")
                    #print(str(ch).encode('utf-8'))

            elif comm_state == cmd_state:
                buff.append(ch)
                comm_state = id_state

            elif comm_state == id_state:
                buff.append(ch)
                comm_state = size_state

            elif comm_state == size_state:
                buff.append(ch)
                data_total = int(ch)
                #print("data total", data_total)
                if data_total == 0:
                    comm_state = crc_1_state
                else:
                    comm_state = data_state

            elif comm_state == data_state:
                buff.append(ch)
                data_recv += 1
                if data_total == data_recv:
                    comm_state = crc_1_state

            elif comm_state == crc_1_state:
                buff.append(ch)
                comm_state = crc_2_state

            elif comm_state == crc_2_state: 
                buff.append(ch)
                comm_state = end_state
            
            #print("Cmd rs485 byte recv",ch, comm_state, ser.in_waiting)
        
        end = time.perf_counter()
        msec = (end - begin) 
        
        if comm_state == end_state:
            comm_state = sync_state
            print("ACK recieved:", len(buff))
            print("Time", msec)
            if(buff[1] == command_map['STATUS_ACK']):
                print_status()
            else:
                print("not status ack")
            #else:
            print("Cmd_ACK: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
            time.sleep(0.005)
            #print(buff)
            return
        #if(comm_state != sync_state): print("ms: ", msec)
        elif comm_state != sync_state and msec > message_timeout:
            missed_packets += 1
            print("command timeout", msec, len(buff), data_recv, data_total)
            print("cmd state: ", comm_state)
            #print("buffer recieved:", buff) 
            print("Cmd fail: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
            comm_state = sync_state
            time.sleep(0.015)
            return

        elif msec > message_timeout:
            print("command timeout", msec)
            print("no cmd state: ", comm_state)
            print("Cmd fail: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
            missed_packets += 1
            time.sleep(0.015)
            return



log_begin = time.perf_counter()
log_end = time.perf_counter()


while True:           
    event, values = window.read(timeout=1) 
    if event != '__TIMEOUT__': print(event, values)       
    cmd_launched = 0

    if event == sg.WIN_CLOSED or event == 'Exit':
        break      
    elif event == '_LOG_TOGGLE_':
        toggle_status()    
    else:
        if event == '_READY_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['READY'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
            pass
        elif event == '_STATUS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STATUS'], cmd_id, 0, 0x20, 0x21])
            print("Cmd: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(6)
            #print(bytearray(rep).hex())
        elif event == '_STOP_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STOP_PROG'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_MANUAL_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_READY_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['READY'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_START_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_LOG_START'], 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_STOP_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_LOG_STOP'], 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_LOG_IDS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_IDS'], 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_LOG_DUMP_':
            cmd_launched = 1
            id = int(window['_LOG_ID_'].get())
            dump(id)

    log_end = time.perf_counter()
    if(cmd_launched == 0 and log_status == 1 and log_end - log_begin > log_delay):
        #send status cmd
        log_speed = 1 / (log_end - log_begin)
        log_begin = time.perf_counter()
        cmd = bytearray([0x55, command_map['STATUS'], cmd_id, 0, 0x20, 0x21])
        ser.write(cmd)
        read_cmd()
        #pass

    time.sleep(0.001)

window.close()
