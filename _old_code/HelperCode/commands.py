import PySimpleGUI as sg
import time
import serial

#mac
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.015) #set read timeout of 1s
#windows
#ser = serial.Serial('COM3', 115200, timeout=0.01) #set read timeout of 1s

message_timeout = 0.3
missed_packets = 0

command_map = {
    "STATUS" : 0,
    "ABORT" : 1,
    "EXEC_PROG" : 2, 
    "STOP_PROG" : 3,
    "FUELING" : 4,

#FLIGHT computer commands
    "READY" : 5,
    "ARM" : 6,

    "LED_ON" : 7,
    "LED_OFF" : 8,

    "IMU_CALIB" : 9,

#FILLING station commands
    "RESUME_PROG" : 10,

#State machine commands
    "ADD_WORK" : 11,
    "REMOVE_WORK" : 12,

#used to get the number of commands 
    "cmd_size" : 13,

#ACKs
    "STATUS_ACK" : 14,
    "FUELING_ACK" : 15,
    "READY_ACK" : 16,
    "ARM_ACK" : 17,
    "ABORT_ACK" : 18, 
    "LED_ON_ACK" : 19,
    "LED_OFF_ACK" : 20,
    "IMU_CALIB_ACK" : 21,
    "EXEC_PROG_ACK" : 22,
    "STOP_PROG_ACK" : 23,
    "RESUME_PROG_ACK" : 24,
    "ADD_WORK_ACK" : 25,
    "REMOVE_WORK_ACK" : 26 
}

sync_state = 1
cmd_state = 2
size_state = 3
data_state = 4
crc_1_state = 5
crc_2_state = 6
end_state = 7

comm_state = sync_state
data_total = 0
data_recv = 0
begin = time.perf_counter()
end = time.perf_counter()
buff = bytearray()

sg.theme('DarkAmber')    # Keep things interesting for your users

layout = [[sg.Text("Missed packets: 0", key = '_PACKETS_', auto_size_text=True, font=('Arial Bold', 16))],
          [sg.Button('Ready', key = '_READY_', size = (10,5)),
           sg.Button('Arm', key = '_ARM_', size = (10,5)), 
           sg.Button('Abort', key = '_ABORT_', size = (10,5)), 
           sg.Button('Log', key = "_LOG_", size = (10,5)),
           sg.Button('IMU_calib', key = "_IMU_calib_", size = (10,5)),
           sg.Button('Toggle log', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]      

window = sg.Window('Window that stays open', layout)      

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
                comm_state = size_state

            elif comm_state == size_state:
                buff.append(ch)
                data_total = int(ch)
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
        
        end = time.perf_counter()
        msec = (end - begin) 
        #if(comm_state != sync_state): print("ms: ", msec)
        if comm_state != sync_state and msec > message_timeout:
            comm_state = sync_state
            print("command timeout", msec)
            print("buffer recieved:", buff) 

            missed_packets += 1
            s = "Missed packets: " + str(missed_packets) 
            window['_PACKETS_'].update(s)
            return

        if msec > message_timeout:
            print("command timeout", msec)
            print("no cmd state: ", comm_state)

            missed_packets += 1
            s = "Missed packets: " + str(missed_packets) 
            window['_PACKETS_'].update(s)
            return

        if comm_state == end_state:
            comm_state = sync_state
            print("ACK recieved:", len(buff))
            print("command time", msec)
            #if(buff[1] == command_map['STATUS_ACK']):  this is now handled by a diferent app
                #print_status()
            print("Cmd_ACK: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
            #print(buff)
            return

def arm():
    cmd = bytearray([0x55, command_map['ARM'], 1, 1, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 6 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[3]) != 1):
        print("ARM1 error", buff)
        return
    else:
        print("ARM 1 ok")

    cmd = bytearray([0x55, command_map['ARM'], 1, 2, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 6 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[3]) != 2):
        print("ARM2 error", buff)
        return
    else:
        print("ARM 2 ok")
    

    cmd = bytearray([0x55, command_map['ARM'], 1, 3, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 6 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[3]) != 3):
        print("ARM3 error", buff)
        return
    else:
        print("ARM 3 ok")


while True:           
    event, values = window.read(timeout=1) 
    if event != '__TIMEOUT__': print(event, values)       
    cmd_launched = 0

    if event == sg.WIN_CLOSED or event == 'Exit':
        break      
    elif event == '_LOG_TOGGLE_':
        #toggle_status()    
        pass
    else:
        if event == '_ARM_':
            cmd_launched = 1
            arm()
            pass
        elif event == '_READY_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['READY'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            pass
        elif event == '_ABORT_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['ABORT'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            pass
        elif event == '_LED_ON_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_ON'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_LED_OFF_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_OFF'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STATUS'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_IMU_calib_':
            print("calibrate imu")
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['IMU_CALIB'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()

    time.sleep(0.01)

window.close()