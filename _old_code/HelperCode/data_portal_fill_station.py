import PySimpleGUI as sg
import time
import serial

#mac
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.015) #set read timeout of 1s
#ser = serial.Serial('/dev/cu.usbserial-3', 115200, timeout=0.015) #set read timeout of 1s
#windows
#ser = serial.Serial('COM3', 115200, timeout=0.01) #set read timeout of 1s

log_delay = 0.01
#log_delay = 1
message_timeout = 0.5

missed_packets = 0
log_speed = 0

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

state_map = {
    "IDLE" : 0,
    "FUELING" : 1,
    "PROG1" : 2,
    "PROG2" : 3,
    "PROG3" : 4,
    "SAFETY" : 5,
    "ABORT": 6,
}

state_map_to_string = {
    0 : "IDLE",
    1 : "FUELING",
    2 : "PROG1",
    3 : "PROG2",
    4 : "PROG3",
    5 : "SAFETY",
    6 : "ABORT",
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

layout = [[sg.Text("Ax  Ay  Az  Gx  Gy  Gz", key = '_IMU_OUT_', size = (40, 5), auto_size_text=True, font=('Arial Bold', 16))],      
          [sg.Button('Toggle log', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]      

window = sg.Window('Window that stays open', layout)      


log_status = 0
def toggle_status():
    global log_status
    if log_status == 0: log_status = 1
    elif log_status == 1: log_status = 0



def print_status():
    global missed_packets
    global log_speed
    if(len(buff) < 16):
        print("bad status")
        return

    state = int.from_bytes(buff[3:4], byteorder='big', signed=True)
    if state < 0 or state >= len(state_map_to_string): 
        print("bad state decoding", state)
        return

    ax = int.from_bytes(buff[4:6], byteorder='big', signed=True)
    ay = int.from_bytes(buff[6:8], byteorder='big', signed=True)
    az = int.from_bytes(buff[8:10], byteorder='big', signed=True)

    ax = (ax * 0.244) / 1000 * 9.80665
    ay = (ay * 0.244) / 1000 * 9.80665
    az = (az * 0.244) / 1000 * 9.80665

    gx = int.from_bytes(buff[10:12], byteorder='big', signed=True)
    gy = int.from_bytes(buff[12:14], byteorder='big', signed=True)
    gz = int.from_bytes(buff[14:16], byteorder='big', signed=True)

    print("state", state)
    s1 = "State: " + state_map_to_string[state] + "\n"
    sa = "Ax: " + str(round(ax, 2)) + " Ay: " + str(round(ay, 2)) + " Az: " + str(round(az, 2)) + "\n"
    sg = "Gx: " + str(round(gx, 2)) + " Gy: " + str(round(gy, 2)) + " Gz: " + str(round(gz, 2)) + "\n"
    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 
    s = s1 + sa + sg + s2
    window['_IMU_OUT_'].update(s)

    print("ax:", ax, "ay:", ay, "az:", az)
    print("gx:", gx, "gy:", gy, "gz:", gz)

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
            missed_packets += 1
            print("command timeout", msec)
            print("buffer recieved:", buff) 
            return

        if msec > message_timeout:
            print("command timeout", msec)
            print("no cmd state: ", comm_state)
            missed_packets += 1
            return

        if comm_state == end_state:
            comm_state = sync_state
            print("ACK recieved:", len(buff))
            print("command time", msec)
            if(buff[1] == command_map['STATUS_ACK']):
                print_status()
            print("Cmd_ACK: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
            #print(buff)
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
        if event == '_STATUS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STATUS'], 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(6)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())

    log_end = time.perf_counter()
    if(cmd_launched == 0 and log_status == 1 and log_end - log_begin > log_delay):
        #send status cmd
        log_speed = 1 / (log_end - log_begin)
        log_begin = time.perf_counter()
        cmd = bytearray([0x55, command_map['STATUS'], 0, 0x20, 0x21])
        ser.write(cmd)
        read_cmd()
        #pass
    
    #while ser.in_waiting > 0:
        #print(ser.read(1).decode('utf-8'), end='')


    time.sleep(0.001)

window.close()
