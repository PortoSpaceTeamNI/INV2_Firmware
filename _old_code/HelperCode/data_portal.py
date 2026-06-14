import PySimpleGUI as sg
import time
import serial

#mac
#ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.015) #set read timeout of 1s
#ser = serial.Serial('/dev/cu.usbmodem1301', 115200, timeout=0.015) #set read timeout of 1s
#windows
ser = serial.Serial('COM6', 115200, timeout=0.01) #set read timeout of 1s

log_delay = 0.01
message_timeout = 0.05
#message_timeout = 0.5

missed_packets = 0
log_speed = 0
last_log = 0


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

#FLASH log commands
    "FLASH_LOG_START" : 11,
    "FLASH_LOG_STOP" : 12,
    "FLASH_IDS" : 13,
    "FLASH_DUMP" : 14,

#used to get the number of commands 
    "cmd_size" : 15,

#ACKs
    "STATUS_ACK" : 16,
    "ABORT_ACK" : 17, 
    "EXEC_PROG_ACK" : 18,
    "STOP_PROG_ACK" : 19,
    "FUELING_ACK" : 20,
    "READY_ACK" : 21,
    "ARM_ACK" : 22,
    "LED_ON_ACK" : 23,
    "LED_OFF_ACK" : 24,
    "IMU_CALIB_ACK" : 25,
    "RESUME_PROG_ACK" : 26,
    "FLASH_LOG_START_ACK" : 27,
    "FLASH_LOG_STOP_ACK" : 28,
    "FLASH_IDS_ACK" : 29,
    "FLASH_DUMP_ACK" : 30,
}

state_map_fill = {
    "IDLE" : 0,
    "FUELING" : 1,
    "PROG1" : 2,
    "PROG2" : 3,
    "PROG3" : 4,
    "STOP" : 5,
    "ABORT": 6,
}

state_map_rocket = {
    "IDLE" : 0,
    "FUELING" : 1,
    "PROG1" : 2,
    "PROG2" : 3,
    "SAFETY" : 4,
    "READY" : 5,
    "ARMED" : 6,
    "LAUNCH": 7,
    "ABORT": 8,
    "IMU_CALIB": 9
}

state_map_to_string_fill = {
    0 : "IDLE",
    1 : "FUELING",
    2 : "PROG1",
    3 : "PROG2",
    4 : "PROG3",
    5 : "SAFETY",
    6 : "ABORT",
}

state_map_to_string_rocket = {
    0 : "IDLE",
    1 : "FUELING",
    2 : "PROG1",
    3 : "PROG2",
    4 : "SAFETY",
    5 : "READY",
    6 : "ARMED",
    7 : "LAUNCH",
    8 : "ABORT",
    9 : "IMU_CALIB"
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

layout = [[sg.Text("Rocket:\n", key = '_ROCKET_OUT_', size = (25, 6), auto_size_text=True, font=('Arial Bold', 16)), sg.Text("Fill Station:\n", key = '_FILL_OUT_', size = (25, 6), auto_size_text=True, font=('Arial Bold', 16))],
          [sg.Text("Statitstics:\n", key = '_STAT_ROCKET_', size = (25, 5), auto_size_text=True, font=('Arial Bold', 16)), sg.Text("Statitstics:\n", key = '_STAT_FILL_', size = (25, 5), auto_size_text=True, font=('Arial Bold', 16))],
          [sg.Button("Reset", key='_RESET_'), sg.Exit()]]      

window = sg.Window('Window that stays open', layout)      

last_rocket_msg = -1
last_fill_msg = -1

total_rocket_timing = 0
count_rocket_timing = 0
total_fill_timing = 0
count_fill_timing = 0

worst_rocket_timming = 0
worst_fill_timming = 0

missed_fill_response = 0
correct_fill_response = 0

def print_rocket():
    global last_rocket_msg, total_rocket_timing, count_rocket_timing, worst_rocket_timming, missed_fill_response, correct_fill_response
    global last_fill_msg
    msg_time = time.perf_counter()

    count_rocket_timing += 1
    if(last_rocket_msg == -1):
        last_rocket_msg = msg_time
    else:
        if(last_fill_msg != -1 and last_fill_msg < last_rocket_msg): 
            missed_fill_response += 1
        else:
            correct_fill_response += 1


        time_diff = msg_time - last_rocket_msg
        last_rocket_msg = msg_time
        total_rocket_timing += time_diff
        print("time diff", time_diff)
        worst_rocket_timming = max(worst_rocket_timming, time_diff)

    state = int.from_bytes(buff[4:5], byteorder='big', signed=False)
    if state < 0 or state >= len(state_map_to_string_rocket): 
        print("bad state decoding")
        return
    
    t1 = int.from_bytes(buff[5:7], byteorder='big', signed=True) 
    t2 = int.from_bytes(buff[7:9], byteorder='big', signed=True) 
    p1 = int.from_bytes(buff[9:11], byteorder='big', signed=True) 
    p2 = int.from_bytes(buff[11:13], byteorder='big', signed=True) 

    print("state", state)
    s1 = "State: " + state_map_to_string_rocket[state] + "\n"
    st1 = "T1: " + str(t1) + '\n'
    st2 = "T2: " + str(t2) + '\n'
    sp1 = "P1: " + str(p1) + '\n'
    sp2 = "P2: " + str(p2) + '\n'

    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 
    s = s1 + st1 + st2 + sp1 + sp2 + s2
    window['_ROCKET_OUT_'].update(s)

    s1 = "Mean time: " + str(round(total_rocket_timing / count_rocket_timing, 5)) + "\n"
    s2 = "Worst time: " + str(round(worst_rocket_timming, 5)) + "\n"
    s3 = "Missed reponses: " + str(missed_fill_response) + "\n"
    s4 = "Got reponses: " + str(correct_fill_response) + "\n"
    s = s1 + s2 + s3 + s4
    window['_STAT_ROCKET_'].update(s)

    return

def print_fill_station():
    global last_fill_msg, total_fill_timing, count_fill_timing, worst_fill_timming

    msg_time = time.perf_counter()

    if(last_fill_msg == -1):
        last_fill_msg = msg_time
        count_fill_timing += 1
    else:
        time_diff = msg_time - last_rocket_msg
        last_fill_msg = msg_time
        total_fill_timing += time_diff
        count_fill_timing += 1
        worst_fill_timming = max(worst_fill_timming, time_diff)
    
    state = int.from_bytes(buff[4:5], byteorder='big', signed=False)
    if state < 0 or state >= len(state_map_to_string_fill): 
        print("bad state decoding")
        return

    s = "State: " + state_map_to_string_fill[state] + "\n"
    window['_FILL_OUT_'].update(s)

    s1 = "Mean time: " + str(round(total_fill_timing / count_fill_timing, 5)) + "\n"
    s2 = "Worst time: " + str(round(worst_fill_timming, 5)) + "\n"
    s = s1 + s2
    window['_STAT_FILL_'].update(s)

    return


def print_status():
    global missed_packets
    global log_speed
    #if(len(buff) < 16):
        #print("bad status")
        #return

    #first 4 bytes of buffer are
    # 0 - 0x55
    # 1 - cmd
    # 2 - id
    # 3 - size
    id = int.from_bytes(buff[2:3], byteorder='big', signed=False)
    print("id: ", id)
    if id == 0x1: return print_rocket()
    elif id == 0x2: return print_fill_station()
    else: return


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
    global last_log, log_speed

    buff = bytearray()
    data_recv = 0
    begin = time.perf_counter()
    msec = 0
    while True: 
        while ser.in_waiting > 0:
            ch = ser.read(1)
            ch = int.from_bytes(ch, 'little')

            if comm_state == sync_state:
                if ch == 0x55:
                    buff = bytearray()
                    data_recv = 0
                    begin = time.perf_counter()
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

            if comm_state == end_state and msec < message_timeout:
                comm_state = sync_state
                print("ACK recieved:", len(buff))
                print("command time", msec)
                log_speed = 1 / ( end - last_log )
                if(len(buff) > 1 and buff[1] == command_map['STATUS']):
                    print_status()
                last_log = end
                print("Cmd_ACK: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
                #print(buff)
                return

        if comm_state != sync_state and msec > message_timeout:
            comm_state = sync_state
            missed_packets += 1
            print("command timeout", msec)
            print("buffer recieved:", buff) 
            return

        if msec > message_timeout:
            print("command timeout2", msec)
            print("no cmd state: ", comm_state)
            missed_packets += 1
            return


log_begin = time.perf_counter()
log_end = time.perf_counter()

while True:           
    event, values = window.read(timeout=1) 
    if event != '__TIMEOUT__': print(event, values)       
    cmd_launched = 0

    if event == sg.WIN_CLOSED or event == 'Exit':
        break      
    
    if event == '_RESET_':
        last_rocket_msg = -1
        last_fill_msg = -1

        total_rocket_timing = 0
        count_rocket_timing = 0
        total_fill_timing = 0
        count_fill_timing = 0

        worst_rocket_timming = 0
        worst_fill_timming = 0

        missed_fill_response = 0
        correct_fill_response = 0 

    #while ser.in_waiting > 0:
        #print(ser.read(1).decode('utf-8'), end='')

    read_cmd()
    #time.sleep(0.01)

window.close()
