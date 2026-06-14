import PySimpleGUI as sg
import time
import serial
import sys

#mac
#ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.015) #set read timeout of 1s
#ser = serial.Serial('/dev/cu.usbserial-3', 115200, timeout=0.015) #set read timeout of 1s
#windows
#ser = serial.Serial('COM3', 115200, timeout=0.01) #set read timeout of 1s

ser_timeout = 0.001
if(len(sys.argv) > 2):
    ser = serial.Serial(sys.argv[2], 115200, timeout=ser_timeout) #set read timeout of 1s
else:
    ser = serial.Serial('COM3', 115200, timeout=ser_timeout) #set read timeout of 1s

log_delay = 0.05
#log_delay = 1
message_timeout = 0.25
#message_timeout = 0.1

missed_packets = 0
log_speed = 0

log_file_size = 4096

cmd_id = 1

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
    "FIRE_PYRO" : 11,

#FLASH log commands
    "FLASH_LOG_START" : 12,
    "FLASH_LOG_STOP" : 13,
    "FLASH_IDS" : 14,
    "FLASH_DUMP" : 15,

#used to get the number of commands 
    "cmd_size" : 16,

#ACKs
    "STATUS_ACK" : 17,
    "ABORT_ACK" : 18, 
    "EXEC_PROG_ACK" : 19,
    "STOP_PROG_ACK" : 20,
    "FUELING_ACK" : 21,
    "READY_ACK" : 22,
    "ARM_ACK" : 23,
    "LED_ON_ACK" : 24,
    "LED_OFF_ACK" : 25,
    "IMU_CALIB_ACK" : 26,
    "RESUME_PROG_ACK" : 27,
    "FIRE_PYRO_ACK" : 28, 
    "FLASH_LOG_START_ACK" : 29,
    "FLASH_LOG_STOP_ACK" : 30,
    "FLASH_IDS_ACK" : 31,
    "FLASH_DUMP_ACK" : 32,
}

state_map_fill = {
    "IDLE" : 0,
    "FUELING" : 1,
    "PROG1" : 2,
    "PROG2" : 3,
    "PROG3" : 4,
    "STOP" : 5,
    "ABORT": 6,
    "READY" : 7,
    "ARMED" : 8,
    "FIRE" : 9,
    "LAUNCH": 10,
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
    7 : "READY",
    8 : "ARMED",
    9 : "FIRE",
    10 : "LAUNCH",
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

layout_rocket = [[sg.Button('LED_ON', key = '_LED_ON_', size = (10,5)), sg.Button('LED_OFF', key = '_LED_OFF_', size = (10,5)), sg.Text("Ax  Ay  Az  Gx  Gy  Gz", key = '_STATUS_OUT_', size = (40, 6), auto_size_text=True, font=('Arial Bold', 16))],      
          [sg.Button('Start Fueling', key = '_FUELING_', size = (10, 5)),
           sg.Button('Stop', key = '_STOP_', size = (10, 5)),
           sg.Button('Exec prog', key = '_EXEC_', size = (10, 5)),
           #sg.Combo(values = (1,2), default_value = 1, key = '_PROG_', size = (10,5), font=('Arial Bold', 16)),
           sg.Input(key = '_PROG_', size = (20,5), font=('Arial Bold', 16))],
          [sg.Button('Ready', key = '_READY_', size = (10,5)),
           sg.Button('Arm', key = '_ARM_', size = (10,5)), 
           sg.Button('Abort', key = '_ABORT_', size = (10,5)), 
           sg.Button('Status', key = "_STATUS_", size = (10,5)),
           sg.Button('IMU_calib', key = "_IMU_calib_", size = (10,5)),
           sg.Button('Toggle log', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]      

layout_fill = [[sg.Button('LED_ON', key = '_LED_ON_', size = (10,5)), sg.Button('LED_OFF', key = '_LED_OFF_', size = (10,5)), sg.Text("Ax  Ay  Az  Gx  Gy  Gz", key = '_STATUS_OUT_', size = (40, 11), auto_size_text=True, font=('Arial Bold', 16))],      
          [sg.Button('Exec prog', key = '_EXEC_', size = (10, 5)),
           #sg.Combo(values = (1,2,3), default_value = 1, key = '_PROG_', size = (10,5), font=('Arial Bold', 16)),
           sg.Input(key = '_PROG_', size = (20,5), font=('Arial Bold', 16))],
          [sg.Button('Start Fueling', key = '_FUELING_', size = (10, 5)),
           sg.Button('Stop', key = '_STOP_', size = (10, 5)),
           sg.Button('Resume', key = '_RESUME_', size = (10, 5)), sg.Button('Abort', key = '_ABORT_', size = (10,5))],
          [sg.Button('Ready', key = '_READY_', size = (10,5)),
           sg.Button('Arm', key = '_ARM_', size = (10,5)), 
           sg.Button('FIRE', key = '_FIRE_', size = (10, 5))],
          [ sg.Button('Status', key = "_STATUS_", size = (10,5)),
           sg.Button('Toggle log', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]  

layout_log_test = [[sg.Text("State: None", key = '_STATUS_OUT_', size = (20, 5), auto_size_text=True, font=('Arial Bold', 16))],
          [sg.Button('Stop', key = '_STOP_', size = (10, 5)),
           sg.Button('Ready', key = '_READY_', size = (10,5)),
           sg.Button('Arm', key = '_ARM_', size = (10,5))],
          [sg.Button('Start log', key = '_START_LOG_', size = (10, 5)),
           sg.Button('Stop log', key = '_STOP_LOG_', size = (10, 5)),
           sg.Button('Log IDs', key = '_LOG_IDS_', size = (10, 5)),
           sg.Button('Log dump', key = '_LOG_DUMP_', size = (10, 5)),
           sg.Input(key = '_LOG_ID_', size = (5,5), font=('Arial Bold', 16)) ],
          [sg.Button('Status', key = "_STATUS_", size = (10,5)),
           sg.Button('Toggle Status', key = '_LOG_TOGGLE_', size = (10,5)), sg.Exit()]]      

window_name = ""
#layout = layout_rocket
if(len(sys.argv) > 1):
    if(sys.argv[1] == 'r'):
        layout = layout_rocket
        state_map = state_map_rocket
        state_map_to_string = state_map_to_string_rocket
        window_name = "Rocket"
        cmd_id = 1
    elif(sys.argv[1] == 'f'):
        layout = layout_fill
        state_map = state_map_fill
        state_map_to_string = state_map_to_string_fill
        window_name = "Fill Station"
        cmd_id = 2
    elif(sys.argv[1] == 'l'):
        layout = layout_log_test
        state_map = state_map_rocket
        state_map_to_string = state_map_to_string_rocket
        window_name = "Falsh logger"

else:
    layout = layout_rocket
    state_map = state_map_rocket
    state_map_to_string = state_map_to_string_rocket

window = sg.Window(window_name, layout)      


log_status = 0
def toggle_status():
    global log_status
    if log_status == 0: log_status = 1
    elif log_status == 1: log_status = 0

def arm():
    cmd = bytearray([0x55, command_map['ARM'], cmd_id, 1, 1, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 7 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[4]) != 1):
        print("ARM1 error", buff)
        return
    else:
        print("ARM 1 ok")

    cmd = bytearray([0x55, command_map['ARM'], cmd_id, 1, 2, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 7 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[4]) != 2):
        print("ARM2 error", buff)
        return
    else:
        print("ARM 2 ok")
    

    cmd = bytearray([0x55, command_map['ARM'], cmd_id, 1, 3, 2, 3])
    ser.write(cmd)
    read_cmd()
    if(len(buff) != 7 or int(buff[1]) != command_map['ARM_ACK'] or int(buff[4]) != 3):
        print("ARM3 error", buff)
        return
    else:
        print("ARM 3 ok")

def dump(id):
    cmd = bytearray([0x55, command_map['FLASH_DUMP'], cmd_id, 1, id, 2, 3])
    ser.write(cmd)

    #size1 = int.from_bytes(ser.read(1), 'little')
    #size2 = int.from_bytes(ser.read(1), 'little')
    ser._timeout = 5 
    
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

    #5 seconds to retreive dump data
    res = ser.read(size)
    print("Log dump: [",''.join('{:02x} '.format(x) for x in res)[:-1], "]")
    #print(res)
    print("Log size recieved", len(res))
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

    #ax = int.from_bytes(buff[5:7], byteorder='big', signed=True)
    #ay = int.from_bytes(buff[7:9], byteorder='big', signed=True)
    #az = int.from_bytes(buff[9:11], byteorder='big', signed=True)

    #ax = (ax * 0.244) / 1000 * 9.80665
    #ay = (ay * 0.244) / 1000 * 9.80665
    #az = (az * 0.244) / 1000 * 9.80665

    #gx = int.from_bytes(buff[11:13], byteorder='big', signed=True)
    #gy = int.from_bytes(buff[13:15], byteorder='big', signed=True)
    #gz = int.from_bytes(buff[15:17], byteorder='big', signed=True)
    
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
    print("state", state)
    s1 = "State: " + state_map_to_string[state] + "\n"
    
    #sa = "Ax: " + str(round(ax, 2)) + " Ay: " + str(round(ay, 2)) + " Az: " + str(round(az, 2)) + "\n"
    #sg = "Gx: " + str(round(gx, 2)) + " Gy: " + str(round(gy, 2)) + " Gz: " + str(round(gz, 2)) + "\n"

    sp = ""
    #sp = "Pressure: " + str(int.from_bytes(buff[5:7], byteorder='big', signed=False)) + "\n"
    sl = ""
    #sl = "Tank Liquid: " + str(int.from_bytes(buff[7:9], byteorder='big', signed=False)) + "\n"

    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 

    #s = s1 + sa + sg + s2
    #s = s1 + sl + sp + s2

    s = s1 + st1 + st2 + sp1 + sp2 + s2
    window['_STATUS_OUT_'].update(s)

    #print("ax:", ax, "ay:", ay, "az:", az)
    #print("gx:", gx, "gy:", gy, "gz:", gz)

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
    
    p_tank = int.from_bytes(buff[5:7], byteorder='big', signed=True) 
    l_tank = int.from_bytes(buff[7:9], byteorder='big', signed=True) 

    t1 = int.from_bytes(buff[9:11], byteorder='big', signed=True) 
    t2 = int.from_bytes(buff[11:13], byteorder='big', signed=True) 
    t3 = int.from_bytes(buff[13:15], byteorder='big', signed=True) 

    p1 = int.from_bytes(buff[15:17], byteorder='big', signed=True) 
    p2 = int.from_bytes(buff[17:19], byteorder='big', signed=True) 
    p3 = int.from_bytes(buff[19:21], byteorder='big', signed=True) 

    v = int.from_bytes(buff[21:23], byteorder='big', signed=True) 

    print("state", state)
    s1 = "State: " + state_map_to_string[state] + "\n"
    sp_tank = "P1: " + str(p_tank) + '\n'
    sl_tank = "L1: " + str(l_tank) + '\n'

    st1 = "T1: " + str(t1) + '\n'
    st2 = "T2: " + str(t2) + '\n'
    st3 = "T3: " + str(t3) + '\n'

    sp1 = "P1: " + str(p1) + '\n'
    sp2 = "P2: " + str(p2) + '\n'
    sp3 = "P3: " + str(p3) + '\n'

    sv = "Ematch: " + str(v) + '\n'

    s1 = "State: " + state_map_to_string[state] + "\n"

    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 

    #s = s1 + sa + sg + s2
    #s = s1 + sl + sp + s2

    #s = s1 + sp_tank + sl_tank +  st1 + st2 + st3 + sp1 + sp2 + sp3 + s2
    s = s1 + sp_tank + sl_tank + sv + s2 
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
        if event == '_ARM_':
            cmd_launched = 1
            arm()
            #read_cmd()
            pass
        elif event == '_READY_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['READY'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
            pass
        elif event == '_ABORT_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['ABORT'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
            pass
        elif event == '_LED_ON_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_ON'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
        elif event == '_LED_OFF_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_OFF'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
        elif event == '_STATUS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STATUS'], cmd_id, 0, 0x20, 0x21])
            print("Cmd: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
            ser.write(cmd)
            read_cmd()
            #rep = ser.read(6)
            #print(bytearray(rep).hex())
        elif event == '_IMU_calib_':
            print("calibrate imu")
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['IMU_CALIB'], cmd_id, 0, 0x20, 0x21])
            print("Cmd: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
            ser.write(cmd)
            read_cmd()
        elif event == '_FUELING_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FUELING'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_STOP_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STOP_PROG'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_RESUME_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['RESUME_PROG'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_EXEC_':
            cmd_launched = 1
            s = window['_PROG_'].get()
            arr = s.split(',')
            prog = int(arr[0])
            p1 = int(arr[1])
            p2 = int(arr[2])
            p3 = int(arr[3])
            l1 = int(arr[4])
            l2 = int(arr[5])
            cmd = bytearray([0x55, command_map['EXEC_PROG'], cmd_id, 11, prog, 
                             (p1 >> 8) & 0xff, (p1) & 0xff, 
                             (p2 >> 8) & 0xff, (p2) & 0xff, 
                             (p3 >> 8) & 0xff, (p3) & 0xff, 
                             (l1 >> 8) & 0xff, (l1) & 0xff, 
                             (l2 >> 8) & 0xff, (l2) & 0xff, 
                             0x20, 0x21])
            print("Comand sent", cmd)
            ser.write(cmd)
            read_cmd()
        elif event == '_FIRE_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FIRE_PYRO'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_START_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FLASH_LOG_START'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_STOP_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FLASH_LOG_STOP'], cmd_id, 0, 0x20, 0x21])
            ser.write(cmd)
            read_cmd()
        elif event == '_LOG_IDS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FLASH_IDS'], cmd_id, 0, 0x20, 0x21])
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

    while ser.in_waiting > 0:
        ch = ser.read(1)
        ch = int.from_bytes(ch, 'little')
        print(chr(ch), end="")
    #while ser.in_waiting > 0:
        #print(ser.read(1).decode('utf-8'), end='')


    time.sleep(0.001)

window.close()
