import PySimpleGUI as sg
import time
import serial
import sys
import queue
import threading


ser_timeout = 0.001
if(len(sys.argv) > 1):
    ser = serial.Serial(sys.argv[1], 115200, timeout=ser_timeout) #set read timeout of 1s
else:
    ser = serial.Serial('COM3', 115200, timeout=ser_timeout) #set read timeout of 1s

#log_delay = 0.05
#log_delay = 0.250
log_delay = 1
message_timeout = 0.25
#message_timeout = 1

missed_packets = 0
log_speed = 0

cmd_id = 0

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

valve_map = {
    "VPU_valve" : 0,
    "Engine_valve" : 1,
    "He_valve" : 2,
    "N2O_valve" : 3,
    "Line_valve" : 4,
}

filling_program_map = {
    "SAFETY_PRESSURE_PROG" : 0,
    "PURGE_PRESSURE_PROG" : 1,
    "PURGE_LIQUID_PROG" : 2,
    "FILL_He_PROG" : 3,
    "FILL_N2O_PROG" : 4,
    "PURGE_LINE_PROG" : 5,
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
begin = time.perf_counter()
end = time.perf_counter()
buff = bytearray()

ground_queue = queue.Queue()
log_queue = queue.Queue()
cmd_queue = queue.Queue()

sg.theme('DarkAmber')    # Keep things interesting for your users


auto_command_layout = [
    [sg.Button('Ready', key = '_READY_', size = (10,5)),
     sg.Button('Arm', key = '_ARM_', size = (10,5)),
     sg.Button('Fire', key = '_FIRE_', size = (10,5)),
     sg.Button('Alow Launch', key = '_LAUNCH_', size = (10,5)),
     sg.Push(),
     sg.OptionMenu(['Rocket', 'Fill Station', 'Broadcast'], default_value='Rocket', key='_ID_MENU_', size = (10,5))],
    [sg.Button('Start Fueling', key = '_FUELING_', size = (10, 5)),
     sg.Button('Exec prog', key = '_EXEC_', size = (10, 5)),
     sg.OptionMenu(["SAFETY_PRESSURE_PROG", "PURGE_PRESSURE_PROG", "PURGE_LIQUID_PROG", "FILL_He_PROG", "FILL_N2O_PROG", "PURGE_LINE_PROG"], default_value='SAFETY_PRESSURE_PROG', key='_PROGRAM_ID_MENU_', size = (10,5)),
     sg.Input(key = '_PROG_', size = (15,5), font=('Arial Bold', 16))],
    [sg.Button('Manual Mode', key = '_MANUAL_', size = (10, 5)),
     sg.Button('Stop', key = '_STOP_', size = (10, 5)),
     sg.Button('Resume', key = '_RESUME_', size = (10, 5)),
     sg.Button('Abort', key = '_ABORT_', size = (10,5)),
     sg.Exit(),
     sg.Push(),
     sg.Button('Status', key = "_STATUS_", size = (10,5)),
     sg.Button('Toggle Status', key = '_LOG_TOGGLE_', size = (10,5))],
]

manual_command_layout = [
    [sg.OptionMenu(['Rocket', 'Fill Station'], default_value='Rocket', key='_ID_MENU_2_', size = (10,5))],
    [sg.Button('Exec Valve', key = '_EXEC_VALVE_', size = (10, 5)),
     sg.OptionMenu(["VPU_valve", "Engine_valve", "He_valve", "N2O_valve", "Line_valve"], default_value='VPU_valve', key='_Valve_ID_MENU_', size = (10,5)),
     sg.Input(key = '_VALVE_', size = (3,5), font=('Arial Bold', 16)),
     sg.Push(),
     sg.Button('Exec Valve ms', key = '_EXEC_VALVE_MS_', size = (12, 5)),
     sg.OptionMenu(["VPU_valve", "Engine_valve", "He_valve", "N2O_valve", "Line_valve"], default_value='VPU_valve', key='_Valve_ID_MENU_MS_', size = (10,5)),
     sg.Input(key = '_MS_', size = (5,5), font=('Arial Bold', 16))],
    [sg.Button('Start log', key = '_START_LOG_', size = (10, 5)),
     sg.Button('Stop log', key = '_STOP_LOG_', size = (10, 5)),
     sg.Button('Log IDs', key = '_LOG_IDS_', size = (10, 5)) ],
]

layout = [
    [sg.Text("Rocket", key = '_STATUS_ROCKET_', size = (60, 15), auto_size_text=True, font=('Arial Bold', 16)), 
     sg.Push(), 
     sg.Text("Fill Station", key = '_STATUS_FILL_', size = (60, 15), auto_size_text=True, font=('Arial Bold', 16))],
    [sg.TabGroup([[sg.Tab("Auto", auto_command_layout), sg.Tab("Manual", manual_command_layout)]])]
]

window = sg.Window("PST Ground Station", layout, finalize=True)      
#window = sg.Window("PST Ground Station", layout)      

def route_message(buff):
    if(buff[2] == 0x0):
        cmd_queue.put(buff)
    else:
        log_queue.put(buff)

buff = bytearray()
data_recv = 0
begin = time.perf_counter()
#reads all messages from the ground
#both log and acks
#we make sure that the ground esp sends complete messages at a time,
#no need to wory about receiving parcial messages
def read_message():
    global comm_state, sync_state, cmd_state, size_state, data_state, crc_1_state, crc_2_state, end_state
    global buff
    global data_recv, data_total
    global begin, end
    global missed_packets

    begin = time.perf_counter()
    
    while ser.in_waiting > 0 and comm_state != end_state:
        ch = ser.read(1)
        ch = int.from_bytes(ch, 'little')
        #print(chr(ch), comm_state)

        if comm_state == sync_state:
            if ch == 0x55:
                begin = time.perf_counter()
                comm_state = cmd_state
                buff = bytearray()
                buff.append(ch)
                data_recv = 0
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
        if(buff[2] == 0):
            print("ACK recieved:", len(buff))
            print("Time", msec)
            print("Cmd_ACK: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
        
        route_message(buff)
        
        #else:
        time.sleep(0.005)
        return
    #message lost
    #this is only needed for the case where there are problems with the usb connection
    #in the ground esp we make sure to only send complete messages to the pc
    elif comm_state != sync_state and msec > message_timeout:
        missed_packets += 1
        print("command timeout", msec, len(buff), data_recv, data_total)
        print("cmd state: ", comm_state)
        #print("buffer recieved:", buff) 
        print("Cmd fail: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
        comm_state = sync_state
        #time.sleep(0.015)
        return

    elif msec > message_timeout:
        print("command timeout", msec)
        print("no cmd state: ", comm_state)
        print("Cmd fail: [",''.join('{:02x} '.format(x) for x in buff)[:-1], "]")
        missed_packets += 1
        comm_state = sync_state
        #time.sleep(0.015)
        return

RUN_FLAG = True
def serial_reader():
    global RUN_FLAG
    while RUN_FLAG:
        #if ground_queue.qsize():  print("ground queu size", ground_queue.qsize())
        if(ground_queue.qsize() > 0):
           cmd = ground_queue.get()
           #print("ground queue:", cmd)

           print("ground queue: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
           #print("ground queue: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
           ser.write(cmd) 

        read_message()

        #time.sleep(0.1)

last_rocket_log = 0
last_fill_log = 0

def print_status_rocket(buff):
    global missed_packets
    global log_speed
    global last_rocket_log 
    #if(len(buff) < 16):
        #print("bad status")
        #return

    state = int.from_bytes(buff[4:5], byteorder='big', signed=True)
    if state < 0 or state >= len(state_map_to_string_rocket): 
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
    
    t3 = int.from_bytes(buff[9:11], byteorder='big', signed=True) 
    t4 = int.from_bytes(buff[11:13], byteorder='big', signed=True) 
    t5 = int.from_bytes(buff[13:15], byteorder='big', signed=True) 
    
    p1 = int.from_bytes(buff[15:17], byteorder='big', signed=True) 
    p2 = int.from_bytes(buff[17:19], byteorder='big', signed=True) 

    #p1 = p1 * 0.0001875
    #p2 = p2 * 0.0001875

    tank_p = int.from_bytes(buff[19:21], byteorder='big', signed=True)
    tank_l = int.from_bytes(buff[21:23], byteorder='big', signed=True) 

    tt1 = int.from_bytes(buff[23:24], byteorder='big', signed=False) 

    w1 = int.from_bytes(buff[24:26], byteorder='big', signed=False) 
    w2 = int.from_bytes(buff[26:28], byteorder='big', signed=False) 
    w3 = int.from_bytes(buff[28:30], byteorder='big', signed=False) 


    #debug
    #ttt1 = int.from_bytes(buff[30:32], byteorder='big', signed=True) 
    #ttt2 = int.from_bytes(buff[32:34], byteorder='big', signed=True) 
    #ttt3 = int.from_bytes(buff[34:36], byteorder='big', signed=True) 

    #print("tactale readings", ttt1, ttt2, ttt3)

    #print("state", state)
    s1 = "State: " + state_map_to_string_rocket[state] + "\n"
    st1 = "TT1: " + str(t1) + '\n'
    st2 = "TT2: " + str(t2) + '\n'
    st3 = "CT3: " + str(t3) + '\n'
    st4 = "CT4: " + str(t4) + '\n'
    st5 = "CT5: " + str(t5) + '\n'
    sp1 = "P1: " + str(round(p1, 4)) + '\n'
    sp2 = "P2: " + str(round(p2, 4)) + '\n'

    s_tank_p = "Pressure: " + str(tank_p) + '\n'
    s_tank_l = "Liquid: " + str(tank_l) + '\n'

    stt1 = "tactile: " + str(bin(tt1)) + '\n'

    sw = "Weights: " + str(w1) + ' , ' + str(w2) + ' , ' + str(w3) + '\n'

    #print("state", state)
    s1 = "State: " + state_map_to_string_rocket[state] + "\n"
    
    #sa = "Ax: " + str(round(ax, 2)) + " Ay: " + str(round(ay, 2)) + " Az: " + str(round(az, 2)) + "\n"
    #sg = "Gx: " + str(round(gx, 2)) + " Gy: " + str(round(gy, 2)) + " Gz: " + str(round(gz, 2)) + "\n"

    sp = ""
    #sp = "Pressure: " + str(int.from_bytes(buff[5:7], byteorder='big', signed=False)) + "\n"
    sl = ""
    #sl = "Tank Liquid: " + str(int.from_bytes(buff[7:9], byteorder='big', signed=False)) + "\n"

    log_speed = 1 / (time.perf_counter() - last_rocket_log)
    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 

    #s = s1 + sa + sg + s2
    #s = s1 + sl + sp + s2

    s = "ROCKET\n" + s1 + st1 + st2 + st3 + st4 + st5 + sp1 + sp2 + s_tank_p + s_tank_l + stt1 + sw + s2
    window['_STATUS_ROCKET_'].update(s)

    last_rocket_log = time.perf_counter()
    #print("ax:", ax, "ay:", ay, "az:", az)
    #print("gx:", gx, "gy:", gy, "gz:", gz)

def print_status_fill(buff):
    global missed_packets
    global log_speed
    global last_fill_log

    #if(len(buff) < 16):
        #print("bad status")
        #return

    state = int.from_bytes(buff[4:5], byteorder='big', signed=True)
    if state < 0 or state >= len(state_map_to_string_fill): 
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

    ematch = int.from_bytes(buff[21:23], byteorder='big', signed=True) 

    w1 = int.from_bytes(buff[23:25], byteorder='big', signed=False) 

    tt1 = int.from_bytes(buff[25:26], byteorder='big', signed=False) 
    stt1 = "tactile: " + str(bin(tt1)) + '\n'

    #print("state", state)
    s1 = "State: " + state_map_to_string_fill[state] + "\n"
    sp_tank = "P1: " + str(p_tank) + '\n'
    sl_tank = "L1: " + str(l_tank) + '\n'

    st1 = "T1: " + str(t1) + '\n'
    st2 = "T2: " + str(t2) + '\n'
    st3 = "T3: " + str(t3) + '\n'

    sp1 = "P1: " + str(round(p1, 4)) + '\n'
    sp2 = "P2: " + str(p2) + '\n'
    sp3 = "P3: " + str(p3) + '\n'

    sematch = "ematch: " + str(ematch) + '\n'

    sw1 = "W: " + str(w1) + '\n'

    #print("state", state)
    s1 = "State: " + state_map_to_string_fill[state] + "\n"

    log_speed = 1 / (time.perf_counter() - last_fill_log)
    s2 = "Log Speed: " + str(round(log_speed, 0)) + "hz\nMissed packets: " + str(missed_packets) + "\n" 

    #s = s1 + sa + sg + s2
    #s = s1 + sl + sp + s2

    s = "FILL STATION\n" + s1 + sp_tank + sl_tank +  st1 + st2 + st3 + sp1 + sp2 + sp3 + sematch + sw1 + stt1 + s2
    window['_STATUS_FILL_'].update(s)

    last_fill_log = time.perf_counter()

log_begin = time.perf_counter()
log_end = time.perf_counter()

log_status = 0
def toggle_status():
    global log_status
    if log_status == 0: log_status = 1
    elif log_status == 1: log_status = 0
    print("status log", log_status)

last_command = time.perf_counter() 
ack_recieved = True
def send_command(event):
    global log_end, log_begin, cmd_id, window_values, ack_recieved, last_command
    cmd_launched = 0


    if event == '_LOG_TOGGLE_':
        toggle_status()    
    elif event != '__TIMEOUT__':
        ##make sure we recieved the ack for the last message sent 
        ##or the last message sent ack as reaced timeout
        #if not ack_recieved and ( time.perf_counter() - last_command < message_timeout ) :
            #return

        id = str(window_values['_ID_MENU_'])
        if id == 'Rocket': 
            cmd_id = 1
        elif id == 'Fill Station':
            cmd_id = 2
        else: 
            cmd_id = 0xFF

        #print("Send command id", cmd_id, id)

        if event == '_ARM_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['ARM'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
            #arm()
            #read_cmd()
            pass
        elif event == '_READY_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['READY'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
            pass
        elif event == '_ABORT_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['ABORT'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
            pass
        elif event == '_LED_ON_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_ON'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
        elif event == '_LED_OFF_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['LED_OFF'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
            #rep = ser.read(5)
            #print(rep.decode('utf-8'))
            #print(bytearray(rep).hex())
        elif event == '_STATUS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STATUS'], cmd_id, 0, 0x20, 0x21])
            print("Cmd: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
            ground_queue.put(cmd)
            #rep = ser.read(6)
            #print(bytearray(rep).hex())
        elif event == '_IMU_calib_':
            print("calibrate imu")
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['IMU_CALIB'], cmd_id, 0, 0x20, 0x21])
            print("Cmd: [",''.join('{:02x} '.format(x) for x in cmd)[:-1], "]")
            ground_queue.put(cmd)
        elif event == '_FUELING_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FUELING'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_STOP_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['STOP_PROG'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_RESUME_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['RESUME_PROG'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_FIRE_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['FIRE_PYRO'], 2, 0, 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_LAUNCH_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['ALLOW_LAUNCH'], 1, 0, 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_EXEC_':
            cmd_launched = 1
            s = window['_PROG_'].get()
            arr = s.split(',')
            prog = filling_program_map[str(window_values['_PROGRAM_ID_MENU_'])]
            p1 = int(arr[0])
            p2 = int(arr[1])
            l1 = int(arr[2])
            cmd = bytearray([0x55, command_map['EXEC_PROG'], cmd_id, 7, prog, 
                            (p1 >> 8) & 0xff, (p1) & 0xff, 
                            (p2 >> 8) & 0xff, (p2) & 0xff, 
                            (l1 >> 8) & 0xff, (l1) & 0xff, 
                            0x20, 0x21])
            print("Comand sent", cmd)
            ground_queue.put(cmd)
        elif event == '_MANUAL_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL'], cmd_id, 0, 0x20, 0x21])
            ground_queue.put(cmd)

        elif event == '_EXEC_VALVE_':
            cmd_launched = 1
            s = window['_VALVE_'].get()
            valve = valve_map[str(window_values['_Valve_ID_MENU_'])]
            state = int(s)
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 3,
                             manual_command_map['MANUAL_VALVE_STATE'], valve, state, 0x20, 0x21])
            
            ground_queue.put(cmd)
        
        elif event == '_EXEC_VALVE_MS_':
            cmd_launched = 1
            s = window['_MS_'].get()
            valve = valve_map[str(window_values['_Valve_ID_MENU_MS_'])]
            ms = int(s)
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 3,
                             manual_command_map['MANUAL_VALVE_MS'], valve, ms, 0x20, 0x21])
            
            ground_queue.put(cmd)

        elif event == '_START_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_LOG_START'], 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_STOP_LOG_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_LOG_STOP'], 0x20, 0x21])
            ground_queue.put(cmd)
        elif event == '_LOG_IDS_':
            cmd_launched = 1
            cmd = bytearray([0x55, command_map['MANUAL_EXEC'], cmd_id, 1, manual_command_map['MANUAL_FLASH_IDS'], 0x20, 0x21])
            ground_queue.put(cmd)

    log_end = time.perf_counter()
    if(cmd_launched == 0 and log_status == 1 and log_end - log_begin > log_delay):
        #send status cmd
        log_speed = 1 / (log_end - log_begin)
        log_begin = time.perf_counter()
        cmd = bytearray([0x55, command_map['STATUS'], cmd_id, 0, 0x20, 0x21])
        cmd_launched = 1
        ground_queue.put(cmd)

    if cmd_launched:
        ack_recieved = False
        last_command = time.perf_counter() 


def GUI():
    global window_values, ack_recieved
    while True:           
        event, window_values = window.read(timeout=1) 
        #window_action, event, window_values = sg.read_all_windows(timeout = 5) 
        if event != '__TIMEOUT__': print(event, window_values)       

        if event == sg.WIN_CLOSED or event == 'Exit':
       # if event == 'Exit':
            break
        else:
            #print("command!", window_action, event, window_values)
            send_command(event)
        
        if(log_queue.qsize() > 0):
            cmd = log_queue.get()
            id = int.from_bytes(cmd[2:3], byteorder='big', signed=False)
            if id == 0x02: 
                print_status_rocket(cmd)
            elif id == 0x01:
                print_status_fill(cmd)

        if(cmd_queue.qsize() > 0):
            ack_recieved = True
            ack = cmd_queue.get()
            cmd = int.from_bytes(ack[1:2], byteorder='big', signed=False)
            #print("cmd queue", ack, cmd)
            if cmd == command_map['STATUS_ACK']:
                id = window_values['_ID_MENU_']
                print("status ack", id)
                if id == 'Rocket': 
                    print_status_rocket(ack)
                elif id == 'Fill Station':
                    print_status_fill(ack)

        time.sleep(0.001)

def main():
    global RUN_FLAG
    X = threading.Thread(target=serial_reader)
    X.start()
    GUI()
    RUN_FLAG = False
    X.join()

main()
window.close()

