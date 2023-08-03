import serial
import time
from gooey import Gooey, GooeyParser
from utils.serial_port import SerialPort
import struct
import signal
import math
import logging

# LOOP_INTERVAL = 0.05 # seconds
MIN_TO_SEC = 60
TACH_CNT = 48
res = 0.02475

DLE = b'\x10' # Data Link Escape
SOH = b'\x01' # Start Of Header
STN = b'\x6F' # Addressing all thrusters
ETX = b'\x03' # End of Text

buffer = bytearray(b'')

# to convert input RPM to tach value
def tachFromRPM(rpm, loopint):
    # from Tecnadyne Thrusters Communication Interface Protocol page 18
    tachInt = int(rpm*loopint*TACH_CNT/MIN_TO_SEC)
    return tachInt.to_bytes(1, "big")

def btoi(bin):
    return int.from_bytes(bin, byteorder="big")

def checksum(data):
    return (sum(list(map(lambda x: int.from_bytes(x, byteorder="big"), data))) & 0xFF).to_bytes(1, "big")

def writeRPM(rpm, loopint, direct):
    tach = tachFromRPM(rpm, loopint)
    CMD = b'\x41'
    DB1 = b'\x21' # closed loop speed control
    DB2 = b'\x0A' # loop time interval count = 10 = 50ms
    DB3 = direct.to_bytes(1, "big")
    # DB3 = b'\x01' # direction normal = 0, reverse = 1
    DB4 = tach  # tach set point
    msg = [DLE, SOH, STN, CMD, DB1, DB2, DB3, DB4, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC
    # print([hex(i) for i in msg])
    return msg

def enableSpeedLoop():
    CMD = b'\x41'
    DB1 = b'\x01'
    DBX = b'\x00'
    msg = [DLE, SOH, STN, CMD, DB1, DBX, DBX, DBX, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    return msg

def disableSpeedLoop():
    CMD = b'\x41'
    DBX = b'\x00'
    msg = [DLE, SOH, STN, CMD, DBX, DBX, DBX, DBX, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    print("stopping")
    logging.info("stopping")
    return msg

def sendMessageToThruster(ser, msg):
    ser.write(bytearray(msg))

def rampThruster(ser, loopint, start, stop, step, ts, wait, direct):
    # Ramping up
    i = start
    try:
        while(i <= stop):
            sendMessageToThruster(ser, writeRPM(i, loopint, direct))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            i += step
    except KeyboardInterrupt:
        print ("KeyboardInterrupt")
        logging.info("KeyboardInterrupt")
        j = i
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint, direct))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())
    except ValueError:
        print ("ValueError")
        logging.info("ValueError")
        j = i
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint, direct))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())

    x = stop
    tach_end = int((i - step) * loopint * TACH_CNT / MIN_TO_SEC)
    print ("Tach End: ", tach_end)
    logging.info("Tach End: ", tach_end)

    # Staying
    try:
        keepAlive(ser, wait)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        logging.info("KeyboardInterrupt")
        j = x
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint, direct))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())
    except ValueError:
        j = x
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint, direct))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())

    # Ramping down
    while(x >= start):
        x -= step
        sendMessageToThruster(ser, writeRPM(x, loopint, direct))
        sendMessageToThruster(ser, enableSpeedLoop())
        time.sleep(ts)
    sendMessageToThruster(ser, disableSpeedLoop())
    tach_stop = int((x + step) * loopint * TACH_CNT / MIN_TO_SEC)
    print("Tach Stop: ", tach_stop)
    logging.info("Tach Stop: ", tach_stop)

def askStatus():
    CMD = b'\x32'
    DBX = b'\x00'
    msg = [DLE, SOH, STN, CMD, DBX, DBX, DBX, DBX, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    return msg

def keepAlive(ser, tim):
    # print("waiting")
    startTime = time.time()
    t2 = 0
    while(time.time() - startTime < tim):
        sendMessageToThruster(ser, askStatus())
        # print("Received:", [hex(i) for i in received_message])
        time.sleep(0.2)
        t = int(time.time() - startTime)
        if t > t2:
            print("Run Time(s): ", t)
            logging.info("Run Time(s): ", t)
            t2 = t
    # print("done waiting")

def datacb(arr):
    global buffer
    buffer += arr
    startidx = -1
    endidx = -1 
    try:
        startidx = buffer.index(b'\x10\x02')
    except:
        return
    try:
        endidx = buffer.index(b'\x10\x03')
    except:
        return

    if (startidx == endidx):
        return
    result = buffer[startidx:endidx+3]
    buffer = buffer[endidx+3:]
    new_res = result[4:-3]
    lng = len(new_res)
    # print ("new_res_len:", lng)
    if lng == 16:
        # for i in range (14, 16):
        #     print ("DB"+str(i+1), hex(new_res[i]))
        curr = current(new_res[2:4])    
        print("Current: ", curr)
        logging.info("Current: ", curr)
        volt = voltage(new_res[0:2])    
        print("Voltage: ", volt)
        logging.info("Voltage: ", volt)
        e_temp = e_temperature(new_res[6:8])    
        print("Electronics Temperature: ", e_temp)
        logging.info("Electronics Temperature: ", e_temp)
        m_temp = m_temperature(new_res[10:12])
        print("Motor Temperature: ", m_temp)
        logging.info("Motor Temperature: ", m_temp)

def current(raw):
    cur_raw = int.from_bytes(raw, byteorder='big')
    cur = cur_raw * res
    return cur

def voltage(raw):
    v_raw = int.from_bytes(raw, byteorder='big')
    v = v_raw * res
    return v

def e_temperature(raw):
    t_raw = int.from_bytes(raw, byteorder='big')
    t = ((t_raw * 4.88) - 480) / 15.6
    return t

def m_temperature(raw):
    m_raw = int.from_bytes(raw, byteorder='big')
    m = ((m_raw * 4.88 * 10) / (5 - (m_raw * 4.88 / 1000)))
    A1 = 0.003354016
    B1 = 0.000256985
    C1 = 2.62013E-06
    D1 = 6.38309E-08
    R_cal = m / 10000
    m_t = calculate_temperature(R_cal, A1, B1, C1, D1)
    return m_t

def constraint(value, min, max):
    if value <= min:
        value = min
    if value >= max:
        value = max
    return value

def calculate_temperature(R, A, B, C, D):
    try:
        ln_R = math.log(R)
        T_inv = A + B * ln_R + C * (ln_R ** 2) + D * (ln_R ** 3)
        temperature_kelvin = 1 / T_inv
        temperature_celsius = temperature_kelvin - 273.15
        return temperature_celsius
    except ValueError:
        # Handle invalid inputs or any other potential errors
        return None

@Gooey(program_name="Tecnadyne thruster control utility",
       shutdown_signal = signal.CTRL_C_EVENT,
       show_stop_warning = False)
def main():
    parser = GooeyParser()
    parser.add_argument("--com", type=str, required=True, default="COM5", metavar="COM Port")
    parser.add_argument("--buadrate", type=int, required=True, default=57600, metavar="Buadrate (default)")
    parser.add_argument("--loopint", type=float, required=True, default=0.05, metavar="Loop Interval (default) (s)")
    parser.add_argument("--dir", type=int, required=True, default=0, metavar="Direction (0 or 1)")
    parser.add_argument("--rampstep", type=int, required=True, default=10, metavar="Ramp Step (default) (RPM)")
    parser.add_argument("--rampstart", type=int, required=True, default=400, metavar="Ramp Start (RPM)")
    parser.add_argument("--rampstop", type=int, required=True, default=900, metavar="Ramp Stop (RPM)")
    parser.add_argument("--ramptimestep", type=float, required=True, default=0.03, metavar="Ramp Time Step (default) (s)")
    parser.add_argument("--rampwaittime", type=int, required=True, default=3, metavar="Steady State Run Time (s)")
    parser.add_argument("--log", action='store_true', widget='CheckBox', default=False, metavar="Save Logs?")
    args = parser.parse_args()

    buadrate = int(args.buadrate)
    ser = SerialPort(args.com, buadrate)
    ser.register_receive_callback(datacb)
    ser.open(args.com, buadrate)
    # ser = serial.Serial(str(args.com), 57600)

    loopint = float(args.loopint)
    rampstart = int(args.rampstart)
    rampstop = int(args.rampstop)
    rampstep = int(args.rampstep)
    rt = float(args.ramptimestep)
    wait = float(args.rampwaittime)
    direct = int(args.dir)

    saveLogs = args.log
    if(saveLogs):
        logging.basicConfig(filename="tecnalog_"+str(math.floor(time.time()))+".log", encoding='utf-8', level=logging.INFO)


    loopint = constraint(loopint, 0.05, 0.1)
    rampstart = constraint(rampstart, 300, 400)
    rampstop = constraint(rampstop, 600, 2100)
    rampstep = constraint(rampstep, 10, 50)
    rt = constraint(rt, 0.01, 10)
    wait = constraint(wait, 0, 65000)
    direct = constraint(direct, 0, 1)

    rampThruster(ser, loopint, rampstart, rampstop, rampstep, rt, wait, direct)
    
if __name__ == "__main__":
    main()