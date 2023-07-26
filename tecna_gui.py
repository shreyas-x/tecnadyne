import serial
import time
from gooey import Gooey, GooeyParser
from utils.serial_port import SerialPort
import struct
import signal

# LOOP_INTERVAL = 0.05 # seconds
MIN_TO_SEC = 60
TACH_CNT = 48
cur_res = 0.02475

DLE = b'\x10' # Data Link Escape
SOH = b'\x01' # Start Of Header
STN = b'\x6F' # Addressing all thrusters
ETX = b'\x03' # End of Text

buffer = bytearray(b'')

# to convert input RPM to tach value
def tachFromRPM(rpm, loopint):
    # from Tecnadyne Thrusters Communication Interface Protocol page 18
    tachInt = int(rpm*loopint*TACH_CNT/MIN_TO_SEC)
    # print(tachInt)
    # return int value in bytes
    return tachInt.to_bytes(1, "big")

def btoi(bin):
    return int.from_bytes(bin, byteorder="big")

def checksum(data):
    return (sum(list(map(lambda x: int.from_bytes(x, byteorder="big"), data))) & 0xFF).to_bytes(1, "big")

def writeRPM(rpm, loopint):
    tach = tachFromRPM(rpm, loopint)
    CMD = b'\x41'
    DB1 = b'\x21' # closed loop speed control
    DB2 = b'\x0A' # loop time interval count = 10 = 50ms
    DB3 = b'\x00' # direction normal = 0, reverse = 1
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
    print("stopping")
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    return msg

def sendMessageToThruster(ser, msg):
    ser.write(bytearray(msg))

def rampThruster(ser, loopint, start, stop, step, ts, wait):
    # Ramping up
    i = start
    try:
        while(i <= stop):
            sendMessageToThruster(ser, writeRPM(i, loopint))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            i += step
    except KeyboardInterrupt:
        j = i
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())

    x = stop
    tach_end = int((i - step) * loopint * TACH_CNT / MIN_TO_SEC)
    print ("Tach End: ", tach_end)
    # print ("i: ", i)

    # Staying
    try:
        keepAlive(ser, wait)
    except KeyboardInterrupt:
        j = x
        while(j >= start):
            sendMessageToThruster(ser, writeRPM(j, loopint))
            sendMessageToThruster(ser, enableSpeedLoop())
            time.sleep(ts)
            j -= step
        sendMessageToThruster(ser, disableSpeedLoop())

    # Ramping down
    while(x >= start):
        x -= step
        sendMessageToThruster(ser, writeRPM(x, loopint))
        sendMessageToThruster(ser, enableSpeedLoop())
        time.sleep(ts)

    sendMessageToThruster(ser, disableSpeedLoop())
    tach_stop = int((x + step) * loopint * TACH_CNT / MIN_TO_SEC)
    print ("Tach Stop: ", tach_stop)
    # print ("x: ", x)

def askStatus():
    CMD = b'\x37'
    DBX = b'\x00'
    msg = [DLE, SOH, STN, CMD, DBX, DBX, DBX, DBX, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    return msg

def keepAlive(ser, tim):
    # print("waiting")
    startTime = time.time()
    while(time.time() - startTime < tim):
        sendMessageToThruster(ser, askStatus())
        # print("Received:", [hex(i) for i in received_message])
        time.sleep(0.2) 
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
        curr = current(new_res[14:16])    
        print("Current: ", curr)

def current(raw):
    cur_raw = int.from_bytes(raw, byteorder='big')
    cur = cur_raw * cur_res
    return cur

@Gooey(program_name="Tecnadyne thruster control utility",
       shutdown_signal = signal.CTRL_C_EVENT)
def main():
    parser = GooeyParser()
    parser.add_argument("--com", type=str, required=True, default="COM5", metavar="COM Port")
    parser.add_argument("--buadrate", type=int, required=True, default=57600, metavar="Buadrate")
    parser.add_argument("--loopint", type=float, required=True, default=0.05, metavar="Loop Interval (s)")
    parser.add_argument("--rampstep", type=int, required=True, default=20, metavar="Ramp Step (RPM)")
    parser.add_argument("--rampstart", type=int, required=True, default=400, metavar="Ramp Start (RPM)")
    parser.add_argument("--rampstop", type=int, required=True, default=600, metavar="Ramp Stop (RPM)")
    parser.add_argument("--ramptimestep", type=float, required=True, default=0.1, metavar="Ramp Time Step (s)")
    parser.add_argument("--rampwaittime", type=int, required=True, default=3, metavar="Steady State Run Time (s)")
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
    
    rampThruster(ser, loopint, rampstart, rampstop, rampstep, rt, wait)


    
if __name__ == "__main__":
    main()