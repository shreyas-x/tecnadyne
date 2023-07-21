import serial
import time
from gooey import Gooey, GooeyParser
from utils.serial_port import SerialPort
import struct


# LOOP_INTERVAL = 0.05 # seconds
MIN_TO_SEC = 60
TACH_CNT = 48

DLE = b'\x10' # Data Link Escape
SOH = b'\x01' # Start Of Header
STN = b'\x6F' # Addressing all thrusters
ETX = b'\x03' # End of Text

buffer = bytearray(b'')

# to convert input RPM to tach value
def tachFromRPM(rpm, loopint):
    # from Tecnadyne Thrusters Communication Interface Protocol page 18
    tachInt = int(rpm*loopint*TACH_CNT/MIN_TO_SEC)
    print(tachInt)
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
    print([hex(i) for i in msg])
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
    i = start
    while(i <= stop):
        i += step
        sendMessageToThruster(ser, writeRPM(i, loopint))
        sendMessageToThruster(ser, enableSpeedLoop())
        time.sleep(ts)
    x = stop
    
    keepAlive(ser, wait)

    while(x >= start):
        x -= step
        sendMessageToThruster(ser, writeRPM(x, loopint))
        sendMessageToThruster(ser, enableSpeedLoop())
        time.sleep(ts)
    sendMessageToThruster(ser, disableSpeedLoop())

def askStatus():
    CMD = b'\x37'
    DBX = b'\x00'
    msg = [DLE, SOH, STN, CMD, DBX, DBX, DBX, DBX, DLE, ETX]
    BCC = checksum(msg)
    msg = b''.join(msg) + BCC 
    return msg

def keepAlive(ser, tim):
    print("waiting")
    startTime = time.time()
    while(time.time() - startTime < tim):
        sendMessageToThruster(ser, askStatus())
        # print("Received:", [hex(i) for i in received_message])
        time.sleep(0.2) 
    print("done waiting")


# Function to receive data
def receive_data(ser):
    # received_data = ser.readline().decode().strip()
    received_data = ser.read()
    print ("rec", receive_data)
    return received_data

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
    
    for i in range (14, 16):
        print ("DB"+str(i+1), hex(new_res[i]))
    

@Gooey(program_name="Tecnadyne thruster control utility")
def main():
    parser = GooeyParser()
    parser.add_argument("--com", type=str, required=True, default="COM5", metavar="COM Port")
    parser.add_argument("--loopint", type=float, required=True, default=0.05, metavar="Loop Interval (s)")
    parser.add_argument("--rampstart", type=int, required=True, default=300, metavar="Ramp Start (RPM)")
    parser.add_argument("--rampstop", type=int, required=True, default=500, metavar="Ramp Stop (RPM)")
    parser.add_argument("--rampstep", type=int, required=True, default=20, metavar="Ramp Step (RPM)")
    parser.add_argument("--ramptimestep", type=float, required=True, default=0.1, metavar="Ramp Time Step (s)")
    parser.add_argument("--rampwaittime", type=float, required=True, default=3, metavar="Ramp Wait Time (s)")
    args = parser.parse_args()
    ser = SerialPort(args.com, 57600)
    ser.register_receive_callback(datacb)
    ser.open(args.com, 57600)
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