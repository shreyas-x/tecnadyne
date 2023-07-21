import serial
import time
from gooey import Gooey, GooeyParser

# LOOP_INTERVAL = 0.05 # seconds
MIN_TO_SEC = 60
TACH_CNT = 48

DLE = b'\x10' # Data Link Escape
SOH = b'\x01' # Start Of Header
STN = b'\x6F' # Addressing all thrusters
ETX = b'\x03' # End of Text

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

@Gooey(program_name="Tecnadyne thruster control utility")
def main():
    parser = GooeyParser()
    parser.add_argument("--com", type=str, required=True, default="COM4", metavar="COM Port")
    parser.add_argument("--loopint", type=float, required=True, default=0.05, metavar="Loop Interval (s)")
    parser.add_argument("--rpm", type=int, required=True, metavar="RPM")
    parser.add_argument("--runtime", type=int, required=True, default=5, metavar="Runtime (s)")
    args = parser.parse_args()

    ser = serial.Serial(str(args.com), 9600)

    loopint = float(args.loopint)
    rpm = int(args.rpm)
    rt = int(args.runtime)

    sendMessageToThruster(ser, writeRPM(rpm, loopint))
    sendMessageToThruster(ser, enableSpeedLoop())
    time.sleep(rt)
    sendMessageToThruster(ser, disableSpeedLoop())


    
if __name__ == "__main__":
    main()