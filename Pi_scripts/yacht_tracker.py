import serial
import time

DEVICE = '/dev/ttyS0'
BAUDRATE = 115200
SERIAL_TIMEOUT = 2

def init_serial():
    ser = serial.Serial(DEVICE , baudrate = BAUDRATE, timeout = SERIAL_TIMEOUT) # this configuratyion is correct
    ser.flushInput()
    pass

def close_serial():
    pass

def main():
    #Todo
    init_serial()
    #To ma zastać na samym końcu
    close_serial()
    pass

if __name__ == "__main__":
    main()