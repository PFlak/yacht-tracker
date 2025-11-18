import serial
import time
from serial_handler import Serial_device_handler as SDH

DEVICE = '/dev/ttyS0'
BAUDRATE = 115200
SERIAL_TIMEOUT = 2
LOOP_TIME = 30

# Todo: get device id

def the_loop():
    
    time.sleep(30)

def main():
    sdh = SDH(DEVICE, BAUDRATE, SERIAL_TIMEOUT)

    print(sdh.get_gps_data())
    #Todo: obsłużyć żyroskop 
    
    #Todo: obsłużyć wyświetlacz

    #Todo: obsłużyć sysyłanie

if __name__ == "__main__":
    main()