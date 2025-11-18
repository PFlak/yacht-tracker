import serial
import time

class Serial_device_handler():
    '''
    Serial device handler for handlingcommunication with GNSS and 4G module
    '''

    def __init__(self, device, baudrate, timeout):
        # self.device = device
        # self.baudrate = baudrate
        # self.tomeout = timeout

        ## initialize connection
        self.ser = serial.Serial(device, baudrate= baudrate, timeout= timeout)
        self.ser.flushInput()
        self.serial_connection_test()

        self.set_gps_on()

    def __del__(self):
        self.ser.close()

    def serial_connection_test(self):
        print (self.send_at('at', False, False).upper())
        # Todo, zrobić tak, żeby dobrze działało
        # if self.send_at('at', False, False).upper()[-2:-1] != 'OK':
        #     print("!!! Serial connection not working !!!")

    def send_at(self, command: str, display_mode = True, debbug_mode = False) -> str:
        ''' Sent at command, wait 1 second , read and return response'''
        self.ser.write((command + '\r\n').encode())
        time.sleep(1)
        response = self.ser.read_all().decode(errors='ignore')
        if display_mode:
            print(f"Command: {command}\nResponse: {response}\n")
        if debbug_mode: # a to nie wiem po co dałem
            pass

        return response
    
    def set_gps_on(self):
        self.send_at('AT+CGNSPWR=1')  # Power on GPS

    def get_gps_data(self) -> dict:
        '''
        Get data from AT+CGNSINF command and return dict with time, latitude, longitude, C/N0 and precision data
        '''
        raw_gps_data = self.send_at('AT+CGNSINF')   # Get GPS info

        # cuting and extracting necesery data
        splited = raw_gps_data.split(',')
        self.gps_data = {'time' : splited[2], 'latitude' : splited[3], 'longitude' : splited[4], 'C/N0': splited[19] , 'HPA' : splited[20] }

        # ToDo: Check if data are credible (time > 20251104173835.000)
        return self.gps_data

    def send_data_up(self):
        #ToDo
        pass

