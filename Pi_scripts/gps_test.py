import serial
import time

ser = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=2) # this configuratyion is correct
ser.flushInput()

def send_at(command):
    ser.write((command + '\r\n').encode())
    time.sleep(1)
    response = ser.read_all().decode(errors='ignore')
    print(f"Command: {command}\nResponse: {response}\n")

send_at('AT')            # Basic check
send_at('AT+CGNSPWR=1')  # Power on GPS
send_at('AT+CGNSINF')    # Get GPS info

ser.close()
