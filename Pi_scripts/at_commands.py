import serial
import time

def send_at(serial: serial.Serial, command: str, display_mode = True, debbug_mode = False) -> str:
    serial.write((command + '\r\n').encode())
    time.sleep(1)
    response = serial.read_all().decode(errors='ignore')

    if display_mode:
        print(f"Command: {command}\nResponse: {response}\n")
    if debbug_mode: # a to nie wiem po co dałem
        pass

    return response

def serial_connection_test(serial_connection):
    if send_at(serial_connection, 'at', False, False).upper() != "OK":
        print("!!! Serial connection not working !!!")

#Todo