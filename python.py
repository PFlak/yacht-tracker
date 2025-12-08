#!/usr/bin/env python3
import time
from smbus import SMBus
from luma.core.interface.serial import spi
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
import time, math

I2C_BUS = 1
bus = SMBus(I2C_BUS)

# adresy urządzeń
MPU_ADDR_1 = 0x68
MPU_ADDR_2 = 0x69
AK8963_ADDR = 0x0C

# dodanie rejestrów od czujnika osiowego
WHO_AM_I      = 0x75  # oczekiwane 0x71
PWR_MGMT_1    = 0x6B
SMPLRT_DIV    = 0x19
CONFIG        = 0x1A
GYRO_CONFIG   = 0x1B
ACCEL_CONFIG  = 0x1C
ACCEL_CONFIG2 = 0x1D
INT_PIN_CFG   = 0x37
USER_CTRL     = 0x6A

ACCEL_XOUT_H  = 0x3B
TEMP_OUT_H    = 0x41
GYRO_XOUT_H   = 0x43


AK_WIA     = 0x00  
AK_HXL     = 0x03  
AK_ST2     = 0x09
AK_CNTL1   = 0x0A
AK_CNTL2   = 0x0B
AK_ASAX    = 0x10



# KONFIG OLED
DC_PIN  = 25
RST_PIN = 27
ROTATE  = 0

# static example text
TEXT1 = "+33.5"
TEXT2 = "-23.22"
TEXT3 = "21.37"

# podstawowe stałe do ekranu oledc
MARGIN      = 1
BOTTOM_PAD  = 6
LINE_GAP    = 25
AMPL_DEG    = 6
FREQ_HZ     = 0.25
LINE_WIDTH  = 2

# instancje klas budulcowych połaczenie z urządeniami
serial = spi(port=0, device=0, gpio_DC=DC_PIN, gpio_RST=RST_PIN)
device = sh1106(serial, width=128, height=64, rotate=ROTATE)

try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
except Exception:
    font = ImageFont.load_default()

def text_wh(draw, txt):
    # Pomiar rozmiaru napisu (Pillow >=8 preferuje textbbox)
    try:
        bbox = draw.textbbox((0, 0), txt, font=font)
        return bbox[2] - bbox[0], bbox[3] - bbox[1]
    except Exception:
        return draw.textsize(txt, font=font)

def line_endpoints(center_x, center_y, length, angle_deg):
    th = math.radians(angle_deg)
    dx = math.cos(th) * (length / 2.0)
    dy = math.sin(th) * (length / 2.0)
    x1 = int(center_x - dx); y1 = int(center_y - dy)
    x2 = int(center_x + dx); y2 = int(center_y + dy)
    return (x1, y1, x2, y2)

t0 = time.time()


def mpu_read(addr, reg, n=1):
    if n == 1:
        return bus.read_byte_data(addr, reg)
    return bus.read_i2c_block_data(addr, reg, n)

def mpu_write(addr, reg, val):
    bus.write_byte_data(addr, reg, val)

def detect_mpu():
    for a in (MPU_ADDR_1, MPU_ADDR_2):
        try:
            v = mpu_read(a, WHO_AM_I)
            if v == 0x71:  # WHO_AM_I dla MPU-9250
                return a
        except Exception:
            pass
    raise RuntimeError("error device, nie ma takiego urządzenia z takim adrsem, bywa :(")

def to_int16(msb, lsb):
    val = (msb << 8) | lsb
    return val - 65536 if val & 0x8000 else val

def read_vec_xyz(addr, start_reg):
    buf = mpu_read(addr, start_reg, 6)
    x = to_int16(buf[0], buf[1])
    y = to_int16(buf[2], buf[3])
    z = to_int16(buf[4], buf[5])
    return x, y, z

def init_mpu(addr):
    #inicjalizacja modułu mpu
    mpu_write(addr, PWR_MGMT_1, 0x00); time.sleep(0.05)
    mpu_write(addr, SMPLRT_DIV, 0x07)
    mpu_write(addr, CONFIG, 0x03)
    mpu_write(addr, GYRO_CONFIG, 0x00)
    mpu_write(addr, ACCEL_CONFIG, 0x00)
    mpu_write(addr, ACCEL_CONFIG2, 0x03)

    mpu_write(addr, USER_CTRL, 0x00)
    mpu_write(addr, INT_PIN_CFG, 0x02)

def init_ak8963():
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00); time.sleep(0.01)
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x0F); time.sleep(0.01)
    asa = bus.read_i2c_block_data(AK8963_ADDR, AK_ASAX, 3)
    adj = [((a - 128) / 256.0) + 1.0 for a in asa]
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00); time.sleep(0.01)
    bus.write_byte_data(AK8963_ADDR, AK_CNTL2, 0x01); time.sleep(0.01)
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x16); time.sleep(0.01)
    return adj

def read_imu(addr, mag_adj):

    ax, ay, az = read_vec_xyz(addr, ACCEL_XOUT_H)
    gx, gy, gz = read_vec_xyz(addr, GYRO_XOUT_H)
    th, tl = mpu_read(addr, TEMP_OUT_H, 2)
    traw = to_int16(th, tl)
    temp_c = (traw / 333.87) + 21.0

    acc = (ax/16384.0, ay/16384.0, az/16384.0)
    gyro = (gx/131.0,    gy/131.0,    gz/131.0)

    m = bus.read_i2c_block_data(AK8963_ADDR, AK_HXL, 7)
    mx = to_int16(m[1], m[0])
    my = to_int16(m[3], m[2])
    mz = to_int16(m[5], m[4])
    mx_uT = mx * mag_adj[0] * 0.15
    my_uT = my * mag_adj[1] * 0.15
    mz_uT = mz * mag_adj[2] * 0.15

    return acc, gyro, temp_c, (mx_uT, my_uT, mz_uT)

def main():
    mpu_addr = detect_mpu()
    print(f"MPU-9250 @ 0x{mpu_addr:02X}, WHO_AM_I=0x71 jest ok")
    init_mpu(mpu_addr)

    try:
        wia = bus.read_byte_data(AK8963_ADDR, AK_WIA)
        if wia != 0x48:
            print(f"AK8963 WIA=0x{wia:02X} (oczekiwane 0x48) – kontynuuję, ale sprawdź połączenia ;_;")
    except Exception as e:
        raise RuntimeError("Nie widzę AK8963 pod 0x0C – sprawdź połoczenie") from e

    mag_adj = init_ak8963()
    print(f"AK8963 ASA adj = {mag_adj}")

    while True:
        acc, gyro, temp_c, mag = read_imu(mpu_addr, mag_adj)

        print(
              f"ACC[g]  : {acc[0]:+6.3f} {acc[1]:+6.3f} {acc[2]:+6.3f} | "
              f"GYR[°/s]: {gyro[0]:+7.2f} {gyro[1]:+7.2f} {gyro[2]:+7.2f} | "
              f"T={temp_c:5.2f}°C | "
              f"MAG[µT]: {mag[0]:+7.2f} {mag[1]:+7.2f} {mag[2]:+7.2f}")


        TEXT1 = f"{gyro[0]:+7.2f}"
        TEXT2 = f"{gyro[1]:+7.2f}"
        TEXT3 = f"{gyro[2]:+7.2f}"

        Nposition = "52.2297"
        Eposition = "21.0122"

        ACC1oled = f"{acc[0]:+6.3f}"
        ACC2oled = f"{acc[1]:+6.3f}"
        ACC3oled = f"{acc[2]:+6.3f}"


        device_screen_width = 128
        with canvas(device) as draw:
            # acc dane
            accw1, acch1 = text_wh(draw, ACC1oled)
            accw2, acch2 = text_wh(draw, ACC2oled)
            accw3, acch3 = text_wh(draw, ACC3oled)

            accw1  = MARGIN
            accw2 = (device.width - accw2) // 2
            accw3  = device.width - (device_screen_width / 3)

            acch1 = device.height - 30
            acch2 = device.height - 30
            acch3 = device.height - 30


            draw.text((accw1, acch1), ACC1oled, fill="white", font=font)
            draw.text((accw2, acch2), ACC2oled, fill="white", font=font)
            draw.text((accw3, acch3), ACC3oled, fill="white", font=font)


            # rozmieszczenie napisów na dole
            w1, h1 = text_wh(draw, TEXT1)
            w2, h2 = text_wh(draw, TEXT2)
            w3, h3 = text_wh(draw, TEXT3)
            h_max = max(h1, h2, h3)

            # ustalamy pozycję oraz treść dla długości geograficznych
            wN, hN = text_wh(draw, Nposition)
            wE, hE = text_wh(draw, Eposition)

            hN = device.height - 64 # Nposition ma być wyżej
            hE = device.height - 54

            wN = MARGIN
            wE = MARGIN

            # bazowa linia tekstu – trochę nad dolną krawędzią
            y_text = device.height - h_max - MARGIN - BOTTOM_PAD

            x_left   = MARGIN
            x_center = (device.width - w2) // 2
            x_right  = device.width - w1 - MARGIN

            draw.text((x_left,   y_text), TEXT2, fill="white", font=font)
            draw.text((x_center, y_text), TEXT3, fill="white", font=font)
            draw.text((x_right,  y_text), TEXT1, fill="white", font=font)

            #drawing mi tekst pozycji geograficznej
            draw.text((wN, hN), Nposition, fill="white", font=font)
            draw.text((wE, hE), Eposition, fill="white", font=font)


            # linia pozioma
            xx1 = 0
            yy1 = device.height - 42

            xx2 = device.width
            yy2 = device.height - 42

            draw.line((xx1, yy1, xx2, yy2), fill="white")

            # linia pionowa

            x_pionowa1 = device.width / 2
            x_pionowa2 = device.width / 2
            y_pionowa1 = device.height - 42
            y_pionowa2 = device.height - 63

            draw.line((x_pionowa1, y_pionowa1, x_pionowa2, y_pionowa2), fill="white")

            """
            # linia nad napisami, kołysząca się 
            # pozycja Y linii tuż NAD napisami
            y_line = y_text - LINE_GAP

            # pełna szerokość ekranu (z niewielkim marginesem, by nie „dotykało” krawędzi)
            length = device.width - 2*MARGIN
            cx = device.width // 2

            # kąt kołysania: sinus czasu
            t = time.time() - t0
            angle = AMPL_DEG * math.sin(2 * math.pi * FREQ_HZ * t)

            x1, y1, x2, y2 = line_endpoints(cx, y_line, length, angle)
            draw.line((x1, y1, x2, y2), fill="white", width=LINE_WIDTH)
            """

            time.sleep(0.1)

if __name__ == "__main__":
    main()