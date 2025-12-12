#!/usr/bin/env python3
import time, math
from smbus import SMBus
from luma.core.interface.serial import spi
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
import RPi.GPIO as gpio
import sys

# --- I2C ---
I2C_BUS = 1
bus = SMBus(I2C_BUS)

# --- Adresy ---
MPU_ADDR_1 = 0x68
MPU_ADDR_2 = 0x69
AK8963_ADDR = 0x0C

# --- Rejestry MPU-9250 (wg. RM v1.6) ---
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

# --- Rejestry AK8963 (magnetometr) ---
AK_WIA     = 0x00  # oczekiwane 0x48
AK_HXL     = 0x03  # HxL, HxH, HyL, HyH, HzL, HzH, ST2
AK_ST2     = 0x09
AK_CNTL1   = 0x0A
AK_CNTL2   = 0x0B
AK_ASAX    = 0x10  # ASAX, ASAY, ASAZ (Fuse ROM)

# --- KONFIG OLED ---
DC_PIN  = 25
RST_PIN = 27
ROTATE  = 2

# --- TEKSTY (inicjalne) ---
TEXT1 = "+33.5"
TEXT2 = "-23.22"
TEXT3 = "21.37"


MIOTA_SCREEN_TIME = 1.0  # było 5
VALUE_OF_MIOTA = 55.0  # było 25
VALUE_OF_MIOTA_m = -55.0  # było 25


# --- PARAMETRY RYSOWANIA ---
MARGIN      = 1           # margines przy krawędziach
BOTTOM_PAD  = 6           # ile pikseli ponad dolną krawędź trzymać tekst
LINE_GAP    = 25          # odstęp między linią a górą napisów (dla kołyszącej linii – obecnie wyłączone)
AMPL_DEG    = 6           # amplituda kołysania linii [stopnie]
FREQ_HZ     = 0.25        # częstotliwość kołysania [Hz]
LINE_WIDTH  = 2           # grubość linii

# --- INICJALIZACJA OLED ---
serial = spi(port=0, device=0, gpio_DC=DC_PIN, gpio_RST=RST_PIN)
device = sh1106(serial, width=128, height=64, rotate=ROTATE)

try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
except Exception:
    font = ImageFont.load_default()

def confPins():
    # ustaw tryb oznaczania GPIO jako BOARD
    # taki uj, 3eba ustawic na BCM bo tak konfiguruje je inne library uzyte w tym programie
    gpio.setmode(gpio.BCM)

    # konfiguracja pinow jako wyjscia
    gpio.setup(5, gpio.OUT)  # wejscie 29  a BCM: 5
    gpio.setup(6, gpio.OUT)  # wejscie 30  a BCM: 6
    gpio.setup(13, gpio.OUT)  # wejscie 33  a BCM: 13

    # ustawy początkową pozycję
    gpio.output(5, gpio.LOW)
    gpio.output(6, gpio.LOW)
    gpio.output(13, gpio.LOW)

   # testmoduleLED()


def testmoduleLED():
     gpio.output(5, gpio.HIGH)
     gpio.output(6, gpio.HIGH)
     gpio.output(13, gpio.HIGH)


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

# --- Funkcje MPU / AK8963 ---

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
    raise RuntimeError("Nie wykryto MPU-9250 pod 0x68 ani 0x69")

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
    # wybudzenie i podstawowa konfiguracja
    mpu_write(addr, PWR_MGMT_1, 0x00); time.sleep(0.05)  # clock=auto, sleep=0
    mpu_write(addr, SMPLRT_DIV, 0x07)   # 1 kHz/(1+7) = 125 Hz
    mpu_write(addr, CONFIG, 0x03)       # DLPF ~44 Hz
    mpu_write(addr, GYRO_CONFIG, 0x00)  # ±250 dps
    mpu_write(addr, ACCEL_CONFIG, 0x00) # ±2 g
    mpu_write(addr, ACCEL_CONFIG2, 0x03)# A-DLPF ~44 Hz

    # wyłącz wewn. I2C master i włącz bypass na złącze (AK8963 będzie widoczny jako 0x0C)
    mpu_write(addr, USER_CTRL, 0x00)
    mpu_write(addr, INT_PIN_CFG, 0x02)  # I2C_BYPASS_EN

def init_ak8963():
    # power-down
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00); time.sleep(0.01)
    # Fuse ROM access (odczyt ASA)
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x0F); time.sleep(0.01)
    asa = bus.read_i2c_block_data(AK8963_ADDR, AK_ASAX, 3)
    # współczynniki korekcji zgodnie z DS: (ASA-128)/256 + 1
    adj = [((a - 128) / 256.0) + 1.0 for a in asa]
    # wróć do power-down
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x00); time.sleep(0.01)
    # reset soft (opcjonalnie)
    bus.write_byte_data(AK8963_ADDR, AK_CNTL2, 0x01); time.sleep(0.01)
    # 16-bit, Continuous Measurement Mode 2 (100 Hz) → 0x16
    bus.write_byte_data(AK8963_ADDR, AK_CNTL1, 0x16); time.sleep(0.01)
    return adj

def read_imu(addr, mag_adj):
    # ACC / GYRO
    ax, ay, az = read_vec_xyz(addr, ACCEL_XOUT_H)
    gx, gy, gz = read_vec_xyz(addr, GYRO_XOUT_H)
    # TEMP (formuła dla MPU-9250): T(°C) = Temp_out/333.87 + 21.0
    th, tl = mpu_read(addr, TEMP_OUT_H, 2)
    traw = to_int16(th, tl)
    temp_c = (traw / 333.87) + 21.0

    # skalowanie do jednostek fizycznych (wg domyślnych zakresów)
    acc = (ax/16384.0, ay/16384.0, az/16384.0)     # g
    gyro = (gx/131.0,    gy/131.0,    gz/131.0)    # deg/s

    # MAG: odczyt 6 bajtów + ST2 (czyści DRDY)
    m = bus.read_i2c_block_data(AK8963_ADDR, AK_HXL, 7)
    # Uwaga: AK8963 zwraca **little-endian** (L,H)
    mx = to_int16(m[1], m[0])
    my = to_int16(m[3], m[2])
    mz = to_int16(m[5], m[4])
    # uwzględnij fabryczną korekcję ASA oraz skalę 16-bit: 0.15 µT/LSB
    mx_uT = mx * mag_adj[0] * 0.15
    my_uT = my * mag_adj[1] * 0.15
    mz_uT = mz * mag_adj[2] * 0.15

    return acc, gyro, temp_c, (mx_uT, my_uT, mz_uT)

# --- KONWERSJA g -> stopnie przechyłu ---

def tilt_degrees_from_g(g_val):
    """
    Zamienia przyspieszenie w osi (w jednostkach 'g', typowo -1.0..+1.0)
    na kąt pochylenia w stopniach względem grawitacji.
    """
    g_clamped = max(-1.0, min(1.0, g_val))  # zabezpieczenie domeny dla asin
    return math.degrees(math.asin(g_clamped))



def show_miota_screen():
    # Ekran alarmowy: cały OLED biały, na środku dwa wiersze:
    # "MIOTA NIM"
    # "JAK SZATAN"
    t_end = time.time() + MIOTA_SCREEN_TIME
    while time.time() < t_end:
        with canvas(device) as draw:
            # całe tło na biało
            draw.rectangle(device.bounding_box, outline="white", fill="white")

            line1 = "UWAGA! WYKRYTO"
            line2 = "NAGŁE MANEWRY"

            w1, h1 = text_wh(draw, line1)
            w2, h2 = text_wh(draw, line2)

            # pozycje tak, by tekst był na środku
            x1 = (device.width  - w1) // 2
            x2 = (device.width  - w2) // 2
            # środek ekranu między liniami
            y_center = device.height // 2
            y1 = y_center - h1
            y2 = y_center + 2

            # tekst na czarno (kontrast na białym tle)
            draw.text((x1, y1), line1, fill="black", font=font)
            draw.text((x2, y2), line2, fill="black", font=font)

        # lekkie opóźnienie żeby nie zapychać I2C
        time.sleep(0.05)






def main():
    mpu_addr = detect_mpu()
    print(f"MPU-9250 @ 0x{mpu_addr:02X}, WHO_AM_I=0x71 OK")
    init_mpu(mpu_addr)

    # Sprawdź identyfikator mag
    try:
        wia = bus.read_byte_data(AK8963_ADDR, AK_WIA)
        if wia != 0x48:
            print(f"AK8963 WIA=0x{wia:02X} (oczekiwane 0x48) – kontynuuję, ale sprawdź połączenia.")
    except Exception as e:
        raise RuntimeError("Nie widzę AK8963 pod 0x0C – czy BYPASS włączony i I2C podłączone?") from e

    mag_adj = init_ak8963()
    print(f"AK8963 ASA adj = {mag_adj}")

    while True:

        acc, gyro, temp_c, mag = read_imu(mpu_addr, mag_adj)  # odczytywanie danych z czujnika

        status = (
              f"ACC[g]  : {acc[0]:+6.3f} {acc[1]:+6.3f} {acc[2]:+6.3f} | "
              f"GYR[°/s]: {gyro[0]:+7.2f} {gyro[1]:+7.2f} {gyro[2]:+7.2f} | "
              f"T={temp_c:5.2f}°C | "
              f"MAG[µT]: {mag[0]:+7.2f} {mag[1]:+7.2f} {mag[2]:+7.2f}")

        print("\r" + status.ljust(120), end="", flush=True)



        # --- przechylenia w stopniach ---

        # Lewo / prawo – oś Y (acc[1])
        # u Ciebie: lewo = plus, prawo = minus.
        acc_lr = acc[1]
        tilt_lr_deg = tilt_degrees_from_g(acc_lr)

        if tilt_lr_deg >= 0:
            tilt_left_deg  = tilt_lr_deg      # przechył w lewo
            tilt_right_deg = 0.0
        else:
            tilt_left_deg  = 0.0
            tilt_right_deg = -tilt_lr_deg     # przechył w prawo (wartość dodatnia)

        # Przód / tył – oś X (acc[0])
        # założenie: przód = minus, tył = plus.
        acc_fb = acc[0]
        tilt_fb_deg = tilt_degrees_from_g(acc_fb)

        if tilt_fb_deg >= 0:
            tilt_back_deg  = tilt_fb_deg      # przechył w tył
            tilt_front_deg = 0.0
        else:
            tilt_back_deg  = 0.0
            tilt_front_deg = -tilt_fb_deg     # przechył w przód
      #  print(f"To jest nasza wartość ================================= {acc_fb}")


        if abs(tilt_lr_deg) > 30 or abs(tilt_fb_deg) > 30:
            print(f"IF od wiecej niz 30 | wartosc: {tilt_lr_deg} | wartosc FB {tilt_fb_deg}")
            print(" === LR | ")
            print(tilt_lr_deg)
            print(" | === FB ")
            print(tilt_fb_deg)
            print("|  === ")
            gpio.output(13, gpio.HIGH)
            gpio.output(5 ,gpio.LOW)
            gpio.output(6, gpio.LOW)

        elif (abs(tilt_lr_deg) > 10 and abs(tilt_lr_deg) < 30) or (abs(tilt_fb_deg) > 10 and abs(tilt_fb_deg) < 30):
            print(f"wiecej niż 10 i mniej niż 30 | wartosc LR {tilt_lr_deg} | wartosc FB {tilt_fb_deg}")
            gpio.output(13, gpio.LOW)
            gpio.output(6 ,gpio.HIGH)
            gpio.output(5, gpio.LOW)

        else:
            print("====== = = = = = mniej niż 10, wartosc LR {tilt_lr_deg} | wartosc FB {tilt_fb_deg}")
            gpio.output(13, gpio.LOW)
            gpio.output(6 ,gpio.LOW)
            gpio.output(5, gpio.HIGH)

            print(" === LR | ")
            print(tilt_lr_deg)
            print(" | === FB | ")
            print(tilt_fb_deg)
            print(" | === ")



        # Teksty z ° (stopniami)
        deg_sign   = u"\N{DEGREE SIGN}"
        left_str   = f"{tilt_left_deg:4.1f}{deg_sign}"
        right_str  = f"{tilt_right_deg:4.1f}{deg_sign}"
        front_str  = f"{tilt_front_deg:4.1f}{deg_sign}"
        back_str   = f"{tilt_back_deg:4.1f}{deg_sign}"


        # Gyro do dolnego wiersza

        if gyro[0] > 0:
            TEXT1 = f"→ {gyro[0]:+7.2f}"
        else:
            TEXT1 = f"← {gyro[0]:+7.2f}"

        if gyro[1] > 0:
            TEXT3 = f"↑ {gyro[1]:+7.2f}"
        else:
            TEXT3 = f"↓ {gyro[1]:+7.2f}"


        #TEXT1 = f"{gyro[0]:+7.2f}"
        #TEXT2 = f"{gyro[1]:+7.2f}"
        #TEXT3 = f"{gyro[2]:+7.2f}"

        # przejście do screenu na 5 sekund
        if gyro[0] > VALUE_OF_MIOTA or gyro[1] > VALUE_OF_MIOTA or gyro[0] < VALUE_OF_MIOTA_m or gyro[1] < VALUE_OF_MIOTA_m:
            show_miota_screen()



        # Pozycja geograficzna – na razie stałe
        Nposition = "52.2297"
        Eposition = "21.0122"

        # Stare surowe ACC na OLED – już nieużywane (zostawiam zakomentowane):
        """
        ACC1oled = f"{acc[0]:+6.3f}"
        ACC2oled = f"{acc[1]:+6.3f}"
        ACC3oled = f"{acc[2]:+6.3f}"
        """

        device_screen_width = 128

        with canvas(device) as draw:
            # --- STARY WYŚWIETLACZ SUROWYCH ACC – WYŁĄCZONY ---
            """
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
            """

            # --- NOWY OPIS PRZECHYLENIA W STOPNIACH ---

            # Wiersz 1: lewo/prawo  ->  wartość_lewo ← →wartość_prawo
            line_lr = f"{left_str}←→{right_str}"

            # Wiersz 2: przód/tył   ->  wartość_przód ↑ ↓wartość_tył
            line_fb = f"{front_str}↑↓{back_str}"

            w_lr, h_lr = text_wh(draw, line_lr)
            w_fb, h_fb = text_wh(draw, line_fb)

            # Ten sam „pas” wysokości, w którym wcześniej były ACC
            y_lr = device.height - 30
            y_fb = device.height - 30
            #  y_lr + h_lr + 1

            x_lr = device_screen_width - 128
            x_fb = (device_screen_width / 2)

            draw.text((x_lr, y_lr), line_lr, fill="white", font=font)
            draw.text((x_fb, y_fb), line_fb, fill="white", font=font)

            # --- rozmieszczenie napisów na dole (GYRO) ---
            w1, h1 = text_wh(draw, TEXT1)
            w2, h2 = text_wh(draw, TEXT2)
            w3, h3 = text_wh(draw, TEXT3)
            h_max = max(h1, h2, h3)

            # bazowa linia tekstu – trochę nad dolną krawędzią
            y_text = device.height - h_max - MARGIN - BOTTOM_PAD

            x_left   = MARGIN
            x_center = (device.width - w2) // 2
            x_right  = device.width - w1 - MARGIN

            draw.text((x_left,   y_text), TEXT3, fill="white", font=font)
            #draw.text((x_center, y_text), TEXT3, fill="white", font=font)
            draw.text((x_right,  y_text), TEXT1, fill="white", font=font)

            # rysowanie tekstu pozycji geograficznej
            wN, hN = text_wh(draw, Nposition)
            wE, hE = text_wh(draw, Eposition)

            hN = device.height - 64  # Nposition ma być wyżej
            hE = device.height - 54

            wN = MARGIN
            wE = MARGIN

            #draw.text((wN, hN), Nposition, fill="white", font=font)
            #draw.text((wE, hE), Eposition, fill="white", font=font)

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
            # --- linia nad napisami, kołysząca się ---
            y_line = y_text - LINE_GAP
            length = device.width - 2*MARGIN
            cx = device.width // 2

            t = time.time() - t0
            angle = AMPL_DEG * math.sin(2 * math.pi * FREQ_HZ * t)

            x1, y1, x2, y2 = line_endpoints(cx, y_line, length, angle)
            draw.line((x1, y1, x2, y2), fill="white", width=LINE_WIDTH)
            """

            time.sleep(0.1)

    gpio.cleanup()
if __name__ == "__main__":
    confPins()
    main()
else:
    print("Nie importuj pliku, to nie library")
    sys.exit()
