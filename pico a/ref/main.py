from machine import I2C, Pin, UART
import time
import struct
import math

# =====================================================
# CONFIG
# =====================================================

P0 = 1016.0
GPS_WARMUP_TIME = 30

# LED
led = Pin(18, Pin.OUT)
led_state = False
last_led_toggle = 0

# I2C
i2c_a = I2C(1, scl=Pin(11), sda=Pin(10), freq=100000)
i2c_b = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)

BMP_A = 0x77
BMP_B = 0x77
IMU_ADDR = 0x68
MAG_ADDR = 0x1E

uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))

# =====================================================
# IMU INIT
# =====================================================

i2c_a.writeto_mem(IMU_ADDR, 0x06, b'\x01')
time.sleep(0.1)
i2c_a.writeto_mem(IMU_ADDR, 0x14, b'\x01')

# =====================================================
# MAG INIT
# =====================================================

i2c_a.writeto_mem(MAG_ADDR, 0x20, b'\x70')
i2c_a.writeto_mem(MAG_ADDR, 0x21, b'\x00')
i2c_a.writeto_mem(MAG_ADDR, 0x22, b'\x00')

# =====================================================
# BMP FUNCTIONS
# =====================================================

def init_bmp(i2c, addr):
    chip = i2c.readfrom_mem(addr, 0x00, 1)[0]
    if chip != 0x60:
        raise Exception("BMP390 not detected")

    i2c.writeto_mem(addr, 0x7E, b'\xB6')
    time.sleep(0.2)

    calib = i2c.readfrom_mem(addr, 0x31, 21)
    par = {}

    par["t1"] = struct.unpack_from("<H", calib, 0)[0] / 2**-8
    par["t2"] = struct.unpack_from("<H", calib, 2)[0] / 2**30
    par["t3"] = struct.unpack_from("<b", calib, 4)[0] / 2**48
    par["p1"] = (struct.unpack_from("<h", calib, 5)[0] - 2**14) / 2**20
    par["p2"] = (struct.unpack_from("<h", calib, 7)[0] - 2**14) / 2**29
    par["p3"] = struct.unpack_from("<b", calib, 9)[0] / 2**32
    par["p4"] = struct.unpack_from("<b", calib,10)[0] / 2**37
    par["p5"] = struct.unpack_from("<H", calib,11)[0] / 2**-3
    par["p6"] = struct.unpack_from("<H", calib,13)[0] / 2**6
    par["p7"] = struct.unpack_from("<b", calib,15)[0] / 2**8
    par["p8"] = struct.unpack_from("<b", calib,16)[0] / 2**15
    par["p9"] = struct.unpack_from("<h", calib,17)[0] / 2**48
    par["p10"] = struct.unpack_from("<b", calib,19)[0] / 2**48
    par["p11"] = struct.unpack_from("<b", calib,20)[0] / 2**65

    i2c.writeto_mem(addr, 0x1C, b'\x03')
    i2c.writeto_mem(addr, 0x1D, b'\x03')
    i2c.writeto_mem(addr, 0x1B, b'\x33')

    return par

def read_bmp(i2c, addr, par):
    data = i2c.readfrom_mem(addr, 0x04, 6)
    raw_press = data[0] | (data[1]<<8) | (data[2]<<16)
    raw_temp  = data[3] | (data[4]<<8) | (data[5]<<16)

    partial = raw_temp - par["t1"]
    t_lin = partial * par["t2"] + (partial**2) * par["t3"]

    offset = par["p5"] + par["p6"]*t_lin + par["p7"]*t_lin**2 + par["p8"]*t_lin**3
    sensitivity = par["p1"] + par["p2"]*t_lin + par["p3"]*t_lin**2 + par["p4"]*t_lin**3

    pressure = sensitivity*raw_press + offset
    pressure += (raw_press**2)*(par["p9"] + par["p10"]*t_lin)
    pressure += (raw_press**3)*par["p11"]

    pressure_hpa = pressure / 100
    altitude = 44330 * (1 - (pressure_hpa / P0) ** 0.1903)

    return pressure_hpa, altitude

par_a = init_bmp(i2c_a, BMP_A)
par_b = init_bmp(i2c_b, BMP_B)

# =====================================================
# GPS
# =====================================================

gps = {"lat":None,"lon":None,"alt":None,"fix":False}
gps_start_time = time.time()
gps_ready = False

def parse_latlon(value, direction):
    if not value or '.' not in value:
        return None
    dot = value.index('.')
    deg_len = dot - 2
    deg = float(value[:deg_len])
    minutes = float(value[deg_len:])
    dec = deg + minutes/60
    if direction in ('S','W'):
        dec = -dec
    return dec

def decode_nmea(line):
    try:
        parts = line.split(',')
        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            gps["fix"] = parts[6] != '0'
            if parts[9]:
                gps["alt"] = float(parts[9])
        elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
            if parts[2] == 'A':
                gps["lat"] = parse_latlon(parts[3], parts[4])
                gps["lon"] = parse_latlon(parts[5], parts[6])
    except:
        pass

# =====================================================
# MAIN LOOP
# =====================================================

print("SYSTEM READY")
print("=================================")

while True:

    current_time = time.time()

    # LED BLINK LOGIC
    if gps_ready:
        blink_interval = 0.2   # fast blink when locked
    else:
        blink_interval = 1.0   # slow blink while acquiring

    if current_time - last_led_toggle > blink_interval:
        led_state = not led_state
        led.value(led_state)
        last_led_toggle = current_time

    # IMU
    accel = i2c_a.readfrom_mem(IMU_ADDR, 0x2D, 6)
    ax = struct.unpack(">h", accel[0:2])[0] / 8192
    ay = struct.unpack(">h", accel[2:4])[0] / 8192
    az = struct.unpack(">h", accel[4:6])[0] / 8192

    # MAG
    mag = i2c_a.readfrom_mem(MAG_ADDR, 0x28 | 0x80, 6)
    mx = struct.unpack("<h", mag[0:2])[0]
    my = struct.unpack("<h", mag[2:4])[0]
    mz = struct.unpack("<h", mag[4:6])[0]

    # BMP
    press_a, alt_a = read_bmp(i2c_a, BMP_A, par_a)
    press_b, alt_b = read_bmp(i2c_b, BMP_B, par_b)

    # GPS
    if uart.any():
        line = uart.readline()
        if line:
            try:
                decode_nmea(line.decode().strip())
            except:
                pass

    elapsed = time.time() - gps_start_time

    print("=== RAIL A DATA ===")
    print("Accel (g):", round(ax,2), round(ay,2), round(az,2))
    print("Mag:", mx, my, mz)
    print("BMP_A:", round(press_a,2),"hPa | Alt:",round(alt_a,2),"m")

    print("=== RAIL B DATA ===")
    print("BMP_B:", round(press_b,2),"hPa | Alt:",round(alt_b,2),"m")

    print("=== GPS DATA ===")

    if not gps_ready:
        if gps["fix"]:
            gps_ready = True
            print("GPS LOCK ACQUIRED ✅")
        elif elapsed < GPS_WARMUP_TIME:
            print("Acquiring... (",int(GPS_WARMUP_TIME-elapsed),"s left)")
        else:
            print("No Fix ❌")
    else:
        print("Lat:",gps["lat"],"Lon:",gps["lon"],"Alt:",gps["alt"])

    print("=================================")
    time.sleep(0.5)
    