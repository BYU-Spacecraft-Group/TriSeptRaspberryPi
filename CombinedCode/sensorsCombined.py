# multiuse
import time
import os

# adc
import glob
import csv
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

SPI_PORT    = 0
SPI_DEVICE  = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# bme280
import board #also: gps, imu, mprls pressure
from adafruit_bme280 import basic as adafruit_bme280

i2c = board.I2C() # also imu
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# gps
import busio
import adafruit_gps
import serial

        # using uart - can use I2C if necessary
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") # turn it on
gps.send_command(b"PMTK220,1000") # set update rate (1Hz)

last_print = time.monotonic() # may be unnecessary - used in making it 1 Hz

# imu 
from adafruit_icm20x import ICM20649, AccelRange, GyroRange

ism = ICM20649(i2c, 0x69)
ism.accelerometer_range = AccelRange.RANGE_30G
ism.gyro_range = GyroRange.RANGE_4000_DPS
ax_max = ay_max = az_max = 0
gx_max = gy_max = gz_max = 0
ism.gyro_data_rate = 125
ism.accelerometer_data_rate = 1125
st = time.monotonic()

# mprls pressure 
import adafruit_mprls

mpr = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)


# ms5803 pressure
import smbus

bus = smbus.SMBus(1)
bus.write_byte(0x76, 0x1E)


# rtc --> sudo hwclock -r

while True:
    # adc ---------------------------------------------------------------------------------
    print("ADC" + "-"*50)
    values = [0]*8
    for i in range(8):
        values[i] = round(mcp.read_adc(i)) # is there a reason to round here?
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    
    # bme280 ------------------------------------------------------------------------------
    print("BME" + "-"*50)
    print("\nTemperature: %0.1f C" % bme280.temperature + 
        "\tHumidity: %0.1f %%" % bme280.relative_humidity + 
        "\tPressure: %0.1f hPa" % bme280.pressure)
    #print("Altitude = %0.2f meters" % bme280.altitude) why is this commented out?

    # gps ---------------------------------------------------------------------------------
    print("GPS" + "-"*50)
    gps.update()
    current = time.monotonic()

    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.
        print("=" * 40)  # Print a separator line.
        print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                gps.timestamp_utc.tm_mday,  # struct_time object that holds
                gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                gps.timestamp_utc.tm_min,  # month!
                gps.timestamp_utc.tm_sec,
            )
        )
        print("Latitude: {0:.6f} degrees".format(gps.latitude))
        print("Longitude: {0:.6f} degrees".format(gps.longitude))
        print("Fix quality: {}".format(gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        if gps.satellites is not None:
            print("# satellites: {}".format(gps.satellites))
        if gps.altitude_m is not None:
            print("Altitude: {} meters".format(gps.altitude_m))
        if gps.speed_knots is not None:
            print("Speed: {} knots".format(gps.speed_knots))
        if gps.track_angle_deg is not None:
            print("Track angle: {} degrees".format(gps.track_angle_deg))
        if gps.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(gps.horizontal_dilution))
        if gps.height_geoid is not None:
            print("Height geoid: {} meters".format(gps.height_geoid))

    # imu ---------------------------------------------------------------------------------
    print(
        "Accel X:%.2f Y:%.2f Z:%.2f ms^2 Gyro X:%.2f Y:%.2f Z:%.2f degrees/s"
        % (ism.acceleration + ism.gyro)
    )

    # mprls pressure ----------------------------------------------------------------------
    print(mpr.pressure, "hPa")

    # ms5803 pressure ---------------------------------------------------------------------
    # needs to be trimed and sleeps need to be removed <-----
    # MS5803_01BA address, 0x76
    #		0x1E(30)	Reset command
    bus.write_byte(0x76, 0x1E)

    time.sleep(0.5)

    # Read 12 bytes of calibration data
    # Read pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA2, 2)
    C1 = data[0] * 256 + data[1]

    # Read pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA4, 2)
    C2 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA6, 2)
    C3 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA8, 2)
    C4 = data[0] * 256 + data[1]

    # Read reference temperature
    data = bus.read_i2c_block_data(0x76, 0xAA, 2)
    C5 = data[0] * 256 + data[1]

    # Read temperature coefficient of the temperature
    data = bus.read_i2c_block_data(0x76, 0xAC, 2)
    C6 = data[0] * 256 + data[1]

    # MS5803_01BA address, 0x76(118)
    #		0x40(64)	Pressure conversion(OSR = 256) command
    bus.write_byte(0x76, 0x40)

    time.sleep(0.5)

    # Read digital pressure value
    # Read data back from 0x00(0), 3 bytes
    # D1 MSB2, D1 MSB1, D1 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D1 = value[0] * 65536 + value[1] * 256 + value[2]

    # MS5803_01BA address, 0x76(118)
    #		0x50(64)	Temperature conversion(OSR = 256) command
    bus.write_byte(0x76, 0x50)

    time.sleep(0.5)

    # Read digital temperature value
    # Read data back from 0x00(0), 3 bytes
    # D2 MSB2, D2 MSB1, D2 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D2 = value[0] * 65536 + value[1] * 256 + value[2]

    dT = D2 - C5 * 256
    TEMP = 2000 + dT * C6 / 8388608
    OFF = C2 * 65536 + (C4 * dT) / 128
    SENS = C1 * 32768 + (C3 * dT ) / 256
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    if TEMP >= 2000 :
        T2 = 0
        OFF2 = 0
        SENS2 = 0
        if TEMP > 4500 :
            SENS2 = SENS2 - ((TEMP - 4500) * (TEMP - 4500)) / 8
    elif TEMP < 2000 :
        T2 = (dT * dT) / 2147483648
        OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000))
        SENS2 = 7 * ((TEMP - 2000) * (TEMP - 2000)) / 8
        if TEMP < -1500 :
            SENS2 = SENS2 + 2 * ((TEMP + 1500) * (TEMP + 1500))

    TEMP = TEMP - T2
    OFF = OFF - OFF2
    SENS = SENS - SENS2
    pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0
    cTemp = TEMP / 100.0
    fTemp = cTemp * 1.8 + 32

    # Output data to screen
    print ("Pressure : %.2f mbar" %pressure)
    print ("Temperature in Celsius : %.2f C" %cTemp)
    print ("Temperature in Fahrenheit : %.2f F" %fTemp)

    # rtc ---------------------------------------------------------------------------------
    #  sudo hwclock -r --- why is this a terminal command - it would be better off only python

    time.sleep(0.01)
    
    