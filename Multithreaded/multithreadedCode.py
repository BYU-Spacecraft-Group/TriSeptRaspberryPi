#!/user/bin/python

import threading
import time

## imports
while True:
    try:
        # adc
        # import glob
        # import csv
        import Adafruit_GPIO.SPI as SPI
        import Adafruit_MCP3008

        SPI_PORT    = 0
        SPI_DEVICE  = 0
        mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

        # bme280
        import board #also: gps, imu, mprls pressure, rtc
        from adafruit_bme280 import basic as adafruit_bme280

        i2c = board.I2C() # also imu, rtc
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
        #bus.write_byte(0x76, 0x1E)

        # rtc --> sudo hwclock -r
        import adafruit_ds3231
        rtc = adafruit_ds3231.DS3231(i2c)

        print("importing done")

    except KeyboardInterrupt:
        break
    except:
        continue
    break

class thread (threading.Thread):
    def __init__(self, threadID, name, funct):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.funct = funct
    def run(self):
        self.funct()

def ms5803():
    #### ms5803 pressure
    startMS58 = time.time()
    try:  #this currently only works sometimes -
            # I'm not sure why. I found a 3rd party library for it 
            # (with looping example code) but it does about the same
        bus.write_byte(0x76, 0x1E)

        time.sleep(0.01)

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

        time.sleep(0.01)

        # Read digital pressure value
        # Read data back from 0x00(0), 3 bytes
        # D1 MSB2, D1 MSB1, D1 LSB
        value = bus.read_i2c_block_data(0x76, 0x00, 3)
        D1 = value[0] * 65536 + value[1] * 256 + value[2]

        # MS5803_01BA address, 0x76(118)
        #		0x50(64)	Temperature conversion(OSR = 256) command
        bus.write_byte(0x76, 0x50)

        time.sleep(0.01)

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
        #fTemp = cTemp * 1.8 + 32

        return f"{pressure}, {cTemp}, " #{fTemp}, "
    except OSError:
        return "-, -, "
    print("MS5803 Time: " + str(time.time() - startMS58))

def i2c():
    try:
        for i in range(100):
            print("hello")
            time.sleep(1)
    except KeyboardInterrupt:
        print("-- KeybaordInturrupt i2c --")


def spi():
    for i in range(100):
        print("hello SPI")
        time.sleep(2)

def uart():
    for i in range(100):
        print("hello UART")
        time.sleep(3)

#create new thread
i2cThread = thread(1, "I2C Thread", i2c)
spiThread = thread(2, "SPI Thread", spi)
uartThread = thread(3, "UART Thread", uart)

#start new thread
i2cThread.start()
spiThread.start()
uartThread.start()

try:
    i2cThread.join()
    spiThread.join()
    uartThread.join()
except Exception as e:
    print(e)

print("done")