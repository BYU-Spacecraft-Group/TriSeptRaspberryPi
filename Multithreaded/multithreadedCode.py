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


def i2c():
    for i in range(100):
        print("hello")
        time.sleep(1)

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

print("done")