"""
Devices in this file: bme280, imu, mprls, rtc
"""

# added to be more robust if any error is raised it will try to execute the code again
# but I don't think any error will happen
while True:
    try: 
        import time
        import os
        from random import random

        #setup file ID using system clock and random()
        ID = "i2c" + time.strftime("%H%M%S", time.localtime()) + str(int(random() * 100))

        STORING_GAP = 10 # how many cycles to wait until you take the time to save

        it = 0
        dataPipe = []

        dataPipe.append("SystemTime")

        #bme280
        import board #also: gps, imu, mprls pressure, rtc
        #from adafruit_bme280 import basic as adafruit_bme280
        import adafruit_bme280.advanced as adafruit_bme280

        i2c = board.I2C() # also imu, rtc
        bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

        bme280.sea_level_pressure = 1013.25
        bme280.standby_period = adafruit_bme280.STANDBY_TC_0_5

        #requires at least OVERSCAN_X1 to be accurate - OVERSCAN_DISABLED is faster but less accurate
        bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X1
        bme280.overscan_temperature = adafruit_bme280.OVERSCAN_X1
        bme280.overscan_humidity = adafruit_bme280.OVERSCAN_X1

        # imu 
        from adafruit_icm20x import ICM20649, AccelRange, GyroRange

        ism = ICM20649(i2c, 0x69)
        ism.accelerometer_range = AccelRange.RANGE_30G
        ism.gyro_range = GyroRange.RANGE_4000_DPS
        ax_max = ay_max = az_max = 0
        gx_max = gy_max = gz_max = 0
        ism.gyro_data_rate = 1100
        ism.accelerometer_data_rate = 1125
        st = time.monotonic()

        # mprls pressure 
        import adafruit_mprls

        mpr = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)

        # rtc --> sudo hwclock -r
        import adafruit_ds3231
        rtc = adafruit_ds3231.DS3231(i2c)
    except:
        continue
    break


