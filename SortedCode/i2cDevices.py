"""
Devices in this file: bme280, imu, mprls, rtc
"""

# added to be more robust if any error is raised it will try to execute the code again
# but I don't think any error will happen
TIMING = True

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

        dataPipe.append("SystemTime, Temp, Hum, Pres, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MPRPressure, RTC, \n")

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

while True: # another retry scheme 
    try:
        while True:
            startCollect = time.time()
            print(it)
            #collecting data
            dataIn = "" #string to save as a line

            #### SystemTime
            startSysTime = time.time()
            dataIn += f"{time.time()}, "
            print("System Time Log: " + str(time.time() - startSysTime)) 

            #### bme280 ----- Currently takes around the longest time - ~0.03s
            startBME = time.time()
            dataIn += f"{str(bme280.temperature)}, {bme280.relative_humidity}, {bme280.pressure}, "
            print("BME Time: " + str(time.time() - startBME))

            #### imu 
            startIMU = time.time()
            dataIn += "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, " % (ism.acceleration + ism.gyro)
            print("IMU Time: " + str(time.time() - startIMU))

            #### mprls pressure
            startMPR = time.time()
            dataIn += f"{mpr.pressure}, "
            print("MPR Time: " + str(time.time() - startMPR))

            #### RTC
            startRTC = time.time()
            t = rtc.datetime

            dataIn += f"{t.tm_hour}:{t.tm_min}:{t.tm_sec}, "
            print("RTC Time: " + str(time.time() - startRTC))

            dataPipe.append(str(dataIn) + "\n")

            print("Data collect time: " + str(time.time() - startCollect))

            #saving data
            it += 1
            if(it % STORING_GAP == 0):
                start = time.time() # for timing
                file = open(f"Results{ID}.csv", 'a')
                file.writelines(dataPipe)
                file.flush()
                os.fsync(file.fileno())
                file.close()
                #clear stored values
                dataPipe.clear()
                    
                print(f"{it}: Write time: " + str(time.time() - start))
                
    except OSError:
        break
    except:
        continue
    # without break because it should never break 
