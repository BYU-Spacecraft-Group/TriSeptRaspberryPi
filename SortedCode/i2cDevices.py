"""
Devices in this file: bme280, imu, mprls, rtc, ms5803
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

        # ms5803 pressure
        import smbus

        bus = smbus.SMBus(1)
        #bus.write_byte(0x76, 0x1E)

        # rtc --> sudo hwclock -r
        import adafruit_ds3231
        rtc = adafruit_ds3231.DS3231(i2c)
    except:
        continue
    break

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

        dataIn += f"{pressure}, {cTemp}, " #{fTemp}, "
    except OSError:
        dataIn += "-, -, "
    print("MS5803 Time: " + str(time.time() - startMS58))

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

            #### ms5803
            ms5803()

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
        print("--OS ERROR on I2C")
        break
    except:
        continue
    # without break because it should never break 
