#!/user/bin/python

## imports
while True:
    try:
        import threading
        import time
        import os
        from random import random

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

        # gps
        import busio
        import adafruit_gps
        import serial

                # using uart - can use I2C if necessary
        uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=10)
        gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") # turn it on
        #gps.send_command(b"PMTK220,1000") # set update rate (1Hz)
        gps.send_command(b"PMTK220,500") # set update rate to 2Hz - according to example code this is the max
        last_print = time.monotonic()

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
        bus.write_byte(0x76, 0x1E)
        MS5803_DELAY = 0.60 / 1000

        # rtc --> sudo hwclock -r
        import adafruit_ds3231
        rtc = adafruit_ds3231.DS3231(i2c)

        print("importing done")

    except KeyboardInterrupt:
        print("Keyboard interrupt in import")
        break
    except Exception as e:
        print("Other exception in import", e)
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
    try:  #this currently only works sometimes -
            # I'm not sure why. I found a 3rd party library for it 
            # (with looping example code) but it does about the same
        # bus.write_byte(0x76, 0x1E) done in setup

        # time.sleep(0.01)

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

        time.sleep(MS5803_DELAY)

        # Read digital pressure value
        # Read data back from 0x00(0), 3 bytes
        # D1 MSB2, D1 MSB1, D1 LSB
        value = bus.read_i2c_block_data(0x76, 0x00, 3)
        D1 = value[0] * 65536 + value[1] * 256 + value[2]

        # MS5803_01BA address, 0x76(118)
        #		0x50(64)	Temperature conversion(OSR = 256) command
        bus.write_byte(0x76, 0x50)

        time.sleep(MS5803_DELAY)

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
    

def i2c():
    while True:
        try:
            #setup file ID using system clock and random()
            ID = time.strftime("%m.%d.%H%M%S", time.localtime()) + str(int(random() * 100))

            STORING_GAP = 10 # how many cycles to wait until you take the time to save

            it = 0
            dataPipe = []

            dataPipe.append("SystemTime, Temp, Hum, Pres, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MPRPressure, MSPressure, MSTemp, RTC, \n")


        except Exception as e:
            print("exception in i2c setup ", e)
            continue
        break
    while True: # retry scheme 
        try:
            while True:

                BMESKIP = 5
                RTCSKIP = 10
                MSSKIP = 5
                MPRSKIP = 5

                startCollect = time.time()
                print(it)
                #collecting data
                dataIn = "" #string to save as a line

                #### SystemTime
                startSysTime = time.time()
                dataIn += f"{time.time()}, "
                print("---> System Time Log: " + str(time.time() - startSysTime)) 

                #### bme280 ----- Currently takes around the longest time - ~0.03s
                if it % BMESKIP == 0:
                    startBME = time.time()
                    dataIn += f"{str(bme280.temperature)}, {bme280.relative_humidity}, {bme280.pressure}, "
                    print("---> BME Time: " + str(time.time() - startBME))
                else:
                    dataIn += "-, -, -, "
                    print("---> SKIPPED BME ")

                #### imu 
                startIMU = time.time()
                dataIn += "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, " % (ism.acceleration + ism.gyro)
                print("---> IMU Time: " + str(time.time() - startIMU))

                #### mprls pressure
                if it % MPRSKIP:
                    startMPR = time.time()
                    dataIn += f"{mpr.pressure}, "
                    print("---> MPR Time: " + str(time.time() - startMPR))
                else:
                    dataIn += "-, "
                    print("---> SKIPPED MPRLS")

                #### ms5803
                if it % MSSKIP:
                    startMS58 = time.time()
                    dataIn += ms5803()
                    print("---> MS5803 Time: " + str(time.time() - startMS58))
                else:
                    dataIn += "-, -, "
                    print("---> SKIPPED MS5803")

                #### RTC
                if it % RTCSKIP == 0:
                    startRTC = time.time()
                    t = rtc.datetime

                    dataIn += f"{t.tm_hour}:{t.tm_min}:{t.tm_sec}, "
                    print("---> RTC Time: " + str(time.time() - startRTC))
                else:
                    dataIn += "-, "
                    print("---> SKIPPED RTC ")

                dataPipe.append(str(dataIn) + "\n")

                print("---> Data collect time: " + str(time.time() - startCollect))

                #saving data
                it += 1
                if(it % STORING_GAP == 0):
                    start = time.time() # for timing
                    file = open(f"ResultsI2C{ID}.csv", 'a')
                    file.writelines(dataPipe)
                    file.flush()
                    os.fsync(file.fileno())
                    file.close()
                    #clear stored values
                    dataPipe.clear()
                        
                    print(f"---> {it}: Write time: " + str(time.time() - start))
        except OSError:
            print("-- OS ERROR on I2C --")
            break
        except KeyboardInterrupt:
            print("-- KeyboardInterrupt on I2C --")
            break
        except Exception as e:
            print("exception in i2c main loop ", e)
            continue
        # without break because it should never break 


def spi():
    while True:
        try:
            #setup file ID using system clock and random()
            ID = time.strftime("%m.%d.%H%M%S", time.localtime()) + str(int(random() * 100))
            print(ID)
            STORING_GAP = 10 # how many cycles to wait until you take the time to save

            it = 0
            dataPipe = []
            
            dataPipe.append("SystemTime, ADC, \n")

        except Exception as e:
            print("exceptoin in spi setup ", e)
            continue
        break
    while True:
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

                #### adc
                startADC = time.time()
                values = [0]*8
                for i in range(8):
                    values[i] = round(mcp.read_adc(i)) # is there a reason to round here?
                dataIn += str(values[0]) + ", "
                print("ADC time: " + str(time.time() - startADC))

                # --------------------------------------------------------------------

                dataPipe.append(str(dataIn) + "\n")
                
                print("Data collect time: " + str(time.time() - startCollect))

                #saving data
                it += 1
                if(it % STORING_GAP == 0):
                    start = time.time() # for timing
                    file = open(f"ResultsSPI{ID}.csv", 'a')
                    file.writelines(dataPipe)
                    file.flush()
                    os.fsync(file.fileno())
                    file.close()
                    #clear stored values
                    dataPipe.clear()
                        
                    print(f"{it}: Write time: " + str(time.time() - start))
                else:
                    time.sleep(0.01)

        except OSError:
            print("-- OSError on SPI --")
            break
        except KeyboardInterrupt:
            print("-- KeyboardInterrupt on SPI --")
            break
        except Exception as e:
            print("exception in SPI main loop ", e)
            continue
        # no break

def uart():
    while True:
        try:
            #setup file ID using system clock and random()
            ID = time.strftime("%m.%d.%H%M%S", time.localtime()) + str(int(random() * 100))
            print(ID)
            STORING_GAP = 10 # how many cycles to wait until you take the time to save

            it = 0
            dataPipe = []

            dataPipe.append("SystemTime, TimeStamp, Latitude, Longitude, FixQuality, Satellites, Altitude, Knots, TrackAngle, HDilution, HGeoID, \n")

            time.sleep(1)

        except Exception as e:
            print("exception in UART setup ", e)
            continue
        break
    while True:
        storageStart = time.time() # for regardless saving
        try:
            while True:
                startCollect = time.time()
                print(it)
                #collecting data
                dataIn = "" #string to save as a line

                #### gps
                startgps = time.time()
                gps.update()
                current = time.monotonic()

                if gps.has_fix and (current - last_print) >= 0.5:

                    #### SystemTime
                    startSysTime = time.time()
                    dataIn += f"{time.time()}, "
                    print("System Time Log: " + str(time.time() - startSysTime))

                    last_print = current
                    if gps.timestamp_utc is not None:
                        dataIn += f"{gps.timestamp_utc.tm_hour}:{gps.timestamp_utc.tm_min}:{gps.timestamp_utc.tm_sec}, "
                    else:
                        dataIn += "-, "
                    if gps.latitude is not None:
                        dataIn += f"{gps.latitude}, "
                    else: 
                        dataIn += "-, "
                    if gps.longitude is not None:
                        dataIn += f"{gps.longitude}, "
                    else:
                        dataIn += "-, "

                    # Some attributes beyond latitude, longitude and timestamp are optional
                    # and might not be present.  Check if they're None before trying to use!
                    if gps.fix_quality is not None:
                        dataIn += f"{gps.fix_quality}, "
                    else: 
                        dataIn += "-, "
                    if gps.satellites is not None:
                        dataIn += f"{gps.satellites}, "
                    else:
                        dataIn += "-, "
                    if gps.altitude_m is not None:
                        dataIn += f"{gps.altitude_m}, "
                    else:
                        dataIn += "-, "
                    if gps.speed_knots is not None:
                        dataIn += f"{gps.speed_knots}, "
                    else:
                        dataIn += "-, "
                    if gps.track_angle_deg is not None:
                        dataIn += f"{gps.track_angle_deg}, "
                    else:
                        dataIn += "-, "
                    if gps.horizontal_dilution is not None:
                        dataIn += f"{gps.horizontal_dilution}, "
                    else:
                        dataIn += "-, "
                    if gps.height_geoid is not None:
                        dataIn += f"{gps.height_geoid}, "
                    else:
                        dataIn += "-, "
                    it += 1
                    dataPipe.append(str(dataIn) + "\n")
                else:
                    dataIn += "-, -, -, -, -, -, -, -, -, -, "
                    time.sleep(0.1)
                print("GPS Time: " + str(time.time() - startgps))

                # --------------------------------------------------------------------
            
                

                print("Data collect time: " + str(time.time() - startCollect))

                #saving data
                #it += 1
                if(it % STORING_GAP == 0 or storageStart - time.time() > 10):
                    storageStart = time.time() # for regardless saving
                    if len(dataPipe) > 0:
                        start = time.time() # for timing
                        file = open(f"ResultsUART{ID}.csv", 'a')
                        file.writelines(dataPipe)
                        file.flush()
                        os.fsync(file.fileno())
                        file.close()
                        #clear stored values
                        dataPipe.clear()
                            
                        print(f"{it}: Write time: " + str(time.time() - start))
                    else:
                        start = time.time() # for timing
                        file = open(f"ResultsUART{ID}.csv", 'a')
                        file.write(f"{time.time()}, -, -, -, -, -, -, -, -, -, -, \n")
                        file.flush()
                        os.fsync(file.fileno())
                        file.close()

                        print(f"{it}: (NO DATA) Write time: " + str(time.time() - start))

        except OSError:
            print("-- OSError in UART --")
            break
        except KeyboardInterrupt:
            print("-- KeyboardInterrupt on UART --")
            break
        except Exception as e:
            print("exception in UART main loop ", e)
            continue
        # no break

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
