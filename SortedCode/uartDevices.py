"""
Devices in this file: GPS
"""

while True:
    try:
        import time
        import os
        from random import random

        #setup file ID using system clock and random()
        ID = "uart" + time.strftime("%H%M%S", time.localtime()) + str(int(random() * 100))
        print(ID)
        STORING_GAP = 10 # how many cycles to wait until you take the time to save

        it = 0
        dataPipe = []

        dataPipe.append("SystemTime, TimeStamp, Latitude, Longitude, FixQuality, Satellites, Altitude, Knots, TrackAngle, HDilution, HGeoID, \n")

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
    
    except:
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
            print("GPS Time: " + str(time.time() - startgps))

            # --------------------------------------------------------------------
        
            

            print("Data collect time: " + str(time.time() - startCollect))

            #saving data
            #it += 1
            if(it % STORING_GAP == 0 or storageStart - time.time() > 10):
                storageStart = time.time() # for regardless saving
                if len(dataPipe) > 0:
                    start = time.time() # for timing
                    file = open(f"Results{ID}.csv", 'a')
                    file.writelines(dataPipe)
                    file.flush()
                    os.fsync(file.fileno())
                    file.close()
                    #clear stored values
                    dataPipe.clear()
                        
                    print(f"{it}: Write time: " + str(time.time() - start))
                else:
                    start = time.time() # for timing
                    file = open(f"Results{ID}.csv", 'a')
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
    except:
        continue
    # no break
	