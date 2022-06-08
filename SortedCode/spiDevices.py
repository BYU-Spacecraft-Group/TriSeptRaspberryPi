"""
Devices in this file: adc (vibration sensor)
"""

while True: # retry block
    try:
        import time
        import os
        from random import random

        #setup file ID using system clock and random()
        ID = "SPI" + time.strftime("%H%M%S", time.localtime()) + str(int(random() * 100))
        print(ID)
        STORING_GAP = 10 # how many cycles to wait until you take the time to save

        it = 0
        dataPipe = []

        dataPipe.append("SystemTime, ADC, \n")
        #adc
        import glob
        import csv
        import Adafruit_GPIO.SPI as SPI
        import Adafruit_MCP3008

        SPI_PORT    = 0
        SPI_DEVICE  = 0
        mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

    except:
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
                file = open(f"Results{ID}.csv", 'a')
                file.writelines(dataPipe)
                file.flush()
                os.fsync(file.fileno())
                file.close()
                #clear stored values
                dataPipe.clear()
                    
                print(f"{it}: Write time: " + str(time.time() - start))

    except OSError:
        print("-- OSError on SPI --")
        break
    except:
        continue
    # no break