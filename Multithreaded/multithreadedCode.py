#!/user/bin/python

import threading
import time

## imports
while True:
    try:
        pass
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