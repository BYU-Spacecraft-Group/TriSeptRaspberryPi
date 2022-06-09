#!/bin/bash

#Used for simultaneously testing the python files in SortedCode

python i2cDevices.py > outputI2C.txt &
python spiDevices.py > outputSPI.txt &
python uartDevices.py > outputUART.txt &

sleep 10

kill $(pgrep -f 'python i2cDevices.py')
kill $(pgrep -f 'python spiDevices.py')
kill $(pgrep -f 'python uartDevices.py')
