#!/bin/bash

#Used for simultaneously testing the python files in SortedCode

python i2cDevices.py > i2cOutput.txt &
python spiDevices.py > spiOutput.txt &
python uartDevices.py > uartOutput.txt &

sleep 10

kill $(pgrep -f 'python i2cDevices.py')
kill $(pgrep -f 'python spiDevices.py')
kill $(pgrep -f 'python uartDevices.py')
