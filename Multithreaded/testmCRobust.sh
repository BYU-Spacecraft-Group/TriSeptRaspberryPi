#!/bin/bash

python mCRobust.py > outputmCRobust.txt &

sleep 10

kill $(pgrep -f 'python mCRobust.py')
