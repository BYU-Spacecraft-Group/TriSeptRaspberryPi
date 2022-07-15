#!/bin/bash

python mCOld.py > outputmCOld.txt &

sleep 10

kill $(pgrep -f 'python mCOld.py')
