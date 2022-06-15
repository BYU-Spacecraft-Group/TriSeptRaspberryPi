#!/bin/bash

python mCFinal.py > outputmCFinal.txt &

sleep 10

kill $(pgrep -f 'python mCFinal.py')
