#!/bin/bash

python multithreadedCode.py > outputMulti.txt &

sleep 20

kill $(pgrep -f 'python multithreadedCode.py')
