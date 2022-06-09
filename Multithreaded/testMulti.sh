#!/bin/bash

python multithreadedCode.py > outputMulti.txt &

sleep 10

kill $(pgrep -f 'python multithreadedCode.py')
