#!/bin/bash

python BYUTriSept.py > output.txt &

sleep 10

kill $(pgrep -f 'python BYUTriSept.py')
