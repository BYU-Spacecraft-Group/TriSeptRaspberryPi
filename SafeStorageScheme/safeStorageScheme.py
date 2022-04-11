"""
Collects data and writes to a csv, 
writes after a determined cycle count to avoid slowdown from writing too often
"""
import RPi.GPIO as gp
import time
from random import random

gp.setmode(gp.BCM)


ID = time.strftime("%H%M%S", time.localtime()) + str(int(random() * 100))
print(ID)
STORING_GAP = 10 # how many cycles to wait until you take the time to save
START_TIME = time.time()

it = 0
data = []

try:
	#main loop
	while True:
		#collecting data
		dataIn = "" #string to save as a line
		
		dataIn += str(time.time()) + ", "

		dataIn += str(random()) + ", " 
	
		data.append(dataIn + "\n")

		#saving data
		it += 1
		if(it % STORING_GAP == 0):
			start = time.time()
			file = open(f"Results{ID}.csv", 'a')
			file.writelines(data)
			file.close()
			#clear stored values
			data.clear()
				
			print(f"{it}: Write time: " + str(time.time() - start))

		time.sleep(0.013)

#clean up for debugging 
except KeyboardInterrupt:
	print("--Interrupted--")
	gp.cleanup()

