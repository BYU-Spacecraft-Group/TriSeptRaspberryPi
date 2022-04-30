 import time
 import os
 from random import random


ID = time.strftime("%H%M%S", time.localtime()) + str(int(random() * 100))
print(ID)
STORING_GAP = 10 # how many cycles to wait until you take the time to save
START_TIME = time.time()

it = 0
data = []

try:
	#main loop
	while True:
		print(it)
		#collecting data
		dataIn = "" #string to save as a line
		
	
		data.append(dataIn + "\n")

		#saving data
		it += 1
		if(it % STORING_GAP == 0):
			start = time.time()
			file = open(f"Results{ID}.csv", 'a')
			file.writelines(data)
			file.flush()
			os.fsync(file.fileno())
			file.close()
			#clear stored values
			data.clear()
				
			#print(f"{it}: Write time: " + str(time.time() - start))

		#time.sleep(0.013)

#clean up for debugging 
except KeyboardInterrupt:
	print("--Interrupted--")
#prcesses error from disk being full
except OSError:
	print("--OSError--")
