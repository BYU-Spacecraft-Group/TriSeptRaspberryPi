import ms5803py
import time

s = ms5803py.MS5803()
while True:
    start = time.time()
    press, temp = s.read(pressure_osr=512)
    print(str(press) + ", " + str(temp))
    print("Read time: " + str(time.time() - start))