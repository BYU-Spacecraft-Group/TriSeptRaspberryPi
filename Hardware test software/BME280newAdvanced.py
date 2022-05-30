import board
import adafruit_bme280.advanced as adafruit_bme280
import time

i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

bme280.sea_level_pressure = 1013.25

bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X1

start = time.time()

temperature = bme280.temperature
relative_humidity = bme280.relative_humidity
pressure = bme280.pressure
altitude = bme280.altitude

print()
print("Sense Time: " + str(time.time() - start))
print("Temp: " + str(temperature))
print("Humidity: " + str(relative_humidity))
print("Pressure: " + str(pressure))
print("Altitude: " + str(altitude))
print()

