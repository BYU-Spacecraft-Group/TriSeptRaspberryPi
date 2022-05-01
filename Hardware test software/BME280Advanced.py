import board
import adafruit_bme280.advanced as adafruit_bme280
import time

i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

bme280.sea_level_pressure = 1013.25
bme280.standby_period = adafruit_bme280.STANDBY_TC_0_5

#RATE = bme280.OVERSCAN_X16
bme280.overscan_pressure = adafruit_bme280.OVERSCAN_DISABLE
bme280.overscan_temperature = adafruit_bme280.OVERSCAN_DISABLE
bme280.overscan_humidity = adafruit_bme280.OVERSCAN_DISABLE

while True:
    start = time.time()
    print(f"Temp: {bme280.temperature}")
    print(f"Hum: {bme280.relative_humidity}")
    print(f"Pres: {bme280.pressure}")
    print("Sense time: " + str(time.time() - start))
    time.sleep(0.1)



