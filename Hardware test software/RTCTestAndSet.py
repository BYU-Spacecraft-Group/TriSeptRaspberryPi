#This is a basic utility that reads the date and time from the DS3231 RTC, 
#and can set the RTC time from the Raspberry pi system time.
#This software is based on the Adafruit example code for the DS3231 board,
#but has been modified to set the time from the Raspberry pi system time instead
#of a user imput time. To set the time, make sure the pi has a network connection,
#change the if False to if True, and run the program

import time
import board
import adafruit_ds3231
from datetime import datetime

i2c = board.I2C()  # uses board.SCL and board.SDA
rtc = adafruit_ds3231.DS3231(i2c)

# Lookup table for names of days (nicer printing).
days = ("Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday")


# pylint: disable-msg=using-constant-test
if False:  # change to True if you want to set the time!
    now = datetime.now()
    t = time.struct_time((now.year, now.month, now.day, now.hour, now.minute, now.second, now.weekday(), -1, -1))
    # you must set year, mon, date, hour, min, sec and weekday
    # yearday is not supported, isdst can be set but we don't do anything with it at this time
    print("Setting time to:", t)  # uncomment for debugging
    rtc.datetime = t
    print()
# pylint: enable-msg=using-constant-test

# Main loop:
while True:
    t = rtc.datetime
    # print(t)     # uncomment for debugging
    print(
        "The date is {} {}/{}/{}".format(
            days[int(t.tm_wday)], t.tm_mon, t.tm_mday, t.tm_year
        )
    )
    print("The time is {}:{:02}:{:02}".format(t.tm_hour, t.tm_min, t.tm_sec))
    time.sleep(1)  # wait a second
