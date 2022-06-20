# Multithreaded
&emsp; A similar scheme to [SortedCode](/SortedCode/) with the difference being that the different communication protocols are separated into threads instead of files. Final version is [mCFinal.py](/Multithreaded/mCFinal.py)
 
&emsp; I strongly suggest using the script [testMulti](/Multithreaded/testMulti.sh) for any sort of testing as it handles killing the processes for you. Because of multithreading and built-in retries, quitting it using ctrl+c can be dicey but should still work, eventually.

#### Current Timings (based on reading counts from 10 second test file):
I2C: ~30 Hz
SPI: ~300 Hz
UART: needs testing