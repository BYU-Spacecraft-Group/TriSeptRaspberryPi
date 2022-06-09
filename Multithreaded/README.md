# Multithreaded
&emsp; A similar scheme to [SortedCode](/SortedCode/) with the difference beeing that the different communication protocol are seperated into threads instead of files.

&emsp; I strongly suggest using the script [testMulti](/Multithreaded/testMulti.sh) for any sort of testing as it handles killing the processes for you. Because of multithreading and built in retries quitting it using ctrl+c can be dicey but should still work, eventually.