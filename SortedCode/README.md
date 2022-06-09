# SortedCode
&emsp; This is a new scheme to use seperate python scripts to run the sensors based on what communication protocol they use. Unfortunately most use I2C so that file is not much faster, but it does detach the GPS (UART) so that won't slow it down and it detaches the vibration sensor which can then run so fast I added a 10ms delay (its sense speed is around 0.001 seconds).
 
&emsp; I also included two scripts for easier testing. The script [testSortedCode](/SortedCode/testSortedCode.sh) will run and detach each python script and then kill them after 5 seconds. The script [cleanup](/SortedCode/cleanup.sh) will remove all files starting with "Results" or "output" which is all of the files that are created by testSortedCode.
