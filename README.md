# TriSeptRaspberryPi
Code for the TriSept launch for the Raspberry Pi team.

## [Combined Code](/CombinedCode/)
Drafts of final code, combining sensor reading code into a single file and saving it with SafeStorageScheme.

## [Hardware Test Software](/Hardware%20test%20software/)
Basic code to get the sensors working individually.

## [Multithreaded](/Multithreaded/)
Takes the code from SortedCode and combines it into one file with multithreading for each communication protocol.

## [SafeStorageScheme](/SafeStorageScheme/)
Template for saving data as an CSV, avoiding data loss and speeding up write to drive.

## [SortedCode](/SortedCode/)
A seperated approach to the final code, splitting it into files that should be able to be run simultaneously based on the communication protocol they use.  

## [StartUpRun](/StartUpRun/)
Files and instructions for setting up the systemd service that will launch the python script.

