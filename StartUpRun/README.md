# How to add the service file:
This will run a python script called trisept.py with the path /home/pi/trisept.py, saving the output files into /home/pi/

I made a script to install it automatically (run it with sudo and run it in the same directory as the file trisept.service file or it won't work). It also might need to be execute enabled with 'chmod u+x autoSetup.sh'

But if it doesn't work do this:
* Copy/Paste the file 'trisept.service' into /lib/systemd/system/
* Then: 'systemctl enable trisept.service'
* And: 'systemctl daemon-reload'

# FINAL SYSTEM CHANGE - not in trisept.service
The final version of the service file is slightly modified with specific paths provided by TriSept and with the removal of the "After=multi-user.target" line to avoid long waits to start saving data after bootup.

## Other useful stuff
* systemctl status trisept.service - will give you info - including if it is running or not
* systemctl stop trisept.service - will stop the service from running until the next reboot

