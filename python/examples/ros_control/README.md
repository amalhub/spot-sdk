# How to run
- Refer to the root README and install the dependencies
- Power on the robot. 
- Connect to the robot wifi hotspot
- Setup the username and password as env vairables (or put it in .bashrc)
```
export BOSDYN_CLIENT_USERNAME=user 
export BOSDYN_CLIENT_PASSWORD=password
```
- Run the python script
```
python ros_control.py 192.168.80.3 #robot_ip
```
- Send commands to spot using ros topics and services.