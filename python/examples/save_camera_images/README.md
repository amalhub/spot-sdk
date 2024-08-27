# How to run

1. Install the Boston Diynamic Python client libraries. Refer to the root folder README.  
2. Update the image save location path at line 47 (IMAGE_SAVE_LOCATION variable)
3. Power on the robot. 
4. Connect to the robot wifi hotspot
5. Run the python script
```
python wasd_image.py 192.168.80.3 #robot_ip
```
6. It will ask for the authentication username and password. Enter them correctly and it will load the dashboard. 
7. Dis-engage the emergency stop by hitting space bar. 
8. Power on the robot by pressing "p"
9. Stand up the robot by pressing "f"
10. Control the robot movement with w a s d q e keys. 
11. Take images by pressing Shift + I key. Each key pess will capture new set of images and save them as new files.
12. Check the image saved location. Each image set is grouped by timestamp and saved. 
13. Follow the instructions in the dashboad.
