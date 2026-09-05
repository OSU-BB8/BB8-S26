Guide to the Files in this Directory:

1. Body_Communication: This is the file that receives commands from the head. If you want to run any APRILTag stuff, run the Body_Communication file on the Body Pi first.

2. Dead_Reckoning: This file allows the robot to locate how far it traveled from where it started. Currently, it doesn't work very well and isn't integrated into the system as a result. 

3. IMU: This file read the IMU data and sends it off the the Body via I2C.

4. Movement_Functions: This file contains all the movement functions and maps them to various motors or servos. If you want to control the robot, you must use a function from this file.

5. PID: This function has a basic PID class in it. The Movement_Functions uses it in its update_balance function.

6. PS5_Controller: This file allows a user to control the BB8 using a PS5 controller. To use it, simply make sure the Pi reads and is paired to your PS5 controller and run this code. When prompted, selected either option 5 or 6.
