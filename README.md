# BB8-S26
Welcome to the BB8 project! This repo has all the files that we used to develop the BB8 as well as a ton of old files from previous groups.

To test out the project, a few things need to be done first.

1. Plug the Pi on the main BB8 (Not the test one in the chassis) to a power source
2. You will need to connect the Pi to a monitor to find out its IP address and connect it to eduroam or whichever network you want to use.
3. The robot has two modes right now: The PS5 controller mode and the head-body communication mode that reads april tags.

PS5 Controller Mode:
1. Pair the PS5 controller with the body Pi (The one in the sphere). You can do this from the terminal or GUI
2. Plug in the 24V LiPo battery to the robot. This connects to everything except the Pi (which is a Pi 4B - definitely could use the upgrade to a 5, which should be in the materials).
3. Plug in the 3.7V with boost converter into the Pi and disconnect anything external from it.
4. Connect the shell so that it encapsulates the entire robot.
5. SSH into the body Pi. The user is "bb8camera" and password is "password" (I know right).
6. Navigate to /'BB8 Files'/'Body Files' 
7. Run the python files "PS5_Controller.py"
8. This will give you a menu of options to select. The two main ones are options 5 and 6. Choose 5 if you want the option that will balance the head and balance the body (Although the pendulum body balancer doesn't work well since the PID values are 0 for now). Option 6 gives the user full control over all the motors and functions.

Head-Body Communication:
1. Instead of pairing the PS5 Controller, go to the file "Expo_Demo.py" in 'BB8 Files'/'Expo Demo'. Replace the IP address in it with the new body IP address.
2. You'll want to power up the head's Pi. This can be found on the picar, which we used for testing and then placed inside the head for Demo purposes.
3. 
