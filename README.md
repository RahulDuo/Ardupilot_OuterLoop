# Ardupilot_OuterLoop
This ROS package is to operate an aerial manipulator (Drone with single ink) in an indoor environment under Vicon cameras.

Codes can be found in the branch name "new_branch".
This Ros package is for running a quadcopter that uses an "Orange Cube" flight controller and has Autopilot firmware installed.
In the src folder, I have kept the executable file.
offborad.py is the main executable file and based on what kind of experiment one is doing we have to import the controller in offboard.py.
1. Controller.py is for setpoint tracking
2. Controller_trajectory.py is for circular and helical trajectory
3. Controller_L_trajectory_tracking_contact.py is for L-shaped trajectory (Used while performing contact experiments)

One can switch into simulation mode(In Gazebo) by changing the variable RUN_TYPE to "Simulation".
