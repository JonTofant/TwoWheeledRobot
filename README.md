# Two Wheeled Robot IsaacSim model documentation
This is the documentation for the Two Wheeled Robot model in IsaacSim. It is still a work in progress, but it provides an overview of the model and its features.

## Overview
The Two Wheeled Robot is a simple differential drive robot that is inherently unstable. It is designed to be a testbed for control algorithms that can stabilize the robot and allow it to move around in a simulated environment. The robot has two wheels that are independently driven, allowing it to turn in place and navigate through tight spaces. The bottom wheels are DDSM115 current controlled motors, while the top motors are Xiaomi Cybergear motors that are in MIT mode, which means they are state space angle and velocity controlled. 

## How to use this
The file to edit and use is `control.py`,(TwoWheeledRobot/source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/control.py) which contains the control loop for the robot. The file contains the measured states of the robot and the odometry information. To run the file run
python scripts/zero_agent.py --task Template-Twowheeledrobot-Direct-v0 from inside the TwoWheeledRobot directory. This will start the simulation and run the control loop for the robot. If no GUI shows up you must likely run xhost +local:docker on host machine to allow the docker container to access the display.