# Comp3766 Final Project | Group 12

Barret Technologies 7 Revolute WAM Robotic Arm
-------------------------------------------------


In this project, we implement a modeled version of the Barret Technologies 7 revolute WAM robotic arm within RViz using ROS. We provide modelling of the forward and inverse kinematcs of the arm, allowing for joint positions to set to manipulate the arm's end-effector, and given a desired position and orientation, use numerical inverse kinematics to find joint positions which put the end-effector in this desired configuration. We also implement this in a way which allows both forward and inverse to be used concurrently without having to relaunch the arm.

Launching the Arm
------------------
Firstly, clone the repository. When opened, re-open the folder in the provided dev container, which will open a port which can be opened in your browser. This port is where the RViz window will be. To then launch the arm, run the following commands:

    catkin_make
    source devel/setup.bash
    roslaunch finalProject finalProject.launch

This will open the arm in RViz in the open port. Note the joint state publisher gui may be open behind the main window, if this happens you can just move the main window to the side to bring the gui to the front.

To run the inverse kinematics script, first go to into the file src/finalProject/scripts/inverse_kinematics.py. Here, there is variables for position and orientation of the end-effector in which you can set. Once set, run the following command to calcualte and move according to the inverse kinematics:

    rosrun finalProject inverse_kinematics.py

If there is a  "couldn’t find executable" error, please just run 

    chmod a+x src/finalProject/scripts/*

This should fix this.

Note that when inverse kinematics are active, the joint state publisher gui will not be active. When the script is ended, then the arm takes its joint positions from the gui again. This allows both to be used without having to relaunch the gui.
