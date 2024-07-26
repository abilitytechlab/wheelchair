# PWAO CSU
Power Wheelchair Add On Control System Unit (PWAO CSU) consists of a set of different control software layers from User Interface(on MyTobii tablet) to Loop Controller (on Teensy).

The software currently has 3 layers: User Interface Layer, Sequence Control Layer and Loop control layer.

The Loop Control Layer is excecuted on a micro-controller, here a Teensy. This Teensy communicates with the Sequence Control Layer on a computer running ROS 2 using micro-ROS library. The communication between the ROS 2 computer and the User interface Layer happens over WebSocket protocol. For details and to understand the working please refer to https://essay.utwente.nl/100464/

Make sure to have the following dependencies before downloading this software

## Intro
This project contains sources for the adaptive wheelchair control system - eventually offering a user assistive options (lane warning, adaptive cruise control) all the way up to fully autonomous (indoor) navigation.

The first series of sources can be found in our [archive](https://github.com/abilitytechlab/archive/tree/main/sjoerd) 

## system overview
The design consists of a wheelchair-bridge system: a small low-level arduino board with digital potentiometers mimicking the joystick input for a (modern) wheelchair. Next up is an arduino teensy running MicroROS, which uses emergency switches and lidar for obstacle detection as main safety layer.

### bridge
This folder will contain the design files of the circuit emulating a joystick. The pinout (9-pin D-sub) is given in the following figure: 

![9-pol d-sub connection for joystick](https://github.com/abilitytechlab/wheelchair/blob/main/bridge/joystick.jpeg)





# Dependencies

1. For the loop control layer on Teensy refer to the Readme file in Arduino directory: https://github.com/abilitytechlab/wheelchair/tree/main/Arduino 

2. The Sequence Control Layer was implemented on ROS 2 Humble, Ubuntu 22.04. Refer to https://docs.ros.org/en/humble/index.html for installation of ROS 2. Make sure to install the desktop version of ROS 2. Pay attention to the warning on the page that says installation of ROS2 on a freshly installed Ubuntu system can cause removal of some critical system packages. After installation of ROS 2, the following ROS 2 packages can be installed to interface with the Rplidar S2 and Oak-D-Lite camera that was used here:

Slamtec Rplidar: https://github.com/Slamtec/sllidar_ros2

Oak D lite: https://github.com/luxonis/depthai-ros

In addition to sensor packages, you also need to install micro_ROS arduino setup on thr ROS 2 computer. https://micro.ros.org/docs/tutorials/core/first_application_linux/. Follow the steps till creating the micro-ROS agent. 

For communication with the user interface, two ways were used. You can use Integration-Service. The installation instructions are here: https://github.com/eProsima/Integration-Service . Here there was a bug that wasn't fixed. It was not possible to view the image stream. The alternative method that worked was using rosbridge_server. Refer to https://github.com/RobotWebTools/rosbridge_suite for installation and tutorials. Both of these methods require a Flask-server to host the html page. You can look at https://flask.palletsprojects.com/en/3.0.x/installation/ for installation instructions. In this project a virtual environent wasn't used for installation of Flask, you may try it.


3. To connect the user interface device to the ROS 2 computer, open a webbrowser and navigate to the url 'http://' + host + ':5000'. If testing on local host then you can go to localhost:5000

## Arduino

The Arduino Folder consist of the ino scripts for loop controller and some other scripts for calibrating the imu and testing the pid controller. It also include Arduino library files which were developed in this work. The instructions for installing micro_ros_arduino are in the this readme file of this directory

## src
The src folder should be extracted in a ROS 2 workspace. It consists of the packages required for the step-motion and continuous-motion modes

## flask-app
The flask-app folder contains the python script for the flask application, called latency_test.py. The templates folder contain the 2 html required templates called onmouse.html for step-motion mode and onmouse2.html for continuous mode

## Running the demo
Two bash scripts are given to run the two different modes.
1. run_demo_continuous.sh to run the software packages for continuous motion
2. run_demo_pid.sh to run the step-motion mode demo.

You will have to change the 'cd' commands in the bash script before running. Make sure to change the path to your system's path.

To run the demo: 

1. Open Terminal in the folder where you installed the run_demo_x script.
2. Run ``` ./run_demo_x.sh ```
3. Make sure the teensy is connected to the wheelchair when you run this script.
4. Open a browser on the same computer and navigate to localhost:5000 to check if a con-
nection has been established.
5. Check if the teensy has a yellow light continuously or blinking. If blinking then discon-
nect Teensy and connect again. If steady yellow light then it should be connected.
6. Turn on the wheelchair and test. Make sure the wheelchair profile is on profile:’5.1’ or ’4.1’
7. To restart, close all terminal windows. Turn of wheelchair, plug out the teensy usb. Then run the script again(from instruction 1). Connect the white usb back to the teensy and wait for it to output a steady yellow light. Make sure to refresh the browser page and it should work.

Warning: Turn Off The Wheelchair Before Plugging Off The Teensy USB Cable
