# Wheelchair
The wheelchair repository

## Intro
This project contains sources for the adaptive wheelchair control system - eventually offering a user assistive options (lane warning, adaptive cruise control) all the way up to fully autonomous (indoor) navigation.

The first series of sources can be found in our [archive](https://github.com/abilitytechlab/archive/tree/main/sjoerd) 

## system overview
The design consists of a wheelchair-bridge system: a small low-level arduino board with digital potentiometers mimicking the joystick input for a (modern) wheelchair. Next up is an arduino teensy running MicroROS, which uses emergency switches and lidar for obstacle detection as main safety layer.

### bridge
This folder will contain the design files of the circuit emulating a joystick. The pinout (9-pin D-sub) is given in the following figure: 

![9-pol d-sub connection for joystick](https://github.com/abilitytechlab/wheelchair/blob/main/bridge/joystick.jpeg)


## Arduino

The Arduino Folder consist of the ino scripts for loop controller and some other scripts for calibrating the imu and testing the pid controller. It also include Arduino library files which were developed in this work. 

## src
The src folder should be extracted in a ROS 2 workspace. It consists of the packages required for the step-motion and continuous-motion modes
