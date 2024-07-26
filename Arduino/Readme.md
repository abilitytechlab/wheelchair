This folder contains the ino scripts for arduino that were developed in this work

The two main scripts are:

1. loop_controller_pid -> Script for running the step motion control
2. wheelchair_continuous -> Script for the continuous mode

These are dependent on the utility libraries which are custom and provided in the libraries folder:
imu_util and PIDController

The following libraries have to be installed from Arduino Libraries:

Adafruit BNO055

Adafruit BusIO

Adafruit Unified Sensor


except the micro_ros_arduino library. The tutorial linked below describes the steps for installing and using micro_ros_arduino
micro_ros_arduino -> https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/
