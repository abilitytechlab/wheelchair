# wheelchair
The wheelchair repository

## intro
This project contains sources for the adaptive wheelchair control system - eventually offering a user assistive options (lane warning, adaptive cruise control) all the way up to fully autonomous (indoor) navigation.

The first series of sources can be found in our [archive](https://github.com/abilitytechlab/archive/tree/main/sjoerd) 

## system overview
The design consists of a wheelchair-bridge system: a small low-level arduino board with digital potentiometers mimicking the joystick input for a (modern) wheelchair. Next up is an arduino teensy running MicroROS, which uses emergency switches and lidar for obstacle detection as main safety layer.


