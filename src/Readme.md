The packages that were developed in this work are included in this folder. This src folder is a ROS 2 src directory and so must be extracted in a ROS2 workspace.

The following packages are developed currently:
1. sequence_controller
2. lidar_visualization
3. obstacle_detection
4. error_handler

## Sequence Controller
The two main ROS 2 source files are:
1. state_controller.cpp : For step motion mode with pid, run this node
2. state_controller_v2.cpp : For continuous motion mode, run this ROS 2 node

## obstacle_detection

For the obstacle detection node to work, the sllidar_ros2 node by Slamtec https://github.com/Slamtec/sllidar_ros2, must be running.
It consists for obstacle_detector node, which is responsible for publishing the status of obstacle (1 or 0) present in the a given region.
The LidarUtility.cpp is a utility class that provides methods which can be used in multiple nodes.

## lidar_visualization
This packages contains the lidarImagePublisher.cpp file which overlays the information about the obstacle status from obstacle_detection node on to a camera image.
For this node to work, both the obstacle_detection and a camera node must be running

## error_handler
The error_handler is not complete and must be tested and improved upon. Currently, it subscribes to some sensor topic and checks if they are publishing. If a sensor node is not publishing messages to it's topic for 2 seconds, the error_handlers ouputs a error message on the terminal


