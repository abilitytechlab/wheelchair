#!/bin/bash

# CAMERA
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
source install/local_setup.bash;
ros2 launch depthai_ros_driver camera.launch.py;
exec /bin/bash'

# LIDAR
gnome-terminal -- /bin/bash -c 'sudo chmod 777 /dev/ttyUSB0;
source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
source install/local_setup.bash;
ros2 launch sllidar_ros2 view_sllidar_s2_launch.py;
echo "Launching Lidar Package..."; 
exec /bin/bash'

# MICRO ROS
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/microros_ws;
source install/local_setup.bash;
echo "Launching micro_ros Agent..."; 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0;
exec /bin/bash'

# OBSTACLE DETECTION
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
colcon build --packages-select obstacle_detection;
source install/local_setup.bash;
ros2 run obstacle_detection obstacle_detector;
echo "Launching ROS Package..."; 
exec /bin/bash'

# LIDAR IMAGE
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
colcon build --packages-select lidar_visualization;
source install/local_setup.bash;
ros2 run lidar_visualization lidarImagePublisher;
echo "Launching ROS Package..."; 
exec /bin/bash'

# empty terminal
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
exec /bin/bash'

#  FLASK SERVER
gnome-terminal -- /bin/bash -c 'cd /home/moro/ros2web/flask-app; 
echo "Starting Flask Server..."; 
flask --app latency_test run --host 0.0.0.0; 
exec /bin/bash'


# ROS BRIDGE WEB
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
echo "Starting ROSBridge Server..."; 
ros2 launch rosbridge_server rosbridge_websocket_launch.xml; 
exec /bin/bash'

# STATE CONTROLLER
gnome-terminal -- /bin/bash -c 'source /opt/ros/humble/setup.bash; 
cd /home/moro/humble_ws;
colcon build --packages-select sequence_controller;
source install/local_setup.bash;
echo "Launching ROS Package..."; 
ros2 run sequence_controller state;
exec /bin/bash'
