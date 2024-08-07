cmake_minimum_required(VERSION 3.8)
project(lidar_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacle_detection REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${obstacle_detection_INCLUDE_DIRS}
)

add_executable(lidarImagePublisher src/lidarImagePublisher.cpp )
  
ament_target_dependencies(lidarImagePublisher 
  rclcpp 
  std_msgs 
  sensor_msgs 
  image_transport 
  cv_bridge 
  message_filters 
  geometry_msgs 
  obstacle_detection )

target_link_libraries(lidarImagePublisher ${OpenCV_LIBRARIES} ${obstacle_detection_LIBRARIES})

install(TARGETS
  lidarImagePublisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
