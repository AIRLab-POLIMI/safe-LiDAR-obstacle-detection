cmake_minimum_required(VERSION 3.8)
project(challenge2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ch2_msg_srv REQUIRED)
find_package(message_filters REQUIRED)

add_executable(lidar_filtering_node src/lidar_filtering_node.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(lidar_filtering_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  geometry_msgs
  message_filters
)

add_executable(clustering_node src/clustering_node.cpp)

# Link the executable with the necessary libraries
ament_target_dependencies(clustering_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  geometry_msgs
  ch2_msg_srv
)

# Install the executable
install(TARGETS lidar_filtering_node clustering_node
  DESTINATION lib/${PROJECT_NAME})
  
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/polygon_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)
  
# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
