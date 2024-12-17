cmake_minimum_required(VERSION 3.8)
project(tf_republisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(tf_republisher_node src/tf_republisher_node.cpp)
ament_target_dependencies(tf_republisher_node rclcpp tf2_msgs)

install(TARGETS
  tf_republisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
