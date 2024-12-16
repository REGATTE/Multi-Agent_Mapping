cmake_minimum_required(VERSION 3.8)
project(tf_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(tf_relay_tf src/tf.cpp)
ament_target_dependencies(tf_relay_tf rclcpp tf2_msgs geometry_msgs)

add_library(tf_relay_tf_static src/tf_static.cpp)
ament_target_dependencies(tf_relay_tf_static rclcpp tf2_msgs geometry_msgs)

add_executable(tf_relay_main src/main.cpp)
target_link_libraries(tf_relay_main tf_relay_tf tf_relay_tf_static)
ament_target_dependencies(tf_relay_main rclcpp tf2_msgs geometry_msgs)

install(TARGETS
  tf_relay_tf
  tf_relay_tf_static
  tf_relay_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
