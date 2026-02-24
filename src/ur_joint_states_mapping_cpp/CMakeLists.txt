cmake_minimum_required(VERSION 3.5)
project(ur_joint_states_mapping_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mapping
  src/mapping.cpp
)

ament_target_dependencies(mapping
  rclcpp
  sensor_msgs
)

install(
  TARGETS mapping
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
