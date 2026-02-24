cmake_minimum_required(VERSION 3.5)
project(ur_rtde_srv)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveGripper.srv"
  "srv/MoveToJoint.srv"
  "srv/MoveToTarget.srv"
  "srv/MoveToTrajectory.srv"
  "srv/SetPinFrame.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# only install launch/ if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

install(
    DIRECTORY resource
    DESTINATION share/${PROJECT_NAME}/
)

install(
    FILES package.xml
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
