cmake_minimum_required(VERSION 3.5)
project(agx_arm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgxArmStatus.msg"
  "msg/GripperStatus.msg"
  "msg/HandStatus.msg"
  "msg/HandCmd.msg"
  "msg/HandPositionTimeCmd.msg"
  "msg/MoveMITMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)



ament_package()