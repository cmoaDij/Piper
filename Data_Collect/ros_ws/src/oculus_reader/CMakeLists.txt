cmake_minimum_required(VERSION 3.8)
project(oculus_reader)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
install(PROGRAMS scripts/pub_pose.py scripts/pub_delta_pose.py scripts/arm_ik_pose_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/oculus_reader.py scripts/buttons_parser.py scripts/FPS_counter.py
  scripts/transformations.py scripts/button_edge.py scripts/gripper_mapping.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/APK DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
