cmake_minimum_required(VERSION 3.8)
project(oculus_reader)

# 1. 查找 ament# 核心包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 2. 查找依赖项（主要用于在构建时检查环境）
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pub_delta_pose test/test_pub_delta_pose.py)
  ament_add_pytest_test(
    test_usb_launch_contract
    test/test_usb_launch_contract.py
  )
  ament_add_pytest_test(
    test_ik_result_contract
    test/test_ik_result_contract.py
  )
  ament_add_pytest_test(
    test_ik_safety_contract
    test/test_ik_safety_contract.py
  )
endif()


# 3. 安装 Python 脚本 (使它们成为可执行文件)
# 假设你的脚本直接放在包的根目录或 scripts 目录下
install(PROGRAMS
  scripts/buttons_parser.py
  scripts/FPS_counter.py
  scripts/install.py
  scripts/oculus_reader.py
  scripts/pub_pose.py
  scripts/pub_delta_pose.py
  scripts/arm_ik_pose_node.py
  scripts/transformations.py
  scripts/check_quest_usb.sh
  scripts/realsense_video_recorder.py

  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/button_edge.py
  scripts/gripper_mapping.py
  scripts/ik_result_contract.py
  scripts/ik_safety_contract.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 如果你有非节点类的 Python 模块 (如 tools.py, piper_control.py 作为库使用)
# 建议将它们放在包名命名的文件夹下，并使用 ament_python_install_package
# ament_python_install_package(${PROJECT_NAME})

# 5. 安装其他资源（如 launch 文件, urdf 等）
install(DIRECTORY
  launch
  scripts
  config
  DESTINATION share/${PROJECT_NAME}
)

# 6. 打包并导出
ament_package()
