# 配置网络连接

danny@PC:~/QuestArmTeleop$ adb tcpip 5555
error: no devices/emulators found
danny@PC:~/QuestArmTeleop$ adb tcpip 5555
restarting in TCP mode port: 5555
danny@PC:~/QuestArmTeleop$ adb shell ip route
192.168.5.0/24 dev wlan0 proto kernel scope link src 192.168.5.32 
danny@PC:~/QuestArmTeleop$ adb connect 192.168.5.32:5555
connected to 192.168.5.32:5555

# 启动应用程序

adb shell am start -n com.rail.oculus.teleop/.MainActivity
结果：Starting: Intent { cmp=com.rail.oculus.teleop/.MainActivity }

# Piper

激活单 CAN：bash ~/QuestArmTeleop/src/agx_arm_ros/scripts/can_activate.sh 
启动遥操机械臂:
source ~/QuestArmTeleop/install/setup.bash 

conda activate vt

ros2 launch  oculus_reader teleop_single_piper.launch.py

# Piper 有线连接运行
  source ~/QuestArmTeleop/install/setup.bash
  ros2 run oculus_reader check_quest_usb.sh
  ros2 launch oculus_reader teleop_single_piper_usb.launch.py

  只使用 rviz：
  ros2 launch oculus_reader teleop_single_piper_rviz_usb.launch.py
