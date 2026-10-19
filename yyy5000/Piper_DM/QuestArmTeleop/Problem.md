## piper的 TCP 坐标系与手柄的坐标系对应不是很正确

颜色 轴 方向
🔴 红色 X轴 前后方向
🟢 绿色 Y轴 左右方向
🔵 蓝色 Z轴 上下方向
        Z (蓝)
        ↑
        |
        |
        o──────→ X (红)
       /
      /
 Y (绿)

        Z (蓝)
        ↑
        |
        |     Yaw ↺
        |
        O────────→ X (红)
       /
      /
 Y (绿)

Roll  : 绕 X 红轴旋转
Pitch : 绕 Y 绿轴旋转
Yaw   : 绕 Z 蓝轴旋转

问题原因：机械臂选择错误，应该运行 piper 而不是 piper x
固件版本：Piper 固件 V1.5.8，joint2、joint3 的 DH 坐标系相差 2°，旧固件应使用 piper_description_old.urdf，新固件使用 piper_description.urdf
