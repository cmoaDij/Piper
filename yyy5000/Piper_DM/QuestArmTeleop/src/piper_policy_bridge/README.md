# piper_policy_bridge

ROS Humble 安全桥。LeRobot 推理服务通过本机 ZeroMQ IPC 返回绝对关节目标；本包负责反馈超时、死手心跳、硬限位、50 Hz 单步限幅以及 teleop/policy 仲裁。

```bash
cd ~/Piper_Project/QuestArmTeleop
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_policy_bridge
source install/setup.bash

ros2 run piper_policy_bridge piper_command_mux
```

真机不要单独运行 bridge，也不使用 shadow；按本文后面的
`PiperLeRobot/scripts/run_live_policy.sh --actuate` 状态机启动。

部署时先停止数据采集器，再用序列号启动两路只读 RGB：

```bash
ros2 launch piper_policy_bridge piper_dual_realsense.launch.py \
  wrist_serial_no:=801212071416 global_serial_no:=211622069098
```

输出话题分别为 `/camera/wrist_camera/color/image_raw` 和
`/camera/global_camera/color/image_raw`。部署相机直接发布模型输入所需的
`320x240 RGB8 @ 30 Hz`，避免先通过 DDS 传输两路 `640x480` 原始图像；bridge 对
其他分辨率仍保留兼容缩放。

安装或移动相机后，在有桌面显示的本机终端使用预览模式：

```bash
ros2 launch piper_policy_bridge piper_dual_realsense.launch.py \
  wrist_serial_no:=801212071416 global_serial_no:=211622069098 \
  preview:=true
```

`preview:=false`（默认）只发布图像话题，供策略部署使用，不打开窗口。

由 `PiperQuestTacDataset` 训练的 `quest-tac-v1` ACT 检查点使用原生
`640x480` 两路图像和 7 维关节位置。启动相机时必须显式选择对应 profile：

```bash
ros2 launch piper_policy_bridge piper_dual_realsense.launch.py \
  wrist_serial_no:=801212071416 global_serial_no:=211622069098 \
  color_profile:=640x480x30
```

运行 live-policy 脚本时还要加入 `--observation-profile quest-tac-v1`。默认的
`legacy-v2`、`320x240` 行为保持不变；IPC 会校验状态维度和图像形状，错误组合会
被推理服务拒绝。

bridge 只发布 `/policy/joint_states`；mux 是 `/control/joint_states` 的唯一发布者。没有持续的 `/policy/enabled` 10 Hz 正心跳时不会输出。

实机启动由 `PiperLeRobot/scripts/run_live_policy.sh --actuate` 强制执行回零状态机。
`piper_home_guard` 在控制门关闭时调用 `/move_home`，并要求六个机械臂关节在默认
`±0.05 rad` 范围内连续稳定 `0.5 s`；策略启动使用位置验收，因为 Piper v158 的
静止速度反馈存在粗量化。`/move_home` 同时将 AGX 夹爪命令到 `0.07 m`；现有采集
数据中完全张开的反馈最大值约为 `0.06692 m`，因此 guard 验证反馈不低于
`0.060 m`，而不要求反馈等于命令值。`Empty` 服务返回本身不算回零成功。
若夹爪在物理完全闭合时反馈零点失效，必须以
`gripper_calibration_enabled:=true` 单次重启控制器，并使用
`piper_gripper_calibrate_closed --confirm-physically-closed`。该命令只调用受保护的
一次性标定服务；服务要求外部控制门关闭、六轴稳定全零、夹爪已使能且无故障、
反馈位于闭合附近。标定成功后仍须用 `piper_home_guard` 打开至 `0.07 m` 并验证
反馈至少 `0.060 m`。正常启动时标定服务默认不存在。
`piper_policy_reset` 显式
清空 LeRobot action queue。actuate bridge 使用
`--startup-hold` 时，即使 heartbeat 存在也不会推理或发布，直到脚本在操作者输入
`RUN` 后调用 `/policy/start_control`。脚本收到首条新策略命令并再次确认机械臂仍在
零位后，才打开 `/control_enable` 并启用绝对 command cursor 与跟踪保护。不要在
标准流程中手工调用这些内部服务。

当前 bridge 订阅腕部与全局 RGB，并通过 `PIPER_LEROBOT_V3` IPC 分别发送 wrist/global
图像、各自时间戳、图像中点观测时间和冻结 state 时间。服务返回完整动作块及其
原始观测时间；ROS 侧把每个动作绑定到绝对 10 Hz 主机时隙，丢弃响应到达时已经
过期的前缀。ACT 时间集成只融合指向同一个绝对执行时隙的预测，权重与原版 ACT
的 `exp(-0.01*i)` 规则一致；非集成队列在耗尽前一时隙预取下一块。
bridge 在每次 10 Hz 请求前检查两路图像都新鲜且互相对齐。
任一路缺失、超时或超过允许 skew 都必须清空策略目标；禁止复制另一视角或沿用
旧帧补齐。每个图像回调同时冻结当时最新的反馈 state；双相机配对后，以两幅图像
时间戳中点为基准，从两个冻结快照中选择最近 state，且要求 state/image 偏差不超过
`0.05 s`。这与训练数据采集器的选择规则一致；未对齐的 camera/state 组合不会送入
策略。控制插值以最后发布的绝对命令为 cursor，不会从存在物理延迟的实时 feedback
重复追赶目标；实时 feedback 只用于初始化和跟踪保护，不使用冻结的观测 state。
任一机械臂关节的 command/feedback 差值超过 `0.25 rad`，或夹爪差值超过
`0.02 m`，bridge 会锁存故障、停止发布并请求关闭 `/control_enable`。启动只验证
夹爪张开位置，不读取夹爪 enable 状态字段。双视角检查点不能通过旧 IPC 使用，
并保持所有运动安全门不变。
