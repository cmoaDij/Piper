# Quest 世界坐标与原有相对坐标

`teleop.coordinate_mode` 默认为 `relative`。启动参数或工作台可以选择 `world`，切换须先停止程序并保存当前 Episode。两种模式共用现有 IK、195 mm TCP、机器人限位与唯一 SDK 指令出口；驱动速度、A/RG 的接入保护及 B 的回零流程沿用现有设置。

| 模式 | 头显输入 | 平移方向 | A / RG / RJ |
|---|---|---|---|
| `relative` | 原有 APK 的头显相对手柄位姿 | 原有 `delta_frame: tcp` 算法 | 沿用原有参考位姿逻辑 |
| `world` | 独立 APK 直接读取 OpenXR LOCAL 的右手柄 grip / aim 位姿 | 首次 A 锁定的基座方向，后续不随头显或握姿改变 | 首次 A 定方向；之后只重设手柄和 TCP 起点 |

## 首次准备

USB 连接 Quest 3/3S，允许调试。世界模式使用独立包 `com.piper.questinput`，原有 `com.rail.oculus.teleop` 不覆盖。

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.quest_world --install
python -m data_collect.cli.quest_world --check --duration 10
```

检查时需头显唤醒、应用在前台、右手柄被有效追踪。若头显提示「创建新的游戏边界或者使用原地边界」，先完成选择并返回应用；站在固定位置操作可选原地边界，需要走动则在真实空旷区域设置房间规模边界。程序显示深色空场景；追踪是否有效看终端 `tracked_frames`。检查只读取 Quest，不连接 CAN、不运行 IK、不控制机械臂。头显放桌面操作时同样需要保持追踪与应用前台状态。

## 采集命令

确认现有 CAN、TCP、传感器与现场工作空间已就绪后，终端二选一：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect

# 世界坐标模式，GoPro 采集
./start.sh --terminal config:=/home/yanyu/Documents/Piper-main/Data_Collect/config/collection_gopro.yaml can_port:=can0 coordinate_mode:=world

# 原有相对坐标模式，GoPro 采集
./start.sh --terminal config:=/home/yanyu/Documents/Piper-main/Data_Collect/config/collection_gopro.yaml can_port:=can0 coordinate_mode:=relative
```

工作台启动方式：

```bash
./start.sh --config /home/yanyu/Documents/Piper-main/Data_Collect/config/collection_gopro.yaml --coordinate-mode world
```

也可以只开工作台，选择「坐标模式」后点击启动：

```bash
source env.sh capture
python -m data_collect.cli.gui --config config/collection_gopro.yaml
```

运行中选择器锁定，须点击「停止程序并保存」后切换。不要同时运行终端采集与工作台启动的采集。

## 世界模式操作

1. 等机械臂 TCP 反馈和 Quest 追踪就绪。松开 RG，将右手柄像指针一样**水平指向基座坐标的 +X 方向**，按 A。这里的 +X 是机器人 `base_link` 的前方，先核对它与桌面布置的关系。
2. 终端显示「世界方向已锁定」。水平指向定义 +X，重力向上定义 +Z，左侧由右手坐标系确定为 +Y。不需要做直线移动标定，也不要求精确保持手柄滚转角。
3. 松开再按住 RG 开始跟随和录制。沿刚才选定的前方移动，TCP 目标沿基座 +X 移动；向左对应 +Y，向上对应 +Z，平移倍率仍为 1。
4. 松开 RG 保持、暂停写入。可以换握姿或移动手柄，再按住 RG 时重新接起点，固定方向不变。A/RJ 重设起点也不重新标定方向。
5. B 结束录制，并沿用当前配置以 5% 速度回零。重新按 A 不会改变本次启动锁定的方向。要重新定义方向，可停止并重新启动世界模式。

首次 A 的手柄水平指向精度决定物理方向对齐精度。手柄接近竖直时无法定义水平朝向，软件拒绝这次方向标定。若首次 A 与 RG 同时按住，必须先松开 RG 再按下，保留原有防跳动行为。

## A 没有反应时

A 只对齐参考位姿，成功后机械臂仍保持当前位置。需要随后按住右手柄侧握键 RG 才开始跟随。简洁终端也会显示以下提示：

- `[设备] Quest 世界输入就绪`：头显应用具有输入焦点，右手柄完整空间追踪有效。
- `[遥操] A 已收到`：已读到 A 的按下事件。
- `[设备] 世界方向已锁定`：本次追踪空间的首次方向标定成功。
- `[遥操] A：已对齐当前 TCP`：允许接入；松开后重新按住 RG 跟随。
- `[遥操] A 未接入：…`：显示未激活、失去焦点或缺少空间追踪等原因。恢复追踪后，须松开并重新按 A；持续按住 A 不会自动接入。

`传感器就绪` 只说明相机、触觉和机器人反馈就绪，并不包含 Quest。Quest 未就绪时每 5 秒单独显示原因。启动摘要和纯遥操状态会明确标出「仅遥操（不录制）」，避免将方向测试误认为数据录制。

若右手柄在 Quest 系统菜单可以移动光标、点击，而应用持续报告「右手柄未激活」，先检查头显内是否有未完成的游戏边界或其他系统提示。2026-11-18 操作员现场确认，未处理的「创建新的游戏边界或者使用原地边界」提示阻断了应用接入，选择原地边界后 A 恢复正常。完成设置并关闭菜单返回应用，确认出现「Quest 世界输入就绪」后再将手柄水平指向 +X，重新按 A。此原因与相机是否录制无关。

边界设置入口参考 [Meta 边界设置说明](https://www.meta.com/zh-cn/help/quest/463504908043519/)：右手柄 Meta 键 → 快捷控制/设置 → 环境设置 → 边界。搬动头显到另一位置或未识别已有空间后，应在启动遥操前确认边界；边界不是机械臂的工作空间限制，也不代替手柄追踪。无需关闭边界系统来使用世界坐标模式。

仅检查遥操、不采集传感器数据时，可以使用：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
./start.sh --terminal \
  config:=/home/yanyu/Documents/Piper-main/Data_Collect/config/collection_gopro.yaml \
  can_port:=can0 coordinate_mode:=world teleop_only:=true
```

这条命令会启动机械臂控制，执行前仍需核对现场工作空间与急停。GoPro 未连接只会阻止录制，不应阻止 A 接入。完整采集须接回已标定的绿联采集卡，再去掉 `teleop_only:=true`；不能用其他摄像头的视频节点替代其设备路径。

## 实际坐标计算

头显端发送 `T_WGrip`、`T_WAim`，直接相对于 `XR_REFERENCE_SPACE_TYPE_LOCAL`；没有乘头显位姿的逆矩阵。

首次 A，将 aim 的 `-Z` 指向投影到水平面，归一化得到 `f`。以 `u=(0,1,0)` 为重力向上：

```text
R_BW = [ fᵀ ; (u × f)ᵀ ; uᵀ ]
p_hand_B = R_BW × p_hand_W
R_hand_B = R_BW × R_hand_W

p_target = p_tcp_ref + (p_hand_B - p_hand_ref_B)
R_target = R_hand_B × inverse(R_hand_ref_B) × R_tcp_ref
```

`R_BW` 只在首次 A 或追踪空间重置后的首次 A 计算。TCP 和手柄原点在每次接入时重取，因此无需测量 Quest 原点到机器人基座的平移。这里实现的是固定空间方向下的增量控制，手柄的绝对位置并不直接作为 TCP 绝对位置。

LOCAL 在一次有效追踪空间内锁定方向；跨应用启动或系统重新居中不保证相同。见 [OpenXR 参考空间定义](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrReferenceSpaceType.html)。软件不把它当作永久房间坐标。

## 中断与记录

世界输入采用一个 ADB 读取器，位姿、按钮、有效性、序号和追踪空间代号在同一数据包中发布。原有相对输入不能被当作世界输入使用。

- 丢失 position/orientation 的有效追踪、应用失焦或输入超过 500 ms 未更新：关闭跟随和遥操接入，保留驱动原有保持行为。恢复后需要重新按 A，并松开再按 RG。
- OpenXR LOCAL 重置、应用重新获得焦点或新会话：方向标定失效，需重新将手柄水平指向 +X 后按 A。不会将坐标跳变作为运动目标。
- 数据 HDF5 的 `config.teleop.coordinate_mode` 和 `delta_frame` 保存实际运行模式；世界模式还保存 `reference`，包含 LOCAL 会话、代号、`world_to_base_rotation` 和起始手柄/TCP 参考矩阵。
- 世界模式强制使用 `delta_frame=base`，显式同时设置 `delta_frame:=tcp` 会在启动前报错。相对模式仍允许原来的 `delta_frame` 参数。

实现源码与构建方法见 [vendor/quest_world/README.md](../vendor/quest_world/README.md)。

## 本机验证（2026-11-18）

- Data_Collect 离线测试 154 项通过；`oculus_reader` ROS 包构建通过，工作台脚本语法和模式选择 HTTP 流程通过。
- 独立 APK 已编译并安装到连接的 Meta Quest 3；原有相对坐标 APK 仍在。
- 在隔离 ROS domain 200 中，仅启动世界输入和增量映射节点：649 帧输入全部具有有效完整空间追踪，控制节点正常接收；未接入遥操，末端运动目标数为 0，没有 CAN 驱动。
- 回到应用前台后原生进程保持同一 PID，焦点代号更新；已修复本机实际触发的 SDK Android/OpenXR 生命周期顺序断言退出。
- 只读实机记录收到两次 A，按下时追踪均有效；将对应的 496 帧窗口按原始时间间隔在隔离 ROS domain 201 中回放，生产节点两次完成参考对齐，末尾失去追踪后关闭接入。该回放使用模拟 TCP，没有 CAN 驱动，未生成末端运动目标；不替代实机运动验证。
- GoPro 和左右触觉真实并发采集检查中，恢复应用输入后收到 3 次 A，均有完整追踪且图像流就绪，输入年龄分别为 10.07、11.28、23.07 ms；图像采集进程未报告故障。该检查不启动 ROS/CAN，不创建训练 Episode。
- 本轮未执行机械臂实际运动，首次使用世界模式仍需核对 +X 方向的小幅运动响应。

本地报告：`data/logs/quest_world_ros_check.json`、`data/logs/quest_a_button_check.json`、`data/logs/quest_a_replay_check.json`、`data/logs/quest_capture_check.json`。
