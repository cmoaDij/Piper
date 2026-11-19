# 当前 Piper-X 采集工程的机械臂延迟测量方案

**采集代码接入（2026-11-17）：** `config/collection_gopro.yaml` 已加载本轮七通道结果，在唯一 SDK 出口记录各分量实际调用起点及预计反馈响应时间，离线生成分段内的响应参考，并导出 `tracking.reference` / `tracking.valid` 诊断字段。标准训练动作仍表示 SDK 发送目标，反馈原始时间不变；未引入实时预测运动或机器人绝对观测延迟修正。启动检查工况一致性，录制嵌入标定快照，旧数据不受后来改测影响。完整启动、手柄操作和审核导出命令见 [本轮采集流程](collect_gopro_with_latency.md)。以下准备记录按历史时点保留。

**最新现场结果（2026-11-17）：纯机器人模式已完成六关节与夹爪测量。** joint1–joint6 的有效跟随滞后单轮估计依次约 **65、80、85、90、85、80 ms**，夹爪约 **50 ms**。七条记录完整，无报告采集错误或队列丢样；全程不启动相机和触觉。工况为 `move_js`、50% 速度参数、50 Hz 控制，关节各自 ±2°，夹爪 65–70 mm。5 ms 为拟合搜索步长，不代表测量精度。汇总标定文件是 `data/calibration/piper_tracking_latency.json`，完整报告及曲线在 `data/calibration/robot_latency_plan/robot_only/results/`。机器人绝对观测延迟仍未知，未改写原始时间戳或自动应用动作补偿。下面保留准备过程与此前首次 joint6 试录的历史说明。

2026-11-17：操作员已确认 TCP、左右触觉身份及无接触基线检查完成，随后已在现场启动 `can0`（1 Mbps）。被动监听 5 秒，收到各组约 1000 个反馈包，关节组周期中位数约 5.000 ms。已启动唯一采集驱动，`auto_enable=false`，GoPro 和左右触觉均有数据；尚未开启跟随或执行测试运动。反馈周期不是动作延迟。

后续现场执行状态：第 6 关节测试已经执行并封盘，`episode_0001.h5` 的有效跟随滞后单次估计约 **80 ms**，相关系数约 0.9519，搜索步长 5 ms，峰值未触及边界。文件完整、无报告丢样；其余关节和夹爪未测，未应用生产补偿。该 Episode 的视频抽查为 **No Signal**，只保留机器人延迟分析，不进入训练或图像动作对齐验证。操作员随后恢复信号，相同 worker/路径/模式下重新只读取像，已目视确认实景（5 秒 173 帧，接收速率约 36.09 fps，不能保证实际 60 fps）。完整记录位于 `data/calibration/robot_latency_plan/joint6_episode_0001_report.md`。

初次运动预览发现 joint2 约 −2.29°、joint3 约 +2.48°，超出 URDF 和 SDK 配置限位（joint2 ≥ 0°、joint3 ≤ 0°），入口拒绝执行。现场随后手动调整姿态；再次只读预览得到 joint2 约 +10.404°、joint3 约 −23.793°，joint6 测试范围 −6.816° 至 −2.816°，限位检查已通过。尚需现场执行测试命令取得实际延迟；不能靠放宽限位或自动回零跳过初始问题。

## 测量对象

| 项目 | 起点与终点 | 能否直接作为图像/反馈时间补偿 |
| --- | --- | --- |
| CAN 反馈新鲜度 | 主机收到反馈包的间隔、包间时间差、缓存年龄 | 不能；不是绝对观测延迟 |
| 关节有效跟随延迟 | 实际 SDK 关节指令调用 → 主机收到相应关节反馈 | 包含控制动态和反馈链路，只能作为该工况的有效跟随估计 |
| 夹爪有效跟随延迟 | 实际 SDK 开度指令调用 → 主机收到开度反馈 | 单独测量，不能套用关节延迟 |
| 机器人绝对观测延迟 | 真实物理关节状态发生 → 主机收到对应反馈 | 需要独立、同钟且延迟已知的参考，测到后才能用于反馈时间修正 |

当前 GoPro 的 QR 估计中位数约 144.4 ms、P95 约 243.5 ms，只描述图像链路估计；不包含机械臂执行延迟，也不足以充当几毫秒精度的机器人反馈时延参考。

## 为什么采用正常采集录制后离线估计

当前 Data_Collect 在唯一 SDK 控制出口记录 `robot_command/cmd_call_begin_ns`、`target_joint_pos` 和 `target_gripper_pos`，同时记录 `robot_feedback/joint_available_host_ts_ns`、`joint_pos`、`gripper_available_host_ts_ns` 和 `gripper_pos`。这些时间均在同一主机 monotonic 时钟下，可用于估计真实采集路径的跟随滞后。

UMI 目录内现有 `calibrate_piper_tracking_latency.py` 和 `calibrate_piper_gripper_latency.py` 依赖本检出不存在的 `data_collect.cli.umi_piper_sdk`、`data_collect.control.piper_sdk_session` 及旧 setup 配置。当前 `collection.yaml` 也不包含这些旧脚本需要的部分字段。因此它们的 `--help` 可用不代表真机执行可用，本方案不直接运行它们的 `--execute` 命令，不补造旧配置，也不建立第二个 SDK 控制出口。

## 1. 环境、CAN 与被动反馈检查

先停止其他遥操、策略和标定控制程序，确认 `can0` 对应当前 Piper-X。下面前三条 CAN 设置会改变接口状态，不发机械臂位置命令：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

机械臂上电后，用已有的纯 SocketCAN 脚本监听 30 秒，不连接控制 SDK、不使能、不运动：

```bash
python /home/yanyu/Documents/Piper-main/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_feedback_latency.py \
  --can can0 \
  --seconds 30 \
  --output "data/calibration/robot_latency_plan/feedback_$(date +%Y%m%d_%H%M%S)"
```

检查关节反馈包是否实际收到、更新周期和组间时间差是否有明显异常或长时间断流。没有反馈时先排查上电、接口和线缆；不能把无数据解释为 0 ms 延迟。这里输出的新鲜度和更新周期不是编码器的物理采样延迟。

## 2. 同步已完成的现场核验状态

已依据操作员本次确认，将现场 JSON 的 `verified` 同步为 true，并更新备注，记录 TCP、左 `X2631410810` / 右 `X2631410795` 和无接触基线核验完成；机械臂及夹爪有效跟随延迟仍为待测，机器人绝对观测延迟未知。配置一致性及 GoPro 延迟文件加载检查已通过，没有填入机械臂延迟数值。

```bash
python - <<'PY'
import json
from pathlib import Path
profile = json.loads(Path('data/calibration/calibration_gopro_v1.json').read_text())
print('verified:', profile['verified'])
print(profile['tracking_lag_notes'])
PY
```

这个状态记录不表示机械臂延迟已经测过，也不自动执行任何机械臂动作。

## 3. 用正式控制链路录制标定轨迹

### 六关节和夹爪：纯机器人记录模式

测量指令到反馈响应不需要相机或触觉。新增 `data_collect.application.collector --robot-latency-only`：继续使用原 CollectionDriver 和唯一 SDK 出口、同一命令观测器及机器人反馈线程，关闭全部相机和触觉采集，只要求机器人反馈满足原有新鲜度检查。输出采用独立 `piper_robot_latency_v1` schema，标记 `not_for_training=true`，只包含 `robot_command` 和 `robot_feedback`，不探测视频编码器，不要求相机延迟文件或图像基线。

现场配置是 `data/calibration/robot_latency_plan/robot_only.yaml`，保持 Piper-X、TCP、`move_js`、50% 速度参数、50 Hz 控制和 200 Hz 反馈轮询。数据单独保存到 `data/calibration/robot_latency_plan/robot_only/raw/`。相机和触觉即使关闭或未接入，也不阻止机器人延迟录制。

助手已将待命采集进程切换为该模式，未启动跟随或生成运动目标。下面不带参数的命令只预览全部范围：

```bash
bash /home/yanyu/Documents/Piper-main/Data_Collect/data/calibration/robot_latency_plan/run_all_robot_latency.sh
```

确认工具、六轴的小幅运动范围及急停、夹爪空载后，现场执行：

```bash
bash /home/yanyu/Documents/Piper-main/Data_Collect/data/calibration/robot_latency_plan/run_all_robot_latency.sh --execute
```

入口先预览全部六轴限位，再顺序测试 joint1–joint6（各自起始目标附近 ±2°，每轴约 16.75 秒），最后测试夹爪（65–70 mm，约 16.75 秒）。夹爪起始反馈为 64.8 mm，所选测试范围大于该已核验开度，不闭合到零。总计约两分钟，产生七条独立 HDF5 和七份执行清单；一项失败立即停止后续项目。每项均先确认录制，再改变测试目标；结束后恢复该项起始目标，保持电机使能。Ctrl+C 中断时停止后续目标并封盘，不执行自动返回或卸力。

该命令要求已有且唯一的机器人标定采集进程；不会自己建立第二个 SDK 连接。若重启了采集进程，先使用下面的原驱动启动命令（不要与已有驱动同时运行）：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.application.collector --robot-latency-only \
  --config data/calibration/robot_latency_plan/robot_only.yaml --ros-args \
  -p can_port:=can0 -p arm_type:=piper_x -p effector_type:=agx_gripper \
  -p pub_rate:=30 -p fast_mode:=true -p auto_enable:=false \
  -p speed_percent:=50 -p home_speed_percent:=5 -p home_position_tolerance:=0.03 \
  -p 'tcp_offset:=[0.0, 0.0, 0.195, 0.0, 0.0, 0.0]' \
  -p control_enabled:=true -p external_command_watchdog_enabled:=false
```

只预览已经在当前真机上通过，全程未发布运动目标。针对机器人记录/反馈新鲜度及原遥操/暂停行为的 24 项离线测试通过；隔离 ROS 域内也验证了夹爪范围、保持机械臂目标、先录制后改变开度，以及录制失败禁止夹爪测试。这些软件检查不是七通道的真机延迟结果；结果须在现场执行后另行分析。

本机已准备配置：`data/calibration/robot_latency_plan/collection.yaml`。它从当前配置复制，保留 Piper-X、TCP、图像、触觉、`fast_mode=true`、`speed_percent=50`、`control_hz=50`；仅关闭 B 后自动回零，修改任务说明和输出目录。标定数据单独保存到 `data/calibration/robot_latency_plan/collection/raw/`，不混入正常示教目录。

本次固定使用 GoPro 方案及实测的稳定 by-id 路径，不再让此标定会话自动切换至 RealSense。

### 现场直接执行关节测试

已准备 `data/calibration/robot_latency_plan/run_joint_steps.py`，作为现有 `/teleop/*` 接口的唯一输入。它不连接控制 SDK、不直接发送 CAN，也不绕过原驱动控制出口。**不要同时启动下文 Quest 方案或其他控制发布者。** 本次由助手运行的采集进程已关闭自动使能及 Quest 输入；只读预览不会使能或发布目标：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python data/calibration/robot_latency_plan/run_joint_steps.py --joint 6 --amplitude-deg 2
```

现场已调整姿态并确认工具、腕部线缆的运动范围及急停后，执行：

```bash
python data/calibration/robot_latency_plan/run_joint_steps.py --joint 6 --amplitude-deg 2 --execute
```

这会使能机械臂，以现场当前姿态为中心，仅给 joint6 发送 ±2° 内的非等周期阶跃，约 17 秒；其余关节和当前夹爪开度保持不变。开始时先对齐保持目标并请求录制，确认 HDF5 已开始后才发送非零测试偏移。录制失败、反馈超出原采集新鲜度限制、状态断流、输入被其他发布者占用或控制门关闭时，停止测试输入并封盘，不自动回零或卸力。正常结束把目标恢复到测试起始姿态，保留电机使能；Ctrl+C 中断时保留最后一个小幅目标，不执行返回运动。物理急停仍由现场操作员掌握。

每轮终端打印实际 `Recording: ...` 路径及 `Run record: ...` 清单，清单记录测试关节、起始姿态、幅度、是否完整结束与故障。命令时间最终以 HDF5 中实际 SDK 调用时间为准，不能用终端打印时间代替。joint6 测完后仅代表该关节和工况；其他关节应在各自范围已核对后逐个选择 `--joint 1` 至 `--joint 5`，夹爪另测。

### 保留的 Quest 手动测试方案

下面是另一种输入方式，必须在上述直接关节测试和其采集进程停止后运行，不能并行占用机械臂。

确认工作区和工具运动范围、线缆及急停后运行。**下面命令会连接并使能机械臂；它不会自动生成或执行一条预设运动轨迹。** 初始化时触觉保持无接触：

```bash
./start.sh --terminal can_port:=can0 \
  config:=/home/yanyu/Documents/Piper-main/Data_Collect/data/calibration/robot_latency_plan/collection.yaml
```

通过正常 Quest 遥操录制：

1. 第一条约 20–30 秒，A 对齐，连续按住 RG，在已确认可运动的小范围内缓慢移动、改变方向并穿插不等间隔停顿。加入不同方向的小幅平移和转动，让需要估计的关节发生可辨识变化；避免只做匀速直线或等周期来回摆动。B 结束封盘。
2. 第二条约 15–20 秒，机械臂保持稳定，连续按住 RG，用食指扳机使空夹爪在两个已确认不会互相挤压触觉垫的开度之间切换，穿插不等间隔停顿，B 结束封盘。夹爪响应与机械臂关节分别分析。
3. 每条尽量使用一个连续 RG 段；需要暂停时不要强行保持操作，暂停后续段在分析时显式选择。每个关节不必强行在一条轨迹里测全；没有充分运动的通道保留为“未测量”。

保持与正式任务相同的控制模式、速度、工具安装和代表性负载。低速试运行只用于确认动作范围，其结果不能直接代替正式模式下的延迟；负载和运动速度也会改变有效跟随结果。当前实际选用 SDK `move_js`，不能用另一个 `move_j` 轨迹模式的测量值直接代替。

等待封盘和程序退出，再运行离线分析；不要另起旧 SDK 标定脚本与遥操争用控制出口。

## 4. 离线分析

已准备本机离线入口：`data/calibration/robot_latency_plan/analyze_episode.py`。该文件只读取 HDF5，复用 UMI 现有 `estimate_signal_delay`，不连接 CAN，也不修改原始文件或生产补偿。

第一条文件通常为 `episode_0001.h5`；以终端实际 `Recording ...` 路径为准，已有文件时编号会递增：

```bash
python data/calibration/robot_latency_plan/analyze_episode.py \
  --input data/calibration/robot_latency_plan/collection/raw/episode_0001.h5 \
  --segment 0 \
  --max-lag-ms 500 \
  --step-ms 5 \
  --output "data/calibration/robot_latency_plan/joints_$(date +%Y%m%d_%H%M%S).json"
```

夹爪轨迹用同一入口，替换为其实际 HDF5 路径和新的输出文件。若按 RG 暂停又恢复，同一 Episode 的后续片段使用 `--segment 1`、`--segment 2`，不跨暂停拼接拟合。

分析将控制指令按零阶保持重建，在实际反馈时间轴上寻找最佳正延迟；分别输出 joint1..joint6 和 gripper 的延迟、相关曲线与搜索边界标记。重复读取 SDK 缓存得到的相同反馈时间戳先去重，避免把轮询频率当成真实反馈更新率。

判断结果时查看：目标和反馈是否都有实际变化、曲线是否存在可辨识峰值、峰值是否触及搜索边界，以及不同代表性轨迹是否给出接近的结论。`delay_ms=null` 表示没有有效估计；`at_search_boundary=true` 需要重新检查搜索范围和轨迹，不能直接当作已标定数值。不同关节可能不同，不强行求一个全机械臂平均值。500 ms 是现有 UMI 拟合工具使用的搜索范围，5 ms 是原跟随标定采用的搜索步长，不是延迟验收阈值，也不代表测量精度。

入口已用已知 80 ms 延迟的合成 HDF5 验证，检查了已知正延迟恢复、静止通道不产生虚假数值及重复缓存时间戳去重。本次未采集真实机械臂延迟，不能使用该合成值作为本机结果。

## 5. 数值如何使用

原始指令和反馈时间戳始终保留；本入口只产生独立测量报告。当前 Data_Collect 尚未接入机械臂动作延迟补偿字段，因此生成报告不等于补偿已经生效。

GoPro 继续按 `图像接收时间 - GoPro 接收延迟估计` 修正图像时间。机械臂有效跟随延迟包含控制/执行动态、SDK 缓存采样和反馈传输，不能再从机器人反馈时间戳中直接减去，也不能直接加到 GoPro 的 144.4 ms 上。

例如，主机在 10.1444 秒收到的图像，按当前中位数估计对应约 10.0000 秒的场景；应当与 10.0000 秒附近的实际机器人状态对齐，而不是与 10.1444 秒的新状态对齐。若机器人绝对观测延迟尚未知，按反馈接收时间匹配仍保留这部分未校准误差，不能宣称已经精确到物理采样时刻。未来若独立确认某模式的动作执行延迟为 100 ms，需要在 10.000 秒达到的目标可计划在 9.900 秒发送；100 ms 在此只是举例，并非本机测量结果。有效跟随滞后受运动形状和负载影响，也不是适用于所有动作的精确常数。

对需要按期达到目标的策略，动作延迟通常用于提前调度未来目标；UMI 上游实现分别使用机器人和夹爪动作延迟安排发送时间，见 [UMI 动作调度实现](https://github.com/real-stanford/universal_manipulation_interface/blob/main/umi/real_world/bimanual_umi_env.py)。这是后续使用方式的参考，不能据此宣称本机已经接入同样逻辑。

如果目标是精确估计机器人状态的物理采样时间，需要独立编码器或已校准光学系统测量同一物理关节角，并映射到同一主机 monotonic 时钟。此时可使用已有只读 `calibrate_piper_observation_latency.py record/analyze` 对照估计绝对观测延迟；时钟偏移和参考链路延迟必须独立已知。仅凭控制指令、CAN 包周期或当前波动较大的 GoPro QR 数值无法分离这项延迟。
