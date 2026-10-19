# Piper 数据采集

这是仓库唯一的数据采集工程，涵盖 Quest 遥操作、Piper-X 控制、双 RealSense / 双 Tac-X 采集、质检审核及 LeRobot v3 导出。全部业务代码、ROS 包、SDK、APK 和模型资源均位于本目录，运行时不依赖其他项目目录。

## 目录与模块

```text
Data_Collect/
├── config/                    # 设备、运动阈值、依赖约束、标定模板
├── launch/collect.launch.py    # ROS 启动入口
├── data_collect/
│   ├── cli/                   # GUI、审核、导出、延迟标定命令
│   ├── application/           # A/B 生命周期与采集流程编排
│   ├── common/                # 路径、配置、执行器、诊断
│   ├── control/               # 目标组合、运动保护、FK
│   ├── sensors/               # 相机、触觉、独立采集进程
│   ├── recording/             # Episode、HDF5、视频读写
│   ├── calibration/           # 相对延迟测量与补偿工具
│   ├── dataset/               # 因果对齐、自动质检、LeRobot 导出
│   └── web/                   # 工作台、审核服务及本地静态资源
├── ros_ws/src/                # Quest/IK、机械臂驱动、消息、模型
├── vendor/                    # 机械臂/触觉 SDK、RealSense wheels
├── tests/                     # 单元测试、离线流程测试、合成数据工具
├── docs/                      # 架构、设备准备、数据格式、验收记录
├── data/                      # 本地标定、录制、审核、导出及日志
├── .runtime/                  # 本地重建的 capture/export 环境（不提交）
├── env.sh
├── environment.yml
└── pyproject.toml
```

ROS 包中同时保留旧标准 Piper 与 Piper-X 模型。当前实机采集配置仍使用 Piper-X；保留旧模型不会自动改变机械臂型号。通用驱动和 SDK 的其他型号/效应器模块仍有导入依赖，暂不裁剪。

## 常用命令

在 `Data_Collect/` 中运行，首次使用先完成环境重建：

```bash
source env.sh
python3 -m data_collect.cli.gui --demo --no-browser --port 8767
python3 -m data_collect.cli.gui
ros2 launch ./launch/collect.launch.py can_port:=can0
python3 -m data_collect.cli.review
python3 -m data_collect.cli.calibrate_latency --help
source env.sh export
python3 -m data_collect.cli.export --help
```

工作台和终端 ROS launch 是两种启动方式，不要同时启动。安装本包后也可使用 `piper-collect-gui`、`piper-collect-review`、`piper-collect-export`、`piper-collect-calibrate-latency` 命令。

## 开发与测试

仅做离线开发时，在独立 Python 3.10/3.11 环境中运行：

```bash
python -m pip install -e '.[dev]'
python -m pytest
python -m pip wheel --no-deps --no-build-isolation . -w /tmp/piper-collect-wheels
```

测试使用假设备及临时合成数据，不连接 CAN、Quest、相机或触觉设备，也不代表实际硬件验收。项目资源相对本检出目录定位；部署使用可编辑安装，并按下文构建内部 ROS overlay。请保留 SDK 和第三方静态资源的许可证。

架构说明见 [docs/architecture.md](docs/architecture.md)，现场操作见 [docs/hardware_setup.md](docs/hardware_setup.md)，格式说明见 [docs/data_format.md](docs/data_format.md)，验证范围见 [docs/validation.md](docs/validation.md)。

## 环境与首次安装

Git 克隆仅包含源码和随附资源，不包含 `.runtime/`、真实采集数据或已验证标定。下文原现场的测试和设备记录属于历史记录，不代表本机环境已经安装或实机已经验收。

参考部署环境为 Ubuntu 24.04，系统 ROS 为 Jazzy/Python 3.12；Tac-X SDK 的原生库只支持 Python 3.10/3.11。采集应在本目录 `.runtime/capture` 重建独立的 RoboStack Jazzy/Python 3.11，包括匹配的 rclpy、Pinocchio 3.2.0、CasADi 3.6.7、NumPy 1.26.4、双相机和触觉依赖。系统 ROS 保留原样。

离线转换环境 `.runtime/export` 使用 Python 3.12、LeRobot 0.4.4、NumPy 2.2.6、PyTorch 2.7.1+cpu、TorchVision 0.22.1+cpu、TorchCodec 0.4.0、datasets 4.0.0 和 PyArrow 21。本项目只采集触觉原始图像、离线转换数据，不需要安装 CUDA Toolkit、TensorRT 或 CuPy。SDK 的 CuPy 提示可忽略。

完成下述环境重建后，每个新终端执行：

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh
```

`env.sh` 清理终端自动加载的系统 ROS 路径，再启用独立环境和内部 ROS overlay；不要在其后再次 source `/opt/ros/jazzy/setup.bash`。切换到转换环境使用 `source env.sh export`，切回采集环境使用 `source env.sh`。

如果重建环境，先准备 `.runtime/bin/micromamba`、`.runtime/bin/uv` 和 Google 官方 platform-tools，再执行：

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
.runtime/bin/micromamba create -y --no-rc \
  --root-prefix "$PWD/.runtime/mamba" --prefix "$PWD/.runtime/capture" \
  -f environment.yml
.runtime/bin/uv pip install --python .runtime/capture/bin/python \
  --find-links vendor/wheels -c config/capture-constraints.txt \
  -e '.[hardware]' ./vendor/pyagxarm ./vendor/dm_sdk colcon-notification
source env.sh
cd ros_ws
colcon --log-base ../data/logs/build build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
cd ..
source env.sh
```

必须使用带 CasADi 绑定的 Pinocchio，不能安装 PyPI 上同名的无关 `pinocchio` 包。触觉 SDK 的安装声明已修正为允许 NumPy 1.26.x，以兼容 ROS 所需的 1.26.4；采集和导出依赖分别由 `config/capture-constraints.txt`、`config/export-constraints.txt` 约束。

RoboStack 环境应避免混入系统 ROS，见 [RoboStack 官方安装说明](https://robostack.github.io/GettingStarted.html)。`.runtime` 含本机路径和二进制依赖，移动目录或换机器后应重建，而不是直接复制环境目录。

## 标定与设备

```bash
mkdir -p data/calibration
cp -n config/calibration.example.json data/calibration/calibration_v1.json
```

修改 `config/collection.yaml` 和现场标定文件，核实相机序列号、左右触觉分配、颜色顺序、无接触基线、任务指令和操作员。实际核验后才将标定文件的 `verified` 设置为 true。

模板预填了原现场的 D455 全局相机、D435i 手腕相机和左右 Tac-X 编号，`verified` 默认为 false。必须根据实际设备核对并完成现场标定，不能直接将模板视为已验证配置。真实标定文件不纳入 Git。

实测异常 Episode 的关节与 SDK TCP 表明应使用 **Piper-X** 模型，原标准 Piper 模型不匹配。驱动 `arm_type`、IK 和保存目标 TCP 的 FK 现统一使用 `piper_x_with_gripper_description.urdf` 及 TCP 偏移 `[0,0,0.113,0,0,0]`。回放 603 条真实反馈，位置误差中位数约 0.260 mm、姿态误差中位数约 0.0016°；运动中的异步反馈最大位置误差约 12.9 mm，因此每次运动前及运动中仍检查反馈新鲜度、关节/TCP 时间差和 FK 误差。

## 可视化工作台（推荐）

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh
python3 -m data_collect.cli.gui
```

程序自动打开 **http://127.0.0.1:8766/**。打开界面只启动本机网页和 ROS 状态观察节点，不连接设备 SDK；点击「启动遥操程序」后才启动现有 Quest、IK、机械臂与采集驱动，并按原配置使能机械臂。选择实际 CAN 接口，完成 CAN/USB 准备后再启动。此模式下不需要另外执行 `ros2 launch`。

同一模式和配置的工作台已经运行时，再执行 `python3 -m data_collect.cli.gui` 会直接打开已有界面并返回终端，不创建第二个服务或 ROS 观察节点；原服务继续运行。若端口由其他程序占用，可使用 `python3 -m data_collect.cli.gui --port 8768`。

- 启动后等待四路传感器都有帧率、机械臂反馈持续刷新，再按手柄 **A** 低速回零、对齐参考，开始增量遥操。设备数据过期或触觉基线未就绪时，A 请求会被拒绝，待恢复后重新按 A。GUI 不绕过 A 的确认流程和运动保护。
- 「停止运动 / 取消回零」与 **B** 使用同一个停止信号，保留程序运行；当前录制结束，不自动回零。
- 「停止程序并保存」停止本界面启动的整组进程，等待数据封盘。关闭 GUI 终端（Ctrl-C）也会清理本界面启动的进程；关闭浏览器页面不会停止后端。
- 已有终端采集程序时，界面可以直接观察其 ROS 状态，并禁止重复启动。GUI 不结束由其他终端启动的进程；可通过停止运动按钮发送 B 信号。

界面显示：

- **三维姿态**：加载本目录 Piper-X URDF 及 11 个 STL 网格；实测模型、下发目标骨架、TCP 轨迹、坐标轴与网格。支持鼠标旋转/缩放/平移，以及透视、正视、俯视。
- **六轴关节**：实际角度、SDK 已下发目标角度、估算速度、跟随误差和 URDF 限位；最近 20 秒曲线可切换位置、速度、误差。
- **运行状态**：启动/回零/已对齐/录制/停止/故障、四个 ROS 节点是否可见、Quest 是否有位姿、机械臂使能和当前反馈延迟。
- **采集状态**：四路传感器帧率/数据延迟、当前 Episode、时长、写盘队列用量，以及可筛选的进程日志。完整启动日志保存在 `data/logs/gui/`。
- 日志面板的「复制日志」复制当前视图中的日志和时间，遵循「只看异常」筛选，最多最近 500 条；「清空视图」只清空浏览器中的显示，不删除日志文件。

监测约 10 Hz 更新，传感器延迟显示驱动采样时的帧龄；显示速度来自相邻采样关节角的时间差，仅用于观察，运动保护仍由采集驱动执行。断线时模型变灰并明确标记最后姿态，不继续显示为实时状态。GUI 不读取相机图像，不增加第二个 CAN/SDK 设备实例。

离线预览（所有数值与状态均为模拟，不连接硬件、不写原始数据）：

```bash
python3 -m data_collect.cli.gui --demo --port 8767
```

可用 `--no-browser` 只启动服务，或用 `--port` 指定其他本机端口。GUI 仅绑定 127.0.0.1，沿用当前终端的 ROS_DOMAIN_ID。所有网页与三维资源均在 Data_Collect 内，无需 npm、外部 CDN 或额外安装 Qt/RViz。

## 采集

确认 USB/CAN 映射、工作空间和急停可用，关闭其他机械臂驱动、相机和触觉占用程序。CAN 需先按现场接口配置为 1 Mbit/s；Quest 需授权 USB 调试，ADB 服务需可用。

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh
adb start-server
adb devices
ros2 launch ./launch/collect.launch.py can_port:=can0
```

独立环境的 `ros2` 使用匹配的 Python 3.11。默认启动使能机械臂，正常运动和 A 键回零均使用 5% 固件规划速度。`fast_mode` 默认 false，当前调试采集拒绝 true，使用 `move_j` 规划路径。`auto_enable:=false` 禁用启动使能；B 后默认不回零（`auto_home_on_stop:=false`）。

CAN 未配置时，在其他驱动已退出、接口确认为 can0 的条件下执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Quest 的 USB 规则已安装到本机 `/etc/udev/rules.d/99-piper-quest.rules`，来源为 `config/99-piper-quest.rules`。`adb devices` 必须列出设备且状态为 `device`；空列表需检查连接及头显开发者模式，`unauthorized` 需在头显中允许调试。ADB 已随独立环境配置，不需要另装 Android Studio。

2026-10-18 已确认 Quest `2G0YC5ZH010727` 完成授权，ADB 状态为 `device`；遥操作应用 `com.rail.oculus.teleop` 已安装，主 Activity 可解析。本次检查未启动头显应用或机械臂。后续如换电脑或主机密钥，仍需在头显中完成相应授权，见 [Meta 官方设备设置](https://developers.meta.com/vr/documentation/native/android/mobile-device-setup/)。

- A：先以 5% 规划速度回到六轴关节零位 `[0,0,0,0,0,0]`。实测六轴均在零位 ±0.5° 内持续 0.3 秒后，将**回零完成时**的当前手柄位姿对应到实测零位 TCP，再允许后续增量遥操。回零期间不录制；手柄保持不动时不产生额外运动指令。首次实际运动或夹爪变化前开始 Episode。
- B：回零期间取消回零并发送当前位置保持目标；遥操期间关闭控制门、发送保持目标并结束 Episode。默认不触发新的回零；下一次 A 重新执行完整回零和对齐流程。
- RJ：保留原来的遥操作参考重定位行为。
- Ctrl-C：退出控制与采集进程，等待文件封盘。

没有独立 mux，不订阅策略指令，也没有原来的手动 start/stop 录制服务。IK 六轴目标和 Quest 夹爪目标仅发送到 `/teleop/joint_states`，采集驱动在内部组合成完整七维向量。它是唯一 SDK 控制出口，不接受外部 `/control/joint_states` 输入。

目标必须在本次 A 之后生成，字段完整、有限值、满足 URDF/夹爪范围，且六轴与夹爪均未过期。默认过期门限 250 ms；真实机器人反馈还需满足 25 ms 新鲜度。首条关节目标与实测关节的差值须 ≤3°。后续 IK 输入单步跳变须 ≤5°；遥操下发轨迹每轴限速 **5°/s**、限加速度 **15°/s²**、单步 **≤0.2°**；目标限制在每轴实测位置 **±3°** 内，快速移动手柄时只跟随附近目标，避免积累较远的待执行位置。URDF 限位内的普通快速输入会被限幅并平滑跟随；连续 IK 输入单次跳变超过 5° 仍判为异常并停止。规划位置与实际位置偏差超过 10°、控制周期中断超过 60 ms，或无法同时满足位移和加速度限制时停止下发。夹爪以 A 后首条扳机值和实际开度为参考，变化速度限制为 0.02 m/s。上述软件轨迹限制用于遥操，回零使用固件 5% 速度规划。

回零和遥操均检查真实关节反馈：每轴相邻真实数据包变化超过 **1.5°**，或 **100 ms 内累计绝对变化超过 2°**（约 20°/s；不足完整窗口但至少 80 ms 时按实际时长折算），将停止下发并发送当前位置保持目标；已录制 Episode 标记失败，需退出并检查原因。累计变化按绝对值求和，可检测快速往返抖动；相同时间戳的缓存读取不作为新反馈。反馈中断超过 60 ms 同样拒绝继续运动。

所有阈值均在 `config/collection.yaml` 的 `motion` 中。调整后重启采集入口生效；这些是软件指令限制及反馈异常检测，不能保证故障硬件的实际速度，也不能代替硬件急停。

回零超时 45 秒、反馈不新鲜、未使能、模型误差 >10 mm/5°、关节与 TCP 反馈时间差 >25 ms 时拒绝进入/继续运动；机械臂完成回零后仍超出 URDF 关节限位时也拒绝开启，不自动修改编码器零点或放宽限位。SDK、传感器、写盘或目标超时发生故障时停止下发；已开始的 Episode 标记失败。传感器或写盘故障需重启入口。

IK 使用配置 `ros_ws/src/oculus_reader/config/arm_ik_pose_node.piper.yaml`，位置权重为 20，并在优化问题内加入与发布检查相同的 10 mm / 5° 精度约束。正则项参考实测关节，平滑项参考上次已发布的解；求解失败或发布前校验不通过不会更新上次接受的解。启动后使用静止反馈预热求解器，不发布运动目标。首条输出仍保持实测关节，之后每次 TCP 求解目标最多从上次接受的位姿前进 2 mm / 1°，参数为 `max_tcp_step_m`、`max_tcp_step_rad`。这是输入步幅限制，实际运动仍由下游关节速度、加速度、单步和跟随范围保护决定；不可达目标仍可能被拒绝，不自动放宽精度门限。

目标超时、目标非法或运动/反馈检查拒绝属于本条轨迹失败：关闭控制门、请求保持当前位置，轨迹标为 `complete=false`、`failure_kind=episode_rejected`；B 停止后可以重新按 A，必须重新完成设备新鲜度/静止检查、低速回零和对齐。SDK 指令异常、传感器/采集进程故障、IPC 与写盘错误仍锁定会话，B 或 A 均不能清除。故障淘汰的文件保留且不能进入训练。IK 失败日志包含手柄请求目标、限制后的目标、具体位置/姿态误差、求解器状态、耗时和反馈年龄；控制端的目标超时日志分别列出六轴与夹爪目标年龄。

正常 SIGINT 退出使用幂等 ROS 关闭，先等待回调与采集资源清理；本条轨迹拒绝不会在退出时再次抛为设备故障。真实设备/写盘错误仍返回非零退出码。GUI 即使在主动停止期间也保留子进程异常，不以 ROS 启动器返回 0 判定全部成功。修改后重启工作台和遥操进程生效；当前安装文件及 YAML 为 symlink，无需重新编译。验证见 `data/logs/environment/ik-recovery-validation.md`。

`/data_collect/status` 提供 recording、armed、aligned、homing、teleop_enabled、error、ready_streams 和 FK 误差。`/data_collect/prepare_home` 是 A 键内部准备服务，成功返回本次已确认的零位反馈。原驱动的急停、使能和回零服务保留；手动关闭控制门会终止录制，不能通过服务绕过 A 直接重新开门。软件门控不是硬件制动。

图像按设备频率异步存储：双 RGB 在线编码为 H.264 视频，触觉灰度和基线以单通道 uint8、LZF 无损压缩保存。主机时间为锁内 SDK 读写打点的 monotonic_ns；关节、TCP、夹爪独立记录新包时间，轮询缓存不刷新真实可用时间。SDK 目标由实际调用记录，目标 TCP 在写线程通过 FK 计算。每条 HDF5 包含任务、标定快照、触觉基线、状态、指令及图像时间索引。

机械臂进程独占 CAN，机器人反馈按 200 Hz 轮询、控制按 50 Hz 运行，ROS 展示反馈按 30 Hz 发布，手柄 ROS 执行器固定 2 个线程。FK 和误差计算使用复制后的数据，在 SDK 锁外执行。双 RGB、双触觉及 HDF5 写盘运行在 `spawn` 创建的独立采集进程中；图像不穿过控制进程，机器人记录经有界队列传输。采集进程回传帧时间、基线就绪、队列与故障状态；就绪后状态超过 250 ms 未更新会锁定故障，阻止继续运动。首次设备初始化允许最多 15 秒。

采集和手柄节点共用 `data_collect/common/executor.py` 的双线程执行器。同一个任务尚在排队或执行时不会再次提交；每次线程池调用完成后清理登记，允许异步服务继续执行。遇到重复任务最多等待 0.2 ms 并让出执行时间，继续调度其他回调；回零仍使用独立回调组。退出先取消回零、停止提交并等待回调线程结束，再销毁 ROS 节点。`env.sh` 和 launch 为安装后的脚本提供共享模块的导入路径，当前 symlink 安装无需重新编译；重启遥操进程即可使用修改。

A 先最多等待 1 秒，每 5 ms 检查新鲜反馈、静止状态、四路图像和触觉基线；等待期间不发运动指令，B 可以取消。满足条件后才进入低速回零。日志每 5 秒及故障时输出“时序诊断(ms)”，包括 SDK 取数、锁等待/持有、FK、反馈轮询和控制循环的 p50/p95/p99/最大耗时，以及各反馈年龄和采集进程状态年龄。超时原因包含具体分量、实测延迟和限制。

诊断中的 `executor` 记录 `workers`、`submitted`、`duplicate_suppressed`、`in_flight`、`peak_in_flight`；`duplicate_suppressed` 增长表示成功拦截重复提交，不表示重复执行了业务回调。2026-10-18 修复验证：同步任务去重、真实 ROS 异步服务恢复、异常传播及线程池退出通过；使用生产 A/B 回调与模拟机械臂/Quest，验证等待、回零和遥操阶段的 B 取消/停止。真实双 RGB、实际 IK、模拟机器人与触觉负载运行 30 秒，6004 次反馈间隔最长 7.83 ms，超过 25 ms 为 0 次，约 456 MB 临时录制正常封盘。验证时系统没有 `can0` 和触觉设备，以上不替代真实 CAN、触觉与机械臂运动验收。记录见 `data/logs/environment/executor-repair-*.log` 和 `executor-repair-validation.md`；临时测试脚本及录制已删除。

这版 Tac-X SDK 的 raw 和基线实测为二维 uint8 灰度图。采集保留 H×W 单通道，不重复存储灰度值；审核显示和 LeRobot 导出时才等值展开为三通道。SDK 如返回真实彩色帧则保留全部通道，按配置转为 RGB。首次等待 SDK 初始化最多 10 秒，正常采集后仍保留 2 秒无新帧故障门限。`env.sh` 将数值库线程数设为 1，避免双 SDK 初始化与相机、控制线程争用 CPU。

新录制采用 `piper_raw_v2`，例如：

```text
data/raw/
├── episode_0005.h5             # 触觉、基线、机器人记录和视频逐帧索引
├── episode_0005.global_rgb.mp4 # 全局相机 H.264
└── episode_0005.wrist_rgb.mp4  # 手腕相机 H.264
```

三份文件需要一起保留、移动和备份；视频相对路径及 SHA-256 写入 HDF5。每路 RGB 保存原始 `available_host_ts_ns`、设备 `frame_id`/`device_ts`，以及从 0 连续编号的 `video_frame_index`、`video_pts`。视频按配置帧率保存，PTS 为 `frame_index/fps`；真实采集抖动及帧间隔以 HDF5 的原始主机时间为准，不能用播放器时间替代同步。编码不补帧、不丢帧；失败按采集故障停止，未完整封盘保留 `.partial`。视频先封盘，随后 HDF5 才改为完整文件。

`config/collection.yaml` 的 `storage` 控制写入：默认累计最多 50 ms / 128 项后按数据流批量扩容和追加；触觉帧及基线用 LZF。`video_encoder: auto` 启动时实际试编码双路视频，优先 `h264_nvenc`，不可用时明确记录原因并回退 `libx264`；也可指定其中一种，指定模式失败会拒绝开始采集。每路编码线程默认 1，`video_quality: 18` 对应 NVENC CQ / libx264 CRF，数值较小质量较高，两者数值不代表完全相同的视觉质量。RGB 为有损 H.264 / yuv420p，触觉为逐像素无损。编码及写盘均在独立采集进程，控制进程不编码图像。

自动质检会验证视频哈希、完整解码帧数、PTS 与索引；人工通过和导出也检查视频完整性。旧 `piper_raw_v1` 三通道 HDF5 仍可审核和导出，不会自动转换或删除已有数据。

队列默认上限 2048 项/256 MiB；溢出显式失败，并记录当前字节数、容量及排队项数。现有 `episode_0001.h5`、`episode_0004.h5` 曾发生 `writer_queue_overflow`；新格式的已有数据重放与独立进程持续写入验证见 `data/logs/environment/storage-v2-validation.md`，不替代真实设备联调。`.h5.partial`、`.mp4.partial` 为写入中或异常残留，完整关闭后才改名；`.reserve` 防止编号冲突。开始时保留最新图像、目标和 250 ms 状态预滚动，但训练网格仍使用实际 Episode 起止范围。

## 自动质检与人工审核

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh
python3 -m data_collect.cli.review
```

有原始文件时先自动审计，再启动 http://127.0.0.1:8765 。没有数据时也能打开空页面。新增采集数据后重启审核命令 会重新审计；页面“刷新列表”读取最新报告。运行服务的终端需保持开启，Ctrl-C 退出。

审计使用完整的 20 Hz 网格，不裁头去尾：观测只取过去最新真实样本，动作优先匹配 ±25 ms 窗口，再回退 ZOH。非静止指令回溯超过 100 ms 拒绝。静止需要历史实测关节/夹爪稳定且接近目标。

| 项目 | 绿色 | 整条淘汰红线 |
|---|---|---|
| RGB age | ≤40 ms | >68 ms |
| 触觉 age | ≤35 ms | >55 ms |
| 关节/TCP/夹爪 age | ≤25 ms | >25 ms |
| 左右触觉时差 | ≤15 ms | >25 ms |
| 黄区 | 占比≤2%、不连续 | 超过2%或连续两步 |
| 夹爪变化前后±0.5 s | 全绿、无图像复用 | 黄区或源帧复用 |

默认要求时长 5–30 s 和实测夹爪张开→闭合→张开。接触窗口按目标/实测夹爪累计变化≥1 mm 保守推断，不是力检测。自动通过后仍需在四视角页面人工确认；自动拒绝不能强制放行。原始文件始终保留。

报告、索引和人工决定保存在 `data/audit/`。最终白名单 `approved_episodes.json` 包含源文件、索引及其 SHA-256，按完整 Episode 划分 train/val，默认比例 20%、种子42；只有一条时全为 train。

## LeRobot v3 转换

完成转换环境重建后，切换并运行：

```bash
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh export
python3 -m data_collect.cli.export --manifest data/audit/approved_episodes.json \
  --output data/lerobot/piper_dataset --repo-id local/piper_dataset
```

重建转换环境时执行：

```bash
.runtime/bin/uv venv --python /usr/bin/python3 .runtime/export
.runtime/bin/uv pip install --python .runtime/export/bin/python \
  --index https://download.pytorch.org/whl/cpu \
  torch==2.7.1 torchvision==0.22.1 torchcodec==0.4.0
.runtime/bin/uv pip install --python .runtime/export/bin/python \
  -c config/export-constraints.txt -e '.[export]'
```

不需要连接硬件，不上传 Hub。读取已缓存的对齐索引，检查源文件完整性及哈希后调用官方 LeRobot create/add_frame/save_episode/finalize。状态和动作均为六轴 rad + 夹爪 m；额外保存实际/目标 TCP 的 xyz m + rpy rad + 夹爪 m。双 RGB 为 H.264/avc1，触觉为 PNG 无损 image 特征，任务语言写入 task 元数据。

train 轨迹在前、val 在后，Episode 和全局帧索引连续。`meta/split_episodes.json` 给出训练/验证 Episode 列表，训练时显式选择；`meta/source_episodes.json` 保存来源对应关系。先写 `<output>.partial`，成功封盘后改名；拒绝覆盖已有目录。

## 随附资源来源

- 机器人驱动、Quest/IK 和配置：当前工作目录中已有的运行代码；保留其保护逻辑和相应许可证，采集入口额外实现 A 回零确认、统一模型/TCP 和连续轨迹保护。
- pyAgxArm：官方 https://github.com/agilexrobotics/pyAgxArm ，提交 d90e234ba8e408cfa964b38aa9120b3131fc9f85，版本1.0.0，许可证见 vendor/pyagxarm/LICENSE。
- 网格：官方 https://github.com/agilexrobotics/agx_arm_urdf ，提交680fa925cd00db7e2628563d2edc5540963045de。新版资源将 gripper 独立到 agx_gripper；这里按随附 URDF 的引用路径保存同名网格，不改关节和几何变换。许可证见模型包 LICENSE。
- GUI 三维库：[Three.js 官方仓库](https://github.com/mrdoob/three.js/tree/r170)，固定 r170（0.170.0）；仅随附 module、OrbitControls、STLLoader 和 MIT 许可证，位于 `data_collect/web/static/vendor/`。
- Quest APK：官方 https://github.com/rail-berkeley/oculus_reader ，提交2cf70ab489348f68a25a40e0ce0878de3f587c3a；保存在 oculus_reader/scripts/APK，供原读取器按需安装。
- 触觉 SDK：当前目录已有的 dmrobotics 1.2.6，仅携带 Linux 3.10/3.11 运行文件，不包含重复 build、示例或 Windows 库。
- RealSense：PyPI 的 pyrealsense2 2.56.5.9235，携带 Linux x86_64 的 cp310/cp311 wheel。

2026-10-18 本机环境验证：独立 Jazzy/Python 3.11 中四个 ROS 包编译通过，ROS 消息通信、六轴 IK/IPOPT 求解、环境切换和依赖检查通过；真实双 RealSense 与双 Tac-X 约 4 秒并行取帧测得 RGB 29.4/29.9 Hz、触觉 60.1/60.1 Hz，双触觉基线可读且线程正常退出。审核页面及 API 返回 HTTP 200；离线环境通过 LeRobot 六帧、双 H.264 视频和双 PNG 图像流写入/读取以及 HDF5 往返检查。

验证日志位于 `data/logs/environment/`。Quest 已授权且遥操作 APK 已安装。本次运动修复使用真实历史数据回放、真实 IK 和假硬件验证，未启动或使能真实机械臂。已验证 A 回零确认、回零超时拒绝、B 中断、错误模型阻断、零位静止保持和速度/加速度/单步限制；实际回零、跟随和长时间采集仍需现场验收。按 A 会主动回零，操作前需确保整段回零路径无碰撞。

17:17 启动日志中，驱动只刷新一次、A 请求一直不返回的问题已复现：触觉 SDK 在 ROS 启动后从工作线程首次加载 ONNX Runtime，会使本机整个采集进程卡住。当前在独立采集进程的主线程预加载该依赖，再启动图像设备线程，机械臂进程不加载图像 SDK。GUI 停止时先通知 ROS launch，由其转发 SIGINT，避免重复中断子进程清理；正常退出不再向已关闭的 ROS 上下文发布状态。

本次延迟优化验证使用真实四路传感器和假机械臂：单进程优化后 30 秒反馈轮询间隔最长 46.0 ms，出现 7 次超过 25 ms，因此启用独立采集进程；隔离后 30 秒测量未出现超过 25 ms。最终版本复测中，反馈轮询间隔 P95 约 5.83 ms、最长 23.30 ms；约 505 MB 的临时四路图像/机器人记录正确写入 HDF5 并封盘，录制期间反馈轮询间隔最长 18.13 ms。A 延迟就绪、1 秒超时、B 取消、IPC 不可用、采集进程退出/心跳超时及 GUI 经真实 ROS launch 停止录制均已验证。展示线程改用可中断的单调时钟等待，避免退出时卡在 ROS Rate 中。日志为 `data/logs/environment/control-*.log`；临时脚本和数据不作为项目源码保留。以上短时验证不代表真实 CAN 与机械臂的实时性能保证，真实运动仍需现场验收。
