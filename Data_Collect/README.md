# Piper 数据采集

这是仓库唯一的数据采集工程，涵盖 Quest 遥操作、Piper-X 控制、RealSense / GoPro 自动选择、双 Tac-X 采集、质检审核及 LeRobot v3 导出。全部业务代码、ROS 包、SDK、APK 和模型资源均位于本目录，正常采集运行时不依赖其他项目目录。

## 目录与模块

```text
Data_Collect/
├── config/                    # 设备、运动阈值、依赖约束、标定模板
├── launch/collect.launch.py    # ROS 启动入口
├── data_collect/
│   ├── cli/                   # GUI、审核、导出、延迟标定命令
│   ├── application/           # A/B 生命周期与采集流程编排
│   ├── common/                # 路径、配置、执行器、诊断
│   ├── control/               # 目标组合、SDK 调用观察、FK
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
./start.sh                              # 自动加载环境、打开工作台并启动遥操
./start.sh --can-port can1                # 选择 CAN 接口
./start.sh --demo --no-browser --port 8767 # 离线演示，不需要 capture/ROS 环境
./start.sh --terminal can_port:=can0      # 无工作台的终端模式
./start.sh --terminal can_port:=can0 teleop_only:=true # 仅遥操，不启动相机、触觉或录制
source env.sh                           # 在当前终端运行审核/标定工具时加载
python3 -m data_collect.cli.review
python3 -m data_collect.cli.calibrate_latency --help
python3 -m data_collect.cli.cameras        # 只读识别相机，不连接机械臂或打开视频流
source env.sh export
python3 -m data_collect.cli.export --help
```

`start.sh` 自动加载 `env.sh`、内部 ROS overlay 并启动 ADB，无需每次手动 source。首次环境安装、ROS 构建、设备授权及 CAN 接口配置仍按下文准备。重复运行会复用相同工作台及其进程。工作台和终端模式不要同时启动。安装本包后也可使用 `piper-collect-gui`、`piper-collect-review`、`piper-collect-export`、`piper-collect-calibrate-latency` 命令。

## 开发与测试

仅做离线开发时，在独立 Python 3.10/3.11 环境中运行：

```bash
python -m pip install -e '.[dev]'
python -m pytest
python -m pip wheel --no-deps --no-build-isolation . -w /tmp/piper-collect-wheels
```

测试使用假设备及临时合成数据，不连接 CAN、Quest、相机或触觉设备，也不代表实际硬件验收。项目资源相对本检出目录定位；部署使用可编辑安装，并按下文构建内部 ROS overlay。请保留 SDK 和第三方静态资源的许可证。

架构说明见 [docs/architecture.md](docs/architecture.md)，现场操作见 [docs/hardware_setup.md](docs/hardware_setup.md)，格式说明见 [docs/data_format.md](docs/data_format.md)，验证范围见 [docs/validation.md](docs/validation.md)。

相机自动选择、GoPro 首次延迟标定及图像/动作对齐方法见 [docs/cameras.md](docs/cameras.md)。默认优先使用已配置的完整双 RealSense 组合；否则使用单台手腕 GoPro 的 HDMI/UVC 采集方案。首次核验 GoPro 设备与延迟后，后续连接任一方案仍使用同一启动命令。

## 环境与首次安装

Git 克隆仅包含源码和随附资源，不包含 `.runtime/`、真实采集数据或已验证标定。下文原现场的测试和设备记录属于历史记录，不代表本机环境已经安装或实机已经验收。

若提示 `环境尚未安装：…/.runtime/capture`，仅遥操可先在工程目录执行 `./setup_teleop.sh`。脚本准备 micromamba、独立 Python 3.11 / ROS / IK 环境和机械臂/Quest Python 依赖，构建内部 ROS 包并检查导入；不启动硬件、不安装 RealSense 或 Tac-X SDK。完成后执行 `./start.sh --terminal can_port:=can0 teleop_only:=true`。系统需已有 `curl`、`tar`、C/C++ 编译工具与 `adb`。若之后需要完整图像采集，再补装下述硬件依赖并核验现场标定。

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

当前驱动、IK、记录目标 TCP 的 FK 以及标定模板统一使用 **Piper-X** 模型与 TCP 偏移 `[0,0,0.195,0,0,0]`（195 mm）。旧 113 mm 配置下的历史反馈回放误差不代表新 TCP 已完成实机标定；已有现场标定文件中的 TCP 也需更新并重新核验。FK 误差仅用于记录、显示和离线分析，不参与遥操运动门控。

当前 Piper-X joint6 的 URDF 限位已与官方对齐为约 ±170°（`[-2.9670597,2.9670597]` rad），SDK 默认范围相同；IK 与网页范围从同一模型加载。重启遥操和工作台后生效。

## 可视化工作台（推荐）

```bash
cd /path/to/Piper-main/Data_Collect
./start.sh
```

`./start.sh` 自动打开 **http://127.0.0.1:8766/** 并启动 Quest、IK、机械臂和采集驱动，按配置使能机械臂。使用 `./start.sh --can-port can1` 指定接口。只希望打开监控界面时，仍可在 `source env.sh` 后运行 `python3 -m data_collect.cli.gui`，再点击「启动遥操程序」。

同一模式和配置的工作台已经运行时，再执行 `python3 -m data_collect.cli.gui` 会直接打开已有界面并返回终端，不创建第二个服务或 ROS 观察节点；原服务继续运行。若端口由其他程序占用，可使用 `python3 -m data_collect.cli.gui --port 8768`。

- 按手柄 **A**：将当前手柄位姿与当前机械臂 TCP 对齐，进入待跟随状态，不回零。按住右手把手侧面的握持扳机 **RG（中指）** 才跟随；松开时冻结最后实际下发的关节目标，重新按住时重设手柄参考，将新增量接到保持目标，不补走暂停期间的位移或转动。暂停不再把负载下的反馈静差反复写成新目标，伺服仍可能完成原目标的短暂收敛。
- Quest 原始轴转换与 QuestArmTeleop 一致，手柄局部修正为 `teleop.ros_to_arm_rpy=[-1.5708,0,-1.5708]`（rad），平移倍率 `[1.0,1.0,1.0]`：手柄移动 10 mm 对应 10 mm 目标 TCP 增量。默认 `teleop.delta_frame: tcp`：按 A 记录当前手柄和当前 TCP，以 `T_tcp0 * inv(T_hand0) * T_hand` 映射增量，使用参考程序的 TCP 局部运动方式。
- `delta_frame:=base` 可切换至上一轮的基座增量方式。终端参数 `ros_to_arm_roll/pitch/yaw` 与参考程序同名，可覆盖 YAML；它们修正手柄局部轴，不是基座或网页坐标旋转。修改后重启并按 A；启动日志打印实际模型、TCP 和映射模式，A/RJ 时打印参考矩阵。FK、网页和手柄坐标的具体区别见 [方向分析](docs/teleop_comparison.md)。
- 手柄 **B**：停止遥操与当前录制，默认延迟 0.5 秒请求回零；回零完成后下一次 A 重新锁定当前参考。配置 `teleop.auto_home_on_stop: false` 可关闭 B 自动回零。
- 界面「停止遥操」关闭活动遥操并结束录制，不发起回零。B 已安排的回零可在发出前通过 A 取消。
- 「停止程序并保存」停止本界面启动的整组进程，等待数据封盘。关闭 GUI 终端（Ctrl-C）也会清理本界面启动的进程；关闭浏览器页面不会停止后端。
- 已有终端采集程序时，界面可以直接观察其 ROS 状态，并禁止重复启动。GUI 不结束由其他终端启动的进程；可通过「停止遥操」按钮发送关闭遥操信号；该界面操作不请求回零。

界面显示：

- **三维姿态**：加载本目录 Piper-X URDF 及 11 个 STL 网格；实测模型、下发目标骨架、TCP 轨迹、坐标轴与网格。支持鼠标旋转/缩放/平移，以及透视、正视、俯视。
- **六轴关节**：实际角度、SDK 已下发目标角度、估算速度、跟随误差和 URDF 限位；最近 20 秒曲线可切换位置、速度、误差。
- **运行状态**：启动/回零/已对齐/录制/停止/故障、四个 ROS 节点是否可见、Quest 是否有位姿、机械臂使能和当前反馈延迟。
- **采集状态**：四路传感器帧率/数据延迟、当前 Episode、时长、写盘队列用量，以及可筛选的进程日志。完整启动日志保存在 `data/logs/gui/`。
- 日志面板的「复制日志」复制当前视图中的日志和时间，遵循「只看异常」筛选，最多最近 500 条；「清空视图」只清空浏览器中的显示，不删除日志文件。

监测约 10 Hz 更新，传感器延迟显示驱动采样时的帧龄；显示速度来自相邻采样关节角的时间差，仅用于观察，当前界面只显示反馈，不执行额外运动保护。断线时模型变灰并明确标记最后姿态，不继续显示为实时状态。GUI 不读取相机图像，不增加第二个 CAN/SDK 设备实例。

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

独立环境的 `ros2` 使用匹配的 Python 3.11。遥操运动设置位于 `config/collection.yaml` 的 `teleop`，默认与 `QuestArmTeleop` Piper-X 启动项一致：`fast_mode: true`、`speed_percent: 50`、回零速度 5%、回零位置容差 0.03 rad。终端模式仍可用 `fast_mode:=false`、`speed_percent:=...`、`auto_home_on_stop:=false` 覆盖。

CAN 未配置时，在其他驱动已退出、接口确认为 can0 的条件下执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Quest 的 USB 规则已安装到本机 `/etc/udev/rules.d/99-piper-quest.rules`，来源为 `config/99-piper-quest.rules`。`adb devices` 必须列出设备且状态为 `device`；空列表需检查连接及头显开发者模式，`unauthorized` 需在头显中允许调试。ADB 已随独立环境配置，不需要另装 Android Studio。

2026-10-18 已确认 Quest `2G0YC5ZH010727` 完成授权，ADB 状态为 `device`；遥操作应用 `com.rail.oculus.teleop` 已安装，主 Activity 可解析。本次检查未启动头显应用或机械臂。后续如换电脑或主机密钥，仍需在头显中完成相应授权，见 [Meta 官方设备设置](https://developers.meta.com/vr/documentation/native/android/mobile-device-setup/)。

- A：当前手柄与当前 TCP 同时重建参考，不下发运动目标或切换到快速关节控制。如果 RG 已握住，先松开再按住；首次跟随等待新参考的完整 IK 结果，之后才下发和录制。A/B 均为按键上升沿切换。
- B：关闭遥操、结束 Episode，默认在 0.5 秒后以 `move_j` 请求六轴零位，即 `joint1..joint6=0 rad`，并沿用原驱动到零位后张开夹爪的行为；尚未按 A 或手柄没有位姿时也能请求回零。正常遥操使用 `move_js`。回零进行中拒绝 A，尚未发出的回零可以由 A 取消。
- RJ：活动遥操期间重新定位当前手柄/TCP 参考，不回零。
- 右把手握持扳机 RG：按住跟随并写入，松开冻结最后已下发目标并暂停所有模态写入；再次按住续写同一 Episode，B 才结束封盘。A 对齐后等待第一次 RG 再开始录制。恢复首帧重复保持目标，后续增量从同一目标继续；忽略上一段跟随尚在求解/传输的 IK 结果。
- 右食指扳机 rightTrig：首次 RG 启动后直接控制 0～70 mm 夹爪开度，松开张开、按下闭合；之后松开 RG 时仍可操作夹爪。
- Ctrl-C：退出控制与采集进程，等待文件封盘。

机械臂驱动仍是唯一 SDK 控制出口，不订阅策略指令。有限关节与夹爪字段按参考 mux 的方式聚合成七维目标，不增加字段年龄、首条跳变、速度、加速度、步幅、目标领先、跟随误差或实际反馈运动限制。驱动外部指令看门狗关闭，与参考遥操启动链一致。IK 保留 URDF 关节位置约束；删除 TCP 2 mm/1° 步进、10 mm/5° 硬精度约束、首条保持和反馈年龄门控。没有新增碰撞拦截，默认碰撞检查关闭。

录制和运动解耦。标定、图像新鲜度、触觉基线、写盘和采集进程故障只限制录制，不关闭遥操；界面会明确显示「录制异常（遥操可继续）」。录制准入、离线质检和审核阈值保留，不能把它们理解成运动保护。故障轨迹仍保留且不能进入训练。

`/data_collect/status` 分别提供 `error` 与 `recording_error`。删除 A 键的 `/data_collect/prepare_home` 服务；原驱动的急停、使能、控制门及回零服务保留。正常退出等待回调和采集资源清理，数据故障仍以非零退出码报告。

双 RGB 在线编码为 H.264 视频，触觉原始图像和基线无损存储；机器人反馈 200 Hz 轮询、完整控制目标 50 Hz 下发、ROS 展示 30 Hz。采集与写盘在独立进程中；采集进程心跳故障不再触发运动门控。SDK 读写打点、真实包时间戳、FK 误差保留用于记录、显示和离线分析。

当前采用“松开 RG 暂停写入、按住续写”：两路 RGB、两路触觉、机器人反馈和指令同时暂停接收新记录，传感器继续运行，已入队数据正常落盘。恢复时补一份最新观测用于起点对齐，不回填整段暂停，文件与 Episode 编号保持不变；界面显示暂停状态，有效录制时长在暂停期间停止增长。松开 RG 时夹爪仍能操作，但其暂停期间的动作不写入数据；要记录夹爪过程需按住 RG。

HDF5 保留原始主机时间戳，`recording_intervals_host_ts_ns` 保存有效区间，各模态的 `recording_segment` 保存分段编号。审核只在各有效区间内建立 20 Hz 网格，导出跳过暂停并保留 `recording_segment`、`source_host_ts_ns` 元数据；长动作窗口采样需按分段边界处理。详见 [采集架构与暂停片段](docs/architecture.md#暂停片段的数据采集选择)。

IK 的剩余算法差异与参数见 [与 QuestArmTeleop 的对比](docs/teleop_comparison.md)；195 mm TCP 下的真实离线求解结果及已发现的零位边界问题见 [精度与速度实测](docs/ik_accuracy_speed_195mm.md)。

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

默认要求有效录制时长 5–30 s。抓取是否完成由人工视频审核确认，不再用固定夹爪开度或张开→闭合→张开循环作为通过条件。接触窗口按目标/实测夹爪累计变化≥1 mm 保守推断，不是力检测。自动通过后仍需在审核页面人工确认；自动拒绝不能强制放行。原始文件始终保留。

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

- 机器人驱动、Quest/IK 和配置：当前工作目录中已有的运行代码；沿用参考遥操的运动路径和相应许可证，采集入口负责数据记录与本地模型/TCP 配置。
- pyAgxArm：官方 https://github.com/agilexrobotics/pyAgxArm ，提交 d90e234ba8e408cfa964b38aa9120b3131fc9f85，版本1.0.0，许可证见 vendor/pyagxarm/LICENSE。
- 网格：官方 https://github.com/agilexrobotics/agx_arm_urdf ，提交680fa925cd00db7e2628563d2edc5540963045de。新版资源将 gripper 独立到 agx_gripper；这里按随附 URDF 的引用路径保存同名网格，不改关节和几何变换。许可证见模型包 LICENSE。
- GUI 三维库：[Three.js 官方仓库](https://github.com/mrdoob/three.js/tree/r170)，固定 r170（0.170.0）；仅随附 module、OrbitControls、STLLoader 和 MIT 许可证，位于 `data_collect/web/static/vendor/`。
- Quest APK：官方 https://github.com/rail-berkeley/oculus_reader ，提交2cf70ab489348f68a25a40e0ce0878de3f587c3a；保存在 oculus_reader/scripts/APK，供原读取器按需安装。
- 触觉 SDK：当前目录已有的 dmrobotics 1.2.6，仅携带 Linux 3.10/3.11 运行文件，不包含重复 build、示例或 Windows 库。
- RealSense：PyPI 的 pyrealsense2 2.56.5.9235，携带 Linux x86_64 的 cp310/cp311 wheel。

历史环境与旧版运动保护的验收记录、当前纯遥操环境安装结果见 `docs/validation.md`。它们不代表当前版本已完成实机验证；克隆或迁移目录后需重建本机环境与内部 ROS overlay，并核验设备及标定。
