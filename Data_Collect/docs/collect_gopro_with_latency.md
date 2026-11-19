# 本轮 GoPro + Piper-X 数据采集流程

使用配置 `config/collection_gopro.yaml`。数据写到 `data/gopro_collection/`，与延迟测试和原有 RealSense 数据分开。左触觉 `X2631410810`，右触觉 `X2631410795`；Piper-X，TCP `[0,0,0.195,0,0,0]`，`move_js`，速度参数 50%，控制 50 Hz、反馈轮询 200 Hz。按 B 结束封盘后，等待 0.5 秒，通过原驱动 `move_j` 以 5% 速度回到六关节零位，到位后张开夹爪。

## 已接入的延迟及含义

| 通道 | 本轮有效跟随滞后 |
| --- | --- |
| joint1 | 65 ms |
| joint2 | 80 ms |
| joint3 | 85 ms |
| joint4 | 90 ms |
| joint5 | 85 ms |
| joint6 | 80 ms |
| gripper | 50 ms |

启动时读取 `data/calibration/piper_tracking_latency.json`，检查七通道测量是否完整，并核对模型、URDF、TCP、控制频率、SDK 模式和速度参数。ROS 参数覆盖造成工况不一致时，在连接 SDK 前拒绝启动。每条正式录制嵌入标定数值和哈希，后续审核和导出使用该快照。

SDK 出口保留实际调用时间，分别为关节及夹爪记录自己的最近调用起点，再加相应通道的实测滞后，生成 `expected_feedback_host_ts_ns[7]`。离线在每个 20 Hz 网格点选取预计已响应的过去目标，导出 `tracking.reference[7]` 和 `tracking.valid[7]`，用于对照反馈和检查跟随效果。每次按 RG 开始的新录制片段，尚无响应历史的分量标记无效，不跨暂停补用目标。

标准训练 `action` 仍为实际 SDK 发送时间对应的目标；机器人反馈仍用原始主机时间。这些数值包含执行动态和反馈链路，不能作为编码器物理采样延迟直接从反馈时间减去，也没有让未知的实时 Quest 动作提前执行。5 ms 是拟合搜索步长，不代表测量精度。改变机械臂/夹爪、工具负载、TCP、控制模式或速度后应重新测量相应工况。

GoPro 使用相同设备和模式的独立接收延迟文件 `data/calibration/gopro_wrist_latency.json`：画面估计时间 = 主机接收时间 − 144.408792 ms。图像按估计画面时间，与原始机器人反馈、实际发送目标以及上述响应参考匹配；不叠加七轴延迟去平移图像。GoPro 本轮 P95 约 243.5 ms，波动明显，这套处理不代表精确曝光同步。

## 1. 连接与启动前核验

先关闭其他机器人控制/标定程序，再连接本次 CAN、GoPro/绿联、左右触觉和 Quest；工具离开桌面，活动范围无障碍，急停可用。触觉垫保持无接触，供 SDK 初始化基线。本轮旧标定进程已退出。

2026-11-17 本次代码交付检查时，电脑已检测不到 `can0`、绿联采集卡及 Quest；因此重新连接后需要执行下面的检查。本次只完成离线配置一致性和自动测试，未重复现场运动或正式采集。

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture

# 确认 can0 对应本次 Piper-X；没有接口时先检查连接，不能继续执行启动。
ip -brief link
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0

# 固定设备身份；重新插拔后 /dev/videoN 编号可以变化，不改用笔记本摄像头。
ls -l /dev/v4l/by-id/usb-MACROSILICON_UGREEN_35287_86477247-video-index0
adb devices
```

CAN 应为 UP、bitrate 1000000，Quest 应显示 `device`；`unauthorized` 需在头显内允许 USB 调试。采集卡必须出现上面的 index0 视频路径，index1 是元数据。视频应为真实 GoPro 手腕视角，不能只根据有帧或采集卡待机画面判断已就绪。

连接后可以运行以下只读配置检查；不会使能或移动机械臂：

```bash
python - <<'PY'
from data_collect.common.config import load_config
from data_collect.sensors.camera_selection import resolve_cameras
from data_collect.application.collector import validate_calibration
cfg = load_config('config/collection_gopro.yaml')
resolve_cameras(cfg)
validate_calibration(cfg)
print('机器人通道延迟(ms):', [v / 1e6 for v in cfg['robot_tracking_latency']['latency_ns']])
print('GoPro接收延迟(ms):', cfg['cameras']['wrist_rgb']['receive_latency_ns'] / 1e6)
print('数据目录:', cfg['data_root'])
PY
```

此检查验证文件、工况和相机路径，不代替触觉实际枚举、实时画面、CAN 通信或工作空间核验。

## 2. 正式采集

下面命令会连接并使能机械臂。先确认现场已准备好，再在一个终端启动；不要同时启动另一个 GUI 自动采集进程。

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
./start.sh --terminal \
  config:=/home/yanyu/Documents/Piper-main/Data_Collect/config/collection_gopro.yaml \
  can_port:=can0
```

在本目录也可简写为 `./start.sh --terminal config:=config/collection_gopro.yaml`，默认使用 can0。

终端默认采用精简输出：显示 A 待跟随、录制开始/暂停/恢复、有效录制秒数、文件封盘结果、回零结果及全部警告/错误；每 5 秒一行传感器就绪情况、图像采样速率和反馈年龄。参数、参考矩阵和详细时序 JSON 不刷屏，完整原始输出仍保存在启动提示的 `launch.log`；ROS 节点日志也保留在 `data/gopro_collection/logs/`。`传感器就绪` 使用原有数据/基线/新鲜度检查，不表示已经识别出 GoPro 画面内容。

排查问题需要原始终端输出时，在启动命令末尾加 `verbose:=true`。该选项只影响日志显示，不改变控制参数、采集频率或写入内容。

需要工作台预览时，在第二个终端运行观察页面，不加 `--start`：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.gui --config config/collection_gopro.yaml --port 8766
```

1. 等待 GoPro、双触觉、机器人反馈及无接触基线就绪。确认画面是当前实景、左右触觉身份正确。若提示“无法开始录制”，先解决具体报错；能够遥操不代表已有数据。
2. 右手柄按 **A** 对齐当前手柄和机械臂姿态。A 只建立参考，不下发运动目标、开始录制或回零。如果按 A 时 RG 已握住，先松开 RG 再按住。
3. 按住右中指把手 **RG**，收到本次参考下完整的新 IK 结果后，开始下发和录制；终端应出现 `Recording ...episode_XXXX.h5`。右食指 **rightTrig** 控制夹爪开度，按实际物体和触觉垫允许的范围操作。首次 RG 启动前不下发夹爪命令；启动后松开 RG 时仍可独立控制夹爪。
4. 松开 RG：暂停该 Episode 的各流写入并保持关节目标；再次按住 RG：恢复同一 Episode，建立新片段。松开 RG 时食指仍能控制夹爪。
5. 确认回零路径无障碍后按 **B**：结束并封盘，0.5 秒后自动以 5% 速度回到六关节零位，到位后张开夹爪。等待驱动报告回零成功，且 `.h5.partial` 消失、最终 `.h5` 和 `.wrist_rgb.mp4` 出现，再进行审核或开始下一条。B 不等于急停或电机失能。回零期间 A 无效；尚未发出的回零请求可由 A 取消。下一条重新按 A、松开后按住 RG。

若首次 RG 提示传感器或基线尚未就绪，修复/等待就绪后，松开并重新按住 RG 可重试，必须看到新的 `Recording ...` 文件路径。已锁定的设备、写盘或标定故障仍需处理后重启；不会在同一次持续握持中自动从中途开始录制。启动 Python 子进程已关闭 stdout 缓冲，以减少 SDK 初始化输出拖到 Ctrl-C 后集中显示造成的时序误判。

启动程序的总时长不等于 Episode 时长：A 前、首次 RG 前、RG 松开期间和 B 后都不写入。需要一条约 1 分钟的动作时，应先确保录制已开始，再在有效动作期间持续按住 RG；当前审核 `max_duration_s: 30` 仅决定能否通过审核，不会在 30 秒截断原始录制。正式任务确需超过 30 秒时，需按任务要求调整审核时长。

先做一条有效录制时间 5–10 秒的试采，检查真实图像及完整性后再批量采集。默认审核要求有效时间 5–30 秒，RG 松开期间不计入。已删除固定夹爪开度和开合循环审核，不需要夹到 20 mm；抓取完成由人工视频确认。图像、时间、新鲜度和接触窗口质量条件仍保留。

## 3. 审核与导出

按 B 封盘后，在另一终端启动审核：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.review --config config/collection_gopro.yaml --port 8765
```

打开 <http://127.0.0.1:8765>，播放 GoPro 和左右触觉、检查关节/指令。确认实景、抓取过程和夹持效果后，人工点击“通过此条”；自动 PASS 仍不进入导出白名单。后续新封盘文件点击“刷新列表”重新审计。此前因旧夹爪规则拒绝的数据也可重新扫描，但不会自动批准。

白名单、索引和审核记录在 `data/gopro_collection/audit/`。只读检查导出输入：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.export \
  --manifest data/gopro_collection/audit/approved_episodes.json --check-only
```

本机本次检查时 `.runtime/export` 尚未安装。需要 LeRobot 时先执行一次工程已有的依赖安装流程（需要下载 CPU PyTorch 和转换依赖）：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
.runtime/bin/uv venv --python /usr/bin/python3 .runtime/export
.runtime/bin/uv pip install --python .runtime/export/bin/python \
  --index https://download.pytorch.org/whl/cpu \
  torch==2.7.1 torchvision==0.22.1 torchcodec==0.4.0
.runtime/bin/uv pip install --python .runtime/export/bin/python \
  -c config/export-constraints.txt -e '.[export]'
```

在已装好的转换环境导出；输出目录必须不存在，重导出请换新名称：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh export
python -m data_collect.cli.export \
  --manifest data/gopro_collection/audit/approved_episodes.json \
  --output data/lerobot/gopro_first_dataset --repo-id local/gopro_first_dataset
```

保留原始 HDF5 与 MP4。导出目录的 `meta/robot_tracking_latency.json` 保存标定快照及动作语义，`meta/source_episodes.json` 保存来源/分段，`tracking.reference` 和 `tracking.valid` 是诊断字段，不作为标准策略观测或替换训练动作。不同标定文件或未标定的录制需分别导出。原有纯机器人标定数据及 No Signal 试录不作为训练数据。
