# RealSense / GoPro 自动采集

## 自动选择

`config/collection.yaml` 的 `camera_selection.mode` 默认为 `auto`：完整的已配置 RealSense 组合连接时使用原来的双相机采集；否则查找 GoPro / HDMI USB 采集卡，使用 `camera_selection.gopro` 中的相机位置和标定。选择在每次启动时完成，录制中不会因断线切换到另一台相机。需要固定方案时可设为 `realsense` 或 `gopro`。

RealSense 的 `data_collect/sensors/cameras.py`、分辨率、帧率、序列号、SDK 读取与时间戳逻辑均未修改。GoPro 实现参考 UMI 的 `umi/real_world/piper_camera_preview.py::CameraReader` 和 `uvc_camera.py`：V4L2、MJPG、1280×960、60 Hz、缓存 1 帧、独立线程持续读取，输出转换为 RGB 后进入本工程的异步 HDF5/MP4 写入链路。

默认一台 GoPro 作为 `wrist_rgb`，所以录制包含一路相机、左右触觉以及机器人反馈/指令。若 GoPro 安装在全局位置，将 GoPro 配置和标定中的相机位置同时改为 `global_rgb`。两台 GoPro 应分别配置两个不同的稳定设备路径；不会自动把同一设备复制成两个视角。不同方案的尺寸或视角清单不同，需要分别导出为 LeRobot 数据集。

自动发现使用 `/dev/v4l/by-id/*video-index0`，识别 GoPro、Elgato、HD60、HDMI、USB_Video 和 UGREEN 35287 标识，排除普通笔记本摄像头及 index1 元数据节点。HDMI 采集卡只暴露采集卡身份，软件无法从该身份证明 HDMI 输入源是 GoPro；现场应将采集卡接到 GoPro。其他品牌或无上述标识的采集卡，在 `device` 中填写稳定的 `/dev/v4l/by-id/...-video-index0` 路径一次，之后仍自动切换。多设备无法确定位置时给出错误，不猜测。

只读检查：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.cameras
```

## GoPro 首次准备

使用与 UMI 相同的 GoPro HDMI → USB 采集卡链路；只用 USB 线传文件的 GoPro 不属于这个实时采集后端。确认 GoPro 视频模式、采集卡支持上述分辨率/帧率，且没有其他程序占用视频设备。

HERO11 Black 的 HDMI 连接应使用 Media Mod 的 Micro-HDMI 输出，再接采集卡的 HDMI 输入；机身 USB-C 不能通过普通 USB-C 转 HDMI 线代替这个输出。按 [GoPro 官方说明](https://gopro.my.site.com/GoProSupportHUB/s/article/Media-Mod-HDMI-Output?language=en_US)，HERO11 实时取景的 HDMI 输出为 60 Hz，机内录像时为 30 Hz。当前 60 Hz 方案应保持实时取景、不启动机内录像，并在同一状态下标定和采集。

先确认电脑采集预览中能看到相机拍摄的实景，再进行二维码标定。UGREEN 待机标志或“无信号 / No Signal”是采集卡生成的画面，不能用于标定；应检查相机输出和线缆是否插紧。当前 UGREEN 35287 实测只有一个视频输入，`video-index0` 是视频，`video-index1` 是元数据，切换到 index1 无法恢复实景。采集卡接受分辨率/帧率设置也不代表它已收到有效 HDMI 输入。

1. 从上面的只读检测输出获取采集卡的稳定路径。复制 `config/calibration.gopro.example.json` 为 `data/calibration/calibration_gopro_v1.json`，将 `camera_serials.wrist_rgb` 改成这个路径，并核对触觉 ID、安装位置和 195 mm TCP。
2. 将 GoPro 对准本机显示的二维码窗口，用 UMI 原有工具测量当前链路的接收延迟。本机检测到的 UGREEN 35287 稳定路径已填入下面的命令，并与现场设备标定、采集时自动选择的路径及现有延迟文件一致。更换采集卡时需重新检测并同步更新路径、重新测量；不要使用会随插拔变化的 `/dev/video4` 代替稳定路径。

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
mkdir -p /home/yanyu/Documents/Piper-main/Data_Collect/data/calibration
python /home/yanyu/Documents/Piper-main/UMI_DM/universal_manipulation_interface/scripts/calibrate_camera_receive_latency.py \
  --camera /dev/v4l/by-id/usb-MACROSILICON_UGREEN_35287_86477247-video-index0 \
  --output /home/yanyu/Documents/Piper-main/Data_Collect/data/calibration/gopro_wrist_latency.json
```

本机已在 2026-11-17 完成 60 个有效样本的接收延迟测量；设备和视频模式未变时无需重复执行并覆盖现有结果。测量结果与实机检查见 [验证记录](validation.md)。

3. 完成上述现场核验后，将 `data/calibration/calibration_gopro_v1.json` 的 `verified` 设为 `true`。这个字段表示整套设备核验完成，包含相机安装位置、实际 TCP、左右触觉设备及无接触基线，不仅表示 GoPro 已经出图或测过延迟。不能把 RealSense 标定文件直接用于 GoPro；设备身份、视频模式、延迟样本和标定缺失/不一致都会阻止开始录制。更换 GoPro、采集卡或视频模式时重新测量。

以后采集用同一条命令：

```bash
./start.sh --terminal can_port:=can0
```

A 对齐；按住 RG 跟随并写入；松开 RG 保持并暂停写入；再按 RG 续写同一 Episode；B 结束并回零。`teleop_only:=true` 仍完全跳过相机检测和采集。

## 图片与动作如何对应

每次成功读取 GoPro 帧立即记录 `time.monotonic_ns()`，得到 `available_host_ts_ns`。使用 UMI 延迟工具产生的实测中位数，计算 `capture_host_ts_ns = available_host_ts_ns - receive_latency_ns`。两种时间都保留，连同设备路径、视频帧索引、延迟配置及其 SHA-256 一起记录。

审核和导出按估计的**画面时间**建立 20 Hz 对齐，机器人反馈取同一时间之前的最新包，动作使用现有 ±25 ms 最近指令匹配。视频通过逐帧索引取图，绝不按视频播放秒数或“第几张图对应第几个动作”配对。GoPro 的离线场景对齐允许使用在网格时间之后才到达主机的帧，索引缓存明确标记 `gopro_calibrated_scene_time_offline_v1`；这不表示该图像在当时已可供在线策略使用。RealSense 仍按原主机可用时间进行因果匹配。

RG 恢复时，迟到但画面属于暂停区间的 GoPro 帧会被丢弃。松开时仍在相机链路中传输的尾部图像可能没有写入，因此每个片段只导出已收到且画面时间属于该片段的覆盖范围；不拿前一张画面补齐缺失尾部。原始有效区间保留不变，同步审核报告记录 `aligned_frames` 和 `aligned_duration_s`。短到没有同步采样点的片段会被拒绝；跨暂停不会匹配图片或动作。

接收延迟修正是 UMI 同样采用的估计方法，并非硬件曝光同步。原 QR 测量包含显示刷新、GoPro、HDMI 和采集卡链路延迟；真实精度受显示刷新和链路抖动影响，不能宣称零误差。需要用现场共同事件复核；本次离线测试只验证代码的时间配对与数据读写，未代替真机同步验收。
