# 数据格式

## 原始录制

每个正常封盘的 Episode 包含 `episode_*.h5` 以及本次选中相机对应的同名前缀 MP4，必须一起保留。双 RealSense 有 `.global_rgb.mp4` 和 `.wrist_rgb.mp4`，默认单 GoPro 只有 `.wrist_rgb.mp4`。HDF5 schema 为 `piper_raw_v2`，记录双触觉原始图像/基线、机器人反馈、控制目标和 RGB 视频索引。旧 `piper_raw_v1` 仍可读取。

- 图像流：`cameras/global_rgb`、`cameras/wrist_rgb`、`tactile/left_raw`、`tactile/right_raw`。
- `available_host_ts_ns` 是数据在主机可用的单调时钟时间，不能用视频播放时间代替。
- 新文件根属性 `image_streams` 声明实际相机/触觉流；相机组属性 `backend` 区分 RealSense/GoPro。GoPro 额外保存 `capture_host_ts_ns`、实测接收延迟与标定哈希，用于离线画面时间对齐；原始接收时间始终保留。详见 [相机说明](cameras.md)。
- RGB 使用 H.264 视频，`video_frame_index` 和 `video_pts` 从零连续；视频相对路径与 SHA-256 存在 HDF5 中。
- 触觉为 uint8 灰度或 RGB，保留同尺寸无接触基线；导出时按需要扩展为三通道。
- `robot_feedback` 包含六轴关节、夹爪、TCP 及各分量的可用时间；`robot_command` 包含目标和 SDK 调用起止时间。
- 使用 `robot_tracking_latency_file` 时，配置属性嵌入七通道标定快照与哈希。`robot_command` 另存 `joint_cmd_call_begin_ns`、`gripper_cmd_call_begin_ns` 和 `expected_feedback_host_ts_ns[7]`；预计响应时间是各分量实际调用起点加实测有效跟随滞后，不修改反馈时间。
- 关节单位 rad，夹爪/位置单位 m，姿态使用 xyz 对应的 rpy rad。

写入阶段保留 `.partial`，视频先封盘，再将 HDF5 标记完整。拒绝轨迹保留 `complete=false` 与 `failure_kind=episode_rejected`；设备/写盘故障同样不能进入训练。原始文件不自动删除。

## 质检、审核与导出

`data/audit/` 保存 `audit_report.json`、CSV、`sync_indices/`、`review_decisions.json` 和 `approved_episodes.json`。自动 PASS 与匹配源文件 SHA-256 的人工 approve 同时满足后，才能进入白名单。

审核按有效录制片段生成 20 Hz 索引：观测时间不晚于网格，动作在 ±25 ms 窗口中优先匹配，再回退零阶保持。GoPro 使用经过实测延迟修正的画面时间，接收时间可以晚于网格；其片段首尾裁至已有画面的范围。RealSense 继续使用原始可用时间。完整阈值由 `config/collection.yaml` 控制。

LeRobot v3 导出包含 `observation.state`、`action`、实际/目标 TCP、本次相机/触觉图像以及 `recording_segment`、`source_host_ts_ns`。Train/Val 按 Episode 划分；来源映射和分区保存在导出目录 `meta/`。不同相机视角清单/图像尺寸的 Episode 分别导出。先写 `<output>.partial`，成功后改名，不覆盖已有输出。合成数据默认拒绝，仅测试时显式使用 `--allow-simulated`。

带七通道标定的录制另有 `tracking.reference[7]`（float32，六关节 rad + 夹爪 m）和 `tracking.valid[7]`（int64 的 0/1 掩码）。参考按预计响应时间选择过去目标，只在各录制片段内匹配；片段起点没有响应历史时填 0 并标记无效，不能将填充值当成真实零位。它们用于诊断，不属于标准 `observation.*` / `action`。训练动作保持 SDK 发送时间语义。同步缓存保存 `tracking_reference_indices`，无效为 -1；导出再次核验该索引，使用录制内的标定快照，不依赖当前外部标定文件。`meta/robot_tracking_latency.json` 保存标定及语义；不同标定的录制分别导出。完整命令见 [本轮采集流程](collect_gopro_with_latency.md)。
