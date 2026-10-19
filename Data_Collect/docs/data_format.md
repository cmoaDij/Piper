# 数据格式

## 原始录制

每个正常封盘的 Episode 包含 `episode_*.h5`、同名前缀的 `.global_rgb.mp4` 和 `.wrist_rgb.mp4`，三者必须一起保留。HDF5 schema 为 `piper_raw_v2`，记录双触觉原始图像/基线、机器人反馈、控制目标和 RGB 视频索引。旧 `piper_raw_v1` 仍可读取。

- 图像流：`cameras/global_rgb`、`cameras/wrist_rgb`、`tactile/left_raw`、`tactile/right_raw`。
- `available_host_ts_ns` 是数据在主机可用的单调时钟时间，不能用视频播放时间代替。
- RGB 使用 H.264 视频，`video_frame_index` 和 `video_pts` 从零连续；视频相对路径与 SHA-256 存在 HDF5 中。
- 触觉为 uint8 灰度或 RGB，保留同尺寸无接触基线；导出时按需要扩展为三通道。
- `robot_feedback` 包含六轴关节、夹爪、TCP 及各分量的可用时间；`robot_command` 包含目标和 SDK 调用起止时间。
- 关节单位 rad，夹爪/位置单位 m，姿态使用 xyz 对应的 rpy rad。

写入阶段保留 `.partial`，视频先封盘，再将 HDF5 标记完整。拒绝轨迹保留 `complete=false` 与 `failure_kind=episode_rejected`；设备/写盘故障同样不能进入训练。原始文件不自动删除。

## 质检、审核与导出

`data/audit/` 保存 `audit_report.json`、CSV、`sync_indices/`、`review_decisions.json` 和 `approved_episodes.json`。自动 PASS 与匹配源文件 SHA-256 的人工 approve 同时满足后，才能进入白名单。

审核按完整 Episode 生成 20 Hz 索引：观测只能来自过去，动作在 ±25 ms 窗口中优先匹配，再回退零阶保持。完整阈值由 `config/collection.yaml` 控制。

LeRobot v3 导出包含 `observation.state`、`action`、实际/目标 TCP 以及四路图像特征。Train/Val 按 Episode 划分；来源映射和分区保存在导出目录 `meta/`。先写 `<output>.partial`，成功后改名，不覆盖已有输出。合成数据默认拒绝，仅测试时显式使用 `--allow-simulated`。
