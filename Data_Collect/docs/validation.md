# 模块迁移验证

## 本次验证范围

2026-10-19 将采集工程从个人目录迁至仓库根目录 `Data_Collect/`，拆分 Python 业务模块，迁移内部导入、ROS 启动文件、网页资源和命令行入口。旧标准 Piper 与 Piper-X 模型同时保留，运动参数未调整。

## 已完成检查

- Python 3.10 / pytest 8.4.2：`python -m pytest -q`，36 项通过。测试在临时虚拟环境中运行，清除系统 ROS 的 `PYTHONPATH`，并通过 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 隔离本机自动加载的 ROS pytest 插件。
- 保留旧工程中看门狗、夹爪标定/使能和示教协议回归测试；新增路径、标定准入、运动限制、反馈新鲜度和因果对齐测试。
- 使用生产 EpisodeWriter 和 libx264 生成小型合成 HDF5 + 双 RGB MP4；验证封盘、质检、人工确认白名单、导出预检，以及视频篡改后的拒绝行为。
- 使用假的 LeRobot dataset 类验证导出状态机、帧映射、最终发布和拒绝覆盖；此项不代表实际 LeRobot 依赖已完成联调。
- 本机 HTTP 演示服务验证工作台 HTML、JavaScript、Three.js、模型网格和状态接口，无硬件连接。
- 可编辑安装及 wheel 构建通过；安装包包含迁移后的网页资源和 Three.js 许可证。
- 18 个业务模块可导入；四个 `python -m data_collect.cli.* --help` 与四个安装命令均可从工程外目录执行。
- 使用系统 Jazzy / Python 3.12 在 `/tmp/piper-collect-ros-check/` 构建四个内部 ROS 包，全部成功。构建产物未写入源码目录。
- 原有 58 个模型、原生库、APK 和 wheel 文件的 SHA-256 在迁移前后全部一致；全部业务/ROS/launch Python 通过语法检查，`env.sh` 通过 Bash 语法检查。

## 尚需现场验证

系统 ROS 构建检查不等同于 Python 3.11 capture 环境重建或真实运行验收。本检出目录尚未安装 `.runtime/capture` 和 `.runtime/export`，也未执行真实机械臂运动、双相机/双触觉持续采集或实际 LeRobot 导出。请按 README 重建环境与 overlay，核对新现场设备映射和标定，再验证 A 回零、B 取消/停止、反馈超时、故障退出和数据封盘。

`config/calibration.example.json` 的 `verified` 默认为 false，必须现场核验。合成测试数据使用临时目录，不纳入生产数据或 Git。
