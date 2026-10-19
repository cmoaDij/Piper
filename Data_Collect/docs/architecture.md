# 采集架构

## 当前模块设计

`Data_Collect/` 是唯一部署单元；Python 业务包、内部 ROS workspace、SDK、模型和运行配置一起保留。目录可以移动到其他机器，但本地 `.runtime/` 和 ROS overlay 必须在新位置重建。

| 模块 | 职责 |
|---|---|
| `cli` | 参数入口；调用业务模块，不持有设备 |
| `application` | A/B 状态机、回零与对齐流程、录制和故障协调 |
| `common` | 稳定资源路径、配置校验、ROS 回调执行器和诊断 |
| `control` | 目标组合、反馈观察、FK、一阶准入与连续运动保护 |
| `sensors` | RealSense、Tac-X 适配及独立图像采集进程 |
| `recording` | 有界队列、Episode、HDF5 与视频编码/读取 |
| `calibration` | 多图像流相对延迟测量工具 |
| `dataset` | 因果时间对齐、自动质检、白名单及 LeRobot 导出 |
| `web` | 本地工作台、人工审核服务和静态资源 |

## 运行与依赖

`launch/collect.launch.py` 启动 Quest 位姿、增量位姿、IK 和 `data_collect.application.collector`。采集驱动继承内部 `agx_arm_ctrl`，保持唯一机械臂 SDK 控制出口；图像 SDK、视频编码和写盘在独立子进程中运行。工作台订阅轻量状态，不额外创建硬件连接。

所有跨模块导入使用完整的 `data_collect.<模块>` 路径。项目根目录只在 `common/paths.py` 中定义；配置路径相对工程根目录解释。ROS 包名和内部资源路径保持兼容，Quest 脚本通过环境提供的 `PYTHONPATH` 使用共享执行器。

`dataset` 的离线流程不启动设备。延迟标定录制通过显式 `record` 子命令启动图像采集；相对补偿辅助函数目前未接入生产审核/导出，不能把测量配置理解为采集时间戳已被修正。

## 模型与第三方边界

同时保留 `agx_arm_urdf/piper/` 与 `piper_x/`，当前采集配置固定为 Piper-X。旧模型仅作为保留资源，不自动切换。当前 Piper-X URDF 的 DAE/STL 引用全部随工程分发。

底层驱动和机械臂 SDK 中其他型号/灵巧手分支仍存在统一导入依赖，本次只调整业务模块与工程位置，不删除这些分支或改变运动阈值。第三方源代码、二进制、APK、wheels 和许可证保留。

## 历史设计方案

以下是保留的原始设计参考；当前实现及命令以本节和工程 README 为准，历史方案中的存储/环境选择不代表当前部署状态。

将流程调整为 **“先逐条独立保存原始 Episode $\rightarrow$ 离线集中质检与筛选 $\rightarrow$ 批量转换为 LeRobot v3 格式”** 是工程上最稳健的做法。这种解耦架构不仅能避免采集时实时视频编码抢占 CPU/磁盘 I/O 导致掉帧，还能避免因中途剔除废片而破坏 LeRobot 数据集的全局连续索引（`episode_index`）和统计量（`stats.json`）。

  

以下是调整后的完整分阶段实施方案：

  

## 一、 整体三阶段流水线架构

1. **阶段一：纯异步轻量化逐条采集（Online Collection）**
    
      
    - 遥操作期间只做一件事：各传感器按原生频率异步读取，每完成一条演示，独立保存为一个自包含的 `raw/episode_XXXX.h5` 文件。不跑视频编码、不调 LeRobot API，确保采集端零阻塞。
        
          
        
2. **阶段二：离线因果质检与人工筛选（Offline Audit & Filtering）**
    
      
    - 采集结束后，运行离线审计脚本对所有 `.h5` 文件进行 **20 Hz 因果对齐体检**，自动剔除掉帧超标的轨迹；同时支持人工剔除操作失误（如抓取滑落、碰撞）的轨迹，最终生成一份合格清单 `approved_episodes.json`。
        
          
        
3. **阶段三：批量转换导出 LeRobot v3（Batch Conversion）**
    
      
    - 转换脚本仅读取 `approved_episodes.json` 中的合格轨迹，先按 Episode 划分 Train/Val，再一次性调用 LeRobot v3 官方 API 顺序生成从 `0` 到 `N-1` 严格连续的标准数据集。
        
          
        

## 二、 阶段一：逐条 Episode 原始采集规范（`episode_XXXX.h5`）

### 1. 源头减负与打点要求

- **触觉减负**：双 Tac-X 仅开启 `raw` 图像流，显式关闭 `deformation`、`shear`、`depth` 等 SDK 派生计算。
    
      
    
- **锁内打点**：所有主机时间戳统一使用 `time.monotonic_ns()`，严格在**获取线程锁之后、SDK 读取成功返回瞬间**记录 `available_host_timestamp_ns`。
    
      
    

### 2. 单条 `episode_XXXX.h5` 内部结构设计

每条轨迹独立落盘，文件内按传感器分 Group 异步存储原始序列，结构如下：

  

Plaintext

```
episode_0001.h5
├── attrs/                               # 元数据：episode_id, task_instruction, calibration_id, operator, record_date
├── cameras/
│   ├── global_rgb/                      # data (N_c1, H, W, 3), available_host_ts_ns, device_ts, frame_id
│   └── wrist_rgb/                       # data (N_c2, H, W, 3), available_host_ts_ns, device_ts, frame_id
├── tactile/
│   ├── left_raw/                        # data (N_t1, H, W, 3), available_host_ts_ns, frame_id, baseline_ref
│   └── right_raw/                       # data (N_t2, H, W, 3), available_host_ts_ns, frame_id, baseline_ref
├── robot_feedback/
│   ├── joint_pos                        # (N_r, 6) 六轴实际角度 (rad)
│   ├── gripper_pos                      # (N_r, 1) 实际夹爪开度 (m)
│   ├── tcp_pose                         # (N_r, 6 或 7) 实际末端位姿 [x, y, z, rx, ry, rz] 或四元数
│   ├── available_host_ts_ns             # (N_r,) 锁内读取完成时间
│   ├── sdk_joint_ts / sdk_tcp_ts        # (N_r,) SDK 原消息时间戳
│   └── is_new_packet                    # (N_r,) bool，区分真实硬件刷新包与 200Hz 轮询缓存包
└── robot_command/
    ├── target_joint_pos                 # (N_cmd, 6) 目标六轴角度 (rad)
    ├── target_gripper_pos               # (N_cmd, 1) 目标夹爪开度 (m)
    ├── target_tcp_pose                  # (N_cmd, 6 或 7) 目标末端位姿（直接下发或离线正运动学 FK 解算）
    ├── cmd_call_begin_ns / end_ns       # (N_cmd,) 锁内下发调用起止时间
    └── cmd_seq                          # (N_cmd,) 独立指令序列号
```

## 三、 阶段二：离线因果质检与 Episode 筛选（不切片）

在采集完一批数据（如 100 条 `.h5`）后，运行离线筛选脚本，执行“双重过滤”：

  

### 1. 第一重过滤：20 Hz 因果对齐与硬件健康度体检（自动）

对每条 `episode_XXXX.h5` 构建严格等步长网格 $t_k = t_0 + k \times 50\text{ ms}$：

  

- **观测（Observation）因果选帧**：每个模态仅从满足 `available_host_ts_ns <= t_k` 且有效的真实样本中选择最新一帧，计算 `age = t_k - available_host_ts_ns`。
    
      
    
- **动作（Action）匹配与零阶保持（ZOH）**：
    
      
    1. 优先在 `cmd_call_begin_ns` 的 $[t_k - 25\text{ ms}, t_k + 25\text{ ms}]$ 窗口内取最近指令。
        
          
        
    2. 若因控制频率波动或短时静止导致窗口内无新指令，自动回退采用 **$t_k$ 之前最后一次下发的指令（零阶保持 ZOH）**，维持控制器真实物理目标。
        
          
        
    3. 若非静止期回溯跨度 **$> 100\text{ ms}$**，直接判定通信断流。
        
          
        
- **整条通过/淘汰红黄绿判据（绝不切片）**：
    
      
    

|**检查项**|**绿区（正常）**|**黄区（偶发单帧复用）**|**淘汰红线（触发即整条标记为 REJECT）**|
|---|---|---|---|
|**RGB 相机 Age**|$\le 40\text{ ms}$|$40 \sim 68\text{ ms}$|任一网格点 **$> 68\text{ ms}$**（连续丢 $\ge 2$ 帧）|
|**触觉 Raw Age**|$\le 35\text{ ms}$|$35 \sim 55\text{ ms}$|任一网格点 **$> 55\text{ ms}$**|
|**机械臂状态 Age (关节+TCP)**|$\le 25\text{ ms}$|_不设黄区_|任一网格点 **$> 25\text{ ms}$**|
|**左右触觉时间差**|$\le 15\text{ ms}$|$15 \sim 25\text{ ms}$|任一网格点 **$> 25\text{ ms}$**|
|**整条黄区累计占比**|$0\%$|**$\le 2\%$** 且不连续|黄区占比 **$> 2\%$**，或连续 $\ge 2$ 步落入黄区|
|**关键接触保护窗口**|全程绿区|**严禁黄区**|夹爪开合接触前后 **$\pm 0.5\text{ s}$** 内出现任何丢帧复用|

> **关键优化**：通过质检的 Episode，脚本直接将其每个 $20\text{ Hz}$ 网格点匹配好的各模态**源数组索引（Index Map）**缓存在 `audit/sync_indices/episode_XXXX_sync.json` 中。阶段三转换时无需二次计算对齐，直接按索引取数。
> 
>   

### 2. 第二重过滤：任务行为与人工抽检（半自动）

- **自动规则检查**：检查轨迹时长是否在合理区间（如 $5 \sim 30\text{ s}$）、夹爪开度是否包含完整的“张开 $\rightarrow$ 闭合 $\rightarrow$ 张开”过程。
    
      
    
- **人工标记剔除**：脚本生成拼接预览小视频或报表，人工划掉遥操作失误（如碰倒物体、姿态奇异）的 Episode。（有可视化界面）
    
      
    
- **输出白名单**：最终生成 `audit/approved_episodes.json`，记录所有 100% 合格的 `episode_XXXX.h5` 文件路径及对应的 `train` / `val` 划分标签。
    
      
    

## 四、 阶段三：批量转换为标准 LeRobot v3 数据集

仅针对白名单 `approved_episodes.json` 中的合格数据进行一次性转换，彻底杜绝废片污染或索引断层。

  

### 1. LeRobot v3 标准特征 Schema（兼容策略与 VLA）

|**LeRobot Feature 键名**|**Dtype / Shape**|**物理含义与封装方式**|
|---|---|---|
|`observation.state`|`float32 (7,)`|**实际关节状态**：`[q1..q6 (rad), gripper_pos (m)]` $\rightarrow$ Parquet|
|`observation.eef_pose`|`float32 (7,)`|**实际末端位姿**：`[x, y, z (m), rx, ry, rz (rad), gripper_pos (m)]` $\rightarrow$ Parquet|
|`action`|`float32 (7,)`|**目标关节指令**：`[q1..q6 (rad), gripper_target (m)]` $\rightarrow$ Parquet|
|`action.eef_pose`|`float32 (7,)`|**目标末端位姿**：`[x, y, z (m), rx, ry, rz (rad), gripper_target (m)]` $\rightarrow$ Parquet|
|`observation.images.camera_global`|`video (H, W, 3)`|全局相机 RGB $\rightarrow$ MP4 (`avc1` 编码)|
|`observation.images.camera_wrist`|`video (H, W, 3)`|手腕相机 RGB $\rightarrow$ MP4 (`avc1` 编码)|
|`observation.images.tactile_left`|`image / video (H, W, 3)`|左触觉 Raw 图 $\rightarrow$ 无损 PNG 或极高码率 MP4|
|`observation.images.tactile_right`|`image / video (H, W, 3)`|右触觉 Raw 图 $\rightarrow$ 无损 PNG 或极高码率 MP4|
|`task`|`string`|**VLA 语言指令**：从 HDF5 `attrs` 读取并传入每帧|

### 2. 批量转换标准执行流

1. **初始化数据集**：调用 `LeRobotDataset.create(repo_id=..., fps=20, features=...)`。
    
      
    
2. **按序遍历合格白名单**：
    
      
    - 读取 `episode_XXXX.h5` 及阶段二生成的 `episode_XXXX_sync.json` 对齐索引表。
        
          
        
    - 遍历该轨迹的所有 20 Hz 网格点 $k = 0 \dots T-1$，按索引抽出对齐后的图像、关节状态、TCP 位姿与动作，调用 `dataset.add_frame(...)`。
        
          
        
    - 单条遍历完成后调用 `dataset.save_episode()`（自动完成视频编码与内部连续 `timestamp = k / 20.0` 生成）。
        
          
        
3. **封盘与统计计算**：所有合格 Episode 转换完毕后，调用 `dataset.finalize()`，生成完整的全局 `meta/info.json` 与归一化统计量 `meta/stats.json`。
    
      
    

## 五、 工程目录结构与操作闭环

Plaintext

```
experiment_piperx/
├── 1_raw_episodes/                    # 【阶段一产物】逐条独立保存的原始异步 HDF5
│   ├── episode_0001.h5
│   ├── episode_0002.h5                # (假设此条掉帧严重，后续被筛除，但原文件保留备查)
│   └── episode_0003.h5
├── calibration/
│   └── calibration_v1.json            # 传感器序列号、相机-触觉相对偏移、跟踪滞后记录
├── 2_audit_and_filter/                # 【阶段二产物】离线质检报告与筛选白名单
	│   ├── sync_indices/                  # 每条合格 Episode 的 20Hz 选帧索引映射缓存
│   ├── audit_report.csv               # 全量 Episode 的体检表（Pass/Reject、拒绝原因、最大 Age、ZOH 比例）
│   └── approved_episodes.json         # 最终确认用于转换的合格 Episode 列表及 Train/Val 划分
└── 3_lerobot_dataset/                 # 【阶段三产物】干净、连续、无切片的标准 LeRobot v3 数据集
    ├── meta/                          # info.json, stats.json, episodes/
    ├── data/                          # Parquet (state, eef_pose, action, action.eef_pose, timestamp)
    └── videos/                        # MP4 视频流
```

**日常工作流总结**：

  

1. **采集时**：运行 `record_raw.py` $\rightarrow$ 专心遥操作，一条存一个 `.h5`。
    
      
    
2. **筛选时**：运行 `audit_and_filter.py` $\rightarrow$ 查看生成的 `audit_report.csv`，自动筛掉红线废片，人工勾选确认生成 `approved_episodes.json`。
    
      
    
3. **转换时**：运行 `convert_to_lerobot.py --manifest approved_episodes.json` $\rightarrow$ 一键批量生成标准 LeRobot v3 数据集，直接送入 ACT / Diffusion Policy 或 VLA 框架训练。