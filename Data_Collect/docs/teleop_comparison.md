# 与 QuestArmTeleop 的遥操及 IK 对比

当前硬件模型与 `Piper_DM (2)/QuestArmTeleop` 的 `teleop_single_piper_x.launch.py` 对比。USB 通用入口和 Piper-X 入口的方向配置不同，不能混用其型号和方向设置。

对当前 Piper-X 遥操及后续采集，Data_Collect 的一键启动、A 上升沿双参考、统一 195 mm TCP 和可选采集模块更符合本工程需求。官方原版更精简，适合复现与作为算法基准。当前手柄变换、TCP 增量及 1 倍平移与官方一致；用户已确认当前实机方向一致，本轮仅调整按键与暂停操作。此前同 TCP 数值实验的残差优势，不代表已经完成与官方原版的同台实机精度或稳定性验收，已有 IK NaN 数值问题也未在本轮修改。

## 官方原版的 TCP 数值与 A 键行为

本节与上面的本地副本区分，核对官方 `ros2` 分支提交 `145b80360cf2c5f2e63179086d0aab2cb563826c`。官方 [Piper-X launch](https://github.com/agilexrobotics/QuestArmTeleop/blob/145b80360cf2c5f2e63179086d0aab2cb563826c/src/oculus_reader/launch/teleop_single_piper_x.launch.py) 的驱动 TCP 为 `[0,0,0.13,0,0,0]`，而 [IK 配置](https://github.com/agilexrobotics/QuestArmTeleop/blob/145b80360cf2c5f2e63179086d0aab2cb563826c/src/oculus_reader/config/arm_ik_pose_node.piper_x.yaml) 的工具平移为 `[0,0,0.1325]`、工具预旋转为零。

驱动的 130 mm 定义了反馈 TCP：`T_base_tcp = T_base_flange * Trans(0,0,0.130)`。也就是以法兰为起点，沿法兰局部 +Z 取一个距其 130 mm 的操作点，再把这个点报告到基座系。它不是基座高度，也不会改变实体机械臂的连杆长度。当前关节遥操中，驱动 TCP 主要用于反馈与 A/RJ 参考；SDK 的偏移是主机端坐标转换。

IK 的 132.5 mm 定义了求解目标点：`T_base_ee(q) = T_base_link6(q) * Trans(0,0,0.1325)`。Piper-X 的 flange_link 与 link6 之间为零位移、零旋转固定连接，所以两条链以相同参考面定义工具点。两组数值并行描述同一个预期操作点，不是先平移 130 mm、再加 132.5 mm。

因此，同一关节姿态下，官方两个软件 TCP 点相差 `R_base_flange * [0,0,0.0025]`：长度恒为 2.5 mm，基座中的方向随法兰旋转。这会造成反馈参考与 IK 模型目标不一致，可能出现静止偏差及转动时的位置耦合；单纯的局部 Z 平移差不会把坐标轴旋转 90°。源码未说明该 2.5 mm 差值的标定原因。Data_Collect 按用户要求已将驱动、IK、记录 FK 统一为 195 mm，本轮不改变该设置。

官方 [pub_delta_pose.py](https://github.com/agilexrobotics/QuestArmTeleop/blob/145b80360cf2c5f2e63179086d0aab2cb563826c/src/oculus_reader/scripts/pub_delta_pose.py) 在约 30 Hz 控制循环中，A 只要为 true，就把当前手柄姿态写入 `start_pose_matrix`。当轮计算 `inv(start_pose_matrix)*current_hand` 为单位变换，目标回到 `zero_matrix`。所以源码行为是按住 A 不断重新设参考，松开后继续跟随；松开并不停止，B 才停止。已遥操移动后再次按 A，手柄参考会重设，而 TCP 的 `zero_matrix` 不在 A 分支一起更新，可能使目标返回先前 TCP 参考。未锁定手柄参考时，反馈回调通过两个矩阵对象的同一性维护 TCP 参考；不能简单理解为永远锁定第一帧 TCP。

Data_Collect 则只在 A 的上升沿同时捕获手柄与 TCP，按住 A 不再重设。当前按用户要求增加右把手 RG：A 后需按住 RG 才跟随，松开 RG 冻结最后实际下发的关节目标；重新按住时重设手柄参考，IK 将本段新增量接到保持关节的 FK 原点，首帧不产生新的关节运动，忽略暂停期间的平移和转动。这样避免反复将负载下的反馈静差写成新目标。B 停止并回零，RJ 可以重定位手柄参考。平移倍率已从 1.2 改为 1.0：RG 持续按住时手柄移动 10 mm，对应 10 mm 目标增量。官方 A 按住期间持续重设手柄参考，目标增量为零，直到松开 A 才保留后续相对移动；官方没有这次新增的 RG 跟随开关。这里比较的是目标生成逻辑，实体运动仍取决于 IK 和驱动执行。

## joint6 与官方范围对齐

当前使用的 Piper-X URDF 的 joint6 已从 `[-2.0943951,2.0943951]`（约 ±120°）改为 `[-2.9670597,2.9670597]` rad（约 ±170°），与官方关联的 [flattened URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/flattened/piper_x/urdf/piper_x_with_gripper_description.urdf) 相同。SDK 原本已为约 ±170°，无需修改；IK 的变量边界和网页关节范围都从同一 URDF 加载。关节原点、旋转轴、TCP 和其余关节范围保持当前配置。

## 方向链路核对与本次修正

| 环节 | QuestArmTeleop USB 通用入口 | QuestArmTeleop Piper-X 入口 | 当前 Data_Collect |
|---|---|---|---|
| 驱动型号 | `piper` | `piper_x` | `piper_x` |
| IK 模型 | 旧标准 Piper URDF | Piper-X URDF | 同一 Piper-X URDF |
| 手柄局部 RPY | `[0,1.0471,0]` | `[-1.5708,0,-1.5708]` | 默认与 Piper-X 相同，可用同名 launch 参数覆盖 |
| Quest 原始位置轴 | `[-z,-x,y]` | 相同 | 相同 |
| 增量参考 | TCP 局部坐标 | TCP 局部坐标 | 默认恢复为 `tcp`；`base` 为上一轮的可选模式 |
| TCP 偏移 | 195 mm | 195 mm | 驱动、IK、记录 FK 均为 195 mm |
| 目标 Pose 坐标标记 | `vr_device`，数值实际用于基座 IK | 相同 | 改为 `base_link` |

上一轮仅对齐 Piper-X 局部 RPY，最新真机日志证实参数已生效，但用户仍报告方向不一致。进一步检查发现，原公式的平移为 `p_t=p_t0+R_t0*S*R_h0^T*(p_h-p_h0)`。手柄与 TCP 初始朝向不同，便会旋转后续位移方向；改局部 RPY 无法使所有起始姿态都按基座轴移动。

此前曾默认切换为 `base`，以位置和姿态分别对齐来实现基座轴移动。现已按用户要求恢复 `teleop.delta_frame: tcp`，并将平移倍率改为 1.0，与官方的 TCP 增量公式一致；保持 Piper-X 局部 RPY 和 A/RJ 当前姿态参考。

`./start.sh --terminal can_port:=can0 teleop_only:=true delta_frame:=tcp` 显式使用 TCP 模式，省略最后一个参数同样使用 TCP。启动打印实际配置、URDF、TCP、RPY 和模式；每次 A/RJ 打印当前手柄、TCP 参考和位移映射矩阵。

对当前 can0 只接收广播帧、不发送指令，Piper-X FK 与固件法兰反馈相差约 0.26 mm、0.0016°；旧 Piper 模型相差约 52.25 mm、118.62°。这支持保留 Piper-X，不能仅根据参考目录的旧 `Problem.md` 改用标准 Piper。该结果是一个静态姿态的模型一致性检查，不是实机定位精度验收。

## data_collect/ 坐标与遥操方向为何看起来不同

| 模块或对象 | 采用的坐标与方向 | 是否修改实际遥操方向 |
|---|---|---|
| `control/kinematics.py` | 从 `base_link` 沿 URDF 到 `link6`，再右乘局部 Z 方向的 195 mm TCP；RPY 为 `Rz(yaw)*Ry(pitch)*Rx(roll)` | 只做 FK 与记录，不修改手柄目标 |
| `control/robot.py` | SDK 法兰反馈右乘同一 TCP 偏移；六轴命令按 `joint1..joint6` 原样记录和转发 | 没有交换、取反或额外旋转关节/位置轴 |
| `application/collector.py` | 聚合 IK 六轴目标和夹爪，再调用原驱动的关节控制接口 | 无额外笛卡尔坐标转换 |
| `web/static/robot.js` | 同一 URDF；`Euler(...rpy,'ZYX')` 与 Python 外旋 `xyz` 等价；相机设置 Z 向上 | 仅显示，不参与控制；相机正视从 +Y 附近看向基座，屏幕左右不等于基座左右 |
| 手柄发布节点 | `T_hand=A*T_raw*C`，`A` 是世界轴转换，`C=r_adj*ros_to_arm` 是两次局部右乘修正 | `C` 决定输入局部轴怎样被解释 |
| TCP 增量节点 | `T_target=T_tcp0*inv(T_hand0)*T_hand`；按手柄局部变化驱动捕获 TCP 的对应局部变化 | 与上一轮基座增量的主要区别在此 |

直接执行网页生产 `makeRig().pose()`，在 URDF 零位、此前 CAN 快照、工作空间中部三个姿态，与 Python FK 的 TCP 矩阵最大元素差小于 `6e-16`；生产 IK 的 FK 与 Python FK 最大差小于 `7e-16`。SDK 位姿转换与 Python 的同一法兰/TCP 变换最大差约 `3.3e-16`。因此没有发现 Python 采集、网页、IK 或 SDK 层之间的固定轴方向差异。

**TCP 局部轴不等于基座轴。** 当前 Piper-X URDF 在六轴零位时，三个 TCP 单位轴在基座中为：

| TCP 局部轴 | 基座 XYZ 分量 | 大致方向 |
|---|---|---|
| +X | `[0,-1,0]` | 基座 -Y |
| +Y | `[0.086982,0,-0.996210]` | 接近向下 |
| +Z | `[0.996210,0,0.086982]` | 接近基座 +X，略向上 |

TCP 在法兰局部 +Z 上偏移 195 mm，且 TCP 旋转偏移为零，所以 TCP 和法兰的轴方向相同。这不表示在基座 +Z 上增加 195 mm。网页基座坐标轴固定，TCP 小坐标轴随末端旋转；均为 X 红、Y 绿、Z 蓝。

**当前手柄局部修正也不是一个孤立的 RPY。** 固定的 `r_adj=RPY(-π,0,-π/2)` 再乘配置 `RPY(-1.5708,0,-1.5708)`，合成 `C` 约为绕原始局部 X 正转 90°。因此在 TCP 模式下，原始手柄矩阵中的局部位移映射为 `+X→TCP +X`、`+Y→TCP -Z`、`+Z→TCP +Y`。这是输入矩阵轴的含义，不能在缺少当前握持姿态时，直接把原始 +Y 等同于操作者的“向上”。

令 `T_raw0=[R_raw0,p_raw0]`，当前倍率为各轴相同的 `s=1.0`，原始世界位移到基座的映射为：

`delta_p_base = s * R_tcp0 * C^T * R_raw0^T * delta_p_raw`

固定世界轴变换 `A` 在 `inv(T_hand0)*T_hand` 中抵消。因此，TCP 模式不要求头显世界系和基座方向相同；改 `adj_mat` 或网页视角不能修复手柄局部轴对应错误。A 只捕获双方参考姿态，没有测量“手柄哪个物理方向对应工具哪个方向”。要判断某次实际操作究竟错了哪个轴，需要那次 A 时的手柄/TCP 参考及操作方向；此前真机日志没有记录参考矩阵，新版 A/RJ 日志会记录，不能据现有日志断言某个固定欧拉角就是现场正确值。

此外，此前 CAN 快照中 joint3 为 `+1.332°`，而当前 URDF 的 joint3 上限为 `0°`。将该快照的真实 TCP 反馈交给现有生产 IK 离线求解，即使目标保持不变，模型内残差仍约 `0.90 mm / 1.23°`；三个基座轴正反向 5 mm 目标的残差约 `0.23–1.57 mm`，并带有其他轴的小分量。它能解释起始修正、斜走或跟随不精确，不能单独证明大幅轴交换。该实验没有修改 URDF 限位、IK 权重或发送运动指令。

## 当前操作流程

在完成首次环境、ROS 构建、CAN 配置与 Quest 授权后运行 `./start.sh`，自动加载项目环境、内部 overlay、ADB，并打开工作台和启动遥操。`./start.sh --terminal can_port:=can0` 使用终端模式；`./start.sh --demo` 为不连接设备的演示。

- A：从当前手柄和当前 TCP 建立增量参考，进入待跟随状态，不回零。
- RG：右把手握持扳机，按住跟随，松开冻结最后已下发目标，重新按住将新的手柄增量接到保持目标，不追赶暂停期间的手柄运动；食指扳机仍独立控制夹爪。
- B：停止遥操与录制，默认延迟 0.5 秒调用原驱动六轴回零；未按 A 或无手柄位姿也能请求，节点自身的停止状态回环不会再取消回零。`teleop.auto_home_on_stop: false` 关闭自动回零。
- RJ：活动遥操中重定位参考，不回零。
- 手柄方向：Quest 转换与 Piper-X 局部 RPY 保持一致；默认 `delta_frame: tcp` 恢复参考 TCP 局部增量。
- 正常运动 `move_js`、速度参数 50%；回零 `move_j`、速度参数 5%，零位容差 0.03 rad、稳定 0.3 秒、超时 45 秒。

松开 RG 不结束 Episode，暂停所有模态写入；再次按住续写同一文件，第一次 RG 才开始录制。HDF5 保存有效区间与各模态分段编号，审核/导出跳过暂停时间；源时间戳保留。实现细节见 [暂停片段的数据采集选择](architecture.md#暂停片段的数据采集选择)。

## 已删除的附加运动保护

采集入口不再叠加每轴速度、加速度、单步、输入跳变、目标领先、跟随误差、控制间隔和实测反馈运动检查。首条目标近邻检查、夹爪相对参考与限速、目标时间戳/字段超时、反馈新鲜度/FK 误差/关节-TCP 时差运动门控，以及 A 的静止/设备就绪/回零前置条件均已删除。驱动指令看门狗关闭，与参考启动链一致。

IK 不再设置 TCP 2 mm/1° 步进、10 mm/5° 精度硬约束、发布前精度检查、反馈年龄门控或额外的求解器 success 标志门控。当前 RG 恢复的首帧及零增量输入直接重复保持目标，是暂停/恢复参考连续性的处理，不是按固定时长冻结或按误差大小拦截。仍使用参考算法中的 URDF 关节位置约束。碰撞检查两者默认关闭；打开时均为发布后的结果报告，不拦截目标。

采集/标定/写盘检查和离线审核继续存在，但只影响录制和数据可用性，不关闭机械臂遥操。界面将其显示为独立的 `recording_error`。

## 移除保护后仍保留的 IK 差异

| 项目 | Data_Collect | QuestArmTeleop / Piper-X |
|---|---|---|
| 运动学与求解器 | Pinocchio + CasADi/IPOPT，六轴位置约束 | 同一求解器；当前 YAML 实际保留夹爪变量，为 7 维优化 |
| 位置残差 | 直接计算基座坐标系中的 XYZ 位置差 | 使用 SE(3) 对数误差 `log6` 的平移分量 |
| `w_pos` / `w_ori` | 20 / 2 | 2 / 2 |
| 位置项权重含义 | 代价为 `sumsqr(w_pos * e_pos)`；同一残差下位置项系数是参考版的 100 倍 | 位置项系数较低 |
| `w_reg=0.01` | 参考实测关节：`||q-q_feedback||²` | 参考零位：`||q||²` |
| `w_smooth=0.1` | 参考上次成功发布的解；反馈更新求解初值与正则参考 | 反馈回调同时重设求解初值和平滑参考 |
| 连续性状态更新 | 关节发布成功后 `accept()`；失败不提交该次解 | 求解返回后即更新历史解 |
| 关节反馈绑定 | 按关节名称匹配 | 按位置数组前 N 个值 |
| 首次求解 | 首次反馈时预热求解器，不发布目标 | 首个目标到达后求解 |
| 工具 TCP | 统一为 0.195 m；支持 TCP RPY 偏移 | 统一为 0.195 m |
| IPOPT 参数 | 最多 50 次迭代，`tol=1e-4`，调用 `solve_limited()` | 相同 |
| 日志和退出 | 保留求解耗时/状态/残差诊断与幂等 ROS 退出 | 主要报告求解异常 |

权重、正则和平滑属于求解偏好，不是速度或误差硬门限。TCP 已按要求统一为 195 mm，手柄方向修正已与参考 Piper-X 启动项统一；夹爪范围仍使用各工程自己的设备配置。

源码位置：`ros_ws/src/oculus_reader/scripts/arm_ik_pose_node.py`、`ros_ws/src/oculus_reader/config/arm_ik_pose_node.piper.yaml`、`launch/collect.launch.py`、`data_collect/application/collector.py`。

195 mm TCP 下已使用真实 Pinocchio/CasADi 执行两套 ArmIK，数值结果及零位边界缺陷见 [精度与速度实测](ik_accuracy_speed_195mm.md)。当前默认已恢复 TCP 增量，坐标一致性、参考公式和实际反馈快照的 IK 行为均只做离线检查，没有执行机械臂运动。
