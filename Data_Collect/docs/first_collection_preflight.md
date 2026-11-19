# 第一次采集前的 TCP、触觉身份与基线核验

本指南适用于当前 Data_Collect / Piper-X / 手腕 GoPro 配置。按“只读检查 → 仅图像试录 → TCP 实体核验 → 首条完整轨迹”执行。`verified` 是人工核验记录，不是自动标定按钮。

2026-11-17 本机检查：GoPro 接收延迟文件已有 60 个有效样本；触觉 SDK 枚举仍为 `[]`。本指南的图像试录和运动步骤尚未在本轮现场执行。

## 1. 加载环境

后续命令均在同一终端运行：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
```

先退出占用相机或触觉的其他采集程序。下面的枚举、配置打印和文件检查不连接机械臂；标有“会使能机械臂”的命令需要完成现场工作区和急停检查后再执行。

## 2. 左右触觉设备：确认实体与序列号一一对应

给两块传感器及其线缆贴固定 L/R 标签，明确标签与夹爪安装位置的对应关系。不要随观察方向或相机画面翻转交换标签。

仅连接 L，执行下面的枚举，记下序列号；再仅连接 R 重复一次；最后同时连接，确认出现两个不同 ID。

```bash
python - <<'PY'
from dmrobotics import listConnectedDevIDs
print("触觉设备：", listConnectedDevIDs())
PY
```

当前配置预填的是：

| 物理标签 | 程序流 | 预填 ID |
| --- | --- | --- |
| L | left_raw | X2631410810 |
| R | right_raw | X2631410795 |

以上 ID 必须与实测相符。若不同，修改下面两处，再重新运行枚举：

```bash
nano config/collection.yaml
nano data/calibration/calibration_gopro_v1.json
```

YAML 修改 `tactile.left_raw.dev_id` / `tactile.right_raw.dev_id`；JSON 修改 `tactile_ids.left_raw` / `tactile_ids.right_raw`。两处必须一致，并保持 `verified=false`。

## 3. 无接触基线：记录“未受压时的参考图像”

基线是触觉传感器未接触物体、未受压时的图像，通常不是全黑或全零图，也不是力传感器归零后的牛顿值。当前工程采集 raw 图像，不要求先完成力值换算标定。

准备时移开物体、手指和线缆，保证两块触觉表面均不受压；检查夹爪开度和安装件是否使传感器相互挤压。采集代码在 SDK 就绪后调用 `getBaseFrame()` 读取参考图，确认其尺寸与 raw 图像一致，并写入每条录制的 `tactile/left_raw/baseline` 和 `tactile/right_raw/baseline`。

下面复用现有图像录制命令检查基线和响应。它打开 GoPro 和左右触觉、保存图像，**不连接机械臂、不发运动命令，也不要求提前设置 `verified=true`**：

```bash
sensor_check_file="data/calibration/sensor_check_$(date +%Y%m%d_%H%M%S).h5"
python -m data_collect.cli.calibrate_latency record \
  --seconds 20 \
  --output "$sensor_check_file"
```

初始化和等待就绪期间两侧保持无接触。终端打印“开始 3 路图像标定录制”后，按下面顺序操作；时间段只用于辨认测试动作，不是验收阈值：

1. 前 5 秒，两侧均不接触。
2. 5–10 秒，用指腹轻触 L，R 不接触。
3. 10–15 秒，松开 L，仅轻触 R。
4. 最后 5 秒，两侧均松开。

工具名称含 latency；这里仅使用 `record` 检查图像、基线和设备对应关系，不运行 `measure` 或 `--write`，不改变已经测得的 GoPro 接收延迟。

完成后检查文件和两个基线：

```bash
python - "$sensor_check_file" <<'PY'
import sys
import h5py

with h5py.File(sys.argv[1], "r") as f:
    print("完整封盘：", bool(f.attrs.get("complete", False)))
    print("采集错误：", f.attrs.get("collection_error", ""))
    print("丢样计数：", f.attrs.get("dropped_samples", 0))
    for name in ("left_raw", "right_raw"):
        group = f["tactile/" + name]
        print(name,
              "帧数=", len(group["frame_id"]),
              "基线尺寸=", group["baseline"].shape,
              "原始图尺寸=", group["data"].shape[1:])
PY
```

导出基线及四个操作阶段的图像，人工检查表面和响应：

```bash
python - "$sensor_check_file" <<'PY'
import sys
from pathlib import Path
import cv2
import h5py
import numpy as np

path = Path(sys.argv[1])
with h5py.File(path, "r") as f:
    start = int(f.attrs["start_host_ts_ns"])
    for name in ("left_raw", "right_raw"):
        group = f["tactile/" + name]
        elapsed = (group["available_host_ts_ns"][:].astype(np.int64) - start) / 1e9
        samples = [("baseline", group["baseline"][:])]
        for seconds in (2, 7, 12, 18):
            index = int(np.argmin(np.abs(elapsed - seconds)))
            samples.append((f"t{seconds}s", group["data"][index]))
        for label, image in samples:
            if image.ndim == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            output = path.with_name(f"{path.stem}.{name}.{label}.png")
            if not cv2.imwrite(str(output), image):
                raise RuntimeError(f"无法保存 {output}")
            print(output.resolve())
PY
```

重点核对：基线对应无接触状态；仅触 L 时 L 图像出现可辨识局部形变，仅触 R 时 R 响应；释放后回到相近的无接触外观。表面本身纹理不同是正常的，不要求左右基线相同。若启动时受压或返回的参考图明显不正确，移开接触并重新初始化、复查；仍不正确时使用厂家支持的基线重建流程。不要用受压图像替代基线来使检查通过。

运行失败时先解决明确的设备缺失、SDK 状态、基线缺失或图像过期错误，保留日志；不要把 `verified` 改为 true 代替排查。

## 4. 实际 TCP：确认软件使用的点就是实体抓取点

TCP 是工具中心坐标系。本工程当前定义为：相对于模型 `link6` 坐标系原点，沿局部 Z 正向 0.195 m，工具姿态偏移 RPY 均为 0：

```yaml
tcp_offset: [0.0, 0.0, 0.195, 0.0, 0.0, 0.0]
```

它不是相机到夹爪的距离，也不自动等于从外壳任意表面量到指尖的距离。先明确实际任务采用的抓取中心点，再依据装配图、CAD 或已知坐标参考面，确认 `link6` 到该点的位移和工具轴方向。夹指、触觉垫或安装件改变了抓取点时，需要重新核对。

先打印配置，命令不连接机械臂：

```bash
python - <<'PY'
import json
from data_collect.common.config import load_config, local_path

cfg = load_config()
profile = cfg["camera_selection"]["gopro"]
cal = json.loads(local_path(profile["calibration_file"]).read_text())
print("模型/参考：", cfg["arm_type"], cfg["base_link"], cfg["tip_link"])
print("配置 TCP：", cfg["tcp_offset"])
print("现场标定 TCP：", cal["tcp_offset"])
PY
```

实体检查分两步：

1. **几何核对**：检查夹爪、触觉垫、工具支架的安装尺寸，确认实际抓取中心与 195 mm 定义相符，工具坐标轴方向符合任务约定。
2. **姿态检查**：在空旷区域、固定目标 TCP 位置时小幅改变工具姿态，观察实体抓取中心是否绕目标点明显画弧。检查时同时确认目标平移没有发生变化；漂移也可能来自遥操输入、跟踪误差或机械间隙，不能仅凭一次目测就修改偏移。

若几何尺寸未知或发现系统性偏移，应使用固定点、多姿态方法重新求工具平移，并单独核对工具姿态。固定点需要不同方向的充分姿态变化；至少三种姿态、另加一种独立姿态复核，是厂商文档介绍的常用做法。该方法的原理可参考 [TCP 固定点标定说明](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-usr-man/software/PolyScope/content/installation_g5/teaching_TCP_position.htm)；这是原理参考，当前 Piper 工程没有该厂商的标定向导，不能把其界面操作当成本项目命令。仅用固定点不能确定工具全部方向，也不能把一根额外伸出的校准针针尖直接当作原抓取中心。

需要通过现有遥操进行姿态检查时，下面命令**会连接并使能机械臂**。先确认 CAN 映射、模型、可运动空间、工具固定和急停；空夹爪开始，不同时运行其他控制程序。下面使用 5% 作为首次人工检查速度，并关闭 B 自动回零；5% 是本项目现有回零速度值，不是安全认证或验收标准：

```bash
./start.sh --terminal can_port:=can0 \
  teleop_only:=true \
  fast_mode:=false \
  speed_percent:=5 \
  auto_home_on_stop:=false
```

A 对齐，按住 RG 才跟随，松开 RG 保持；本模式不录制。姿态检查结束退出该进程。不要启动上述命令期待自动完成 TCP 标定。

如果测得 TCP 与配置不同，修改 `config/collection.yaml` 的 `tcp_offset` 和 `data/calibration/calibration_gopro_v1.json` 的同名字段，再重启。当前启动文件将 YAML 同时传给驱动、IK 和 FK；本次方案不修改其他项目的控制器。

## 5. 完成核验后启用整体标定，再录第一条短轨迹

完成实体 TCP 核对、L/R 身份确认、无接触基线及响应检查后，记录操作员、安装状态和核验日期，再修改：

```bash
nano data/calibration/calibration_gopro_v1.json
```

将 `verified` 从 false 改为 true，并更新现场核验备注；不要重新覆盖为模板。软件检查只能确认配置及标定字段一致，不能替人判断实际安装和无接触状态：

```bash
python - <<'PY'
from data_collect.common.config import load_config
from data_collect.sensors.camera_selection import resolve_cameras
from data_collect.application.collector import validate_calibration

cfg = load_config()
resolve_cameras(cfg)
validate_calibration(cfg)
print("设备标识、TCP 配置和 GoPro 延迟标定文件检查通过。")
PY
```

然后运行完整采集。下面命令会使能机械臂，保持前述现场准备；初始化期间两侧触觉保持无接触：

```bash
./start.sh --terminal can_port:=can0 \
  fast_mode:=false \
  speed_percent:=5 \
  auto_home_on_stop:=false
```

等待所有图像流和两个基线就绪，A 对齐，按住 RG 做一条约 5–10 秒的简单抓取/释放轨迹，松开暂停，B 结束封盘。必须看到 `Recording .../episode_XXXX.h5`；仅有遥操运动不表示已写入。核对 `data/raw/` 中实际 HDF5 和同名手腕 MP4，再进行审核。5–10 秒的安排落在当前审核的 5–30 秒时长范围内；其他动作和接触质检仍按现有规则判断。

基线会在每次采集运行时重新读取；一次准备试录成功不代表以后可以在传感器受压状态下初始化。首次短轨迹还应检查 GoPro 图像、机器人反馈/指令和触觉事件的对应关系；已有 QR 延迟估计不能替代这一步。
