# 现场设备准备与采集

`start.sh` 自动加载环境、启动 ADB 和内部 ROS 节点。设备授权和 CAN 配置需要首次完成；后续从工程目录执行 `./start.sh` 即可。

首次使用先按 [工程 README](../README.md) 重建环境与内部 ROS overlay；克隆不包含已验证现场标定。旧版 Piper 模型保留，当前采集仍配置为 Piper-X。

**1. 首次配置 CAN**

```
cd /path/to/Piper-main/Data_Collect
source env.sh

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

**2. 检查 Quest，启动工作台**
```
./start.sh --can-port can0
```

**3. 打开工作台：** [http://127.0.0.1:8766/](http://127.0.0.1:8766/)

**4. 界面上开始采集**
1. `start.sh` 已自动启动 ROS 和工作台并使能机械臂；停止程序后可在界面再次启动。
2. 等待四个节点、机械臂反馈、所选相机及左右触觉就绪。
3. 按手柄 **A**：将当前手柄与当前 TCP 对齐；此时等待 RG，不回零。
4. 按住把手 **RG** 才跟随并开始录制；以终端 `Recording ...` 和界面录制状态为准。松开 RG 保持并暂停写入，再按住续写同一条轨迹；图像或标定尚未就绪会显示录制错误，遥操继续。
5. 完成一条轨迹后按 **B**，等待封盘及默认自动回零；下一条按 **A** 重新对齐。

**5. 结束采集**

点击 **「停止程序并保存」**，等待进程退出和写盘完成。关闭浏览器页面不会停止程序。
原始数据保存在：
```
/home/l1/Piper_All/Piper/Data_Collect/data/raw/
```

若审核页没有刚才的轨迹，先查看采集终端是否出现 `Recording .../episode_XXXX.h5`。`无法开始录制，遥操继续` 表示该次仅完成遥操，没有生成数据文件。审核启动时会打印本检出目录下的实际 `data/raw` 扫描路径；页面“刷新列表”会重新扫描和质检已封盘文件。`.h5.partial` 会以未完成文件列出，不能通过审核。

相机只读检查使用 `source env.sh capture` 后运行 `python -m data_collect.cli.cameras`。只安装 `setup_teleop.sh` 的环境尚无完整采集依赖，需要按 README 补装 RealSense、Tac-X SDK 和运行库，并完成实际设备标定。GoPro HDMI/UVC 连接与延迟标定见 [相机说明](cameras.md)。



## 标定

**1. 首次采集前核对配置**

```
cd /path/to/Piper-main/Data_Collect
source env.sh

mkdir -p data/calibration
cp -n config/calibration.example.json data/calibration/calibration_v1.json
nano config/collection.yaml
nano data/calibration/calibration_v1.json 
```

确认任务指令、操作员、左右触觉设备和 TCP 参数。当前两套遥操统一使用 `[0,0,0.195,0,0,0]`，已有现场标定文件也需同步更新并重新核验。完成实际标定核验后，将标定文件中的 `"verified": false` 改为 `"verified": true`。这一步不是自动标定命令。

**2、延迟标定**

nano data/calibration/calibration_v1.json 中：

```

  "relative_offsets_ns": {
    "global_rgb": 0,
    "wrist_rgb": 0,
    "left_raw": 0,
    "right_raw": 0
  },
  
  "tracking_lag_notes": "待测量；available_host_ts_ns 不做回拨校正"
```


**3、延迟标定测量脚本**

```bash
python3 -m data_collect.cli.calibrate_latency record --seconds 20
python3 -m data_collect.cli.calibrate_latency measure --input data/calibration/latency_实际时间.h5
```

录制仅连接图像设备，不连接机械臂。测量成功后可加 `--write` 写入标定文件；相对补偿尚未接入生产审核/导出，原始可用时间戳不回拨。
