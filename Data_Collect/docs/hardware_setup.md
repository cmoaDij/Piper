# 现场设备准备与采集

首次使用先按 [工程 README](../README.md) 重建环境与内部 ROS overlay；克隆不包含已验证现场标定。旧版 Piper 模型保留，当前采集仍配置为 Piper-X。

**1. 进入环境并检查 CAN**

```
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

**2. 检查 Quest，启动工作台**
```
adb start-server
adb devices

python3 -m data_collect.cli.gui
```

**3.打开：**[http://127.0.0.1:8766/](http://127.0.0.1:8766/)****

**4. 界面上开始采集**
1. CAN 接口填写 `can0`，点击 **「启动遥操程序」**，机械臂会按配置使能。
2. 等待四个节点、机械臂反馈和四路传感器就绪。
3. 按手柄 **A**：低速回零，完成后自动对齐当前手柄参考。
4. 开始移动手柄或改变夹爪开度，自动开始录制。
5. 完成一条轨迹后按 **B**，等待封盘；下一条重新按 **A**。

**5. 结束采集**

点击 **「停止程序并保存」**，等待进程退出和写盘完成。关闭浏览器页面不会停止程序。
原始数据保存在：
```
/home/l1/Piper_All/Piper/Data_Collect/data/raw/
```



## 标定

**1. 首次采集前核对配置**

```
cd /home/l1/Piper_All/Piper/Data_Collect
source env.sh

mkdir -p data/calibration
cp -n config/calibration.example.json data/calibration/calibration_v1.json
nano config/collection.yaml
nano data/calibration/calibration_v1.json 
```

确认任务指令、操作员、左右触觉设备和 TCP 参数。完成实际标定核验后，将标定文件中的 `"verified": false` 改为 `"verified": true`。这一步不是自动标定命令。

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
