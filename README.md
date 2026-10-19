# Piper

Piper 遥操作与多传感器数据采集工程。全部采集代码、ROS 包、设备 SDK、模型、测试和运行说明位于 [`Data_Collect/`](Data_Collect/README.md)。

```text
Piper/
├── Data_Collect/    # 独立、分模块的数据采集工程
├── .gitignore
└── README.md
```

## 开始使用

```bash
git clone https://github.com/cmoaDij/Piper.git
cd Piper/Data_Collect
# 按 Data_Collect/README.md 重建环境和 ROS overlay
source env.sh
python3 -m data_collect.cli.gui --demo --no-browser --port 8767
```

当前采集使用 Piper-X + AGX 夹爪、Quest、双 RealSense 和双 Tac-X，同时保留旧版 Piper 模型。首次实机运行前须核对设备映射、标定与急停。

## 协作

每项任务从最新 main 创建功能分支，提交后创建面向 main 的 Pull Request，经审核后合并。PR 说明行为变化、验证结果及设备假设；界面变化附截图。禁止强制推送及提交设备密钥、密码、Token、`.env`、本地环境或真实采集数据。
