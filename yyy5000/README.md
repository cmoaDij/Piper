# yyy5000 个人开发目录

`Piper_DM/` 包含 Piper 遥操作、双相机与双触觉数据采集项目，运行说明见 [Data_Collect/README.md](Piper_DM/Data_Collect/README.md)。

- `Piper_DM/Data_Collect/`：独立采集工程、可视化工作台、自动质检、人工审核和 LeRobot 导出。
- `Piper_DM/QuestArmTeleop/`：原遥操作及策略桥接工程。
- `Piper_DM/DM_SDK/`：触觉 SDK。
- `Piper_DM/docs/`：设计方案。

采集数据、录制文件、日志、构建目录、本机 Python 环境和设备认证密钥不纳入 Git；SDK、URDF、网格、安装包等必要资源保留。新机器需要按采集工程说明重建环境。

## 协作流程

```bash
git clone https://github.com/cmoaDij/Piper.git
cd Piper
git switch main
git pull --ff-only origin main
git switch -c feat/yyy5000-任务名
# 仅修改 yyy5000/ 内本人的文件
git add yyy5000/
git commit -m "Describe the change"
git push -u origin feat/yyy5000-任务名
```

推送后创建指向 `main` 的 Pull Request，等待审核后合并。修改 `shared/` 或其他成员目录前先沟通；禁止强制推送以及提交密码、Token、API Key、`.env` 等敏感信息。

本次上传分支为 `feat/yyy5000-piper-dm`，基于仓库 `main` 创建；不会直接修改或合并 `main`。
