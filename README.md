# Piper

多人协作开发仓库。

## 目录结构

```text
Piper/
├── yyy5000/        # yyy5000 的个人开发目录
│   └── Piper_DM/   # 遥操作、数据采集工程、SDK 和文档
├── .gitignore
└── README.md
```

当前代码统一位于 `yyy5000/Piper_DM/`，运行说明见 [个人目录说明](yyy5000/README.md)。其他成员后续可创建各自的个人目录。目录划分是协作约定，**不是 GitHub 权限隔离**。

## 协作约定

1. 首次使用：`git clone https://github.com/cmoaDij/Piper.git`。
2. 开始任务前更新本地 `main`：`git switch main && git pull --ff-only origin main`。
3. 每项任务从 `main` 新建分支：`git switch -c feat/用户名-任务名`。
4. 只修改本人负责的目录；修改 `shared/` 或他人目录前先沟通。
5. 提交本次任务相关文件，推送任务分支并创建 Pull Request，审核后合并到 `main`。
6. 禁止强制推送、覆盖他人代码或提交密码、Token、API Key、`.env` 等敏感信息。

> 建议仓库管理员为 `main` 配置分支保护，要求通过 Pull Request 合并。该规则需在 GitHub 设置中单独启用。
