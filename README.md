# Piper

多人协作开发仓库。

## 目录结构

```text
Piper/
├── member1/        # 组员 1 的代码
├── member2/        # 组员 2 的代码
├── member3/        # 组员 3 的代码
├── shared/         # 公共代码与共享资源
├── .gitignore
└── README.md
```

`member1`、`member2`、`member3` 是初始占位目录，可根据实际成员或模块重命名、扩展。目录划分是协作约定，**不是 GitHub 权限隔离**。

## 协作约定

1. 首次使用：`git clone https://github.com/cmoaDij/Piper.git`。
2. 开始任务前更新本地 `main`：`git switch main && git pull --ff-only origin main`。
3. 每项任务从 `main` 新建分支：`git switch -c feat/用户名-任务名`。
4. 只修改本人负责的目录；修改 `shared/` 或他人目录前先沟通。
5. 提交本次任务相关文件，推送任务分支并创建 Pull Request，审核后合并到 `main`。
6. 禁止强制推送、覆盖他人代码或提交密码、Token、API Key、`.env` 等敏感信息。

> 建议仓库管理员为 `main` 配置分支保护，要求通过 Pull Request 合并。该规则需在 GitHub 设置中单独启用。
