# Git管理

## 从github直接拉取最新版本的仓库，覆盖本地文件

```bash
# 1. 获取远程最新信息
git fetch origin

# 2. 强制将当前分支重置为远程分支的状态（例如 main 或 master）
git reset --hard origin/main
```