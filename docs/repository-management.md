# 仓库维护

远程仓库为 `git@github.com:zhangfuyong888/franka_planning.git`，主分支为 `main`。当前机器的仓库目录是 `/home/zfy/franka_planning/panda-motion-planning-lab`；其他人克隆后直接在 `franka_planning/` 根目录工作。

## 提交与同步

修改前确认工作区状态；工作区干净时拉取远程更新：

```bash
git status
git pull --ff-only
```

完成一个可说明的修改后，检查差异、运行相关测试，再提交推送：

```bash
source .venv/bin/activate
python -m pytest -q
git diff
git add README.md docs/ src/ tests/ configs/ examples/ scripts/ requirements/ pyproject.toml .gitignore .github/
git diff --cached --stat
git commit -m "说明本次修改的功能或修复"
git push
```

按实际修改选择 `git add` 路径。不要使用 `git push --force` 覆盖别人的提交。推送被远程更新拒绝时，先 `git fetch origin` 并检查 `git log --oneline --graph --all -15`，再决定合并方式。

多人开发建议使用独立分支：

```bash
git switch -c feature/describe-change
# 修改、测试、提交后
git push -u origin feature/describe-change
```

随后在 GitHub 创建 Pull Request，由测试和代码审查确认后合入 main。

## SSH 身份

当前维护者机器通过仓库本地 `core.sshCommand` 指定账号 `zhangfuyong888` 对应的私钥。GitHub 的 SSH 用户始终是 `git`，账号身份由密钥决定。此配置只保存在本机 `.git/config`，不会随克隆传播。

```bash
git remote -v
git config --local --get core.sshCommand
```

如果需要使用本机已经配置好的 `zfy-personal` SSH 别名和 SOCKS 代理，可对当前仓库设置：

```bash
git config --local core.sshCommand 'ssh -o Hostname=github.com -o HostKeyAlias=github.com -o ProxyCommand="nc -X 5 -x 127.0.0.1:7890 %h %p" -i ~/.ssh/id_ed25519_zhangfuyong888 -o IdentitiesOnly=yes'
git push -u origin main
```

该代理选项仅适用于维护者本机已经运行 7890 端口 SOCKS 代理的情况。其他贡献者使用自己的 GitHub SSH 密钥；只克隆学习可使用 README 的 HTTPS 地址，无需写权限。

## 上传范围与检查

纳入版本管理：源码、完整模型资产、配置、示例、测试、安装脚本、中文文档、第三方许可及已有教学验收报告。忽略：`.venv*`、`outputs/`、构建产物、缓存、私钥、`.env` 和本机账号配置。历史验收报告对应生成时的代码版本，不代表未来每次修改都已重新验收。

GitHub Actions 在 push 和 Pull Request 时，对 Linux 的 Python 3.10、3.12 执行依赖检查和无窗口测试。首次云端结果以仓库 Actions 页面为准；显示窗口和 EGL 渲染仍需在支持的机器单独验证：

```bash
python scripts/validate_ompl.py
PANDA_TEST_RENDER=1 python -m pytest -q
```

发布版本前检查测试、更新版本与文档，再用新标签标记经过验证的提交；不要移动已有发布标签。
