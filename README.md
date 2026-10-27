# Panda Motion Planning Lab

在同一个 Franka Panda 场景里，比较关节直连、RRT-Connect、RRT*、PRM，以及 none、人工势场 APF、距离约束速度 QP。规划输出七维关节路径，执行由 MuJoCo 真实动力学、直接力矩 PD 和重力补偿完成。夹爪固定，七轴参与运动。

## 从 GitHub 克隆并学习

仓库：<https://github.com/zhangfuyong888/franka_planning>。仓库根目录就是 Python 项目根目录，无需再进入 `panda-motion-planning-lab` 子目录。下面以 Linux、Python 3.10–3.12 为例：

```bash
git clone https://github.com/zhangfuyong888/franka_planning.git
cd franka_planning
PYTHON=python3.12 bash scripts/install.sh
source .venv/bin/activate
python -m panda_planning.demo --scene single_obstacle --planner rrt_connect --backend ompl --trace --output outputs/first-run
```

将 `python3.12` 换成机器上已有的 `python3.10` 或 `python3.11` 也可以。需要先安装相应 Python 的 venv 支持；没有桌面显示时为演示添加 `--headless`。虚拟环境与 `outputs/` 不上传，模型、第三方许可和现成教学报告随仓库保存。

生成网页回放：

```bash
python -m panda_planning.replay --run-dir outputs/first-run --html outputs/first-run/replay.html
xdg-open outputs/first-run/replay.html
```

学习顺序：本 README → [算法教程](docs/algorithms.md) → [架构与数据流](docs/architecture.md) → [实验结果](docs/experiments.md)。维护者提交、同步和 SSH 设置见 [仓库维护说明](docs/repository-management.md)。

本次完善新增：真实 EGL 离屏图片、APF 停滞教学配置、同一路径的 hybrid 三策略套件、故障记录与命令监测、已经用真实 OMPL 2.0.1 验证的规划后端。运行记录和 PNG 帧均带实际 run_id，不将回放冒充新的动力学执行。

OMPL 2.0.1 已在项目虚拟环境安装，RRT-Connect、RRT*、PRM cold/warm 的真实集成测试通过。修复了 PRM 短查询时间片及回调引用环；未将近似解当成功，也未屏蔽 nanobind 泄漏提示。默认 auto 现优先调用真实 OMPL，替代后端仍可显式指定。

用户已在宿主机确认网页可打开，MuJoCo 实时演示运行成功（run `187ced8983e6`，8.77 s、末端误差约 0.681 mm、监测干预 0）。浏览器/窗口能打开与全部交互按钮目视验收分开记录。没有移动障碍或在线重规划。

## 先看成果

* [OMPL 最新报告](docs/evidence/ompl-release/tutorial-report.html)、[OMPL 完整验收](docs/evidence/ompl-release/acceptance.json)：真实库、真实 trace、cold/warm 和独立动力学执行。
* [宿主机显示确认与逐项交互状态](docs/evidence/ompl-release/human-validation.json)。

下面的 v0.2.0 报告保留为替代后端的历史记录，其路径和数值未改写成 OMPL 结果：

* [最新验收与全局/局部报告](docs/evidence/completion/tutorial-report.html)、[最新验收记录](docs/evidence/completion/acceptance.json)。
* [真实 MuJoCo 全局四栏图片](docs/evidence/completion/frames/global/comparison-t-4.png)、[局部停滞三栏图片](docs/evidence/completion/frames/local/comparison-t-8.png)。
* [APF 停滞对照报告](docs/evidence/completion/apf-trap-report.html)、[同一 plan_id 的 hybrid 报告](docs/evidence/completion/hybrid-report.html)。

以下原始交付入口保留为 2026-10-24 历史实验，其运行 ID 和数值没有改写成新版本：

* [交互对照报告](docs/evidence/tutorial-report.html)：全局 2×2、局部三栏，共用物理时间轴、规划快照时间轴、整臂与固定手指、真实障碍、路径/搜索图/最近点图层、距离/误差/局部诊断曲线。浏览器直接打开，无 CDN、账号或服务。
* [验收记录](docs/evidence/acceptance.json) 和 [实际实验表](docs/evidence/results.csv)。报告是“基于记录状态的回放”，不是重新计算的动力学或效果动画。
* 默认 seed=42 单障碍：直连失败；RRT-Connect、PRM 规划并执行成功；RRT* 在短预算内超时。失败保留，不用零路径长度代替无解。

## 当前工作区立即运行

```bash
cd /home/zfy/franka_planning/panda-motion-planning-lab
source .venv/bin/activate
python -m panda_planning.demo --scene single_obstacle --planner rrt_connect --headless --trace --output outputs/first-run
python -m panda_planning.replay --run-dir outputs/first-run --html outputs/first-run/replay.html
python -m pytest -q
```

无 `--headless` 时打开 MuJoCo passive viewer；需要本机显示权限。空格暂停/继续，N 单步，P 路径，G 搜索图末端投影，D 距离，C 视觉网格/碰撞网格切换。GUI 使用英文数值 HUD 避免 MuJoCo 默认字体缺中文；教程和 HTML 为中文。规划在执行前离线进行，GUI 显示规划墙钟耗时；过程快照在 HTML 的规划滑块中查看。

## 在新环境安装

推荐 Python 3.11/3.12；也支持本机实际验证的 3.10.20。当前 3.12 解释器可用但缺少离线依赖轮子，因此选择 3.10 在独立 `.venv` 中复制已有发行包后运行 `pip check`、wheel 安装和真实测试；没有修改全局环境，也不依赖原项目的源码或资产路径。网络正常的新机器可执行：

```bash
cd franka_planning
PYTHON=python3.12 VENV_DIR=.venv-new bash scripts/install.sh
source .venv-new/bin/activate
# 默认安装脚本已包含 OMPL；单独补装或验证可执行：
python -m pip install 'ompl==2.0.1'
python scripts/validate_ompl.py
python -m panda_planning.demo --scene empty --headless --backend ompl
```

主依赖精确固定在 pyproject.toml；完整已测发行包列表在 `requirements/locked-py310-linux.txt`，安装使用排除 pip 的 `requirements/locked-runtime.txt`，避免把 Python 3.12 自带的 pip 降级成旧 3.10 环境版本。安装脚本包含 OMPL，不覆盖已有环境；最小安装仍支持显式替代后端。OMPL 缺失时 `auto` 只在创建规划器时选择标注的替代后端；`--backend ompl` 则明确失败，不在求解失败后偷换算法。重新求解受墙钟和机器负载影响，同 seed 不保证逐位相同；保存记录可确定回放。

## 新增：真实图片、APF 停滞与 hybrid 比较

```bash
# 无 GUI 的真实 MuJoCo 实测状态渲染；显式创建 EGL Renderer。
python -m panda_planning.render --run-dir outputs/first-run --times 0 4 9 --output outputs/frames
# 增大 APF 自身增益后可观察实际 stalled，而不是把 guarded_stop 称为局部极小。
python -m panda_planning.demo --scene local_trap --mode local --avoider apf --config configs/apf-trap.yaml --headless --output outputs/apf-stall
# 只规划一次，再由三种策略读取同一保存路径。
python -m panda_planning.benchmark --suite hybrid --headless --trace --output outputs/hybrid-comparison
# 在有 EGL 的机器执行包括真实离屏渲染的完整测试。
PANDA_TEST_RENDER=1 python -m pytest -q
```

普通 `--headless` 仍不会创建 Renderer，不要求 OpenGL。`render --compare 运行目录1 运行目录2 ... --times 4 --output outputs/compare-frames` 可同相机拼图；拒绝不同 scene_id，结束早的运行保持实际末态。每帧 frames.json 保存请求时间、实际记录时间和运行/场景/路径 ID。

## 选择实验

```bash
# 对照：同一保存场景，独立进程，预算相同；默认有限时长结束。
python -m panda_planning.benchmark --suite tutorial --headless --trace --budget 3 --output outputs/tutorial-benchmark
python -m panda_planning.report --input outputs/tutorial-benchmark --output outputs/tutorial-benchmark/report.html
# 局部实验：目标关节反馈没有经过全局避障验证。
python -m panda_planning.demo --scene local_trap --mode local --avoider apf --headless --output outputs/local-apf
python -m panda_planning.demo --scene local_trap --mode local --avoider distance_qp --headless --output outputs/local-qp
# 组合：三种局部方法应使用同一 plan.json / plan_id。
python -m panda_planning.demo --scene-file outputs/first-run/scene.json --mode hybrid --avoider distance_qp --plan-file outputs/first-run/plan.json --headless --output outputs/hybrid-qp
# 随机场景先冻结，再求解。
python -m panda_planning.scenes --generate random_static --seed 42 --output outputs/scene-42.json
python -m panda_planning.demo --scene-file outputs/scene-42.json --headless --output outputs/random
# 同场景十个查询的 PRM cold/warm：结果为 JSON，成本不混入全局排行榜。
python -m panda_planning.benchmark --suite prm --headless --output outputs/prm-queries
# 真实记录的 MuJoCo 回放；不会改变新的运行实验。
python -m panda_planning.replay --run-dir outputs/first-run
```

可选 `--budget`、`--max-time`、`--seed`、`--config configs/local.yaml`、`--target x y z`（世界坐标 m）、`--scene-file`。固定场景包括 empty、single_obstacle、elbow_obstacle、narrow_passage、local_trap、random_static、multi_query、invalid_cases。`invalid_cases` 的不可达 IK 与规划超时分开保存。

每次输出 scene/request/plan/trace/telemetry/summary/collision_pairs/display JSON，包含输入、独立规划/执行状态、种子、环境、代码指纹、原始/处理后路径、真实 q/dq/力矩/诊断。JSON 无 NaN、无无解的假零代价。未执行的规划失败保持起始姿态并显示原因。

## 阅读与练习

1. 本 README：运行并看同场景区别。
2. [架构与数据流](docs/architecture.md)：接口、状态隔离、日志和执行。
3. [算法教程](docs/algorithms.md)：公式、一次迭代、对应代码和小练习。
4. [实验与局限](docs/experiments.md)：实测表、失败解释、复现和改进。

`examples/01_collision.py`、`02_global_compare.py`、`03_local_step.py`、`04_prm_queries.py` 调用实际模块。第三方来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。场景和模型随 wheel 打包，从其他工作目录也能运行，首次启动不用下载资产。
