# 从绕开一个球开始理解算法

先运行 `python examples/01_collision.py`。起点和终点分别合法，整条边却不合法。七个关节角组成向量 q∈R⁷，同一个末端位置可能对应不同肘部姿态。只盯着末端，无法判断整条机械臂有没有碰到球。本项目把“选择构型”“连接构型”“给速度”“真实执行”分开学习。

## FK、Jacobian、IK 与共同几何

正运动学 FK 把 q 变成末端位置 x=f(q)。Jacobian 是局部线性关系：ẋ≈J(q)q̇，J 的每一列表示相应关节小转动对末端位置的影响。`robot.py: Robot.fk / jacobian` 调用 MuJoCo FK 和 mj_jacSite；不依赖自己写的简化七杆模型。

IK 是反过来从目标位置找关节角。`ik.py: solve_ik` 从固定 HOME 初值创建 mink.Configuration，设 FrameTask，迭代求受限小速度并积分，每次核验 FK 误差。位置权重 1，姿态软权重默认 0.01；`orientation_cost`、迭代数和容差是可配置参数。返回后检查有限位关节和终点碰撞。比较前固定并保存同一个 q_goal，规划器不各自重选 IK 分支。不可达的 [3,0,3] 单独记录 ik_unreachable。

```text
configuration ← 固定初值
重复至预算：
    v ← mink.solve_ik(位置主任务 + 姿态软偏好 + 关节限位)
    configuration.integrate_inplace(v, dt)
    达到位置容差 → 再核验边界与终点碰撞
```

练习：`python -m panda_planning.demo --scene empty --target .42 .05 .60 --headless --output outputs/ik-exercise`。比较保存的 target、q_goal、FK 位置，不用关节角相近替代位置误差验证。

`collision.py` 对 robot collision mesh、固定手指、环境球/盒/胶囊和必要自碰对统一查询。相邻连杆/固定装配允许接触，基座允许安装在平面；具体名单在每次输出的 collision_pairs.json，不把所有自碰关闭。距离不使用渲染网格，视觉网格和凸碰撞网格可用 C 切换观察。

`mj_geomDistance` 返回 signed distance：正数表示分离，负数表示穿透。但超过 distmax 时返回的是截断上界，最近点不能用于梯度。本项目只返回激活范围内的样本，最近点退化/穿透/非有限值时 valid=False，局部策略明确失败并触发统一停止，绝不把坏法向除以零。

最近点 A 指向 B 的单位法向 n，有：

\[
\nabla_q d=n^T(J_B-J_A).
\]

静态障碍的 J_B=0；自碰要算两个连杆的点 Jacobian。这里的点在 world，梯度单位 m/rad。MuJoCo 3.3.7 的 native CCD 在本机出现分离网格 distance=0、两最近点却分离的异常；`Robot.__init__` 禁用 nativeccd、提高 CCD 精度后，使用见证点一致性与简单指垫盒/球有限差分回归。网格接触特征切换处不可微，有限差分误差不能推广为解析式错误或安全证明。

边检查取 n=ceil(||q_b−q_a||/h)，包含两端。h=0.025 rad，规划后用 0.0125 rad 再复核。所有规划与捷径使用 W=I 的距离 D(a,b)=sqrt((a−b)ᵀW(a−b))。七个关节有限位，不进行角度绕回。离散检查只说明这些采样点通过，不能宣称严格连续无碰。

## direct：最容易解释的对照

问题：如果关节一起从起点线性插值到终点，能否直接通过？输入固定两个构型，输出合法的两点路径或 blocked。`planners/direct.py: Direct.plan` 只调用统一边检查，保存第一个阻挡构型。不搜索，不自动换算法。

```text
起点/终点不合法 → invalid_start / invalid_goal
整边合法 → [q_start, q_goal]
否则 → blocked + blocked_q
```

报告里的橙色阻挡整臂来自实际检查位置。实测 single_obstacle 的两端合法但 direct 被阻挡；empty 则可直接执行。练习：分别执行 `python -m panda_planning.demo --scene empty --planner direct --headless --output outputs/direct-empty` 与把 empty 换为 single_obstacle；检查 blocked_q 对应哪一对几何。

## RRT-Connect：从两边一起探索

RRT 是快速探索随机树：在关节空间采样一个构型，找树中最近构型，沿连接方向扩展并检查碰撞。随机点并非都可接受，边上的中间点也要检查。RRT-Connect 交替从起点树和终点树生长，另一棵树尝试连续连接新节点；两树碰面才形成完整路径。

```text
树 A ← 起点；树 B ← 终点
直到预算：
    q_rand ← 随机采样
    q_new ← 从 A 的最近节点向 q_rand 合法扩展
    从 B 尝试连接到 q_new
    若接通：回溯父节点并拼接路径
    交换或按树大小选择 A/B
```

优先后端 `ompl_backend.py` 使用 og.RRTConnect；本机实际后端 `fallback.py` 调用随包 motion_planners.rrt_connect，并由统一 extend/collision 回调约束所有插值。真实节点来自 TreeNode 构造，父节点给真实边。上游扩展接口需要包含两端，反向连接才不会丢掉连接位置。仅显示真实快照，不按最终图随机生成“生长动画”；当前快照没有可靠的逐节点树标签，所以不虚构起终树颜色。

RRT-Connect 常快速找到绕路，但第一次路径可能很长。实测单障碍 raw_cost≈7.36 rad，统一 shortcut 后约 4.02 rad，再经动力学执行成功。图是七维搜索图的末端投影，投影重合并不表示构型相同。图边先插值关节再 FK，避免把末端两点的直线冒充真实轨迹。

练习：`python -m panda_planning.demo --scene single_obstacle --planner rrt_connect --trace --seed 7 --headless --output outputs/connect-7`，从该记录生成 report，比较 raw/processed 长度。换 seed 可能改变路径，不要套用 seed42 的时间。

## RRT*：有解后继续改进

RRT* 与 RRT 的区别不只是运行更久。新构型附近会选使总路径代价最小且连接合法的父节点，再尝试让邻居通过新节点降低代价，称为“重连”。有限时间输出当前最好解。渐近最优说的是在相应采样/连接假设和样本数趋于无穷时收敛，不是本次 3 秒一定最好。

\[
parent(q)=\arg\min_{u\in N(q),\ edge(u,q)\ valid}\{cost(u)+D(u,q)\}.
\]

```text
采样并扩展到 q_new
在近邻中选最便宜合法父节点
对每个邻居 u：若经 q_new 更便宜且边合法，则重连 u
若存在目标节点，记录真实当前最好代价；继续至预算
```

`ompl_backend.py` 指向 og.RRTstar，分段 solve 保留对象而不重新初始化；本机 `fallback.py` 调用 motion_planners.rrt_star，半径 1.5 rad、informed=False。对上游 filter 迭代器改为 list，父节点选择后重连循环才会真的执行。不会拿 RRT-Connect 冒充 RRT*。当前不记录逐次重连事件，只有周期真实图与 best_cost。

实测：single_obstacle 3 秒、额外 15 秒运行均未找到解，曲线空白；empty 很快有直接解，在预算内保留约 0.9941 rad 的最好代价，没有凭空绘制下降。随机树在高维、狭窄可行域容易把大量预算花在碰撞检查上。`first_solution` 与 `elapsed` 分列。

练习：`python -m panda_planning.demo --scene empty --planner rrt_star --trace --budget 2 --headless --output outputs/star-empty`；再换 single_obstacle、逐步增加预算，检查曲线是否真的有解，以及 first_solution 是否小于预算结束时间。没有解时绝不填零代价。

## PRM：先修路网，再问怎么走

PRM 是概率路网：采样许多合法构型，尝试连接邻近构型并保存合法边，查询时把起终点接入路网，用图搜索取路径。这适合同一个静态场景重复查询。建图失败的边不进入路网。

```text
建图：采样合法 q；连接近邻且整边通过 → 保存边
查询：加入起点与终点；图搜索；回溯关节路径
场景或碰撞配置改变 → 清空重建
```

`ompl_backend.py` 使用 og.PRM；本机 `fallback.py` 调用 motion_planners.prm.DegreePRM，目标度数 8、最多 70 个采样构型，替代后端建图使用约 65% 时间；OMPL 经短查询回归修复后为 50%，并在同一剩余总预算内完成整个查询。边权是相同欧氏关节距离，A* 用到目标的距离作启发。cold 新建全部路网，warm 复用原来 roadmap，并单列最初建图和各次查询时间；不是把 warm 查询耗时与 RRT 完整求解时间直接排名。到预算时不伪造直线兜底。首次 warm 仍需要建图，stats.prm_mode 会明确为 cold。

实测单障碍的 cold PRM 找到并执行路径；`multi_query.json` 保存同场景的每组端点、cold/warm、建图/查询/累计成本依据。小型 tutorial 使用 3 个查询，完整示例用 10 个。

练习：`python examples/04_prm_queries.py`。把 cold 总耗时求和，再算首次 warm 建图+所有 warm 查询，观察查询次数多少时值得预先建图；不能只比较最后一个 warm 查询。

## 从几何路径到带时间运动

路径只是一组关节构型，没有“几秒到达”的含义。`shortcut` 随机挑两个路径点，只有整个捷径通过共同边检查才删除中间点；固定次数与独立 postprocess_seed。raw_path 与 processed_path 都保留。

`Trajectory` 每段使用：

\[
q(t)=q_a+s(r)(q_b-q_a),\quad s(r)=10r^3-15r^4+6r^5,\quad r=t/T.
\]

s' 峰值 1.875，|s''| 峰值 10/√3，所以：

\[
T\ge\max_j\left(1.875|\Delta q_j|/v_{max,j},\sqrt{(10/\sqrt3)|\Delta q_j|/a_{max,j}}\right).
\]

每个顶点停稳，因此不改变已检查的几何折线。执行用名义速度加位置反馈；APF/QP 改速度时按修正量暂停/减慢轨迹进度，接回路径前验边，不追逐远处路点穿过障碍。最终 τ 由直接力矩 PD+重力得到，真实状态通过 mj_step 演化，绝不设置 qpos 让算法“走动画”。测试验证静态保持、小位移和固定绕障。

练习：复制 configs/default.yaml，把 velocity 改成 0.3、acceleration 改成 0.8，运行同一个保存场景和路径，比较执行时长与速度峰值。几何代价应相同，执行时长会变化。

## none：把共同限制和避障贡献分开

`avoiders/none.py` 对名义速度只应用所有策略共有的关节速度/加速度/一步位置限位，不加入距离项。输入 RobotState、dq_nominal 和 context，输出 AvoidanceResult.dq_cmd。共同监测器可以停止机器人，但这是 guarded_stop，不是 none 成功绕行。

```text
low/high ← 速度、一步限位、上次发送速度的加速度约束
dq_cmd ← clip(dq_nominal, low, high)
```

实测 local_trap 的 none 触发 guarded_stop，显示未到达。练习：`python -m panda_planning.demo --scene local_trap --mode local --avoider none --headless --output outputs/local-none`，核对 interventions，不把“没发生穿透”直接等同算法避障成功。

## APF：在名义运动上叠加排斥速度

APF 是人工势场：越靠近障碍，排斥修正越大；离开激活范围就变为零。已有名义运动承担吸引，不另做全局搜索。

\[
U_i=\frac\eta2(1/d_i-1/d_0)^2\ (0<d_i<d_0),\quad
\dot q=\dot q_{nom}+\sum_i\eta(1/d_i-1/d_0)d_i^{-2}\nabla_q d_i.
\]

d0 默认环境 0.10 m，跨非邻接连杆自碰激活 0.03 m；η=2×10⁻⁵。rad 视为无量纲时 η 等效单位 m²/s；这是速度势，不是焦耳意义的物理势能。距离数值保护 max(d,0.005 m)，穿透/无效梯度先失败，不用保护掩盖碰撞。输出原始修正、势值、活跃对，再施加共同基础限制。

```text
对所有有效近距几何对：累加势梯度修正
名义速度 + 修正 → 共同运动限制 → 命令
```

`avoiders/apf.py: APF.step` 作用于整臂距离，不只末端。过大的修正可能被加速度限制延迟；吸引与排斥可能互相抵消，也可能震荡或在目标附近排斥。当前 local_trap 实测 APF 也触发监测停止，不能把该失败泛称“局部极小”；应查看势值、命令限制与距离曲线，区分加速度响应和真正停滞。

练习：`python examples/03_local_step.py` 先看同一已知构型的修正；再把 configs/local.yaml 的 apf_gain 改为 0.0002，用 local 模式重跑，不自动全局重规划。观察速度限制、势值和最终失败类型。

## 距离 QP：尽量保留名义速度，同时满足线性距离限制

QP 是二次规划：对七个关节速度解一个小优化，尽量少改名义速度。

\[
\min_v\frac12(v-v_{nom})^TI(v-v_{nom})+\frac\lambda2v^Tv,
\qquad \nabla d_i\,v\ge-\alpha(d_i-d_{safe,i}).
\]

α=5 s⁻¹，λ=0.001；环境 dsafe=0.025 m，自碰 dsafe=0.015 m。d>dsafe 时允许有限接近，越靠近下限允许的接近速度越小。左边的 m/rad×rad/s 与右边 m/s 一致。dsafe 是局部预测阈值，规划余量 0.012 m、实际停止阈值 0.003 m 是另外两件事。

`avoiders/distance_qp.py` 调 OSQP：P=(I+λI)，线性项 q=−v_nom，A 前七行是单位阵，后面每行为距离梯度；l/u 前七行组合速度上下限、(q_limit±余量−q)/dt、v_previous±amax dt，后续距离行下界为 −α(d−dsafe)，上界 +∞。QP 后不再裁剪速度，否则可能破坏约束。

```text
建立基础速度行 + 有效距离梯度行
OSQP 解 l≤Av≤u
检查真实求解状态、primal/dual 残差门限、最小余量
不可行/坏梯度 → 明确状态 + 统一停止，不删除距离行
```

日志输出 solver_status、primal/dual residual、distance_slack、min_slack、求解时间与速度修正。这是一阶速度距离约束，不是真实动力学的形式化安全证明。真实机器人与命令之间还有惯性、采样、网格误差，始终保留监测器。实测 local_trap 中 QP 到达，末端约 7.46 mm；失败注入测试则验证不可行和坏梯度会明确退出。

练习：同一 scene-file 运行 none/APF/distance_qp。先缩小 max_time 观察 timeout，再调大 safe_distance（保持 activation_distance 更大）观察是否停滞或不可行。任何失败都保留；不要无限放松约束来提高成功率。

## 资料与版本边界

本机真实 API 验证来自已安装 MuJoCo 3.3.7、mink 0.0.13、OSQP 1.0.4 源码和运行测试。可选 OMPL 适配针对新 nanobind（allocState、直接 Python callable、独立 MotionValidator），与旧 Boost.Python 的 ob.State/StateValidityCheckerFn 教程不同，当前已用真实 2.0.1 轮子通过三个算法及 cold/warm 集成验证。官方资料：[MuJoCo API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)、[mink](https://github.com/kevinzakka/mink)、[OMPL Python](https://ompl.kavrakilab.org/python.html)、[状态/运动验证](https://ompl.kavrakilab.org/stateValidation.html)、[OSQP Python](https://osqp.org/docs/interfaces/python.html)。

## 补充实测：局部停滞与官方绑定审核

`configs/apf-trap.yaml` 只将 APF 增益提高到 0.0002，保持控制、关节限制和到达判据不变。实测在 local_trap 中约 7.97 s 触发 stalled，末端误差约 0.01151 m；运动量窗口判断停滞，没有把阈值放宽到 0.012 m 来伪造成功。相同冻结输入的 QP 到达，none 被共同监测停止。观察命令与势值曲线可区分吸引/排斥平衡和仅仅走得慢；这不是所有场景都适用的结论。

可选 OMPL 适配已按 [官方 2.0.1 源码](https://github.com/ompl/ompl/tree/2.0.1/py-bindings)核对 MotionValidator(si)、PlannerData.getEdges(index)、PRM.constructRoadmap/clearQuery、ProblemDefinition 中间解回调及 PathGeometric.getStates。逐文件 blob SHA 在 evidence/ompl-api-audit.json。RNG 每个进程只初始化一次，进程内 warm 查询继续同一随机流；stats 同时记录 request_seed 与 process_seed，不承诺重复全局播种。中间解回调可用时记录真正改进时间；若轮子不支持该函数转换，明确标记为 solve 返回时的观测时间。源码核对不代替真实轮子的运行验证。

OMPL 运行验证补充：短预算 PRM 曾因“建图/接入耗时超过查询时间片”返回零目标差距的近似解。现通过预建图阶段的可中断边检查和完整查询时间段修复，不改变近似解的状态定义，不补造直线路径。普通 PRM 显式目标阈值为 infinity，保持与 RRT-Connect 一样的首条精确可行解停止语义；RRT* 则继续优化。重连事件仍只在 API 实际可用时展示。
