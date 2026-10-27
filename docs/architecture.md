# 从一个目标到一次真实执行

先在 single_obstacle 里看现象：两端关节构型合法，直接插值却让手掌穿过球。全局规划器要在七维关节空间找一条折线；局部策略只改变当前关节速度。二者通过同一个碰撞查询器理解世界，不能各自用不同的机器人半径或安全标准。

## 模块边界

| 文件 | 工作与主要入口 |
| --- | --- |
| types.py / config.py | SceneSpec、PlanningRequest、PlanResult、RobotState、DistanceSample、AvoidanceResult、TraceSnapshot、RunSummary；Config.load 拒绝未知参数 |
| robot.py | 七轴名称映射、随包模型、FK、world Jacobian、连杆和固定手指代理 |
| ik.py | mink 固定种子差分 IK，独立 Configuration，核验目标位置/关节限位/终点碰撞 |
| collision.py | Collision.is_state_valid / is_edge_valid / query_distances；独立 MjData、157 个基准几何对以及显式排除清单 |
| scenes.py | 固定场景、随机拒绝原因、保存/读取与稳定 scene_id |
| planners/ | GlobalPlanner.plan(request, context, trace_sink=None) → PlanResult，reset(context)；注册表选 direct/真实采样规划后端 |
| avoiders/ | LocalAvoider.step(state, dq_nominal, context) → AvoidanceResult，reset(context)；none/APF/OSQP |
| trajectory.py | 已验边 shortcut、带解析导数的五次分段轨迹、关节边采样后的 FK 投影 |
| controller.py / monitor.py | 七个直接力矩 motor 的 PD+重力；独立检查真实状态、距离、跟踪、饱和、时间单调性与停滞 |
| runner.py / sim.py | 输入与保存路径核验、三种模式编排；独立 execute 管理路径进度、真实执行、故障记录和分开的成功定义 |
| telemetry.py | 严格 JSON、包版本、平台、代码 SHA256；记录不由可视化层猜测 |
| overlays.py / visualizer.py / replay.py | passive viewer user_scn 装饰；实测记录使用单独 playback_data |
| render.py | EGL/OSMesa 显式离屏渲染实测记录，完整 MuJoCo 网格、最近点和活跃连杆高亮；图像与帧 ID 一同保存 |
| report.py / assets/report_template.html | JSON → 离线 Plotly.js 整臂/真实障碍回放、同组共用尺度与时间轴 |
| demo.py / benchmark.py | CLI、独立子进程比较、独立 cold/warm PRM 实验 |

Python 包采用 src/panda_planning，模型资产和默认 YAML 在包内；根目录 configs 是便于编辑的副本。不要将所有代码塞进一个入口，也不需要热加载、ROS 或服务进程。

## 一次调用的顺序

```mermaid
flowchart LR
 A[场景 + 固定输入/目标] --> B[mink IK 或读取保存 q_goal]
 B --> C[统一碰撞核验]
 C --> D[全局插件 plan]
 D --> E[shortcut + 更细整路径复核]
 E --> F[五次轨迹参考]
 F --> G[名义关节速度]
 G --> H[局部插件 step]
 H --> I[统一监测器]
 I --> J[PD + 重力 / mj_step]
 J --> K[真实状态与诊断日志]
 K --> G
 K --> L[实时显示 / 离线报告]
```

`global` 强制局部 none；`local` 跳过全局搜索，直接用 1.5(q_goal−q) 形成名义速度；`hybrid` 要求保存的 plan.json，核对 scene_id、端点和完整路径，不重新随机规划。场景哈希包含模型标识、完整障碍、起终点、目标、生成元数据；warm PRM 额外绑定关节边界、边检查精度和规划余量。查询时可以换端点，但要使用同一冻结场景标识。任何几何变化都清空路网。

## 状态隔离与单位

所有位置与最近点在 world，长度 m，角度 rad，时间 s，速度 rad/s，距离梯度 m/rad。七轴顺序固定为 joint1…joint7，通过 qpos/dof/actuator 名称索引取得。夹爪无自由度；motor gear=1，gain=1，bias=0，力矩限值前四轴 87 Nm、后三轴 12 Nm。

运行 data 只由初始化和 `mj_step` 推进。FK 有独立 fk_data，距离有独立 data，IK 使用 mink.Configuration，重力用 qvel=0 的独立 data。图快照在规划对象释放前复制为 Python 数组。离线回放设置的是专用 playback_data 的记录 q/dq，不计为新的动力学实验。批量运行使用独立子进程；motion-planners 节点捕获临时替换模块类，只允许进程内单线程，finally 恢复。

固定度量为单位阵 W 的关节欧氏距离，不进行角度绕回。`extension_range` 只对 OMPL 生效；替代后端沿上游连接扩展，日志如实标注。边检查步长 0.025 rad 与独立复核 0.0125 rad，不是末端直线距离，更不是严格连续碰撞证明。

## 执行与进度

控制周期 0.01 s，物理步长 0.002 s，周期必须整除。轨迹给 q_ref、dq_ref；名义速度为 dq_ref+3(q_ref−q)。局部策略修改速度后，按修改量和跟踪偏差减慢/暂停轨迹时间。参考偏差超过 0.08 rad 时验证接回当前参考的线段，超过跟踪阈值停止；积分误差超过 0.05 rad 时将命令锚回实测状态，避免无限累积。速度、一步关节限位、相对上次发送速度的加速度限制在 none/APF 的共同限幅或 QP 矩阵内部完成，QP 后不再裁剪速度。

控制层执行 τ=Kp(qcmd−q)+Kd(dqcmd−dq)+g(q)。g 由静态 data 的 qfrc_bias 计算；每个物理步重新算 PD、重力并限力矩。这里没有 MPC。实际速度可能有动力学超调，日志保存峰值；监测阈值为命令速度的 1.5 倍。局部速度约束不能直接变成真实动力学的安全保证。

到达需要末端位置误差 <0.01 m、速度 <0.02 m/s 持续 0.2 s。规划成功、执行成功独立保存。求解器失败、距离无效、碰撞、guarded_stop、停滞、跟踪过大、关节限位、数值异常/意外时间回退分别记录；共同监测停止绝不记为 none/APF 绕障。监测器同时核验命令形状/有限值、已发送命令的仿真时间戳与 command_timeout、实际力矩、MuJoCo warning 计数及时间回退。离线仿真没有异步命令队列，命令过期基于仿真时间，不能把 headless 计算墙钟当成实时硬件看门狗；真实硬件通信仍不在范围内。

## 保存什么，怎样解释时间

* scene.json/request.json：完整输入、固定 q_start/q_goal、目标和种子。
* plan.json：状态、原因、raw_path/processed_path、真实长度、plan_id、阶段成本；trace.json：真实周期快照，不补造拒绝采样或重连事件。
* telemetry.json：t 是已执行的物理时间，q/dq/ee/nearest 属于该时刻；q_command/torque/applied_dq_command 是上一控制区间施加的值，applied_t 标出该区间起点。dq_nominal/dq_command 是在当前 command_t 计算、用于下一区间的速度。首行尚未施加力矩，因此为零。
* summary.json：独立成功判据、干预列表、模型/场景/路径 ID、配置、包版本、平台与源码指纹。失败代价为 null。
* collision_pairs.json：检查/排除的具体几何名称与原因。排除相邻/固定装配体和基座安装平面；不以 ncon>0 代替整臂检查。
* display.json：从该次路径与机器人 FK 产生的展示数据。report 从同一个运行目录读取，不拼贴另一次运行的快路径和快耗时。

规划使用墙钟，执行使用仿真时间。learning 开启真实图快照并单列 trace_overhead；benchmark 关闭昂贵快照。图最多显示 250 个节点、HTML 最多 120 条边，完整算法统计不截断。不同算法较早结束时保持末态，不把速度归一化隐藏。

## 完善后的失败语义

局部模式也核验起终构型；保存 q_goal 与末端目标不一致时拒绝，不自动偷偷换 IK 分支。hybrid 检查保存 plan_id 的内容哈希、场景和端点，再重新验整条路径。全局/局部插件异常保留 reason、部分遥测和 summary。距离/状态数值异常的 null 保留在原遥测，聚合跳过无效值而不再次崩溃。

QP 不再把 max_iter 和数值失败统称 infeasible；分别检查求解状态、primal/dual residual 和约束余量。成功还必须满足物理时间戳上的完整 settle_time，不把第一帧提前算成已持续一个周期。

环境/源码指纹在 run 开始冻结，并记录 source_changed_during_run。验收脚本接受 --input/--output，审核记录哈希、时间递增、CLI失败和缺失运行；不会仅凭旧报告存在就写成通过。

## OMPL 2.0.1 实测修复

`_CollisionQueries` 只持有 MuJoCo 碰撞查询器，顶层 `_MotionValidator` 不通过局部类闭包捕获 si，也不持有全局规划器自身。这样打断 si→Python回调→planner→si、si→validator类→si 两类跨语言引用环。进程退出测试直接检查 stderr，未关闭 nanobind 的泄漏检查。

所有 OMPL 边仍使用共同插值和碰撞余量；只增加检查点之间的预算中断。未检查完的边被拒绝，并计入 interrupted_edges，不把它作为通过的边。PRM cold 预建图使用总预算的 50%，剩余时间用于完整查询，避免以 snapshot_interval 截断起终点接入和原生解检查线程。PRM 的目标仍是关节路径长度；普通查询的满足阈值设为 infinity，在首条精确可行路径处返回，warm 不会因默认零阈值而无谓跑满预算。RRT* 保持预算内继续优化。

OMPL 轮子没有可用的中间解 std::function caster，adapter 如实记录 callback_error 与 first_solution_source=solve_return_observation，首次解时间是分段 solve 返回时的观测上界。RRT* 代价曲线来自真实周期观测，不伪造更细事件。
