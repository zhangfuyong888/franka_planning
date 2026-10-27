# 第三方来源与修改

项目自有代码使用根目录 MIT LICENSE。第三方文件保持各自许可。

* Panda：Google DeepMind MuJoCo Menagerie，提交 `cda5236f98383f6fbaa4820c6b18dba892cd087f`，目录 `franka_emika_panda`。来源 https://github.com/google-deepmind/mujoco_menagerie/tree/cda5236f98383f6fbaa4820c6b18dba892cd087f/franka_emika_panda 。Apache-2.0 许可、upstream.xml、67 个 mesh 的来源与校验信息保存在 `src/panda_planning/assets/panda/`。从用户已有 panda-mpc-tracking 的已验证派生模型复制，源项目只读。保留质量、惯量、几何；固定手指在 ±0.03 m，移除手指 joint/tendon/equality，使用七个 gear=1 的直接力矩 motor，清除位置伺服 gain/bias，保留 ee_site/home。运行时组合场景、命名碰撞体、添加地面，针对 MuJoCo 3.3.7 的实测距离异常禁用 nativeccd 并设置 CCD tolerance=1e-8、iterations=100，原资产不改。
* motion-planners：Caelan Garrett，MIT，https://github.com/caelan/motion-planners 。在 `_vendor/motion_planners/` 保留 LICENSE、逐文件 Git blob SHA 和 URL（provenance.json）。复制真实 rrt_connect、rrt_star、prm、rrt、utils、primitives；仅移除 primitives 未使用的轨迹导入，并把 RRT* neighbors 的 Python 3 filter 转成 list，避免第二次遍历为空而漏掉重连。没有将其称为 OMPL。适配器以真实节点构造和 roadmap 对象取得快照，扩展函数包含两端，避免上游反向扩展丢端点。
* Plotly.js gl3d：v3.1.0，MIT，https://github.com/plotly/plotly.js/tree/v3.1.0 ，原始 dist/plotly-gl3d.min.js blob `e1b9da3ebd4502c983829abbe09885784578571d`，保留 PLOTLY-LICENSE。通过 GitHub 连接器获取完整 blob，未改动，内嵌在离线报告中。Python Plotly 发行包在当前终端无法安装，因此直接使用官方 Plotly.js；曲线由 canvas 绘制。
* mink 0.0.13（Apache-2.0）只通过运行依赖调用，不复制算法源码。FrameTask、Configuration、ConfigurationLimit、solve_ik 的 API 依据实际安装源码核对。距离梯度按 world 点 Jacobian 自行实现。
* MuJoCo 3.3.7、OSQP 1.0.4、NumPy、SciPy、PyYAML、Matplotlib、pytest 均为独立发行依赖。其许可证随各安装发行包保存。
* 可选 OMPL 2.0.1（BSD-3-Clause）适配 nanobind API；当前环境已安装并完成真实集成及回调生命周期回归；历史替代后端测试与 OMPL 结果仍分开保存。
