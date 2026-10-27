"""有限时长的真实动力学执行；任何求解失败都先记录，再停止实验。"""
from dataclasses import dataclass, field
import math
import time
import numpy as np
from .avoiders import create as avoider_create
from .controller import Controller
from .monitor import Monitor
from .trajectory import Trajectory
from .types import AvoidanceResult, plain


@dataclass
class Execution:
    status: str = 'not_executed'
    rows: list = field(default_factory=list)
    interventions: list = field(default_factory=list)
    wall_time: float = 0.


def execute(context, scene, request, plan, *, planner, avoider, mode, headless):
    """正常状态只由 mj_step 演化。观测在 t，候选速度用于下一个控制区间。"""
    r, c, cfg = context.robot, context.collision, context.config
    out = Execution(status='timeout')
    data = r.data(request.q_start)
    controller, guard = Controller(r, cfg), Monitor(context)
    trajectory = Trajectory(plan.processed_path, cfg.velocity, cfg.acceleration) if plan.status == 'success' else None
    substeps = round(cfg.dt / r.model.opt.timestep)
    if substeps < 1 or not np.isclose(substeps * r.model.opt.timestep, cfg.dt):
        raise ValueError('dt 必须是 MuJoCo timestep 的整数倍')
    qcmd, tau = request.q_start.copy(), np.zeros(7)
    progress, settle_since = 0., None
    saturated, viewer, last_sent_t = False, None, 0.
    initial_warnings = data.warning.number.copy()
    wall = time.perf_counter()
    try:
        strategy = avoider_create(avoider)
        strategy.reset(context)
        if not headless:
            from .visualizer import LiveViewer
            label = 'target_feedback' if mode == 'local' else planner
            viewer = LiveViewer(r, data, scene, plan, label, avoider)
        index = 0
        # ceil 后不额外走超时物理步；最后一个时刻也保存一次观测。
        steps = int(math.floor(cfg.max_time / cfg.dt + 1e-9))
        while index <= steps:
            state = r.state(data)
            if viewer and not viewer.running():
                out.status = 'viewer_closed'
                break
            if viewer and viewer.paused and not viewer.single_step:
                paused_at = time.perf_counter()
                viewer.sync(state, [], out.rows)
                time.sleep(.02)
                wall += time.perf_counter() - paused_at  # 暂停不造成继续后的快进追时。
                continue
            if viewer:
                viewer.single_step = False
            if trajectory:
                qr, vr, _ = trajectory.sample(progress)
                if np.linalg.norm(qr - state.q) > .08 and not c.is_edge_valid(state.q, qr):
                    out.status = 'rejoin_blocked'
                    out.interventions.append({'t': state.t, 'status': out.status})
                    break
                nominal = vr + 3. * (qr - state.q)
            else:
                nominal = 1.5 * (request.q_goal - state.q)
            started = time.perf_counter()
            try:
                result = strategy.step(state, nominal.copy(), context)
                if not isinstance(result, AvoidanceResult):
                    raise TypeError('局部插件必须返回 AvoidanceResult')
            except Exception as exc:
                result = AvoidanceResult(np.zeros(7), 'error', {'exception': f'{type(exc).__name__}: {exc}'})
            command_valid = np.shape(result.dq_cmd) == (7,) and np.all(np.isfinite(result.dq_cmd))
            # 日志用有限占位值表示拒绝的命令；停止前不会把它发送到执行器。
            if not command_valid:
                result.diagnostics['rejected_command'] = plain(result.dq_cmd)
                result.status = 'invalid_command'
            command = np.asarray(result.dq_cmd) if command_valid else np.zeros(7)
            samples = c.query_distances(state.q)
            distance = min((s.distance for s in samples), default=cfg.activation_distance)
            nearest = min(samples, key=lambda s: s.distance) if samples else None
            warnings = data.warning.number - initial_warnings
            status = guard.check(state, qcmd, distance, progress, saturated, result.status,
                                 torque=tau, command=result.dq_cmd, command_t=last_sent_t,
                                 numerical_warning=bool(np.any(warnings > 0)))
            if not np.isfinite(distance) or any(not np.isfinite(s.distance) for s in samples):
                status = status or 'invalid_distance'
            jac = r.jacobian(data)
            row = {
                't': state.t, 'q': state.q, 'dq': state.dq,
                'ee': state.ee, 'ee_velocity': state.ee_velocity,
                'skeleton': r.skeleton(state.q), 'gripper': r.gripper(state.q),
                'dq_nominal': nominal, 'dq_command': command,
                'applied_dq_command': context.previous_velocity.copy(),
                'command_t': state.t, 'applied_t': last_sent_t if index else None,
                'q_command': qcmd.copy(), 'torque': tau.copy(),
                'min_distance': distance, 'distance_censored': not bool(samples),
                'nearest': plain(nearest), 'progress': progress,
                'error': float(np.linalg.norm(state.ee - scene.target)),
                'diagnostics': plain(result.diagnostics), 'solver_status': result.status,
                'saturated': saturated, 'mujoco_warning_counts': warnings,
                'command_compute_time': time.perf_counter() - started,
                'velocity_arrows': {'nominal': jac @ nominal, 'command': jac @ command,
                                    'correction': jac @ (command - nominal)},
            }
            out.rows.append(plain(row))
            if viewer:
                viewer.sync(state, samples, out.rows)
            if status:
                out.status = status
                break
            arrived = row['error'] < cfg.goal_tolerance and np.linalg.norm(state.ee_velocity) < cfg.settle_speed
            settle_since = (state.t if settle_since is None else settle_since) if arrived else None
            if settle_since is not None and state.t - settle_since >= cfg.settle_time - 1e-9:
                out.status = 'success'
                break
            if index >= steps:
                break
            # 各策略已包含共同限制，尤其 QP 后不再裁速度。
            context.previous_velocity = command.copy()
            if np.linalg.norm(qcmd - state.q) > .05:
                qcmd = state.q.copy()
            qcmd = qcmd + command * cfg.dt
            if trajectory:
                correction = np.linalg.norm(command - nominal)
                factor = max(0., 1. - correction / (np.linalg.norm(nominal) + .02))
                if np.linalg.norm(qr - state.q) > .06:
                    factor = 0.
                progress += cfg.dt * factor
            last_sent_t, saturated = state.t, False
            for _ in range(substeps):
                tau, clipped = controller.step(data, qcmd, command)
                saturated = saturated or clipped
            index += 1
            if viewer:
                time.sleep(max(0., state.t + cfg.dt - (time.perf_counter() - wall)))
    except Exception as exc:
        # 插件 reset、距离查询、GUI 或物理执行异常不能使整次记录消失。
        out.status = 'viewer_unavailable' if not headless and viewer is None else 'execution_error'
        out.interventions.append({'t': float(data.time), 'status': out.status,
                                  'exception': f'{type(exc).__name__}: {exc}'})
    finally:
        if viewer:
            viewer.close()
        out.interventions.extend(guard.events)
        out.wall_time = time.perf_counter() - wall
    return out
