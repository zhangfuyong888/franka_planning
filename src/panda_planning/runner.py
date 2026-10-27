"""规划阶段使用墙钟；执行阶段只由 mj_step 推进真实状态。"""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
import time,uuid
import numpy as np
from .robot import Robot
from .collision import Collision,interpolate
from .types import PlanningRequest,PlanResult,RunSummary,plain
from .planners import create as planner_create
from .trajectory import shortcut,Trajectory,cost,projected_path
from .sim import execute
from .telemetry import write_json,environment,execution_metrics
from .scenes import save
from .ik import solve_ik

class Context(SimpleNamespace):pass

def build_context(scene,config):
    r=Robot(scene)
    return Context(robot=r,collision=Collision(r,config),config=config,previous_velocity=np.zeros(7))

def _usable_path(path):
    try:
        array = np.asarray(path, dtype=float)
        return array.ndim == 2 and array.shape[1] == 7 and np.all(np.isfinite(array))
    except (TypeError, ValueError):
        return False

def run(scene,config,planner='rrt_connect',avoider='none',mode='global',seed=42,output='outputs/run',trace=False,headless=True,plan_file=None,planner_instance=None):
    if mode=='global' and avoider!='none':raise ValueError('global 模式局部策略必须为 none；组合实验请用 hybrid')
    if mode=='hybrid' and not plan_file:raise ValueError('hybrid 必须 --plan-file 复用保存的同一 plan_id')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    run_environment = environment()
    ctx=build_context(scene,config);r=ctx.robot;c=ctx.collision
    request=PlanningRequest(np.array(scene.q_start),np.array(scene.q_goal),config.budget,seed,scene.scene_id,scene.target)
    run_id=uuid.uuid4().hex[:12];interventions=[];rows=[];execution_status='not_executed'
    # 读取保存目标时绝不静默更换 IK 分支；局部模式也必须检查起终构型。
    if not c.is_state_valid(request.q_start):
        plan = PlanResult('invalid_start', reason='起点超限、无效或碰撞')
    elif not c.is_state_valid(request.q_goal):
        plan = PlanResult('invalid_goal', reason='目标构型超限、无效或碰撞')
    elif np.shape(scene.target) != (3,) or not np.all(np.isfinite(scene.target)):
        plan = PlanResult('invalid_goal', reason='末端目标必须是 world 中的有限三维位置')
    elif np.linalg.norm(r.fk(request.q_goal) - scene.target) >= config.ik_tolerance:
        ik = solve_ik(r, scene.target, c, orientation_cost=config.ik_orientation_cost,
                      max_iterations=config.ik_iterations, tolerance=config.ik_tolerance)
        status = 'ik_unreachable' if ik['status'] == 'ik_unreachable' else 'invalid_goal'
        plan = PlanResult(status, reason='保存 q_goal 与末端目标不一致；请显式重新生成并冻结 IK')
    elif mode == 'local':
        plan = PlanResult('not_requested', stats={'backend': 'none'})
    else:
        try:
            if plan_file:
                import json
                saved = json.loads(Path(plan_file).read_text())
                plan = PlanResult(**{k:v for k,v in saved.items() if k != 'plan_id'})
                if saved.get('plan_id') != plan.plan_id:
                    raise ValueError('保存路径的 plan_id 内容校验失败')
                if plan.stats.get('scene_id') != scene.scene_id:
                    raise ValueError('保存路径的 scene_id 不匹配')
                if plan.status != 'success' or len(plan.processed_path) < 2:
                    raise ValueError('保存路径不是成功的完整路径')
                if not np.allclose(plan.processed_path[0], scene.q_start) or not np.allclose(plan.processed_path[-1], scene.q_goal):
                    raise ValueError('保存路径端点不匹配')
            else:
                algorithm = planner_instance or planner_create(planner, config.backend)
                plan = algorithm.plan(request, ctx, True if trace else None)
                if not isinstance(plan, PlanResult):
                    raise TypeError('全局插件必须返回 PlanResult')
        except Exception as exc:
            plan = PlanResult('error', reason=f'{type(exc).__name__}: {exc}')
    planning_success=False;post_time=0.;plan_min=None
    if plan.status == 'success':
        try:
            if not _usable_path(plan.raw_path) or len(plan.raw_path) < 2:
                raise ValueError('成功路径必须包含至少两个七轴构型')
            if not plan_file:
                t = time.perf_counter()
                plan.processed_path = shortcut(plan.raw_path, c, config.shortcut_attempts, seed+1)
                post_time = time.perf_counter() - t
            path = np.asarray(plan.processed_path)
            planning_success = bool(path.ndim == 2 and path.shape[1] == 7 and
                                    np.allclose(path[0], request.q_start, atol=1e-7, rtol=0) and
                                    np.linalg.norm(path[-1]-request.q_goal) < 1e-5 and
                                    c.validate_path(path))
            if not planning_success:
                raise ValueError('更细分辨率的完整路径或端点复核未通过')
            plan_min = min(c.minimum(q) for a,b in zip(path[:-1],path[1:])
                           for q in interpolate(a,b,config.validation_step))
        except Exception as exc:
            planning_success = False
            plan.status = 'validation_failed'
            plan.reason = f'{type(exc).__name__}: {exc}'
            # 保留原路径供诊断，展示只接受已知形状的数据。
            if not _usable_path(plan.raw_path):
                plan.stats['rejected_raw_path'] = plain(plan.raw_path)
                plan.raw_path = []
            if not _usable_path(plan.processed_path):
                plan.stats['rejected_processed_path'] = plain(plan.processed_path)
                plan.processed_path = []
    plan.stats.update(scene_id=scene.scene_id,raw_cost=cost(plan.raw_path),processed_cost=cost(plan.processed_path),postprocess_time=post_time,plan_min_distance=plan_min)
    planning_calls=c.calls;planning_seconds=c.seconds
    execution = None
    if (planning_success or (mode == 'local' and plan.status == 'not_requested')):
        execution = execute(ctx, scene, request, plan, planner=planner,
                            avoider=avoider, mode=mode, headless=headless)
        rows = execution.rows
        execution_status = execution.status
        interventions = execution.interventions
    metrics = execution_metrics(rows)
    metrics.update({
        'planning_time': plan.elapsed, 'first_solution': plan.first_solution,
        'postprocess_time': post_time, 'raw_cost': plan.stats.get('raw_cost'),
        'processed_cost': plan.stats.get('processed_cost'), 'planning_min_distance': plan_min,
        'collision_queries_planning': planning_calls, 'collision_seconds_planning': planning_seconds,
        'collision_queries_total': c.calls, 'collision_seconds_total': c.seconds,
        'interventions': len(interventions),
        'wallclock_execution': execution.wall_time if execution is not None else None,
    })
    summary=plain(RunSummary(run_id,planning_success,execution_status=='success',execution_status,metrics,interventions))
    summary.update(scene_id=scene.scene_id,plan_id=plan.plan_id,model_id=r.model_id,mode=mode,planner='not_requested' if mode=='local' else planner,avoider=avoider,planning_status=plan.status,planning_reason=plan.reason,backend=plan.stats.get('backend'),config=asdict(config),seeds={'scene':scene.seed,'planner':seed,'postprocess':seed+1},environment=run_environment,source_changed_during_run=environment()['source_sha256']!=run_environment['source_sha256'],recording='learning' if trace else 'benchmark')
    save(scene,output/'scene.json');write_json(output/'request.json',request);write_json(output/'summary.json',summary)
    pdata=plain(plan);pdata['plan_id']=plan.plan_id
    write_json(output/'plan.json',pdata);write_json(output/'telemetry.json',rows);write_json(output/'trace.json',plan.trace);write_json(output/'collision_pairs.json',c.checklist())
    write_json(output/'display.json',{'raw_path':projected_path(r,plan.raw_path),'processed_path':projected_path(r,plan.processed_path),'start_skeleton':r.skeleton(request.q_start),'goal_skeleton':r.skeleton(request.q_goal),'target':scene.target,'start_gripper':r.gripper(request.q_start)})
    return summary
