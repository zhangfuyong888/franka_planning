"""按官方 OMPL 2.0.1 nanobind 源码核对的适配；缺包不伪造成功。"""
import time
import numpy as np
from ompl import base as ob, geometric as og, util as ou
from .base import GlobalPlanner
from ..types import PlanResult, TraceSnapshot, plain
from ..trajectory import cost
from ..collision import interpolate

_PROCESS_SEED = None


class _CollisionQueries:
    """回调只持有 MuJoCo 查询器，不持有 planner/si，避免跨语言引用环。"""
    def __init__(self, collision):
        self.collision = collision
        self.deadline = float('inf')
        self.interrupted_edges = 0

    @staticmethod
    def vector(state):
        return np.array([state[i] for i in range(7)])

    def is_valid(self, state):
        if time.perf_counter() >= self.deadline:
            return False
        return self.collision.is_state_valid(self.vector(state))

    def edge_valid(self, a, b):
        # 与 Collision.is_edge_valid 同一插值/余量。预算到期只拒绝未检查完的边，
        # 不把它接受成合法边，也不抛异常穿过 C++/线程边界。
        collision = self.collision
        for q in interpolate(self.vector(a), self.vector(b), collision.config.edge_step):
            if time.perf_counter() >= self.deadline:
                self.interrupted_edges += 1
                return False
            if not collision.is_state_valid(q):
                return False
        return True


class _MotionValidator(ob.MotionValidator):
    # 顶层类的方法不捕获创建时的 si；局部 class 会经类型闭包形成 si→validator→si 环。
    def __init__(self, si, queries):
        super().__init__(si)
        self.queries = queries

    def checkMotion(self, a, b):
        return self.queries.edge_valid(a,b)


class OMPLPlanner(GlobalPlanner):
    def __init__(self, name):
        self.name, self.key, self.objects = name, None, None
        self.queries = None
        self.objective = None

    def reset(self, context):
        self.key, self.objects, self.queries, self.objective = None, None, None, None

    def plan(self, request, context, trace_sink=None):
        global _PROCESS_SEED
        c, cfg, r = context.collision, context.config, context.robot
        for label, q in [('start', request.q_start), ('goal', request.q_goal)]:
            if not c.is_state_valid(q):
                return PlanResult('invalid_' + label)
        vec = lambda state: np.array([state[i] for i in range(7)])
        key = (request.scene_id, r.model_id, tuple(r.limits.ravel()), cfg.edge_step, cfg.planning_margin)
        warm = self.name == 'prm' and self.key == key and self.objects is not None
        if not warm:
            # OMPL 的全局 RNG 在创建 RNG 对象后不应重复 setSeed。
            # CLI 每个随机规划独立进程；进程内多查询明确记录初始种子及流的延续。
            if _PROCESS_SEED is None:
                ou.RNG.setSeed(request.seed)
                _PROCESS_SEED = request.seed
            space = ob.RealVectorStateSpace(7)
            bounds = ob.RealVectorBounds(7)
            for i, (lo, hi) in enumerate(r.limits):
                bounds.setLow(i, float(lo))
                bounds.setHigh(i, float(hi))
            space.setBounds(bounds)
            si = ob.SpaceInformation(space)
            self.queries = _CollisionQueries(c)
            si.setStateValidityChecker(self.queries.is_valid)
            validator = _MotionValidator(si, self.queries)
            si.setMotionValidator(validator)
            si.setup()
            planner = {'rrt_connect': og.RRTConnect, 'rrt_star': og.RRTstar, 'prm': og.PRM}[self.name](si)
            if hasattr(planner, 'setRange'):
                planner.setRange(cfg.extension_range)
            self.objective = ob.PathLengthOptimizationObjective(si)
            if self.name == 'prm':
                # PRM 普通查询与 RRT-Connect 一样在第一条精确路径处结束。
                # 显式提供路径长度目标后，OMPL 不会自动设置这个阈值；默认0
                # 会让 warm 查询一直优化到预算结束，而不是快速复用路网。
                self.objective.setCostThreshold(ob.Cost(float('inf')))
            self.objects, self.key = (space, si, planner, validator), key
        space, si, planner, validator = self.objects
        self.queries.collision = c
        if warm:
            planner.clearQuery()
        pd = ob.ProblemDefinition(si)
        start, goal = space.allocState(), space.allocState()
        for i in range(7):
            start[i], goal[i] = float(request.q_start[i]), float(request.q_goal[i])
        pd.setStartAndGoalStates(start, goal, 1e-6)
        pd.setOptimizationObjective(self.objective)
        planner.setProblemDefinition(pd)
        if not warm:
            planner.setup()
        begin = time.perf_counter()
        self.queries.deadline = begin + request.budget
        self.queries.interrupted_edges = 0
        first, first_source, trace, improvements = None, None, [], []
        overhead, build_time, query_time = 0., 0., 0.
        raw, status, reason = [], 'timeout', ''

        def improvement(_planner, _states, value):
            nonlocal first, first_source
            at = time.perf_counter() - begin
            improvements.append({'t': at, 'best_cost': float(value.value())})
            if first is None:
                first, first_source = at, 'intermediate_callback'

        callback_error = None
        try:
            pd.setIntermediateSolutionCallback(improvement)
        except (TypeError, RuntimeError) as exc:
            # 某些轮子未导出 std::function caster；保留真实观测精度，不编造首次时间。
            callback_error = str(exc)
        graph_counts = {'nodes': None, 'edges': None}

        def snapshot(record=True, label='实际搜索图快照'):
            nonlocal overhead
            started = time.perf_counter()
            data = ob.PlannerData(si)
            planner.getPlannerData(data)
            count, edge_count = data.numVertices(), data.numEdges()
            graph_counts['nodes'] = planner.milestoneCount() if self.name == 'prm' else count
            graph_counts['edges'] = planner.edgeCount() if self.name == 'prm' else edge_count
            if record:
                limit = min(count, cfg.graph_limit)
                nodes = [vec(data.getVertex(i).getState()).tolist() for i in range(limit)]
                edges = [[i, int(j)] for i in range(limit) for j in data.getEdges(i) if j < limit]
                snap = plain(TraceSnapshot(time.perf_counter()-begin, nodes, edges, cost(raw), label))
                snap.update(total_nodes=count, total_edges=edge_count, truncated=count>limit)
                trace.append(snap)
            overhead += time.perf_counter()-started

        qt = None
        try:
            if self.name == 'prm' and not warm:
                bt = time.perf_counter()
                try:
                    self.queries.deadline = begin + request.budget * .5
                    planner.constructRoadmap(ob.timedPlannerTerminationCondition(max(0., self.queries.deadline-time.perf_counter())))
                finally:
                    build_time = time.perf_counter() - bt
                if trace_sink is not None:
                    snapshot(label='PRM真实预建图末快照；尚未接入查询端点')
            self.queries.deadline = begin + request.budget
            qt = time.perf_counter()
            while time.perf_counter() - begin < request.budget:
                remaining = request.budget - (time.perf_counter() - begin)
                if remaining <= 0:
                    break
                # PRM 在 solve 内先接入查询点，再启动原生解检查线程；不能用
                # 图快照间隔截断这次查询，导致线程还未检查就只返回近似解。
                duration = remaining if self.name == 'prm' else min(cfg.snapshot_interval, remaining)
                planner.solve(ob.timedPlannerTerminationCondition(duration))
                if pd.hasExactSolution():
                    status = 'success'
                    # nanobind Path/State 指针在底层对象存活时复制，日志不保存悬空引用。
                    raw = [vec(s).copy() for s in pd.getSolutionPath().getStates()]
                    if first is None:
                        first, first_source = time.perf_counter()-begin, 'solve_return_observation'
                elif pd.hasApproximateSolution():
                    status = 'approximate_solution'
                if trace_sink is not None:
                    snapshot()
                if status == 'success' and self.name != 'rrt_star':
                    break
        except Exception as exc:
            status, reason = 'error', f'{type(exc).__name__}: {exc}'
            raw = []
        finally:
            if qt is not None:
                query_time = time.perf_counter()-qt
            self.queries.deadline = float('inf')
        if not trace:
            snapshot(record=False)
        stats = {'backend': 'ompl-2.0.1', 'implementation': planner.getName(),
                 'trace_overhead': overhead, 'build_time': build_time, 'query_time': query_time,
                 'prm_mode': ('warm' if warm else 'cold') if self.name == 'prm' else None,
                 'process_seed': _PROCESS_SEED, 'request_seed': request.seed,
                 'rng_policy': 'process_initialized_once_then_continued',
                 'first_solution_source': first_source, 'callback_error': callback_error,
                 'nodes': graph_counts['nodes'], 'edges': graph_counts['edges'],
                 'improvements': improvements, 'prm_build_fraction': .5 if self.name == 'prm' else None,
                 'interrupted_edges': self.queries.interrupted_edges,
                 'budget_overrun': max(0., time.perf_counter()-begin-request.budget)}
        return PlanResult(status, raw_path=raw, elapsed=time.perf_counter()-begin,
                          first_solution=first, stats=stats, trace=trace, reason=reason)
