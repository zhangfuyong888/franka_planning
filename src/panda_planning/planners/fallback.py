"""明确标记的 motion-planners 后端；不把替代实现的时间写成 OMPL。"""
import random,time,contextlib,io
import numpy as np
from .base import GlobalPlanner
from ..types import PlanResult,TraceSnapshot,plain
from ..trajectory import cost
from ..collision import interpolate
from .._vendor.motion_planners import rrt_connect as rc,rrt_star as rs,prm as pm, primitives as primitives

class BudgetExpired(Exception):pass

class MotionPlanners(GlobalPlanner):
    def __init__(self,name):self.name=name;self.roadmap=None;self.key=None
    def reset(self,context):self.roadmap=None;self.key=None
    def plan(self,request,context,trace_sink=None):
        c=context.collision; cfg=context.config; r=context.robot
        for label,q in [('start',request.q_start),('goal',request.q_goal)]:
            if not c.is_state_valid(q):return PlanResult('invalid_'+label)
        random.seed(request.seed); rng=np.random.default_rng(request.seed)
        start=time.perf_counter(); deadline=start+request.budget
        snapshots=[]; first=None; overhead=0.; last_snapshot=-1.; nodes=[]; roadmap=None
        self.best_path=[]
        def distance(a,b):return float(np.linalg.norm(np.array(b)-a))
        def collision(q):
            if time.perf_counter()>=deadline:raise BudgetExpired()
            return not c.is_state_valid(q)
        def extend(a,b):return [tuple(q) for q in interpolate(np.asarray(a),np.asarray(b),cfg.edge_step)]
        def sample():return tuple(rng.uniform(r.limits[:,0],r.limits[:,1]))
        # 上游构造节点时抓取真实对象。只在本进程单线程求解，退出后还原类。
        originals=(rc.TreeNode,rs.OptimalNode,primitives.TreeNode)
        def capture(original):
            def factory(*a,**kw):
                node=original(*a,**kw);nodes.append(node);return node
            return factory
        def snapshot(force=False):
            nonlocal first,last_snapshot,overhead
            now=time.perf_counter()-start
            solutions=[n for n in nodes if hasattr(n,'cost') and np.linalg.norm(np.array(n.config)-request.q_goal)<1e-6]
            if solutions:
                best=min(solutions,key=lambda n:n.cost);self.best_path=best.retrace()
                if first is None:first=now
            if trace_sink is None:return
            if not force and now-last_snapshot<cfg.snapshot_interval:return
            ot=time.perf_counter()
            if roadmap is not None:
                ns=list(roadmap.vertices.values()); qs=[v.q for v in ns];index={id(n):i for i,n in enumerate(ns)}
                edges=[[index[id(e.v1)],index[id(e.v2)]] for e in roadmap.edges]
            else:
                ns=list(nodes);qs=[n.config for n in ns]; index={id(n):i for i,n in enumerate(ns)}
                edges=[[index[id(n.parent)],i] for i,n in enumerate(ns) if n.parent is not None]
            # 显示截断仅影响快照，不影响算法统计和完整路径。
            limit=cfg.graph_limit
            snapshots.append(plain(TraceSnapshot(now,qs[:limit],[e for e in edges if max(e)<limit],cost(self.best_path))))
            snapshots[-1]['total_nodes']=len(ns);snapshots[-1]['truncated']=len(ns)>limit
            last_snapshot=now; overhead+=time.perf_counter()-ot
        orig_sample=sample
        def sample():snapshot();return orig_sample()
        rc.TreeNode=capture(originals[0]);rs.OptimalNode=capture(originals[1]);primitives.TreeNode=capture(originals[2])
        path=None; status='timeout'; reason='预算内未找到精确解';build_time=0.;query_time=0.;warm=False;bt=None;qt=None
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                if self.name=='rrt_connect':
                    path=rc.rrt_connect(tuple(request.q_start),tuple(request.q_goal),distance,sample,extend,collision,max_iterations=float('inf'),max_time=request.budget,tree_frequency=3)
                    if path:first=time.perf_counter()-start
                elif self.name=='rrt_star':
                    path=rs.rrt_star(tuple(request.q_start),tuple(request.q_goal),distance,sample,extend,collision,radius=1.5,max_time=request.budget,informed=False)
                else:
                    # 路网缓存绑定整个场景与碰撞配置；变化就重新构建。
                    key=(request.scene_id,tuple(r.limits.ravel()),cfg.edge_step,cfg.planning_margin)
                    warm=self.roadmap is not None and self.key==key
                    if not warm:
                        self.roadmap=pm.DegreePRM(distance,extend,collision,target_degree=8)
                        self.key=key
                    roadmap=self.roadmap;roadmap.collision_fn=collision
                    # 预留查询时间，建图可跨 warm 查询保留。
                    bt=time.perf_counter()
                    build_deadline=start+request.budget*.65
                    count=0
                    while not warm and count<cfg.prm_samples and time.perf_counter()<build_deadline:
                        q=sample()
                        if not collision(q):roadmap.grow([q]);count+=1
                    build_time=time.perf_counter()-bt
                    qt=time.perf_counter();path=roadmap(tuple(request.q_start),tuple(request.q_goal));query_time=time.perf_counter()-qt
                    if path:first=time.perf_counter()-start
        except BudgetExpired:
            if self.name=='rrt_star':snapshot(force=True);path=self.best_path or None
            reason='碰撞检查预算到期；timeout 不证明无路'
        except Exception as e:status='error';reason=f'{type(e).__name__}: {e}'
        finally:
            # 即使预算在建图/查询内部到期，也记录已消耗的该阶段成本。
            if self.name=='prm':
                if qt is not None:query_time=time.perf_counter()-qt
                elif bt is not None:build_time=time.perf_counter()-bt
            rc.TreeNode,rs.OptimalNode,primitives.TreeNode=originals
        elapsed=time.perf_counter()-start
        snapshot(force=True)
        if self.name=='rrt_star' and not path:path=self.best_path
        if path:status='success';reason=''
        stats={'backend':'motion-planners (vendored, patched)','implementation':self.name,'nodes':len(roadmap.vertices) if roadmap is not None else len(nodes),'edges':len(roadmap.edges) if roadmap is not None else sum(n.parent is not None for n in nodes),'trace_overhead':overhead,'prm_mode':'warm' if warm else 'cold' if self.name=='prm' else None,'build_time':build_time,'query_time':query_time,'budget':request.budget,'range_note':'fallback 使用上游连接扩展；extension_range 仅 OMPL 生效'}
        return PlanResult(status,raw_path=[np.asarray(q) for q in path] if path else [],elapsed=elapsed,first_solution=first,stats=stats,trace=snapshots,reason=reason)
