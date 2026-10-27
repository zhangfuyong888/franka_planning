"""七变量速度投影；所有速度/加速度/限位约束在同一个 OSQP 内。"""
import time
import numpy as np
import scipy.sparse as sp
import osqp
from .base import LocalAvoider
from .none import bounds
from ..types import AvoidanceResult,plain
class DistanceQP(LocalAvoider):
    def step(self,state,dq_nominal,context):
        t=time.perf_counter();c=context.config
        if np.shape(dq_nominal) != (7,) or not np.all(np.isfinite(dq_nominal)):
            return AvoidanceResult(np.zeros(7),'invalid_command')
        low,high=bounds(state,context)
        if np.any(low>high):return AvoidanceResult(np.zeros(7),'infeasible',{'reason':'基础运动限制冲突'})
        samples=context.collision.query_distances(state.q)
        rows=[np.eye(7)];ls=[low];us=[high];active=[]
        for s in samples:
            if not s.valid or not np.all(np.isfinite(s.gradient)):
                return AvoidanceResult(np.zeros(7),'invalid_distance',{'pair':s.pair})
            safe=c.safe_distance if 'obstacle' in s.pair[1] or s.pair[1]=='floor' else .015
            # OSQP 用 l<=Av<=u。梯度单位 m/rad，乘关节速度后为 m/s。
            rows.append(s.gradient[None,:]);ls.append(np.array([-c.alpha*(s.distance-safe)]));us.append(np.array([np.inf]));active.append(plain(s))
        A=sp.csc_matrix(np.vstack(rows));l=np.concatenate(ls);u=np.concatenate(us)
        problem=osqp.OSQP()
        problem.setup(P=sp.eye(7,format='csc')*(1+c.regularization),q=-np.asarray(dq_nominal),A=A,l=l,u=u,verbose=False,eps_abs=1e-6,eps_rel=1e-6,max_iter=4000,polishing=False)
        result=problem.solve(raise_error=False)
        status=result.info.status
        if result.info.status_val not in (1,2) or result.x is None or not np.all(np.isfinite(result.x)):
            # max_iter/数值失败不等于已经证明约束不可行。
            failure = 'infeasible' if result.info.status_val in (3,4,5,6) else 'solver_failure'
            return AvoidanceResult(np.zeros(7),failure,{'solver_status':status,'primal_residual':result.info.prim_res,'dual_residual':result.info.dual_res,'solve_time':time.perf_counter()-t})
        v=result.x;slack=np.minimum(A@v-l,u-A@v)
        diagnostics={'solver_status':status,'primal_residual':result.info.prim_res,'dual_residual':result.info.dual_res,'min_slack':float(min(slack)),'distance_slack':slack[7:],'active':active,'correction':v-dq_nominal,'solve_time':time.perf_counter()-t}
        if not np.isfinite(result.info.prim_res) or not np.isfinite(result.info.dual_res) or result.info.prim_res > c.qp_primal_tolerance or result.info.dual_res > c.qp_dual_tolerance:
            return AvoidanceResult(np.zeros(7),'residual_failure',diagnostics)
        if min(slack)<-c.qp_primal_tolerance:return AvoidanceResult(np.zeros(7),'constraint_violation',diagnostics)
        return AvoidanceResult(v,diagnostics=diagnostics)
