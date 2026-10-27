import time
from .base import GlobalPlanner
from ..types import PlanResult

class Direct(GlobalPlanner):
    def plan(self,request,context,trace_sink=None):
        t=time.perf_counter(); c=context.collision
        for name,q in [('start',request.q_start),('goal',request.q_goal)]:
            if not c.is_state_valid(q):return PlanResult('invalid_'+name,reason=f'{name} 不合法')
        ok=c.is_edge_valid(request.q_start,request.q_goal)
        path=[request.q_start.copy(),request.q_goal.copy()] if ok else []
        return PlanResult('success' if ok else 'blocked',raw_path=path,elapsed=time.perf_counter()-t,first_solution=time.perf_counter()-t if ok else None,stats={'backend':'direct','blocked_q':None if ok else c.blocked.tolist(),'attempted_path':[request.q_start.tolist(),request.q_goal.tolist()]},reason='' if ok else '直连中间构型被阻挡')
