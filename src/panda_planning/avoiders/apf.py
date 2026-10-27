"""连续到激活边界的 U=η/2(1/d-1/d0)^2；输出速度修正，不是物理力。"""
import numpy as np
from .base import LocalAvoider
from .none import bounds
from ..types import AvoidanceResult,plain
class APF(LocalAvoider):
    def step(self,state,dq_nominal,context):
        c=context.config
        if np.shape(dq_nominal) != (7,) or not np.all(np.isfinite(dq_nominal)):
            return AvoidanceResult(np.zeros(7),'invalid_command')
        samples=context.collision.query_distances(state.q)
        correction=np.zeros(7);potential=0.;active=[]
        for s in samples:
            if not s.valid or not np.all(np.isfinite(s.gradient)):
                return AvoidanceResult(np.zeros(7),'invalid_distance',{'pair':s.pair})
            # 固定装配附近的跨腕部自碰距离很小，采用独立激活范围避免永久排斥。
            d0=c.activation_distance if 'obstacle' in s.pair[1] or s.pair[1]=='floor' else .03
            if s.distance>=d0:continue
            d=max(s.distance,.005);eta=c.apf_gain
            potential+=eta/2*(1/d-1/d0)**2
            correction+=eta*(1/d-1/d0)/d**2*s.gradient
            active.append(plain(s))
        low,high=bounds(state,context)
        if np.any(low>high):return AvoidanceResult(np.zeros(7),'infeasible')
        v=np.clip(dq_nominal+correction,low,high)
        return AvoidanceResult(v,diagnostics={'potential':potential,'correction':correction,'active':active,'base_limit_delta':float(np.linalg.norm(v-dq_nominal-correction))})
