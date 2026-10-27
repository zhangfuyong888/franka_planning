import numpy as np
from .base import LocalAvoider
from ..types import AvoidanceResult

def bounds(state,context):
    c=context.config;r=context.robot;dt=c.dt
    previous=context.previous_velocity
    low=np.maximum.reduce([-np.ones(7)*c.velocity,(r.limits[:,0]+.005-state.q)/dt,previous-c.acceleration*dt])
    high=np.minimum.reduce([np.ones(7)*c.velocity,(r.limits[:,1]-.005-state.q)/dt,previous+c.acceleration*dt])
    return low,high
class NoneAvoider(LocalAvoider):
    def step(self,state,dq_nominal,context):
        if np.shape(dq_nominal) != (7,) or not np.all(np.isfinite(dq_nominal)):
            return AvoidanceResult(np.zeros(7),'invalid_command')
        low,high=bounds(state,context)
        if np.any(low>high):return AvoidanceResult(np.zeros(7),'infeasible',{'reason':'基础运动限制不可行'})
        v=np.clip(dq_nominal,low,high)
        return AvoidanceResult(v,diagnostics={'base_limit_delta':float(np.linalg.norm(v-dq_nominal)),'correction':np.zeros(7)})
