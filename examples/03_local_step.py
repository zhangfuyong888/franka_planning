"""同一构型/名义速度下查看势场和速度投影，不执行物理运动。"""
import numpy as np
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context
from panda_planning.config import Config
from panda_planning.avoiders import create
s=make_scene();ctx=build_context(s,Config());q=.9*np.array(s.q_start)+.1*np.array(s.q_goal)
state=ctx.robot.state(ctx.robot.data(q));nominal=.3*(np.array(s.q_goal)-q)
for name in ('none','apf','distance_qp'):
    result=create(name).step(state,nominal,ctx)
    print(name,result.status,'v=',result.dq_cmd,'potential=',result.diagnostics.get('potential'),'QP slack=',result.diagnostics.get('min_slack'))
