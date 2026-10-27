"""从项目根目录执行：.venv/bin/python examples/01_collision.py。"""
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context
from panda_planning.config import Config
s=make_scene();c=build_context(s,Config()).collision
print('start:',c.is_state_valid(s.q_start),'goal:',c.is_state_valid(s.q_goal),'edge:',c.is_edge_valid(s.q_start,s.q_goal))
print('阻挡构型:',c.blocked)
print('最近几何:',[(d.pair,round(d.distance,4)) for d in c.query_distances(s.q_start)])
