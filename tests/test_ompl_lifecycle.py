"""真实 OMPL 回归：短查询预算和 Python/C++ 对象生命周期。"""
import importlib.util
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context
from panda_planning.planners import create
from panda_planning.types import PlanningRequest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('ompl') is None, reason='需要真实OMPL轮子')


def test_prm_query_not_cut_short_by_snapshot_interval():
    scene = make_scene('empty')
    context = build_context(scene, Config(snapshot_interval=.001))
    planner = create('prm', 'ompl')
    request = PlanningRequest(np.array(scene.q_start), np.array(scene.q_goal), .8, 42, scene.scene_id)
    result = planner.plan(request, context, True)
    assert result.status == 'success', (result.reason, result.stats)
    assert context.collision.validate_path(result.raw_path)
    assert result.stats['prm_mode'] == 'cold'
    assert result.stats['budget_overrun'] < .15
    other = build_context(scene, Config(snapshot_interval=.001))
    warm = planner.plan(request, other, True)
    assert warm.status == 'success' and warm.stats['prm_mode'] == 'warm'
    assert other.collision.calls > 2
    planner.reset(other)
    assert planner.objects is None


def test_native_callbacks_do_not_retain_python_planners():
    script = '''
import gc, weakref
import numpy as np
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context
from panda_planning.planners import create
from panda_planning.types import PlanningRequest
s=make_scene('empty'); ctx=build_context(s,Config())
for name in ('rrt_connect','rrt_star','prm'):
    planner=create(name,'ompl')
    result=planner.plan(PlanningRequest(np.array(s.q_start),np.array(s.q_goal),.8,42,s.scene_id),ctx,True)
    assert result.status=='success', (name,result.status)
    if name=='prm':
        for _ in range(3):
            warm=planner.plan(PlanningRequest(np.array(s.q_start),np.array(s.q_goal),.8,42,s.scene_id),ctx)
            assert warm.status=='success' and warm.stats['prm_mode']=='warm'
    reference=weakref.ref(planner)
    del planner
    gc.collect()
    assert reference() is None, name+' callback retained planner'
print('native lifecycle clean')
'''
    proc = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert 'native lifecycle clean' in proc.stdout
    assert 'nanobind: leaked' not in proc.stderr, proc.stderr
    assert 'Planner setup called multiple times' not in proc.stderr, proc.stderr


def test_ompl_budget_expiry_keeps_failure_and_raw_path_empty():
    s = make_scene('empty')
    context = build_context(s, Config())
    for name in ('rrt_connect','rrt_star','prm'):
        planner = create(name,'ompl')
        result = planner.plan(PlanningRequest(np.array(s.q_start),np.array(s.q_goal),1e-9,42,s.scene_id),context)
        assert result.status == 'timeout' and result.raw_path == []
