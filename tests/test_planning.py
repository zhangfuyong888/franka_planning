import importlib.util
import numpy as np
import pytest
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context
from panda_planning.planners import create
from panda_planning.types import PlanningRequest,PlanResult
from panda_planning.trajectory import shortcut,Trajectory,cost

def request(s,budget=1.5):return PlanningRequest(np.array(s.q_start),np.array(s.q_goal),budget,42,s.scene_id)

def test_direct_blocked_and_invalid():
    s=make_scene();c=build_context(s,Config());p=create('direct').plan(request(s),c)
    assert p.status=='blocked' and not p.raw_path and p.stats['blocked_q']
    req=request(s);req.q_start[3]=2
    assert create('direct').plan(req,c).status=='invalid_start'

def test_rrtconnect_and_shortcut_time_scaling():
    s=make_scene();c=build_context(s,Config());p=create('rrt_connect','motion_planners').plan(request(s),c,True)
    assert isinstance(p,PlanResult) and p.status=='success'
    assert c.collision.validate_path(p.raw_path)
    assert len(p.trace[-1]['nodes'])>2 and p.trace[-1]['edges']
    processed=shortcut(p.raw_path,c.collision,seed=43)
    assert c.collision.validate_path(processed)
    assert cost(processed)<=cost(p.raw_path)+1e-8
    t=Trajectory(processed,c.config.velocity,c.config.acceleration)
    for x in np.linspace(0,t.total,300):
        q,v,a=t.sample(x)
        assert max(abs(v))<=c.config.velocity+1e-8
        assert max(abs(a))<=c.config.acceleration+1e-8
    np.testing.assert_allclose(t.sample(t.total)[0],s.q_goal)

def test_true_rrtstar_and_budget_timeout():
    s=make_scene('empty');ctx=build_context(s,Config());p=create('rrt_star','motion_planners').plan(request(s,.3),ctx,True)
    assert p.stats['implementation']=='rrt_star' and p.status=='success'
    assert cvalid(ctx,p)
    costs=[x['best_cost'] for x in p.trace if x['best_cost'] is not None]
    assert costs and all(a>=b-1e-8 for a,b in zip(costs[:-1],costs[1:]))
    assert create('rrt_connect','motion_planners').plan(request(s,1e-10),ctx).status=='timeout'

def cvalid(ctx,p):return ctx.collision.validate_path(p.raw_path)

def test_prm_cold_warm_and_scene_invalidation():
    s=make_scene('empty');ctx=build_context(s,Config(prm_samples=8));alg=create('prm','motion_planners')
    cold=alg.plan(request(s),ctx);warm=alg.plan(request(s),ctx)
    assert cold.status==warm.status=='success'
    assert cold.stats['prm_mode']=='cold' and warm.stats['prm_mode']=='warm'
    assert warm.stats['build_time']<.001
    req=request(s);req.scene_id='changed-geometry'
    assert alg.plan(req,ctx).stats['prm_mode']=='cold'

@pytest.mark.skipif(importlib.util.find_spec('ompl') is None,reason='终端网络被禁止，OMPL 未安装；替代后端测试不冒充 OMPL 验证')
def test_real_ompl_integration():
    s=make_scene('empty');ctx=build_context(s,Config())
    p=create('rrt_connect','ompl').plan(request(s),ctx,True)
    assert p.status=='success' and ctx.collision.validate_path(p.raw_path)

@pytest.mark.skipif(importlib.util.find_spec('ompl') is None, reason='OMPL未安装；需真实绑定验证')
@pytest.mark.parametrize('name', ['rrt_star', 'prm'])
def test_real_ompl_remaining_algorithms(name):
    s = make_scene('empty')
    ctx = build_context(s, Config())
    alg = create(name, 'ompl')
    result = alg.plan(request(s, .8), ctx, True)
    assert result.status == 'success'
    assert ctx.collision.validate_path(result.raw_path)
    assert result.trace[-1]['nodes']
    if name == 'prm':
        other_context = build_context(s, Config())
        warm = alg.plan(request(s, .8), other_context, True)
        assert warm.stats['prm_mode'] == 'warm'
        assert warm.stats['build_time'] == 0
        assert warm.status == 'success' and other_context.collision.calls > 2
