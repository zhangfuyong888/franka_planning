import numpy as np
import pytest
from panda_planning.config import Config
from panda_planning.scenes import make_scene,save,load
from panda_planning.runner import build_context
from panda_planning.robot import HOME
from panda_planning.ik import solve_ik

@pytest.fixture(scope='module')
def setup():
    scene=make_scene('single_obstacle');return scene,build_context(scene,Config())

def test_endpoints_do_not_prove_edge_valid(setup):
    s,c=setup
    assert c.collision.is_state_valid(s.q_start)
    assert c.collision.is_state_valid(s.q_goal)
    assert not c.collision.is_edge_valid(s.q_start,s.q_goal)
    assert c.collision.blocked.shape==(7,)

def test_elbow_risk_is_not_ee_risk():
    s=make_scene('elbow_obstacle');ctx=build_context(s,Config());q=(np.array(s.q_start)+s.q_goal)/2
    assert ctx.collision.is_state_valid(s.q_start) and ctx.collision.is_state_valid(s.q_goal)
    assert not ctx.collision.is_state_valid(q)
    assert np.linalg.norm(ctx.robot.fk(q)-s.obstacles[0]['pos'])>s.obstacles[0]['size'][0]+.1
    assert any(x.distance<0 for x in ctx.collision.query_distances(q) if 'obstacle' in x.pair[1])

def test_gradient_and_witness_consistency(setup):
    s,ctx=setup;q=np.array(s.q_start)*.8+np.array(s.q_goal)*.2
    samples=ctx.collision.query_distances(q,activation_distance=.3)
    sample=next(x for x in samples if x.pair[0]=='right_finger_collision_3' and x.pair[1]=='obstacle_0')
    assert sample.valid
    derivative=[]
    for i in range(7):
        a=q.copy();b=q.copy();a[i]+=1e-5;b[i]-=1e-5
        def d(q):return next(x.distance for x in ctx.collision.query_distances(q,activation_distance=.3) if x.pair==sample.pair)
        derivative.append((d(a)-d(b))/2e-5)
    np.testing.assert_allclose(sample.gradient,derivative,atol=1e-4,rtol=1e-3)
    for x in samples:
        if x.distance>0:assert abs(np.linalg.norm(x.points[1]-x.points[0])-x.distance)<1e-6
    assert not ctx.collision.query_distances(s.q_start,activation_distance=.00001)

def test_independent_ik_and_unreachable():
    s=make_scene('empty');ctx=build_context(s,Config());r=ctx.robot;data=r.data(s.q_start);before=data.qpos.copy()
    result=solve_ik(r,r.fk(HOME)+[.05,.04,.01],ctx.collision)
    assert result['status']=='success'
    assert np.linalg.norm(r.fk(result['q'])-(r.fk(HOME)+[.05,.04,.01]))<.002
    np.testing.assert_array_equal(data.qpos,before)
    assert solve_ik(r,[3,0,3],max_iterations=30)['status']=='ik_unreachable'

def test_scene_roundtrip_random_rejections(tmp_path):
    s=make_scene('random_static',42);save(s,tmp_path/'scene.json');new=load(tmp_path/'scene.json')
    assert s.scene_id==new.scene_id
    assert new.metadata['start_valid'] and new.metadata['goal_valid']
    assert new.metadata['feasibility']=='unknown'
    assert len(new.obstacles)==3
    assert new.obstacles==make_scene('random_static',42).obstacles

def test_configuration_rejects_typo():
    with pytest.raises(TypeError):Config.load(None,velocty=2)
