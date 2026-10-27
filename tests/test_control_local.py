from types import SimpleNamespace
import numpy as np
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context,run
from panda_planning.controller import Controller
from panda_planning.avoiders import create
from panda_planning.types import DistanceSample,AvoidanceResult

def test_real_dynamics_hold_and_small_step():
    s=make_scene('empty');ctx=build_context(s,Config());r=ctx.robot;d=r.data(s.q_start);control=Controller(r,ctx.config)
    q=np.array(s.q_start)
    for _ in range(500):control.step(d,q,np.zeros(7))
    assert np.max(abs(d.qpos-q))<1e-4
    command=q.copy();command[0]+=.05
    for _ in range(1000):control.step(d,command,np.zeros(7))
    assert np.linalg.norm(d.qpos-command)<.003 and abs(d.qpos[0]-q[0])>.045

def test_local_corrections_and_constraints():
    s=make_scene();ctx=build_context(s,Config(acceleration=100.));q=np.array(s.q_start)*.9+np.array(s.q_goal)*.1
    state=ctx.robot.state(ctx.robot.data(q));nom=(np.array(s.q_goal)-q)*.4
    apf=create('apf').step(state,nom,ctx);qp=create('distance_qp').step(state,nom,ctx)
    assert isinstance(apf,AvoidanceResult) and apf.status=='ok'
    assert np.linalg.norm(apf.diagnostics['correction'])>0
    assert apf.diagnostics['potential']>0
    assert qp.status=='ok' and qp.diagnostics['min_slack']>=-1e-4
    for sample in ctx.collision.query_distances(q):
        safe=ctx.config.safe_distance if 'obstacle' in sample.pair[1] or sample.pair[1]=='floor' else .015
        assert sample.gradient@qp.dq_cmd>=-ctx.config.alpha*(sample.distance-safe)-1e-4

def test_infeasible_and_bad_gradient_are_explicit():
    s=make_scene('empty');ctx=build_context(s,Config());state=ctx.robot.state(ctx.robot.data(s.q_start))
    class Injected:
        def query_distances(self,q):return [DistanceSample(('robot','obstacle_0'),.001,np.array([[0,0,0],[.001,0,0]]),np.ones(7)*1e-5,True)]
    ctx.collision=Injected()
    assert create('distance_qp').step(state,np.zeros(7),ctx).status=='infeasible'
    ctx.collision.query_distances=lambda q:[DistanceSample(('robot','obstacle_0'),.1,np.zeros((2,3)),np.ones(7)*np.nan,False)]
    for name in ('apf','distance_qp'):assert create(name).step(state,np.zeros(7),ctx).status=='invalid_distance'

def test_fixed_real_obstacle_execution_and_hybrid(tmp_path):
    s=make_scene();summary=run(s,Config(max_time=16),output=tmp_path/'global',trace=True)
    assert summary['planning_success'] and summary['execution_success']
    assert summary['metrics']['final_error']<.01 and not summary['interventions']
    assert summary['metrics']['execution_min_distance']>0
    hybrid=run(s,Config(max_time=.1),mode='hybrid',avoider='distance_qp',output=tmp_path/'hybrid',plan_file=tmp_path/'global'/'plan.json')
    assert hybrid['plan_id']==summary['plan_id']
    assert hybrid['execution_status']=='timeout'
