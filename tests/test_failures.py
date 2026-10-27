"""故障注入只验证退出/记录语义；动力学成功仍由真实集成测试证明。"""
import json
from dataclasses import replace
import numpy as np
import pytest
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.runner import run, build_context
from panda_planning.monitor import Monitor
from panda_planning.types import PlanResult


@pytest.mark.parametrize('values', [
    {'dt': float('nan')}, {'budget': float('inf')}, {'alpha': -1}, {'kp': 0},
    {'regularization': -1}, {'graph_limit': 2.5}, {'backend': 'typo'},
    {'safe_distance': .001}, {'validation_step': .05}, {'command_timeout': .001},
])
def test_config_validates_direct_construction(values):
    with pytest.raises(ValueError):
        Config(**values)


@pytest.mark.parametrize('kwargs, expected', [
    ({'min_distance': float('nan')}, 'numerical_error'),
    ({'command_t': -1}, 'stale_command'),
    ({'command': np.array([float('nan')]*7)}, 'invalid_command'),
    ({'torque': np.array([100.]*7)}, 'torque_limit'),
    ({'numerical_warning': True}, 'numerical_error'),
])
def test_monitor_faults(kwargs, expected):
    scene = make_scene('empty')
    ctx = build_context(scene, Config())
    state = ctx.robot.state(ctx.robot.data(scene.q_start))
    values = dict(min_distance=.1, command=np.zeros(7), command_t=state.t, torque=np.zeros(7))
    values.update(kwargs)
    monitor = Monitor(ctx)
    assert monitor.check(state, state.q, progress=0., **values) == expected
    assert monitor.events[-1]['status'] == expected


def test_fixed_goal_mismatch_does_not_silently_change_branch(tmp_path):
    scene = make_scene('empty')
    scene.target = (np.array(scene.target)+[.02, 0, .01]).tolist()
    summary = run(scene, Config(), output=tmp_path)
    assert summary['planning_status'] == 'invalid_goal'
    assert summary['execution_status'] == 'not_executed'
    assert json.loads((tmp_path/'telemetry.json').read_text()) == []


def test_invalid_start_rejected_also_in_local_mode(tmp_path):
    scene = make_scene('empty')
    scene.q_start[3] = 2.
    summary = run(scene, Config(), mode='local', output=tmp_path)
    assert summary['planning_status'] == 'invalid_start'
    assert not summary['execution_success']


def test_planner_exception_and_malformed_success_keep_records(tmp_path):
    class BrokenPlanner:
        def plan(self, *args):
            raise RuntimeError('injected planner fault')
    class MalformedPlanner:
        def plan(self, *args):
            return PlanResult('success', raw_path=[[0, 1], [1]])
    for name, planner, expected in [('exception', BrokenPlanner(), 'error'),
                                     ('malformed', MalformedPlanner(), 'validation_failed')]:
        dest = tmp_path/name
        summary = run(make_scene('empty'), Config(), planner_instance=planner, output=dest)
        assert summary['planning_status'] == expected
        assert not summary['execution_success']
        assert (dest/'summary.json').exists() and (dest/'telemetry.json').exists()


def test_local_plugin_exception_preserves_physical_observation(tmp_path, monkeypatch):
    from panda_planning import sim
    class BrokenAvoider:
        def reset(self, context): pass
        def step(self, *args): raise RuntimeError('injected local fault')
    monkeypatch.setattr(sim, 'avoider_create', lambda name: BrokenAvoider())
    summary = run(make_scene('empty'), Config(), mode='local', output=tmp_path)
    assert summary['execution_status'] == 'solver_failure'
    rows = json.loads((tmp_path/'telemetry.json').read_text())
    assert len(rows) == 1 and 'injected local fault' in rows[0]['diagnostics']['exception']
    assert np.allclose(rows[0]['q'], json.loads((tmp_path/'scene.json').read_text())['q_start'])


def test_saved_plan_hash_and_scene_are_enforced(tmp_path):
    scene = make_scene('empty')
    first = run(scene, Config(max_time=.02), planner='direct', output=tmp_path/'first')
    path = tmp_path/'first'/'plan.json'
    saved = json.loads(path.read_text())
    saved['processed_path'][1][0] += .01
    path.write_text(json.dumps(saved))
    result = run(scene, Config(max_time=.02), mode='hybrid', plan_file=path, output=tmp_path/'second')
    assert result['planning_status'] == 'error' and not result['planning_success']
    assert 'plan_id' in result['planning_reason']


def test_timeout_log_includes_exact_last_physical_state(tmp_path):
    result = run(make_scene('empty'), Config(max_time=.1), planner='direct', output=tmp_path)
    rows = json.loads((tmp_path/'telemetry.json').read_text())
    assert rows[-1]['t'] == pytest.approx(.1)
    assert result['metrics']['execution_time'] == pytest.approx(.1)


def test_nonfinite_distance_failure_keeps_partial_log(tmp_path, monkeypatch):
    from panda_planning import sim
    from panda_planning.telemetry import execution_metrics
    # 单元级故障数据用于验证聚合，不冒充真实碰撞观测。
    rows = [{'t':0., 'min_distance':.02, 'dq':[0.]*7, 'torque':[0.]*7,
             'ee':[0.,0.,0.], 'error':.1, 'diagnostics':{}, 'saturated':False},
            {'t':.01, 'min_distance':None, 'dq':[None]*7, 'torque':[None]*7,
             'ee':[None]*3, 'error':None, 'diagnostics':{}, 'saturated':False}]
    metrics = execution_metrics(rows)
    assert metrics['execution_min_distance'] == .02
    assert metrics['final_error'] is None
