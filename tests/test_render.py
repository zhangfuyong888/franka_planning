import json
import os
from pathlib import Path
import subprocess
import sys
import mujoco as mj
import numpy as np
import pytest
from panda_planning.scenes import make_scene
from panda_planning.runner import build_context, run
from panda_planning.config import Config
from panda_planning.overlays import draw


def test_overlay_does_not_mutate_physics():
    spec = make_scene('empty')
    context = build_context(spec, Config())
    robot = context.robot
    rgba = robot.model.geom_rgba.copy()
    data = robot.data(spec.q_start)
    q = data.qpos.copy()
    scene = mj.MjvScene(robot.model, maxgeom=100)
    draw(scene, robot, spec, {'raw_path': [], 'processed_path': []})
    assert scene.ngeom == 2  # 只生成起始/目标装饰；没有任何物理写入。
    np.testing.assert_array_equal(robot.model.geom_rgba, rgba)
    np.testing.assert_array_equal(data.qpos, q)


@pytest.mark.skipif(os.environ.get('PANDA_TEST_RENDER') != '1', reason='可选真实EGL离屏测试，PANDA_TEST_RENDER=1启用')
def test_real_egl_record_render(tmp_path):
    from PIL import Image
    result = run(make_scene('empty'), Config(max_time=.03), planner='direct', output=tmp_path/'run')
    proc = subprocess.run([sys.executable,'-m','panda_planning.render','--run-dir',str(tmp_path/'run'),
                           '--times','0','.03','--width','480','--height','360','--output',str(tmp_path/'frames')],
                          capture_output=True,text=True,timeout=60)
    assert proc.returncode == 0, proc.stderr
    entries = json.loads((tmp_path/'frames'/'frames.json').read_text())
    assert entries[-1]['run_id'] == result['run_id']
    assert entries[-1]['recorded_time'] == pytest.approx(.03)
    assert entries[-1]['recording'] == 'measured_state_playback'
    picture = np.asarray(Image.open(tmp_path/'frames'/entries[-1]['file']))
    assert picture.shape == (360,480,3) and picture.std() > 10
