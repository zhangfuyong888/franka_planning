import json,subprocess,os,sys,shutil
import pytest
from pathlib import Path
from panda_planning.runner import run
from panda_planning.config import Config
from panda_planning.scenes import make_scene
from panda_planning.report import generate

def test_logs_report_and_headless_without_display(tmp_path):
    env=os.environ.copy();env.pop('DISPLAY',None);env['MUJOCO_GL']='disable'
    cmd=[sys.executable,'-m','panda_planning.demo','--scene','empty','--planner','direct','--headless','--max-time','.1','--output',str(tmp_path/'run')]
    subprocess.run(cmd,cwd='/tmp',env=env,check=True,capture_output=True)
    root=tmp_path/'run';summary=json.loads((root/'summary.json').read_text());plan=json.loads((root/'plan.json').read_text());rows=json.loads((root/'telemetry.json').read_text())
    assert summary['plan_id']==plan['plan_id'] and summary['scene_id']==plan['stats']['scene_id']
    assert rows and rows[0]['q']==json.loads((root/'scene.json').read_text())['q_start']
    output=generate(root,tmp_path/'report.html');text=output.read_text()
    assert summary['run_id'] in text and '基于记录状态的回放' in text
    assert '<script src=' not in text and 'plotly.js (gl3d - minified) v3.1.0' in text
    checker=Path(__file__).resolve().parents[1]/'scripts'/'check_report.cjs'
    if not shutil.which('node'):pytest.skip('Node 不可用；HTML 数据已验证，JS 交互验证待补')
    subprocess.run(['node',str(checker),str(output)],check=True,capture_output=True,text=True)
