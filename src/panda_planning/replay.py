"""专用 playback_data 回放实测状态；无显示器可用 --html 导出。"""
import argparse,time,json
from pathlib import Path
import numpy as np
from .scenes import load
from .robot import Robot
from .types import PlanResult,DistanceSample

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',required=True);p.add_argument('--html');p.add_argument('--speed',type=float,default=1.)
    a=p.parse_args();root=Path(a.run_dir)
    if a.html:
        from .report import generate
        print(generate(root,a.html));return
    if a.speed<=0:raise ValueError('speed 必须大于零')
    scene=load(root/'scene.json');robot=Robot(scene);rows=json.loads((root/'telemetry.json').read_text());raw=json.loads((root/'plan.json').read_text())
    plan=PlanResult(**{k:v for k,v in raw.items() if k!='plan_id'})
    from .visualizer import LiveViewer
    data=robot.data(scene.q_start);viewer=LiveViewer(robot,data,scene,plan,'实测状态回放','非动力学重算')
    print('实测状态回放：设置专用 playback_data，不冒充新的动力学执行。')
    try:
        for i,row in enumerate(rows):
            if not viewer.running():break
            while viewer.paused and not viewer.single_step and viewer.running():viewer.viewer.sync();time.sleep(.02)
            viewer.single_step=False
            data.qpos[:]=row['q'];data.qvel[:]=row['dq'];data.time=row['t'];state=robot.state(data)
            sample=row.get('nearest');samples=[]
            if sample:
                samples=[DistanceSample(tuple(sample['pair']),sample['distance'],np.array(sample['points']),np.array(sample['gradient']),sample['valid'],tuple(sample['geom_ids']))]
            viewer.sync(state,samples,rows[:i+1])
            interval=(rows[i+1]['t']-row['t']) if i+1<len(rows) else 0.
            time.sleep(max(0.,interval)/a.speed)
    finally:viewer.close()
if __name__=='__main__':main()
