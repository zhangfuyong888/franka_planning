"""固定场景保存真实 q_goal；随机拒绝只基于输入合法性，不按算法结果筛选。"""
import argparse,json
from pathlib import Path
import numpy as np
from .types import SceneSpec,plain
from .robot import Robot,HOME
from .config import Config
from .collision import Collision
from .ik import solve_ik

NAMES=['empty','single_obstacle','elbow_obstacle','narrow_passage','local_trap','random_static','multi_query','invalid_cases']

def save(scene,path):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(plain(scene),ensure_ascii=False,indent=2))

def load(path):return SceneSpec(**json.loads(Path(path).read_text()))

def make_scene(name='single_obstacle',seed=42):
    if name not in NAMES:raise ValueError(name)
    r=Robot(); start=HOME.copy(); start[0]=-.7
    nominal=HOME.copy(); nominal[0]=.7
    target=r.fk(nominal)
    ik=solve_ik(r,target)
    if ik['status']!='success':raise RuntimeError(ik)
    goal=ik['q']; mid=(start+goal)/2
    p=r.fk(mid)
    sphere=lambda pos,size:dict(shape='sphere',pos=list(pos),size=[size])
    obstacles=[]; meta={'generator':'v1','ik':plain(ik),'feasibility':'unknown','rejected':{}}
    if name in ('single_obstacle','multi_query','local_trap'):
        obstacles=[sphere(p,.075)]
    elif name=='elbow_obstacle':
        elbow=r.skeleton(mid)[5]; obstacles=[sphere(elbow+[0,0,.06],.035)]
    elif name=='narrow_passage':
        obstacles=[sphere(p,.075),sphere(p+[.20,0,.08],.075)]
    elif name=='invalid_cases':
        target=np.array([3.,0.,3.]); meta['expected']='ik_unreachable'
    elif name=='random_static':
        rng=np.random.default_rng(seed); rejected={}; attempts=0
        while len(obstacles)<3 and attempts<100:
            attempts+=1
            shape=rng.choice(['sphere','box','capsule'])
            size=([float(rng.uniform(.035,.07))] if shape=='sphere' else ([.045,.06,.10] if shape=='box' else [.035,.10]))
            candidate=dict(shape=str(shape),pos=rng.uniform([.20,-.35,.25],[.65,.35,.85]).tolist(),size=size)
            s=SceneSpec(name,obstacles+[candidate],start.tolist(),goal.tolist(),target.tolist(),seed)
            c=Collision(Robot(s),Config())
            if not c.is_state_valid(start) or not c.is_state_valid(goal):
                rejected['invalid_endpoint']=rejected.get('invalid_endpoint',0)+1;continue
            # 包围球避免静态障碍之间重叠；保守过滤只影响生成分布。
            def radius(o):return float(np.linalg.norm(o['size']))
            if any(np.linalg.norm(np.array(candidate['pos'])-o['pos'])<radius(candidate)+radius(o)+.01 for o in obstacles):
                rejected['obstacle_overlap']=rejected.get('obstacle_overlap',0)+1;continue
            obstacles.append(candidate)
        meta.update(rejected=rejected,attempts=attempts,feasibility='unknown',selection_bias='仅筛选端点与障碍合法性，未按规划结果筛选')
    scene=SceneSpec(name,obstacles,start.tolist(),goal.tolist(),target.tolist(),seed,meta)
    c=Collision(Robot(scene),Config())
    scene.metadata['start_valid']=c.is_state_valid(start)
    scene.metadata['goal_valid']=c.is_state_valid(goal)
    return scene

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--generate',choices=NAMES,default='random_static');p.add_argument('--seed',type=int,default=42);p.add_argument('--output',required=True)
    a=p.parse_args();s=make_scene(a.generate,a.seed);save(s,a.output);print(s.scene_id)
if __name__=='__main__':main()
