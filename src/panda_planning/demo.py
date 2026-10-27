"""有限时长的规划/局部避障实验入口。"""
import argparse,json
from .config import Config
from .scenes import make_scene,load,NAMES
from .runner import run

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',choices=NAMES,default='single_obstacle');p.add_argument('--scene-file')
    p.add_argument('--mode',choices=['global','local','hybrid'],default='global')
    p.add_argument('--planner',choices=['direct','rrt_connect','rrt_star','prm'],default='rrt_connect')
    p.add_argument('--avoider',choices=['none','apf','distance_qp'],default='none')
    p.add_argument('--backend',choices=['auto','ompl','motion_planners'])
    p.add_argument('--target',type=float,nargs=3,help='世界坐标末端位置 x y z / m')
    p.add_argument('--seed',type=int,default=42);p.add_argument('--config');p.add_argument('--budget',type=float);p.add_argument('--max-time',type=float)
    p.add_argument('--headless',action='store_true');p.add_argument('--trace',action='store_true');p.add_argument('--plan-file');p.add_argument('--output',default='outputs/run')
    return p

def main():
    a=parser().parse_args();config=Config.load(a.config,budget=a.budget,max_time=a.max_time,backend=a.backend)
    scene=load(a.scene_file) if a.scene_file else make_scene(a.scene,a.seed)
    if a.target:
        from .ik import solve_ik
        from .runner import build_context
        ctx=build_context(scene,config)
        ik=solve_ik(ctx.robot,a.target,ctx.collision,orientation_cost=config.ik_orientation_cost,max_iterations=config.ik_iterations,tolerance=config.ik_tolerance)
        scene.target=a.target
        if ik['q'] is not None:scene.q_goal=ik['q'].tolist()
        from .types import plain
        scene.metadata['target_override_ik']=plain(ik)
    result=run(scene,config,a.planner,a.avoider,a.mode,a.seed,a.output,a.trace,a.headless,a.plan_file)
    print(json.dumps({k:result[k] for k in ('run_id','planning_status','execution_status','metrics')},ensure_ascii=False,indent=2))
    if result['planning_status'] == 'error' or result['execution_status'] in ('execution_error', 'viewer_unavailable'):
        raise SystemExit(2)
if __name__=='__main__':main()
