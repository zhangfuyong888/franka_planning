"""小型对照实验，各随机规划运行用独立子进程；warm PRM 独立实验组。"""
import argparse,subprocess,sys,json
from pathlib import Path
import numpy as np
from .scenes import make_scene,save,load
from .config import Config
from .runner import run,build_context
from .planners import create
from .types import PlanningRequest,plain
from .telemetry import write_json


def multi_query(output,budget=2.,seed=42,queries=10,backend='auto'):
    """同一几何下固定端点插值生成多查询，显式保存建图与查询成本。"""
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    scene=make_scene('multi_query',seed);ctx=build_context(scene,Config(budget=budget,backend=backend));save(scene,output/'scene.json')
    warm=create('prm',backend);records=[]
    for i in range(queries):
        a=np.asarray(scene.q_start).copy();b=np.asarray(scene.q_goal).copy()
        # 同一场景；仅变关节端点，障碍、边界和验证参数保持不变。
        a[0]+=.02*(i%5);b[0]-=.02*(i//5)
        req=PlanningRequest(a,b,budget,seed+i,scene.scene_id,ctx.robot.fk(b).tolist())
        for mode in ('cold','warm'):
            alg=create('prm',backend) if mode=='cold' else warm
            plan=alg.plan(req,ctx)
            valid=plan.status=='success' and ctx.collision.validate_path(plan.raw_path)
            records.append({'query':i,'group':mode,'request':plain(req),'status':'validation_failed' if plan.status=='success' and not valid else plan.status,'solver_status':plan.status,'validated':valid,'elapsed':plan.elapsed,'stats':plan.stats})
    write_json(output/'multi_query.json',records)
    return records

def hybrid_suite(output, seed=42, budget=2., max_time=20., backend='auto', trace=False):
    """先保存唯一全局路径，三个局部插件全部读取该 plan_id。"""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    scene_file = (output/'frozen-scene.json').resolve()
    save(make_scene('single_obstacle', seed), scene_file)
    reference = output/'reference'
    common = [sys.executable, '-m', 'panda_planning.demo', '--scene-file', str(scene_file),
              '--seed', str(seed), '--budget', str(budget), '--max-time', str(max_time), '--headless']
    manifest = []
    def launch(cmd, directory):
        process = subprocess.run(cmd, capture_output=True, text=True)
        (output/f'{directory.name}.log').write_text(process.stdout+process.stderr)
        manifest.append({'command':cmd,'returncode':process.returncode,'run_dir':str(directory)})
        return process.returncode
    command = common + ['--planner','rrt_connect','--backend',backend,'--output',str(reference)]
    if trace: command.append('--trace')
    failed = launch(command, reference)
    if failed or not (reference/'plan.json').exists() or json.loads((reference/'plan.json').read_text())['status'] != 'success':
        write_json(output/'manifest.json',manifest)
        raise RuntimeError('全局参考路径失败，hybrid 比较未启动；原因已保存')
    for name in ('none','apf','distance_qp'):
        directory = output/f'hybrid-{name}'
        command = common + ['--mode','hybrid','--avoider',name,'--plan-file',str((reference/'plan.json').resolve()),'--output',str(directory)]
        failed = launch(command,directory) or failed
    write_json(output/'manifest.json',manifest)
    from .report import generate
    generate(output,output/'report.html')
    if failed: raise RuntimeError('部分 hybrid 子进程运行错误；查看 manifest 与对应日志')
    return manifest

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',choices=['tutorial','prm','extended','hybrid'],default='tutorial');p.add_argument('--headless',action='store_true');p.add_argument('--output',default='outputs/tutorial-benchmark');p.add_argument('--budget',type=float,default=2.);p.add_argument('--max-time',type=float,default=20.);p.add_argument('--seeds',type=int,nargs='+',default=[42]);p.add_argument('--backend',choices=['auto','motion_planners','ompl'],default='auto');p.add_argument('--trace',action='store_true')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    if a.suite=='prm':multi_query(out,a.budget,a.seeds[0],10,a.backend);return
    if a.suite=='hybrid':
        for seed in a.seeds: hybrid_suite(out/str(seed),seed,a.budget,a.max_time,a.backend,a.trace)
        return
    manifest=[]
    for seed in a.seeds:
        scenes=['single_obstacle'] if a.suite=='tutorial' else ['empty','single_obstacle','elbow_obstacle','narrow_passage','random_static']
        for name in scenes:
            scene=make_scene(name,seed);scene_file=(out/f'{name}-{seed}.json').resolve();save(scene,scene_file)
            for planner in ('direct','rrt_connect','rrt_star','prm'):
                dest=out/f'{name}-{seed}-{planner}'
                cmd=[sys.executable,'-m','panda_planning.demo','--scene-file',str(scene_file),'--planner',planner,'--backend',a.backend,'--budget',str(a.budget),'--max-time',str(a.max_time),'--seed',str(seed),'--headless','--output',str(dest)]
                if a.trace:cmd.append('--trace')
                result=subprocess.run(cmd,capture_output=True,text=True)
                (out/f'{dest.name}.log').write_text(result.stdout+result.stderr)
                manifest.append({'command':cmd,'returncode':result.returncode,'run_dir':str(dest)})
                if result.returncode:print(result.stderr,file=sys.stderr)
        scene_file=(out/f'local_trap-{seed}.json').resolve();save(make_scene('local_trap',seed),scene_file)
        for avoider in ('none','apf','distance_qp'):
            dest=out/f'local-{seed}-{avoider}'
            cmd=[sys.executable,'-m','panda_planning.demo','--scene-file',str(scene_file),'--mode','local','--avoider',avoider,'--headless','--seed',str(seed),'--max-time',str(a.max_time),'--output',str(dest)]
            result=subprocess.run(cmd,capture_output=True,text=True);(out/f'{dest.name}.log').write_text(result.stdout+result.stderr)
            manifest.append({'command':cmd,'returncode':result.returncode,'run_dir':str(dest)})
    write_json(out/'manifest.json',manifest)
    multi_error = None
    try:
        multi_query(out/'prm-multi-query',a.budget,a.seeds[0],3 if a.suite=='tutorial' else 10,a.backend)
    except Exception as exc:
        multi_error = f'{type(exc).__name__}: {exc}'
        write_json(out/'multi_query_error.json',{'error':multi_error})
    from .report import generate
    print(generate(out,out/'report.html'))
    if multi_error or any(entry['returncode'] for entry in manifest):
        raise SystemExit(2)
if __name__=='__main__':main()
