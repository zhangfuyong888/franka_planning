"""纯离线、同场景同尺度的实测状态回放；所有 JS 和诊断都内嵌。"""
import argparse,json,html
from pathlib import Path
import numpy as np
from .types import plain
from .scenes import load
from .robot import Robot
from .trajectory import projected_path

def read(path):return json.loads(Path(path).read_text())

def collect(root):
    root=Path(root);summaries=sorted(root.rglob('summary.json')) if not (root/'summary.json').exists() else [root/'summary.json']
    records=[]
    for file in summaries:
        d=file.parent;summary=read(file);scene=read(d/'scene.json');rows=read(d/'telemetry.json');plan=read(d/'plan.json')
        # 以共同物理时间采样；绝不将每条记录归一化成相同长度。
        interval=rows[1]['t']-rows[0]['t'] if len(rows)>1 else .01
        stride=max(1,round(.05/max(interval,1e-6)))
        sampled=rows[::stride]
        if rows and (not sampled or sampled[-1]!=rows[-1]):sampled.append(rows[-1])
        robot=Robot(load(d/'scene.json'));snapshots=[]
        for snap in read(d/'trace.json'):
            qs=snap['nodes'];edges=snap['edges']
            # 实际关节边经插值再 FK，非末端投影直线。显示上限不改变统计。
            polylines=[projected_path(robot,[qs[i],qs[j]],.15) for i,j in edges[:120]]
            snapshots.append(dict(t=snap['t'],edges=polylines,nodes=[robot.fk(q).tolist() for q in qs],best_cost=snap['best_cost'],total_nodes=snap.get('total_nodes',len(qs)),displayed_edges=len(polylines)))
        if plan['stats'].get('blocked_q') is not None:
            plan['blocked_skeleton']=robot.skeleton(plan['stats']['blocked_q']).tolist()
        records.append(plain(dict(summary=summary,scene=scene,rows=sampled,plan=plan,display=read(d/'display.json'),snapshots=snapshots)))
    if not records:raise ValueError('没有找到 summary.json')
    return records

def generate(root,output):
    records=collect(root)
    js=(Path(__file__).parent/'assets'/'plotly-gl3d.min.js').read_text()
    template=(Path(__file__).parent/'assets'/'report_template.html').read_text()
    payload=json.dumps(records,ensure_ascii=False,allow_nan=False).replace('</','<\\/')
    queries=[]
    for qfile in Path(root).rglob('multi_query.json'):
        queries.extend(read(qfile))
    qpayload=json.dumps(queries,ensure_ascii=False,allow_nan=False).replace('</','<\\/')
    result=template.replace('/*PLOTLY_BUNDLE*/',js).replace('/*RUN_DATA*/',payload).replace('/*PRM_QUERIES*/',qpayload)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(result)
    return output

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();print(generate(a.input,a.output))
if __name__=='__main__':main()
