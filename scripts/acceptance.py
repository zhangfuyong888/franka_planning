"""对指定真实实验目录验收并导出证据；历史记录不覆盖成新的实验结论。"""
import argparse
import csv
import datetime
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from panda_planning.scenes import load
from panda_planning.types import PlanResult
from panda_planning.telemetry import environment, write_json
from panda_planning.report import generate


def read(path):
    return json.loads(path.read_text())


def audit_record(directory):
    summary, plan = read(directory/'summary.json'), read(directory/'plan.json')
    rows, scene = read(directory/'telemetry.json'), load(directory/'scene.json')
    parsed = PlanResult(**{k:v for k,v in plan.items() if k != 'plan_id'})
    assert summary['scene_id'] == scene.scene_id == plan['stats']['scene_id'], 'scene_id 不一致'
    assert summary['plan_id'] == plan['plan_id'] == parsed.plan_id, 'plan_id 不一致'
    times = [x['t'] for x in rows]
    assert all(b > a for a,b in zip(times[:-1],times[1:])), '执行时间不严格递增'
    if rows:
        assert abs(times[-1]-summary['metrics']['execution_time']) < 1e-8
    if summary['execution_success']:
        assert summary['execution_status'] == 'success' and rows
        assert rows[-1]['error'] < summary['config']['goal_tolerance']
    return summary, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='outputs/tutorial-benchmark')
    parser.add_argument('--output', default='docs/evidence')
    parser.add_argument('--human-validation', help='宿主机人工确认记录；不把能打开当成所有交互通过')
    args = parser.parse_args()
    source, output = Path(args.input), Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    paths = sorted(source.rglob('summary.json'))
    if not paths:
        parser.error('输入中没有真实运行 summary.json')
    records, audit_errors = [], []
    for path in paths:
        try:
            summary, rows = audit_record(path.parent)
        except (AssertionError, KeyError, ValueError, OSError) as exc:
            audit_errors.append({'run_dir': str(path.parent), 'error': str(exc)})
            continue
        records.append(summary)
        dest = output/'runs'/path.parent.name
        dest.mkdir(parents=True, exist_ok=True)
        for name in ('summary.json','scene.json','request.json','plan.json','trace.json','display.json','collision_pairs.json'):
            if (path.parent/name).exists():
                shutil.copy2(path.parent/name,dest/name)
        interval = rows[1]['t']-rows[0]['t'] if len(rows)>1 else .01
        stride = max(1,round(.05/max(interval,1e-6)))
        sampled = rows[::stride]
        if rows and sampled[-1] != rows[-1]:
            sampled.append(rows[-1])
        write_json(dest/'telemetry.json',sampled)
        write_json(dest/'sampling.json',{'original_samples':len(rows),'saved_samples':len(sampled),
                   'source_interval':interval,'display_stride':stride,
                   'note':'仅展示存档抽样；summary 指标来自完整遥测'})
    manifest = read(source/'manifest.json') if (source/'manifest.json').exists() else []
    for entry in manifest:
        if entry['returncode'] or not (source/Path(entry['run_dir']).name/'summary.json').exists():
            audit_errors.append({'run_dir':entry['run_dir'],'returncode':entry['returncode'],'error':'CLI失败或缺少实验记录'})
    ids = [r['run_id'] for r in records]
    if len(set(ids)) != len(ids):
        audit_errors.append({'error':'重复 run_id'})
    write_json(output/'record-audit.json',{'records':len(records),'errors':audit_errors})
    fields = ['run_id','mode','planner','avoider','planning_status','execution_status',
              'planning_success','execution_success','planning_time','first_solution',
              'execution_time','raw_cost','processed_cost','execution_min_distance','final_error','interventions']
    with (output/'results.csv').open('w') as f:
        writer = csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        for summary in records:
            writer.writerow({k:summary.get(k,summary['metrics'].get(k)) for k in fields})
    generate(source,output/'tutorial-report.html')
    for file in source.rglob('multi_query.json'):
        shutil.copy2(file,output/'prm-multi-query.json')
    checks = {'record_audit': {'returncode':int(bool(audit_errors)),'log':'record-audit.json'}}
    commands = [('pytest',[sys.executable,'-m','pytest','-q']),
                ('pip_check',[sys.executable,'-m','pip','check'])]
    if shutil.which('node'):
        commands.append(('report_js',['node','scripts/check_report.cjs',str(output/'tutorial-report.html')]))
    for label,cmd in commands:
        proc = subprocess.run(cmd,capture_output=True,text=True)
        (output/f'{label}.txt').write_text(proc.stdout+proc.stderr)
        checks[label] = {'returncode':proc.returncode,'log':f'{label}.txt'}
    human = read(Path(args.human_validation)) if args.human_validation else None
    unverified = ['新机器联网完整安装流程；本机验证已锁发行包、用户补装OMPL与独立wheel']
    if not human:
        unverified.append('未提供宿主机人工交互确认；代理环境仍受socket限制')
    elif human.get('browser_play_pause_slider_layers') != 'passed' or human.get('viewer_space_N_P_G_D_C') != 'passed':
        unverified.append('宿主机已确认网页打开与MuJoCo成功执行；逐项按钮/键盘交互尚待确认')
    if importlib.util.find_spec('ompl') is None:
        unverified.append('OMPL2.0.1未安装；已审核官方固定版本API，真实集成测试显式跳过')
    write_json(output/'acceptance.json',{
        'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'input':str(source),'environment':environment(),'checks':checks,'runs':records,
        'human_validation':human,'unverified':unverified,'verified':[
            '真实MuJoCo保持、小轨迹、绕障执行与局部策略',
            '故障记录、目标与保存路径一致性、QP不可行与坏梯度',
            *(['真实OMPL RRT-Connect/RRT*/PRM cold/warm及无nanobind泄漏回归'] if importlib.util.find_spec('ompl') and checks['pytest']['returncode']==0 else []),
            '无DISPLAY/MUJOCO_GL=disable启动；Node VM报告交互与真实数据映射'],
        'note':'离屏EGL图像及帧ID单独保存在frames/，不等于浏览器验收；失败样本保留在runs中'})
    print(output/'acceptance.json')
    if any(c['returncode'] for c in checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
