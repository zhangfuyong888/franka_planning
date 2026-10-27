"""从真实日志产生可导出的指标图，不伪造截图或填补失败代价。"""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('outputs/tutorial-benchmark');out=Path('docs/evidence');out.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(1,3,figsize=(14,4),layout='constrained')
for name,color in [('none','#607d9d'),('apf','#c47236'),('distance_qp','#27877b')]:
    d=root/f'local-42-{name}';rows=json.loads((d/'telemetry.json').read_text());s=json.loads((d/'summary.json').read_text());t=[x['t'] for x in rows]
    axes[0].plot(t,[x['error'] for x in rows],label=f"{name}: {s['execution_status']}",color=color)
    axes[1].plot(t,[x['min_distance'] for x in rows],label=name,color=color)
    if name=='apf':axes[2].plot(t,[x['diagnostics'].get('potential',0) for x in rows],label='APF artificial potential',color=color)
axes[0].set(ylabel='EE error / m',xlabel='Physical time / s',title='Same local target, seed 42')
axes[1].axhline(.003,color='black',ls='--',label='monitor 0.003 m');axes[1].set(ylabel='Minimum geometry distance / m',xlabel='Physical time / s',title='Actual full-arm minimum')
axes[2].set(ylabel='Velocity potential (artificial units)',xlabel='Physical time / s',title='APF diagnostic, not physical energy')
for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.savefig(out/'local-metrics.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for name in ('rrt_connect','prm'):
    s=json.loads((root/f'single_obstacle-42-{name}'/'summary.json').read_text());axes[0].plot(['raw','shortcut'],[s['metrics']['raw_cost'],s['metrics']['processed_cost']],'o-',label=name)
axes[0].set(ylabel='Joint path length / rad (W=I)',title='Only validated solutions have a path cost')
axes[0].text(.03,.03,'direct: blocked; RRT*: timeout (no cost)',transform=axes[0].transAxes,fontsize=8)
star=Path('outputs/rrt-star-empty/trace.json')
if star.exists():
    trace=json.loads(star.read_text());axes[1].step([x['t'] for x in trace],[x['best_cost'] for x in trace],where='post',label='empty: measured best_cost')
axes[1].set(xlabel='Planning wall time / s',ylabel='Best joint path cost / rad',title='RRT*: no invented improvement')
for ax in axes:ax.legend();ax.grid(alpha=.2)
fig.savefig(out/'planning-metrics.png',dpi=160);plt.close(fig)
