"""严格 JSON，失败指标用 null；环境和源代码内容指纹与记录绑定。"""
from pathlib import Path
from dataclasses import asdict
import json,platform,sys,importlib.metadata as md,hashlib
from .types import plain

def write_json(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(plain(obj),ensure_ascii=False,indent=2,allow_nan=False))

def environment():
    packages={}
    for name in ('mujoco','mink','numpy','scipy','osqp','PyYAML','matplotlib','ompl'):
        try:packages[name]=md.version(name)
        except md.PackageNotFoundError:packages[name]=None
    digest=hashlib.sha256()
    for p in sorted(Path(__file__).parent.rglob('*.py')):digest.update(p.relative_to(Path(__file__).parent).as_posix().encode());digest.update(p.read_bytes())
    return {'python':sys.version,'platform':platform.platform(),'packages':packages,'source_sha256':digest.hexdigest()}


def execution_metrics(rows):
    """数值故障的原始 null 留在遥测；聚合时不让 None 引发第二次崩溃。"""
    import numpy as np

    def finite(values):
        return [float(x) for x in values if x is not None and np.isfinite(x)]

    def peak(name):
        values = finite(v for row in rows for v in (row.get(name) or []))
        return max((abs(v) for v in values), default=None)

    def norm(value):
        try:
            array = np.asarray(value, dtype=float)
            return float(np.linalg.norm(array)) if np.all(np.isfinite(array)) else None
        except (TypeError, ValueError):
            return None

    length = 0.
    for a, b in zip(rows[:-1], rows[1:]):
        delta = np.asarray(b['ee'],dtype=float)-np.asarray(a['ee'],dtype=float)
        value = norm(delta)
        if value is not None:
            length += value
    distances = finite(row.get('min_distance') for row in rows)
    corrections = finite(norm(row.get('diagnostics',{}).get('correction')) for row in rows)
    return {
        'execution_time': rows[-1]['t'] if rows else None,
        'execution_min_distance': min(distances,default=None),
        'final_error': rows[-1]['error'] if rows else None,
        'velocity_peak': peak('dq'), 'torque_peak': peak('torque'),
        'ee_path_length': length if rows else None,
        'local_correction_peak': max(corrections,default=None),
        'saturation_samples': sum(row.get('saturated',False) for row in rows),
    }
