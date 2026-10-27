"""捷径保持统一碰撞检查，五次曲线在每个顶点停稳，不改变几何线段。"""
import numpy as np
from .collision import interpolate

def cost(path):
    return float(np.linalg.norm(np.diff(np.asarray(path),axis=0),axis=1).sum()) if len(path)>1 else None

def shortcut(path,collision,attempts=60,seed=43):
    out=[np.asarray(q).copy() for q in path]
    # 去除共线中间点仍必须验边，避免将弯路误简化。
    rng=np.random.default_rng(seed)
    for _ in range(attempts):
        if len(out)<=2:break
        i,j=sorted(rng.choice(len(out),2,replace=False))
        if j>i+1 and collision.is_edge_valid(out[i],out[j]):out[i+1:j]=[]
    return out

class Trajectory:
    def __init__(self,path,velocity=.5,acceleration=1.5):
        self.path=np.asarray(path)
        delta=np.abs(np.diff(self.path,axis=0))
        # s'(r) 的峰值 1.875，|s''(r)| 的峰值 10/sqrt(3)。
        self.durations=np.maximum(np.max(1.875*delta/velocity,axis=1),np.sqrt(np.max((10/np.sqrt(3))*delta/acceleration,axis=1)))
        self.durations=np.maximum(.05,self.durations)
        self.ends=np.cumsum(self.durations);self.total=float(self.ends[-1])
    def sample(self,t):
        if t>=self.total:return self.path[-1].copy(),np.zeros(7),np.zeros(7)
        i=int(np.searchsorted(self.ends,max(0,t),side='right'));T=self.durations[i]
        r=np.clip((t-(self.ends[i-1] if i else 0))/T,0,1);delta=self.path[i+1]-self.path[i]
        s=10*r**3-15*r**4+6*r**5
        ds=(30*r**2-60*r**3+30*r**4)/T
        dds=(60*r-180*r**2+120*r**3)/T**2
        return self.path[i]+s*delta,ds*delta,dds*delta

def projected_path(robot,path,step=.08):
    if not path:return []
    qs=[path[0]]
    for a,b in zip(path[:-1],path[1:]):qs.extend(interpolate(np.asarray(a),np.asarray(b),step)[1:])
    return [robot.fk(q).tolist() for q in qs]
