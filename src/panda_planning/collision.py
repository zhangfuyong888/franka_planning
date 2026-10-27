"""统一整臂几何标准；距离截断不是精确距离，不能用于梯度。"""
import time
from itertools import combinations
import mujoco as mj
import numpy as np
from .types import DistanceSample

def interpolate(a,b,step):
    n=max(1,int(np.ceil(np.linalg.norm(np.asarray(b)-a)/step)))
    return np.linspace(a,b,n+1)

class Collision:
    def __init__(self,robot,config):
        self.robot=robot; self.config=config; self.model=robot.model
        self.data=mj.MjData(self.model); self.calls=0; self.seconds=0.; self.blocked=None
        m=self.model
        arm=[i for i in range(m.ngeom) if m.geom_group[i]==3]
        env=[i for i in range(m.ngeom) if m.geom_bodyid[i]==0]
        # 相邻/焊接装配体允许接触；其余跨连杆对均检查。基座只排除安装平面。
        rank={m.body(f'link{i}').id:i for i in range(8)}
        rank.update({m.body(n).id:8 for n in ('hand','left_finger','right_finger')})
        self.pairs=[]; self.excluded=[]
        for a in arm:
            for b in env:
                if m.geom_bodyid[a]==m.body('link0').id and m.geom(b).name=='floor':
                    self.excluded.append((a,b,'基座固定安装于地面')); continue
                self.pairs.append((a,b))
        for a,b in combinations(arm,2):
            ra,rb=rank[int(m.geom_bodyid[a])],rank[int(m.geom_bodyid[b])]
            # hand 与 link6/7 属于紧邻腕部装配；手指与手掌为固定装配。
            if abs(ra-rb)<=1 or (min(ra,rb)>=6):
                self.excluded.append((a,b,'相邻或固定腕部装配')); continue
            self.pairs.append((a,b))
    def checklist(self):
        name=lambda i:self.model.geom(i).name
        return {'checked':[[name(a),name(b)] for a,b in self.pairs], 'excluded':[[name(a),name(b),why] for a,b,why in self.excluded]}
    def query_distances(self,q,scene=None,activation_distance=None,gradients=True):
        t=time.perf_counter(); self.calls+=1
        maxd=activation_distance if activation_distance is not None else self.config.activation_distance
        self.data.qpos[self.robot.qids]=q; mj.mj_forward(self.model,self.data)
        out=[]; m=self.model; d=self.data
        for a,b in self.pairs:
            points=np.zeros(6)
            distance=mj.mj_geomDistance(m,d,a,b,maxd,points)
            if distance>=maxd-1e-10:continue
            points=points.reshape(2,3)
            delta=points[1]-points[0]; norm=np.linalg.norm(delta)
            valid=bool(np.isfinite(distance) and np.all(np.isfinite(points)) and norm>1e-9 and distance>0)
            grad=np.zeros(7)
            if valid and gradients:
                ja=np.zeros((3,m.nv)); jb=np.zeros_like(ja); jr=np.zeros_like(ja)
                mj.mj_jac(m,d,ja,jr,points[0],int(m.geom_bodyid[a]))
                mj.mj_jac(m,d,jb,jr,points[1],int(m.geom_bodyid[b]))
                # 法向从 A 指向 B，机器人 A 向 B 运动应使距离减小。
                grad=(delta/norm)@(jb-ja)[:,self.robot.dids]
                valid=bool(np.all(np.isfinite(grad)))
            out.append(DistanceSample((m.geom(a).name,m.geom(b).name),float(distance),points.copy(),grad,valid,(a,b)))
        self.seconds+=time.perf_counter()-t
        return out
    def is_state_valid(self,q,scene=None,margin=None):
        q=np.asarray(q)
        if q.shape!=(7,) or not np.all(np.isfinite(q)):return False
        if np.any(q<self.robot.limits[:,0]) or np.any(q>self.robot.limits[:,1]):return False
        margin=self.config.planning_margin if margin is None else margin
        return not self.query_distances(q,activation_distance=max(margin,1e-5),gradients=False)
    def is_edge_valid(self,a,b,scene=None,step=None):
        for q in interpolate(np.asarray(a),np.asarray(b),step or self.config.edge_step):
            if not self.is_state_valid(q):self.blocked=q.copy(); return False
        return True
    def validate_path(self,path):
        return len(path)>=2 and all(self.is_edge_valid(a,b,step=self.config.validation_step) for a,b in zip(path[:-1],path[1:]))
    def minimum(self,q):
        samples=self.query_distances(q,activation_distance=2.0,gradients=False)
        return min((s.distance for s in samples),default=2.0)
