"""模型自包含；FK 与距离查询的 data 不与动力学运行 data 共享。"""
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib
import mujoco as mj
import numpy as np
from .types import RobotState

ASSETS=Path(__file__).parent/'assets'/'panda'
HOME=np.array([0,-0.6,0,-2.0,0,1.5,0.785398])

class Robot:
    def __init__(self, scene=None):
        root=ET.parse(ASSETS/'panda.xml').getroot()
        root.find('compiler').set('meshdir',str(ASSETS/'assets'))
        root.find('option').set('cone','elliptic')
        # 本机 3.3.7 native CCD 在分离 mesh 上出现 distance=0、见证点分离；
        # 禁用 nativeccd 后核验距离与见证点一致，并在测试中保留回归。
        root.find('option').set('ccd_tolerance','1e-8')
        root.find('option').set('ccd_iterations','100')
        ET.SubElement(root.find('option'),'flag',nativeccd='disable')
        world=root.find('worldbody')
        ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 0.05',rgba='.87 .89 .92 1')
        if scene:
            for i,o in enumerate(scene.obstacles):
                ET.SubElement(world,'geom',name=f'obstacle_{i}',type=o['shape'],pos=' '.join(map(str,o['pos'])),size=' '.join(map(str,o['size'])),quat=' '.join(map(str,o.get('quat',[1,0,0,0]))),rgba='.9 .48 .15 1',group='0')
        # 给原来无名的碰撞几何命名，日志中的几何对可直接追溯到连杆。
        for body in root.iter('body'):
            for i,g in enumerate(body.findall('geom')):
                if g.get('class')!='visual': g.set('name',f"{body.get('name')}_collision_{i}")
        self.model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        m=self.model
        self.joints=np.array([m.joint(f'joint{i}').id for i in range(1,8)])
        self.qids=m.jnt_qposadr[self.joints]; self.dids=m.jnt_dofadr[self.joints]
        self.aids=np.array([m.actuator(f'motor{i}').id for i in range(1,8)])
        self.limits=m.jnt_range[self.joints].copy()
        self.torque=m.actuator_ctrlrange[self.aids,1].copy()
        self.site=m.site('ee_site').id
        self.fk_data=mj.MjData(m)
        self.body_ids=[m.body(f'link{i}').id for i in range(8)]+[m.body('hand').id]
        self.model_id=hashlib.sha256((ASSETS/'panda.xml').read_bytes()).hexdigest()
        assert m.nq==m.nv==m.nu==7
        assert np.all(m.actuator_biastype==0) and np.all(m.actuator_gainprm[:,0]==1)
    def data(self,q=HOME):
        d=mj.MjData(self.model); d.qpos[self.qids]=q; mj.mj_forward(self.model,d); return d
    def forward(self,q):
        self.fk_data.qpos[self.qids]=q; mj.mj_forward(self.model,self.fk_data)
        return self.fk_data
    def fk(self,q): return self.forward(q).site_xpos[self.site].copy()
    def skeleton(self,q):
        d=self.forward(q)
        return np.vstack([d.xpos[self.body_ids],d.site_xpos[self.site]]).copy()
    def gripper(self,q):
        d=self.forward(q);m=self.model;paths=[]
        for name in ('left_finger','right_finger'):
            b=m.body(name).id
            paths.append(np.vstack([d.xpos[m.body('hand').id],d.xpos[b],d.xpos[b]+d.xmat[b].reshape(3,3)@np.array([0,0,.045])]).tolist())
        return paths
    def jacobian(self,d):
        jp=np.zeros((3,self.model.nv)); jr=np.zeros_like(jp)
        mj.mj_jacSite(self.model,d,jp,jr,self.site)
        return jp[:,self.dids]
    def state(self,d):
        mj.mj_forward(self.model,d)
        return RobotState(d.time,d.qpos[self.qids].copy(),d.qvel[self.dids].copy(),d.site_xpos[self.site].copy(),self.jacobian(d)@d.qvel[self.dids])
