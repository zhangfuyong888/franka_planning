"""直接力矩执行：PD + 独立静态 data 计算的重力补偿。"""
import mujoco as mj
import numpy as np
class Controller:
    def __init__(self,robot,config):
        self.robot=robot;self.config=config;self.gravity_data=robot.data();self.saturation_count=0
    def step(self,data,q_command,dq_command):
        r=self.robot;c=self.config;g=self.gravity_data
        g.qpos[:]=data.qpos;g.qvel[:]=0.;mj.mj_forward(r.model,g)
        tau=c.kp*(q_command-data.qpos[r.qids])+c.kd*(dq_command-data.qvel[r.dids])+g.qfrc_bias[r.dids]
        clipped=np.clip(tau,-r.torque,r.torque)
        saturated=bool(np.any(np.abs(clipped-tau)>1e-8))
        self.saturation_count+=saturated
        data.ctrl[r.aids]=clipped
        mj.mj_step(r.model,data)
        return clipped.copy(),saturated
