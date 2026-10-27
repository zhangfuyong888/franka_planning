"""固定初值的 mink 差分 IK；位置主任务，姿态仅为软偏好。"""
import mink
import numpy as np
from .robot import HOME

def solve_ik(robot,target,collision=None,seed=HOME,orientation_cost=0.01,max_iterations=250,tolerance=0.002):
    configuration=mink.Configuration(robot.model)
    configuration.update(np.asarray(seed).copy())
    task=mink.FrameTask(frame_name='ee_site',frame_type='site',position_cost=1.,orientation_cost=orientation_cost,lm_damping=1.)
    pose=configuration.get_transform_frame_to_world('ee_site','site')
    task.set_target(mink.SE3.from_rotation_and_translation(pose.rotation(),np.asarray(target)))
    limits=[mink.ConfigurationLimit(robot.model)]
    for i in range(max_iterations):
        velocity=mink.solve_ik(configuration,[task],dt=0.02,solver='osqp',damping=1e-4,limits=limits)
        configuration.integrate_inplace(velocity,0.02)
        q=configuration.q.copy(); error=float(np.linalg.norm(robot.fk(q)-target))
        if error<tolerance:
            valid=np.all(q>=robot.limits[:,0]) and np.all(q<=robot.limits[:,1])
            if collision:valid=valid and collision.is_state_valid(q)
            return {'status':'success' if valid else 'invalid_goal','q':q,'error':error,'iterations':i+1}
    return {'status':'ik_unreachable','q':None,'error':error,'iterations':max_iterations}
