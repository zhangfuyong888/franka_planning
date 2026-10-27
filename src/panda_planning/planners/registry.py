"""只在创建时选后端；ompl 请求失败明确抛错，不在求解途中换算法。"""
import importlib.util
from .direct import Direct
from .fallback import MotionPlanners
REGISTRY={'direct':Direct,'rrt_connect':MotionPlanners,'rrt_star':MotionPlanners,'prm':MotionPlanners}
def create(name,backend='auto'):
    if name not in REGISTRY:raise ValueError(name)
    if name=='direct':return Direct()
    if backend=='ompl' or (backend=='auto' and importlib.util.find_spec('ompl')):
        from .ompl_backend import OMPLPlanner
        return OMPLPlanner(name)
    if backend not in ('auto','motion_planners'):raise ValueError(backend)
    return MotionPlanners(name)
