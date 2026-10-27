"""小配置统一安全参数，拒绝拼写错误以防参数悄悄失效。"""
from dataclasses import dataclass, asdict
from pathlib import Path
import math
import yaml

@dataclass
class Config:
    ik_orientation_cost: float = 0.01
    ik_iterations: int = 250
    ik_tolerance: float = 0.002
    dt: float = 0.01
    budget: float = 2.0
    max_time: float = 25.0
    edge_step: float = 0.025
    validation_step: float = 0.0125
    planning_margin: float = 0.012
    activation_distance: float = 0.10
    safe_distance: float = 0.025
    monitor_distance: float = 0.003
    velocity: float = 0.5
    acceleration: float = 1.5
    kp: float = 180.0
    kd: float = 28.0
    alpha: float = 5.0
    apf_gain: float = 0.00002
    regularization: float = 0.001
    shortcut_attempts: int = 60
    snapshot_interval: float = 0.25
    graph_limit: int = 250
    backend: str = 'auto'
    extension_range: float = 0.5
    prm_samples: int = 70
    goal_tolerance: float = 0.01
    settle_speed: float = 0.02
    settle_time: float = 0.2
    stall_window: float = 4.0
    tracking_limit: float = 0.2
    command_timeout: float = 0.05
    qp_primal_tolerance: float = 0.0001
    qp_dual_tolerance: float = 0.001

    def __post_init__(self):
        """直接构造与 YAML 入口执行同一验证，NaN 不能绕过大小比较。"""
        for name, value in asdict(self).items():
            if name == 'backend':
                if value not in ('auto', 'ompl', 'motion_planners'):
                    raise ValueError('backend 必须为 auto / ompl / motion_planners')
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} 必须为有限数值')
            nonnegative = name in ('ik_orientation_cost', 'apf_gain', 'regularization', 'monitor_distance', 'shortcut_attempts')
            if value < 0 or (value == 0 and not nonnegative):
                raise ValueError(f'{name} 超出允许范围')
        for name in ('ik_iterations', 'shortcut_attempts', 'graph_limit', 'prm_samples'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} 必须为整数')
        if not 0 <= self.monitor_distance < self.planning_margin < self.safe_distance < self.activation_distance:
            raise ValueError('阈值顺序应为 monitor < planning < safe < activation')
        if self.validation_step > self.edge_step:
            raise ValueError('validation_step 不能比规划边检查更粗')
        if self.command_timeout < self.dt:
            raise ValueError('command_timeout 不能小于一个控制周期')
    @classmethod
    def load(cls,path=None,**overrides):
        values = yaml.safe_load(Path(path).read_text()) if path else {}
        values = values or {}
        values.update({k:v for k,v in overrides.items() if v is not None})
        return cls(**values)
