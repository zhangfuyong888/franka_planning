"""插件间只传数据；规划路径和速度命令故意使用不同类型。"""
from dataclasses import dataclass, field, asdict
import hashlib, json
import numpy as np

def plain(x):
    if hasattr(x, '__dataclass_fields__'): return plain(asdict(x))
    if isinstance(x, np.ndarray): return plain(x.tolist())
    if isinstance(x, np.generic): return plain(x.item())
    if isinstance(x, dict): return {str(k): plain(v) for k,v in x.items()}
    if isinstance(x, (list,tuple)): return [plain(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x): return None
    return x

def stable_id(x):
    return hashlib.sha256(json.dumps(plain(x),sort_keys=True,ensure_ascii=False).encode()).hexdigest()[:16]

@dataclass
class SceneSpec:
    name: str
    obstacles: list
    q_start: list
    q_goal: list
    target: list
    seed: int = 42
    metadata: dict = field(default_factory=dict)
    model: str = 'menagerie-panda-fixed-fingers-v1'
    @property
    def scene_id(self): return stable_id(self)

@dataclass
class PlanningRequest:
    q_start: np.ndarray
    q_goal: np.ndarray
    budget: float = 2.0
    seed: int = 42
    scene_id: str = ''
    target: list = field(default_factory=list)

@dataclass
class PlanResult:
    status: str
    raw_path: list = field(default_factory=list)
    processed_path: list = field(default_factory=list)
    elapsed: float = 0.0
    first_solution: float | None = None
    stats: dict = field(default_factory=dict)
    trace: list = field(default_factory=list)
    reason: str = ''
    @property
    def plan_id(self): return stable_id([self.processed_path, self.stats.get('scene_id')])

@dataclass
class RobotState:
    t: float
    q: np.ndarray
    dq: np.ndarray
    ee: np.ndarray
    ee_velocity: np.ndarray

@dataclass
class DistanceSample:
    pair: tuple
    distance: float
    points: np.ndarray
    gradient: np.ndarray
    valid: bool
    geom_ids: tuple = (0,0)

@dataclass
class AvoidanceResult:
    dq_cmd: np.ndarray
    status: str = 'ok'
    diagnostics: dict = field(default_factory=dict)

@dataclass
class TraceSnapshot:
    t: float
    nodes: list
    edges: list
    best_cost: float | None = None
    label: str = '实际搜索图快照'

@dataclass
class RunSummary:
    run_id: str
    planning_success: bool
    execution_success: bool
    execution_status: str
    metrics: dict
    interventions: list
