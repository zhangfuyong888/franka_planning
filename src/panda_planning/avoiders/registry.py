from .none import NoneAvoider
from .apf import APF
from .distance_qp import DistanceQP
REGISTRY={'none':NoneAvoider,'apf':APF,'distance_qp':DistanceQP}
def create(name):return REGISTRY[name]()
