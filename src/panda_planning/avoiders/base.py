from abc import ABC,abstractmethod
class LocalAvoider(ABC):
    def reset(self,context):pass
    @abstractmethod
    def step(self,state,dq_nominal,context):...
