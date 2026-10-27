from abc import ABC,abstractmethod
class GlobalPlanner(ABC):
    def reset(self,context):pass
    @abstractmethod
    def plan(self,request,context,trace_sink=None):...
