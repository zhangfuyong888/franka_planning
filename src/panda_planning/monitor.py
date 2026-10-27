"""统一监测真实状态和命令；停止实验的贡献与算法分开记录。"""
from collections import deque
import numpy as np


class Monitor:
    def __init__(self, context):
        self.context = context
        self.history = deque()
        self.events = []
        self.saturation = 0

    def check(self, state, qcmd, min_distance, progress, saturated=False,
              solver_status='ok', *, torque=None, command=None, command_t=None,
              numerical_warning=False):
        c, r = self.context.config, self.context.robot
        status = None
        values = np.r_[state.q, state.dq, state.ee, state.ee_velocity, qcmd,
                       state.t, progress, min_distance]
        if not np.all(np.isfinite(values)) or numerical_warning:
            status = 'numerical_error'
        elif torque is not None and (not np.all(np.isfinite(torque)) or
                                    np.any(np.abs(torque) > r.torque + 1e-6)):
            status = 'torque_limit'
        elif command is not None and (np.shape(command) != (7,) or
                                      not np.all(np.isfinite(command))):
            status = 'invalid_command'
        elif command_t is not None and (not np.isfinite(command_t) or
                                        state.t - command_t > c.command_timeout or
                                        command_t > state.t + 1e-9):
            status = 'stale_command'
        elif min_distance < 0:
            status = 'collision'
        elif min_distance < c.monitor_distance:
            status = 'guarded_stop'
        elif solver_status != 'ok':
            status = 'solver_failure'
        elif np.any(state.q < r.limits[:, 0] - .002) or np.any(state.q > r.limits[:, 1] + .002):
            status = 'joint_limit'
        elif np.max(np.abs(state.dq)) > c.velocity * 1.5:
            status = 'velocity_limit'
        elif np.linalg.norm(qcmd - state.q) > c.tracking_limit:
            status = 'tracking_error'
        self.saturation = self.saturation + 1 if saturated else 0
        if self.saturation * c.dt > .5:
            status = status or 'persistent_saturation'
        if self.history and state.t <= self.history[-1][0]:
            status = status or 'simulation_reset'
        self.history.append((state.t, state.q.copy(), progress))
        while self.history and state.t - self.history[0][0] > c.stall_window + c.dt:
            self.history.popleft()
        if self.history and state.t - self.history[0][0] >= c.stall_window:
            _, oldq, oldp = self.history[0]
            if np.linalg.norm(state.q - oldq) < .015 and progress - oldp < .02:
                status = status or 'stalled'
        if status:
            self.events.append({'t': state.t, 'status': status,
                                'distance': min_distance, 'solver_status': solver_status})
        return status
