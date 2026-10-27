"""MuJoCo passive viewer；装饰和实测回放标识与离屏渲染共用。"""
import time
import numpy as np
import mujoco as mj
import mujoco.viewer
from .trajectory import projected_path
from .overlays import camera, draw, graph_projection


class LiveViewer:
    def __init__(self, robot, data, scene, plan, planner, avoider):
        self.robot, self.data, self.scene, self.plan = robot, data, scene, plan
        self.label = f'{planner}/{avoider}'
        self.playback = '回放' in planner
        self.paused, self.single_step = False, False
        self.layers = {'path': True, 'graph': True, 'distance': True}
        self.last_sync = 0.

        def key(code):
            ch = chr(code).lower() if code < 256 else ''
            if code == 32:
                self.paused = not self.paused
            if ch == 'n':
                self.single_step = True
            for name, letter in [('path','p'), ('graph','g'), ('distance','d')]:
                if ch == letter:
                    self.layers[name] = not self.layers[name]
            if ch == 'c':
                with self.viewer.lock():
                    self.viewer.opt.geomgroup[2] = 1 - self.viewer.opt.geomgroup[2]
                    self.viewer.opt.geomgroup[3] = 1 - self.viewer.opt.geomgroup[3]

        self.viewer = mujoco.viewer.launch_passive(robot.model, data, key_callback=key)
        cam = camera()
        with self.viewer.lock():
            self.viewer.cam.azimuth, self.viewer.cam.elevation = cam.azimuth, cam.elevation
            self.viewer.cam.distance, self.viewer.cam.lookat[:] = cam.distance, cam.lookat
            self.viewer.opt.geomgroup[3] = 0
        self.paths = {'raw_path': projected_path(robot, plan.raw_path),
                      'processed_path': projected_path(robot, plan.processed_path)}
        self.graph = graph_projection(robot, plan)
        print(f'{self.label}；空格暂停 N单步 P路径 G搜索图 D距离 C碰撞模型；规划 {plan.elapsed:.3f}s')

    def running(self):
        return self.viewer.is_running()

    def sync(self, state, samples, rows):
        if time.perf_counter() - self.last_sync < 1/30:
            return
        row = rows[-1] if rows else None
        with self.viewer.lock():
            self.viewer.user_scn.ngeom = 0
            draw(self.viewer.user_scn, self.robot, self.scene, self.paths, row, rows, samples,
                 graph=self.graph, show_paths=self.layers['path'], show_graph=self.layers['graph'],
                 show_distance=self.layers['distance'])
        nearest = min(samples, key=lambda s:s.distance) if samples else None
        pair = ' / '.join(nearest.pair) if nearest else 'outside activation range'
        mode = 'MEASURED STATE PLAYBACK (not a new execution)' if self.playback else 'PHYSICAL EXECUTION'
        status = row.get('solver_status', 'ok') if row else 'ok'
        error = np.linalg.norm(state.ee - self.scene.target)
        texts = (f'{mode} | {self.label}\nt={state.t:.2f}s error={error:.4f}m solver={status}\n'
                 f'{pair} d={nearest.distance if nearest else .1:.4f}m\n'
                 f'planning wall time={self.plan.elapsed:.3f}s\n'
                 'Space pause | N step | P path | G graph | D distance | C collision')
        self.viewer.set_texts((mj.mjtFontScale.mjFONTSCALE_150, mj.mjtGridPos.mjGRID_TOPLEFT, texts, None))
        self.viewer.sync()
        self.last_sync = time.perf_counter()

    def close(self):
        self.viewer.close()
