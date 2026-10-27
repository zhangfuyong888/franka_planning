"""只添加可视化几何；passive viewer 与离屏实测回放共用同一图例。"""
import mujoco as mj
import numpy as np
from .trajectory import projected_path


def camera():
    cam = mj.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 130, -23, 2.1
    cam.lookat[:] = [.2, 0, .55]
    return cam


class Painter:
    def __init__(self, scene):
        self.scene = scene

    def point(self, position, color, size=.014):
        scene = self.scene
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, np.full(3, size),
                       np.asarray(position), np.eye(3).ravel(), np.array(color, dtype=np.float32))

    def line(self, a, b, color, width=.003):
        if np.linalg.norm(np.asarray(b) - a) < 1e-8:
            return
        scene = self.scene
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                       np.eye(3).ravel(), np.array(color, dtype=np.float32))
        mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a), np.asarray(b))

    def path(self, points, color, width=.003, dashed=False):
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            if not dashed or i % 2 == 0:
                self.line(a, b, color, width)


def draw(scene, robot, spec, paths, row=None, history=(), samples=(), *, graph=(),
         show_paths=True, show_graph=True, show_distance=True):
    """装饰永不进入 MjModel。paths 是该次真实路径的 FK 投影。"""
    p = Painter(scene)
    p.point(spec.target, [.55, .25, .8, 1])
    p.point(robot.fk(spec.q_start), [.12, .5, .8, 1])
    if show_paths:
        p.path(paths.get('raw_path', []), [.45, .55, .7, .55], .002, dashed=True)
        p.path(paths.get('processed_path', []), [0, .68, .75, 1], .003)
        p.path([entry['ee'] for entry in history][-500:], [.7, .2, .65, 1], .004)
    if show_graph:
        for edge in graph:
            p.path(edge, [.45, .6, .7, .2], .001)
    if show_distance:
        for sample in sorted(samples, key=lambda s: s.distance)[:3]:
            p.line(*sample.points, [.95, .45, .05, 1], .004)
            p.point(sample.points[0], [1, .55, .08, 1], .01)
    if row:
        active = row.get('diagnostics', {}).get('active', [])
        for sample in active:
            if sample['pair'][1].startswith('obstacle'):
                p.line(*sample['points'], [1, .4, .08, 1], .004)
        for name, color in [('nominal', [.3, .4, .8, 1]), ('command', [.2, .7, .4, 1]),
                            ('correction', [.9, .4, .15, 1])]:
            v = np.array(row.get('velocity_arrows', {}).get(name, [0, 0, 0]))
            v = v / max(1, np.linalg.norm(v) / .4)
            p.line(row['ee'], np.array(row['ee']) + .4 * v, color, .005)


def highlight(scene, robot, row):
    """仅修改 mjvScene 中的显示颜色，不改变物理材质或碰撞模型。"""
    if not row:
        return
    active = row.get('diagnostics', {}).get('active', [])
    body_ids = set()
    obstacle_ids = set()
    for sample in active:
        if not sample['pair'][1].startswith('obstacle'):
            continue
        a, b = sample['geom_ids']
        body_ids.add(int(robot.model.geom_bodyid[a]))
        obstacle_ids.add(b)
    for geom in scene.geoms[:scene.ngeom]:
        if geom.objtype != mj.mjtObj.mjOBJ_GEOM or geom.objid < 0:
            continue
        if geom.objid in obstacle_ids or int(robot.model.geom_bodyid[geom.objid]) in body_ids:
            geom.rgba[:3] = [.98, .57, .20]


def graph_projection(robot, plan, limit=120):
    if not plan.trace:
        return []
    snap = plan.trace[-1]
    return [projected_path(robot, [snap['nodes'][i], snap['nodes'][j]], .15)
            for i, j in snap['edges'][:limit]]
