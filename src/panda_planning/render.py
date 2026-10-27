"""离屏渲染真实记录状态，不启动 viewer，不重新计算或冒充动力学执行。"""
import argparse
import json
import os
import math
from pathlib import Path


def read(path):
    return json.loads(Path(path).read_text())


def frame(run_dir, at, width=960, height=720, collision_geoms=False):
    # 此模块的入口先选 GL 后端，再导入 MuJoCo；普通 headless 路径不会创建 Renderer。
    import mujoco as mj
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    from .robot import Robot
    from .scenes import load
    from .types import PlanResult, DistanceSample
    from .overlays import camera, draw, highlight
    root = Path(run_dir)
    summary, rows = read(root/'summary.json'), read(root/'telemetry.json')
    scene = load(root/'scene.json')
    raw = read(root/'plan.json')
    plan = PlanResult(**{k:v for k,v in raw.items() if k != 'plan_id'})
    display = read(root/'display.json')
    robot = Robot(scene)
    selected = [x for x in rows if x['t'] <= at + 1e-9]
    row = selected[-1] if selected else None
    data = robot.data(row['q'] if row else scene.q_start)
    if row:
        data.qvel[:] = row['dq']
        data.time = row['t']
        mj.mj_forward(robot.model, data)
    robot.model.vis.global_.offwidth = max(width, robot.model.vis.global_.offwidth)
    robot.model.vis.global_.offheight = max(height, robot.model.vis.global_.offheight)
    option = mj.MjvOption()
    option.geomgroup[2] = not collision_geoms
    option.geomgroup[3] = collision_geoms
    samples = []
    if row and row.get('nearest'):
        s = row['nearest']
        samples = [DistanceSample(tuple(s['pair']), s['distance'], np.asarray(s['points']),
                                  np.asarray(s['gradient']), s['valid'], tuple(s['geom_ids']))]
    header_height = 90
    with mj.Renderer(robot.model, height=height-header_height, width=width) as renderer:
        renderer.update_scene(data, camera=camera(), scene_option=option)
        highlight(renderer.scene, robot, row)
        draw(renderer.scene, robot, scene, display, row, selected, samples)
        content = Image.fromarray(renderer.render()).convert('RGB')
    picture = Image.new('RGB',(width,height),(23,48,68))
    picture.paste(content,(0,header_height))
    font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    font = ImageFont.truetype(str(font_path), max(14, width//55)) if font_path.exists() else ImageFont.load_default()
    painter = ImageDraw.Draw(picture)
    painter.rectangle((0, 0, width, header_height-1), fill=(23, 48, 68))
    label = f"{summary['planner']} / {summary['avoider']} | {summary['planning_status']} / {summary['execution_status']}"
    physical_time = row['t'] if row else 0.
    text = f"实测状态回放（非重新执行）  {label}\n请求时间 {at:.2f} s / 记录状态 {physical_time:.2f} s | run {summary['run_id']}\n青:处理后路径  紫:实际轨迹  橙:最近点/激活对  蓝/绿:名义/命令速度"
    painter.multiline_text((12, 9), text, font=font, fill='white', spacing=3)
    metadata = {'run_id': summary['run_id'], 'scene_id': summary['scene_id'], 'plan_id': plan.plan_id,
                'requested_time': at, 'recorded_time': physical_time,
                'gl_backend': os.environ.get('MUJOCO_GL'), 'recording': 'measured_state_playback',
                'geometry': 'collision' if collision_geoms else 'visual', 'width': width, 'height': height}
    return picture, metadata


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sources = p.add_mutually_exclusive_group(required=True)
    sources.add_argument('--run-dir')
    sources.add_argument('--compare', nargs='+', help='同一场景的多个运行目录')
    p.add_argument('--times', nargs='+', type=float, default=[0., 4., 9.])
    p.add_argument('--output', required=True)
    p.add_argument('--gl', choices=['egl', 'osmesa'], default='egl')
    p.add_argument('--width', type=int, default=960)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--collision-geoms', action='store_true')
    args = p.parse_args()
    if min(args.width, args.height) < 240 or any(not math.isfinite(t) or t < 0 for t in args.times):
        p.error('宽高至少240，时间必须非负')
    os.environ['MUJOCO_GL'] = args.gl
    from PIL import Image
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    directories = args.compare or [args.run_dir]
    ids = {read(Path(d)/'summary.json')['scene_id'] for d in directories}
    if len(ids) != 1:
        p.error('并排对照必须使用同一个 scene_id')
    entries = []
    for at in args.times:
        pictures = []
        for i, directory in enumerate(directories):
            picture, metadata = frame(directory, at, args.width, args.height, args.collision_geoms)
            name = f'run-{i}-t-{at:g}.png'
            picture.save(output/name)
            entries.append(dict(metadata, file=name))
            pictures.append(picture)
        if len(pictures) > 1:
            columns = 3 if len(pictures) == 3 else 2
            rows = (len(pictures)+columns-1)//columns
            sheet = Image.new('RGB', (columns*args.width, rows*args.height), '#eef2f7')
            for i, picture in enumerate(pictures):
                sheet.paste(picture, ((i%columns)*args.width, (i//columns)*args.height))
            sheet.save(output/f'comparison-t-{at:g}.png')
    (output/'frames.json').write_text(json.dumps(entries, ensure_ascii=False, indent=2))
    print(output/'frames.json')


if __name__ == '__main__':
    main()
