"""Project recorded point clouds and checked TCP trajectories for the demo display."""
from functools import lru_cache
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from motion.kinematics import W, joint_frames, m

BASIS = np.array([[.85, -.53, 0], [-.24, -.39, -.89], [.47, .76, -.46]])
PATH_COLOR = '#72a9ff'
CONTACT_COLOR = '#ffe18a'
MESHES = Path(__file__).resolve().parents[2]/'src/TUMST_ARM_description/meshes'
STL_FACET = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
LINKS = ['electronics_base_link', m['joints'][0]['parent'], *(j['child'] for j in m['joints'])]


@lru_cache(maxsize=1)
def visual_meshes():
    visuals = []
    for visual in m['visuals']:
        with (MESHES/visual['file']).open('rb') as stream:
            stream.seek(80)
            count = int.from_bytes(stream.read(4), 'little')
            facets = np.fromfile(stream, dtype=STL_FACET, count=count)
        # Merge nearby vertices to keep the original silhouette practical to render in Tk.
        points = facets['vertices'].reshape(-1, 3)
        _, first, ids = np.unique(np.round(points/.0025).astype(int), axis=0,
                                  return_index=True, return_inverse=True)
        faces = ids.reshape(-1, 3)
        ordered = np.sort(faces, axis=1)
        valid = np.all(np.diff(ordered, axis=1) != 0, axis=1)
        faces = faces[valid]
        _, distinct = np.unique(ordered[valid], axis=0, return_index=True)
        triangles = points[first][faces[distinct]]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        good = lengths > 1e-12
        visuals.append((LINKS.index(visual['link']), triangles[good],
                        normals[good]/lengths[good, None], np.array(visual['color'][:3])*255))
    return visuals


@lru_cache(maxsize=1)
def fixed_mesh_bounds():
    vertices = np.concatenate([mesh.reshape(-1, 3) @ (BASIS @ W[:3, :3]).T + BASIS @ W[:3, 3]
                               for link, mesh, _, _ in visual_meshes() if link == 0])
    return np.array([vertices.min(axis=0), vertices.max(axis=0)])


def draw_robot(draw, frames, screen):
    triangles, colors = [], []
    for link, mesh, normals, color in visual_meshes():
        transform = frames[link]
        rotation = BASIS @ transform[:3, :3]
        projected = mesh @ rotation.T + BASIS @ transform[:3, 3]
        light = np.clip((normals @ rotation.T) @ np.array([-.25, -.45, -.85]), 0, 1)
        shade = (.56 + .44*light[:, None])*color + 12
        triangles.append(projected)
        colors.append(np.clip(shade, 0, 255).astype(np.uint8))
    triangles = np.concatenate(triangles)
    colors = np.concatenate(colors)
    depth_order = np.argsort(triangles[:, :, 2].mean(axis=1))[::-1]
    xy = screen(triangles).round().astype(int).reshape(-1, 6)
    for i in depth_order:
        draw.polygon(tuple(xy[i]), fill=tuple(colors[i]))



def report(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


@lru_cache(maxsize=24)
def cloud(path, stamp):
    with np.load(path) as data:
        points = data['points']
        points = points[np.isfinite(points).all(axis=1)]
        return points[::max(1, (len(points)+1999)//2000)]


def load_cloud(path):
    return cloud(path, path.stat().st_mtime_ns) if path.is_file() else np.empty((0, 3))


def candidate(point, normal):
    if point is None or normal is None:
        return None
    point, normal = np.asarray(point), np.asarray(normal)
    return np.array([point + .035*normal, point])


@lru_cache(maxsize=12)
def checked_paths(path, stamp):
    paths = []
    for entry in report(path).values():
        joint_path = entry['trajectory']['joint_trajectory']
        names = joint_path['joint_names']
        order = [names.index(f'joint{i}') for i in range(1, 5)]
        points = joint_path['points']
        stride = max(1, len(points)//120)
        sampled = points[::stride]
        if points and sampled[-1] is not points[-1]:
            sampled.append(points[-1])
        if sampled:
            poses = []
            for point in sampled:
                frames = joint_frames([point['positions'][i] for i in order])
                poses.append([W, *frames])
            paths.append(np.array(poses))
    return paths


def geometry(scan, colors, show_paths):
    plan_files = sorted(scan.glob('batch_*/batch_plan.json'))
    plan = report(plan_files[-1]) if show_paths and plan_files else {}
    stages = plan.get('stages', []) if plan.get('all_stages_ready') else []
    if stages:
        stage = next((s for s in stages if s.get('execution_status') != 'complete'), stages[-1])
        session = Path(stage['prepared_session'])
        data = report(session/'candidate.json').get('selected_candidate', {})
        normal = data.get('commanded_tool_axis_table')
        contact = candidate(data.get('nominal_contact_position_table_m'),
                            -np.array(normal) if normal is not None else data.get('normal_table'))
        path = session/'reviewed_trajectories.json'
        paths = checked_paths(path, path.stat().st_mtime_ns) if path.is_file() else []
        color = colors[(int(stage['object'].split('_')[-1])-1) % len(colors)]
        return [(load_cloud(session/'analysis/cloud.npz'), color)], [contact], paths, stage['object']
    clouds = [(load_cloud(obj/'analysis/cloud.npz'), colors[i % len(colors)])
              for i, obj in enumerate(sorted((scan/'objects').glob('object_*')))]
    contacts = []
    for obj in report(scan/'inclined_pick_plan.json').get('objects', []):
        matches = obj.get('matched_faces', [])
        if matches:
            face = matches[0]['face']
            contacts.append(candidate(face.get('point_table_m'), face.get('normal_table')))
    return clouds, contacts, [], None


def preview_pose(paths, elapsed):
    """Loop through saved samples without inventing moves between segments."""
    if not paths:
        return None
    segment, fraction = divmod(elapsed / 2.8, 1)
    poses = paths[int(segment) % len(paths)]
    return poses[min(int(fraction * len(poses)), len(poses)-1)]


def render(width, height, clouds, contacts, paths, robot=None):
    clouds = [(xyz @ BASIS.T, color) for xyz, color in clouds if len(xyz)]
    contacts = [xyz @ BASIS.T for xyz in contacts if xyz is not None]
    arms = [segment[:, :, :3, 3] @ BASIS.T for segment in paths]
    tips = [(segment[:, -1, :3, :3] @ m['tcp_xyz'] + segment[:, -1, :3, 3]) @ BASIS.T
            for segment in paths]
    arrays = [xyz for xyz, _ in clouds] + contacts + tips + [arm.reshape(-1, 3) for arm in arms]
    if arms:
        arrays.append(fixed_mesh_bounds())
    if not arrays:
        return None
    joined = np.concatenate(arrays)[:, :2]
    margin = .055 if arms else 0
    lo, hi = joined.min(axis=0)-margin, joined.max(axis=0)+margin
    scale = min(max(1, width-48)/max(hi[0]-lo[0], .01),
                max(1, height-52)/max(hi[1]-lo[1], .01))
    center = (lo+hi)/2

    def screen(xyz):
        return (xyz[..., :2]-center)*scale + [width/2, height/2]

    image = Image.new('RGB', (width, height), '#131f30')
    draw = ImageDraw.Draw(image)
    for xyz, color in clouds:
        draw.point([tuple(p) for p in screen(xyz)], fill=color)
    if robot is not None:
        draw_robot(draw, robot, screen)
    for xyz in tips:
        coords = [tuple(p) for p in screen(xyz)]
        draw.line(coords, fill=PATH_COLOR, width=3)
    for xyz in contacts:
        start, tip = screen(xyz)
        x, y = tip
        draw.line([tuple(start), tuple(tip)], fill=CONTACT_COLOR, width=3)
        draw.ellipse((x-4, y-4, x+4, y+4), fill=CONTACT_COLOR, outline='#131f30')
    return image
