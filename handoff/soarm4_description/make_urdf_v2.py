"""SO-ARM4 URDF v2 generator - real meshes (official SO-101 CAD + our vacuum tool + electronics box).

Kinematics come from the official SO-101 URDF (so101_new_calib.urdf, TheRobotStudio/SO-ARM100).
Our joint convention is kept from v1 so soarm_bridge / servo_map.yaml do not change:
  zero pose  = arm stretched forward and horizontal (upper-arm part, fore-arm part and tool all level)
  joint1 +   = CCW seen from above,  joint2/3/4 + = lifts the link up
Every link frame sits on its joint axis, on the arm mid-plane, and is parallel to base_link at zero pose.

base_link = table surface directly under the J1 axis. x = arm forward (= box front wall), y = left, z = up.
"""
import os, sys, math, shutil, json
import numpy as np, trimesh
from trimesh.transformations import euler_matrix, euler_from_matrix, rotation_matrix
sys.path.insert(0, '/home/claude/urdf_work')
from fk import URDF

SO = '/tmp/claude-0/-home-claude/8b264096-9bb9-57df-a27f-a86b5b55ca9d/scratchpad/soarm_repo/Simulation/SO101'
UP = '/root/.claude/uploads/8b264096-9bb9-57df-a27f-a86b5b55ca9d'
OLD = '/home/claude/soarm/soarm4_description'          # v1 package (cup / adapter meshes, configs)
OUT = '/home/claude/handoff/soarm4_description'
TOOL = '/home/claude/soarm/J4_Vacuum_Tool_v5_long25.stl'

# ---------------- parameters (mm unless noted) ----------------
BOX_H = 95.0                 # table -> top-plate top surface (bottom plate -95 .. top 0 in box CAD)
J1_ON_PLATE = (17.7, 27.5)   # J1 axis in box-CAD coords (from photo): base along CAD x, footprint centred on the 4 holes (x +-35, y 10/45)
BOX_YAW = 0.0               # arm forward = box CAD +x (open side), Pi wall (CAD -y) on the robot right
CUP_LIP_X = 104.0            # tool x of cup lip  (block face 74 + adapter neck 8 + cup 22)
CUP_AXIS_Z = -17.8           # cup axis in tool coords (tool z = J4 horn axis, horn face at z=0)
SMAP = {'joint1': (1, 2047, -1, 860, 3472), 'joint2': (2, 4095, -1, 1879, 4095),
        'joint3': (3, 0, -1, 0, 2047), 'joint4': (4, 2047, -1, 660, 3273)}
EFF = {'joint1': 2.9, 'joint2': 2.9, 'joint3': 2.9, 'joint4': 1.9}     # N*m stall (STS3250 / STS3215 @12V)
VEL = {'joint1': 2.36, 'joint2': 2.36, 'joint3': 2.36, 'joint4': 3.30}
# official joint -> ours:  q_official = OFF + SIGN_O * q_ours
MAP = {'shoulder_pan': ('joint1', 0.0, -1), 'shoulder_lift': ('joint2', math.pi / 2, -1),
       'elbow_flex': ('joint3', -math.pi / 2, -1), 'wrist_flex': ('joint4', 0.0, -1)}

C2R = 2 * math.pi / 4096
def c2r(j, c): _, ctr, sg, _, _ = SMAP[j]; return sg * (c - ctr) * C2R
def to_official(q):  # q: dict joint1..4 -> dict official names
    return {o: off + s * q.get(j, 0.0) for o, (j, off, s) in MAP.items()}

def Tm(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = euler_matrix(*rpy, 'sxyz'); T[:3, 3] = xyz; return T

# ---------------- official model placed on the box ----------------
so = URDF(f'{SO}/so101_new_calib.urdf')
J1_LOCAL = np.array([0.0388353, 0, 0])                       # J1 axis in official base frame (x,y)
ARM_BASE = Tm((-J1_LOCAL[0], 0, BOX_H / 1000 + 0.0024))      # official base_link in our base_link (base bottom at -2.4 mm)
BOX_T = Tm((0, 0, BOX_H / 1000), (0, 0, BOX_YAW)) @ Tm((-J1_ON_PLATE[0] / 1000, -J1_ON_PLATE[1] / 1000, 0))

# tool in official wrist_link frame: tool x -> -y (out), tool z (horn axis) -> -z (into motor), horn face at z = 0
TOOL_IN_WRIST = np.array([[0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], float)
S_MM = np.diag([1e-3, 1e-3, 1e-3, 1])

def official_world(qo):
    P = so.fk(qo); return {k: ARM_BASE @ v for k, v in P.items()}

ZERO = official_world(to_official({}))
# link frames at zero pose: on the official joint axis, projected to the mid-plane y = 0, parallel to base_link
def axis_pt(child):
    p = ZERO[child][:3, 3].copy(); p[1] = 0.0; return p
G = {'base_link': np.eye(4)}
for name, child in [('link1', 'shoulder_link'), ('link2', 'upper_arm_link'), ('link3', 'lower_arm_link'), ('tool_link', 'wrist_link')]:
    G[name] = Tm(axis_pt(child))
OF2OURS = {'shoulder_link': 'link1', 'upper_arm_link': 'link2', 'lower_arm_link': 'link3', 'wrist_link': 'tool_link'}
# sanity: joint axes in world at zero
for c, ax in [('shoulder_link', [0, 0, -1]), ('upper_arm_link', [0, 1, 0]), ('lower_arm_link', [0, 1, 0]), ('wrist_link', [0, 1, 0])]:
    assert np.allclose(ZERO[c][:3, 2], ax, atol=1e-4), (c, ZERO[c][:3, 2])

# ---------------- collect meshes per link, in link frame ----------------
KEEP = {'base_link': ['base_so101_v2', 'base_motor_holder_so101_v1', 'sts3215_03a_v1'],
        'shoulder_link': ['sts3215_03a_v1', 'motor_holder_so101_base_v1', 'rotation_pitch_so101_v1'],
        'upper_arm_link': ['sts3215_03a_v1', 'upper_arm_so101_v1'],
        'lower_arm_link': ['under_arm_so101_v1', 'motor_holder_so101_wrist_v1', 'sts3215_03a_v1']}
parts = {}   # (link, material) -> [meshes in link frame]
def add(link, mat, m): parts.setdefault((link, mat), []).append(m)
for olink, vis in so.links.items():
    if olink not in KEEP: continue
    ours = 'base_link' if olink == 'base_link' else OF2OURS[olink]
    for (To, fn, sc, _) in vis:
        stem = os.path.basename(fn)[:-4]
        if stem not in KEEP[olink]: continue
        m = trimesh.load(f'{SO}/{fn}', force='mesh')
        m.apply_transform(np.linalg.inv(G[ours]) @ ZERO[olink] @ To)
        add(ours, 'servo' if stem.startswith('sts3215') else 'print', m)
# vacuum tool + adapter + cup (tool CAD in mm)
Wt = np.linalg.inv(G['tool_link']) @ ZERO['wrist_link'] @ TOOL_IN_WRIST
tool = trimesh.load(TOOL); tool.apply_transform(Wt @ S_MM); add('tool_link', 'print', tool)
def tool_pt(x, y, z): return (Wt @ np.array([x / 1000, y / 1000, z / 1000, 1]))[:3]
for fn, x0, mat in [('ig1_adapter.stl', 60.5, 'alu'), ('fga25_cup.stl', 82.0, 'cup')]:
    m = trimesh.load(f'{OLD}/meshes/{fn}')                                  # metres, axis = +z from z=0
    m.apply_transform(Wt @ Tm((x0 / 1000, 0, CUP_AXIS_Z / 1000), (0, math.pi / 2, 0)))
    add('tool_link', mat, m)
CUP_TIP = tool_pt(CUP_LIP_X, 0, CUP_AXIS_Z)
# electronics box (CAD in mm, 4 identical posts at +-87, +-72)
box = {'top': trimesh.load(f'{UP}/adc9d8ab-01a_top_plate.stl'),
       'wall_front': trimesh.load(f'{UP}/648a2c2b-01b_wall_front.stl'),
       'wall_back': trimesh.load(f'{UP}/7e37de35-01b_wall_back.stl'),
       'bottom': trimesh.load(f'{UP}/17c8e06e-01d_bottom_plate.stl')}
post = trimesh.load(f'{UP}/4d3a7cbf-01c_post.stl')
posts = []
for sx in (1, -1):
    for sy in (1, -1):
        p = post.copy(); p.apply_translation((-87, -72, 0)); p.apply_translation((87 * sx, 72 * sy, 0)); posts.append(p)
box['posts'] = trimesh.util.concatenate(posts)
for k, m in box.items():
    mm = m.copy(); mm.apply_transform(BOX_T @ S_MM); add('base_link', 'box_plate' if k in ('top', 'bottom') else 'box_wall', mm)

# ---------------- write package ----------------
_keep = open(f'{OUT}/README.md').read() if os.path.exists(f'{OUT}/README.md') else None
if os.path.exists(OUT): shutil.rmtree(OUT)
for d in ('urdf', 'meshes/visual', 'meshes/collision', 'launch', 'rviz', 'config'):
    os.makedirs(f'{OUT}/{d}', exist_ok=True)
MATS = {'print': '0.92 0.92 0.88 1', 'servo': '0.10 0.10 0.10 1', 'alu': '0.75 0.77 0.80 1',
        'cup': '0.10 0.55 0.95 1', 'box_plate': '0.30 0.42 0.62 1', 'box_wall': '0.62 0.46 0.36 1'}
DENS = {'print': 550.0, 'alu': 2700.0, 'cup': 1100.0, 'box_plate': 550.0, 'box_wall': 550.0}
SERVO_KG = {'base_link': 0.060, 'link1': 0.060, 'link2': 0.060, 'link3': 0.055}
links_xml, inert = {}, {}
for link in ['base_link', 'link1', 'link2', 'link3', 'tool_link']:
    vis, coll, mass, com_acc = [], [], 0.0, np.zeros(3)
    allm = []
    for (l, mat), ms in parts.items():
        if l != link: continue
        m = trimesh.util.concatenate(ms); allm.append(m)
        fn = f'{link}_{mat}.stl'; m.export(f'{OUT}/meshes/visual/{fn}')
        vis.append(f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="package://soarm4_description/meshes/visual/{fn}"/></geometry><material name="{mat}"/></visual>')
        h = m.convex_hull
        kg = SERVO_KG[link] if mat == 'servo' else h.volume * DENS[mat] * (0.5 if mat == 'print' else 1.0)
        if link == 'base_link' and mat.startswith('box'): kg = h.volume * 0.0 + {'box_plate': 0.35, 'box_wall': 0.20}[mat]
        mass += kg; com_acc += kg * h.center_mass
    # collision: arm links -> one convex hull; base -> SO-101 base hull + electronics box as a box
    if link == 'base_link':
        arm_base = trimesh.util.concatenate([m for (l, mat), ms in parts.items() if l == link and mat in ('print', 'servo') for m in ms])
        arm_base.convex_hull.export(f'{OUT}/meshes/collision/base_link.stl')
        bb = trimesh.util.concatenate([m for (l, mat), ms in parts.items() if l == link and mat.startswith('box') for m in ms]).bounds
        c, s = bb.mean(0), bb[1] - bb[0]
        coll.append(f'    <collision><origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" rpy="0 0 0"/><geometry><box size="{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}"/></geometry></collision>')
    else:
        trimesh.util.concatenate(allm).convex_hull.export(f'{OUT}/meshes/collision/{link}.stl')
    coll.append(f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="package://soarm4_description/meshes/collision/{link}.stl"/></geometry></collision>')
    com = com_acc / mass
    hull = trimesh.util.concatenate(allm).convex_hull; I = hull.moment_inertia * (mass / hull.volume) if hull.volume > 0 else np.eye(3) * 1e-5
    # shift inertia from hull centroid to the mass-weighted com is small; use hull tensor about its centroid
    I = (I + I.T) / 2
    inert[link] = (mass, com)
    links_xml[link] = '\n'.join(vis + coll) + (
        f'\n    <inertial><origin xyz="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" rpy="0 0 0"/><mass value="{mass:.4f}"/>'
        f'<inertia ixx="{I[0,0]:.3e}" ixy="{I[0,1]:.3e}" ixz="{I[0,2]:.3e}" iyy="{I[1,1]:.3e}" iyz="{I[1,2]:.3e}" izz="{I[2,2]:.3e}"/></inertial>')

LIM = {j: tuple(sorted((c2r(j, v[3]), c2r(j, v[4])))) for j, v in SMAP.items()}
def f3(v): return ' '.join(f'{x:.5f}' for x in v)
chain = [('joint1', 'base_link', 'link1', '0 0 1'), ('joint2', 'link1', 'link2', '0 -1 0'),
         ('joint3', 'link2', 'link3', '0 -1 0'), ('joint4', 'link3', 'tool_link', '0 -1 0')]
J = {}
for j, p, c, ax in chain:
    o = (np.linalg.inv(G[p]) @ G[c])[:3, 3]; J[j] = o
mat_xml = ''.join(f'<material name="{k}"><color rgba="{v}"/></material>' for k, v in MATS.items())
cup_in_tool = CUP_TIP
j1h = G['link1'][2, 3]
urdf = f'''<?xml version="1.0"?>
<!-- SO-ARM4 (SO-ARM101 based 4-DOF + vacuum tool v5) on the electronics box.  Generated by make_urdf_v2.py - edit that, not this.
     Geometry = official SO-101 CAD (TheRobotStudio so101_new_calib.urdf) + our parts.
     base_link : table surface right under the J1 axis.  x = forward (box front wall), y = left, z = up.
     zero pose : arm stretched forward and level.  joint1 + = CCW from above, joint2/3/4 + = link goes up.
     servo count <-> angle : config/servo_map.yaml  (same as v1, soarm_bridge unchanged)
     key numbers (mm, base_link) : box top z {BOX_H:.1f} | J1 output z {j1h*1000:.1f} | J2 axis x {G['link2'][0,3]*1000:.1f} z {G['link2'][2,3]*1000:.1f}
                                   J2->J3 ({J['joint3'][0]*1000:.1f}, {J['joint3'][2]*1000:.1f}) | J3->J4 ({J['joint4'][0]*1000:.1f}, {J['joint4'][2]*1000:.1f}) | J4->cup lip ({cup_in_tool[0]*1000:.1f}, {cup_in_tool[1]*1000:.1f}, {cup_in_tool[2]*1000:.1f}) -->
<robot name="soarm4">
  {mat_xml}
  <link name="world"/>
  <joint name="world_fixed" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
'''
for link in ['base_link', 'link1', 'link2', 'link3', 'tool_link']:
    urdf += f'\n  <link name="{link}">\n{links_xml[link]}\n  </link>\n'
    for j, p, c, ax in chain:
        if p == link:
            lo, hi = LIM[j]
            urdf += (f'  <joint name="{j}" type="revolute"><parent link="{p}"/><child link="{c}"/>\n'
                     f'    <origin xyz="{f3(J[j])}" rpy="0 0 0"/><axis xyz="{ax}"/>\n'
                     f'    <limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{EFF[j]}" velocity="{VEL[j]}"/>\n  </joint>\n')
urdf += f'''
  <!-- suction point: centre of the cup lip, x = approach direction -->
  <joint name="cup_tip_fixed" type="fixed"><parent link="tool_link"/><child link="cup_tip"/><origin xyz="{f3(cup_in_tool)}" rpy="0 0 0"/></joint>
  <link name="cup_tip"/>
  <!-- helper frames -->
  <joint name="box_top_fixed" type="fixed"><parent link="base_link"/><child link="box_top"/><origin xyz="0 0 {BOX_H/1000:.4f}" rpy="0 0 0"/></joint>
  <link name="box_top"/>
  <joint name="box_cad_fixed" type="fixed"><parent link="base_link"/><child link="box_cad"/><origin xyz="{f3(BOX_T[:3,3])}" rpy="0 0 {BOX_YAW:.6f}"/></joint>
  <link name="box_cad"/>
</robot>
'''
open(f'{OUT}/urdf/soarm4.urdf', 'w').write(urdf)

# configs / launch / rviz
for f in ('config/servo_map.yaml', 'config/joint_limits.yaml'):
    shutil.copy(f'{OLD}/{f}', f'{OUT}/{f}')
shutil.copy(__file__, f'{OUT}/make_urdf_v2.py')
open(f'{OUT}/package.xml', 'w').write('''<?xml version="1.0"?>
<package format="3">
  <name>soarm4_description</name>
  <version>2.0.0</version>
  <description>SO-ARM4 (SO-101 based 4-DOF vacuum arm) URDF with real meshes and electronics box</description>
  <maintainer email="swj4875@gmail.com">wanjae</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
''')
open(f'{OUT}/CMakeLists.txt', 'w').write('''cmake_minimum_required(VERSION 3.8)
project(soarm4_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz config DESTINATION share/${PROJECT_NAME})
ament_package()
''')
open(f'{OUT}/launch/display.launch.py', 'w').write('''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('soarm4_description')
    desc = open(os.path.join(share, 'urdf', 'soarm4.urdf')).read()
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        # gui:=true  -> sliders (no robot needed)
        # gui:=false -> follow /joint_states from soarm_bridge (real robot); run bringup with use_rsp:=false
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'view.rviz')]),
    ])
''')
open(f'{OUT}/rviz/view.rviz', 'w').write('''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Cell Size: 0.05
      Plane Cell Count: 20
      Reference Frame: base_link
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Value: /robot_description
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Marker Scale: 0.06
      Show Names: true
      Frames:
        All Enabled: false
        base_link: {Value: true}
        link1: {Value: true}
        link2: {Value: true}
        link3: {Value: true}
        tool_link: {Value: true}
        cup_tip: {Value: true}
      Value: true
  Global Options:
    Fixed Frame: base_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.9
      Focal Point: {X: 0.1, Y: 0, Z: 0.15}
      Pitch: 0.45
      Yaw: 3.9
''')
if _keep: open(f'{OUT}/README.md', 'w').write(_keep)
json.dump({'G': {k: v.tolist() for k, v in G.items()}, 'J': {k: v.tolist() for k, v in J.items()},
           'cup_tip': cup_in_tool.tolist(), 'mass': {k: [v[0], v[1].tolist()] for k, v in inert.items()}},
          open('/home/claude/urdf_work/v2_numbers.json', 'w'), indent=1)
print('J', {k: (v * 1000).round(1).tolist() for k, v in J.items()}, 'cup', (cup_in_tool * 1000).round(1))
print('mass', {k: round(v[0], 3) for k, v in inert.items()})
print('limits', {k: (round(math.degrees(a), 1), round(math.degrees(b), 1)) for k, (a, b) in LIM.items()})
