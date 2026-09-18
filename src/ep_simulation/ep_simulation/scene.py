"""Upright bottle on the floor; all task dimensions are in meters."""
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .model import fk, ik, TOOL_OFFSET


# 读取瓶子参数并提前验证尺寸、夹爪开口和所有任务端点可达性。
def load_config(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('ep_simulation'))/'config/bottle.json'
    cfg = json.loads(Path(path).read_text())
    numeric = ['height_m', 'diameter_m', 'empty_mass_kg', 'water_ml', 'friction',
               'pick_x_m', 'place_x_m', 'grasp_height_m', 'lift_height_m',
               'release_clearance_m', 'open_half_gap_m', 'grip_compression_m']
    if not all(isinstance(cfg.get(k), (int, float)) and math.isfinite(cfg[k]) for k in numeric):
        raise ValueError('Bottle settings must contain finite numeric values')
    if not (0 < cfg['height_m'] < 0.4 and 0 < cfg['diameter_m'] < 0.1
            and 0 < cfg['empty_mass_kg'] and 0 <= cfg['water_ml'] <= 550
            and 0 < cfg['friction'] <= 2 and 0 < cfg['lift_height_m']
            and 0 < cfg['release_clearance_m'] <= 0.01
            and 0 < cfg['grip_compression_m'] < cfg['diameter_m']/2):
        raise ValueError('Invalid bottle dimensions, mass, friction or clearance')
    if not cfg['diameter_m']/2+0.003 < cfg['open_half_gap_m'] <= 0.05:
        raise ValueError('Bottle must fit inside the opened 100 mm gripper')
    if not 0.03 < cfg['grasp_height_m'] < cfg['height_m']-0.03:
        raise ValueError('Grasp must be on the bottle body above the ground')
    targets(cfg)  # Fail before starting Gazebo if any endpoint is unreachable.
    return cfg


# 把夹取、抬升、搬运、放置与撤离位置转换为关节角。
def targets(cfg):
    x, z, lift = cfg['pick_x_m'], cfg['grasp_height_m'], cfg['lift_height_m']
    px, pz = cfg['place_x_m'], z+cfg['release_clearance_m']
    return {'pick': ik(x, z), 'approach': ik(x, z+lift),
            'lift': ik(x, z+lift), 'transfer': ik(px, z+lift),
            'place': ik(px, pz), 'retract': ik(px, z+lift)}


# 瓶子原点位于几何中心，因此落地时中心高度为瓶高的一半。
def bottle_start(cfg):
    return [cfg['pick_x_m'], TOOL_OFFSET[1], cfg['height_m']/2]


# 目标点使用同一机械臂平面，固定底盘不提供横向搬运自由度。
def bottle_goal(cfg):
    return [cfg['place_x_m'], TOOL_OFFSET[1], cfg['height_m']/2]


# 按水密度约 1 g/ml，将装水量换算为 kg 并加上空瓶质量。
def bottle_mass(cfg):
    return cfg['empty_mass_kg']+cfg['water_ml']/1000


# 生成含重力、地面和刚体瓶子的 Gazebo 世界；标记仅用于显示。
def world_sdf(cfg):
    x, y, z = bottle_start(cfg)
    h, r, mass, mu = cfg['height_m'], cfg['diameter_m']/2, bottle_mass(cfg), cfg['friction']
    ixx, izz = mass*(3*r*r+h*h)/12, mass*r*r/2
    # Rigid cylinder collision/inertia proxy; colored sleeve, shoulder and cap
    # are visual approximations, not official CAD or a fluid simulation.
    return f'''<sdf version="1.6"><world name="ep_pick_world">
      <gravity>0 0 -9.81</gravity>
      <physics name="ode" type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate><ode><solver><iters>100</iters></solver></ode></physics>
      <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.82 0.86 0.91 1</background><shadows>true</shadows></scene>
      <light name="sun" type="directional"><pose>0 0 2 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.3 0.1 -1</direction></light>
      <gui><camera name="user_camera"><pose>0.9 -0.8 0.6 0 0.43 2.35</pose></camera></gui>
      <model name="ground"><static>true</static><link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry><material><ambient>0.65 0.69 0.73 1</ambient></material></visual>
      </link></model>
      <model name="place_marker"><static>true</static><pose>{cfg['place_x_m']} {y} 0.0002 0 0 0</pose><link name="link">
        <visual name="visual"><geometry><cylinder><radius>{r+0.008}</radius><length>0.0002</length></cylinder></geometry><material><ambient>0.1 0.65 0.3 0.6</ambient></material><cast_shadows>false</cast_shadows></visual>
      </link></model>
      <model name="target_bottle"><pose>{x} {y} {z+0.0001} 0 0 0</pose><link name="link">
        <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{izz}</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry><surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction><contact><ode><kp>100000</kp><kd>10</kd><min_depth>0.0005</min_depth><max_vel>0.05</max_vel></ode></contact></surface></collision>
        <visual name="body"><pose>0 0 {-h*0.08} 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>{h*0.84}</length></cylinder></geometry><material><ambient>0.65 0.85 0.92 1</ambient><diffuse>0.65 0.85 0.92 1</diffuse></material></visual>
        <visual name="shoulder"><pose>0 0 {h*0.34} 0 0 0</pose><geometry><sphere><radius>{r}</radius></sphere></geometry><material><ambient>0.65 0.85 0.92 1</ambient></material></visual>
        <visual name="cap"><pose>0 0 {h*0.47} 0 0 0</pose><geometry><cylinder><radius>{r*0.5}</radius><length>{h*0.06}</length></cylinder></geometry><material><ambient>0.85 0.05 0.03 1</ambient></material></visual>
        <visual name="label"><pose>0 0 {-h*0.07} 0 0 0</pose><geometry><cylinder><radius>{r+0.0002}</radius><length>{h*0.27}</length></cylinder></geometry><material><ambient>0.85 0.06 0.04 1</ambient></material></visual>
      </link></model>
      <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>30</update_rate></plugin>
    </world></sdf>'''


# 从适配后的 URDF 构建 MoveIt 分组、预设姿态和碰撞忽略对。
def semantic(urdf):
    r = ET.fromstring(urdf)
    out = ET.Element('robot', name='ep_fixed_arm')
    arm_joints = [j.get('name') for j in r.findall('joint')
                  if j.get('type') != 'fixed' and 'finger' not in j.get('name')]
    for group, names in [('arm', arm_joints),
                         ('gripper', ['left_finger_joint', 'right_finger_joint'])]:
        g = ET.SubElement(out, 'group', name=group)
        for n in names:
            ET.SubElement(g, 'joint', name=n)
    for name, group, values in [('home', 'arm', {'arm_1_joint': 0.35, 'arm_2_joint': -0.35}),
                                 ('open', 'gripper', {'left_finger_joint': 0.04}),
                                 ('closed', 'gripper', {'left_finger_joint': 0.011})]:
        s = ET.SubElement(out, 'group_state', name=name, group=group)
        for j, v in values.items():
            ET.SubElement(s, 'joint', name=j, value=str(v))
    # Every non-collision visual linkage can be ignored; retain meaningful arm,
    # chassis and finger pairs except mechanically adjacent parts.
    coll = {l.get('name') for l in r.findall('link') if l.find('collision') is not None}
    adjacent = {frozenset(p) for p in [('chassis_base_link','arm_1_link'),
                 ('arm_1_link','arm_2_link'), ('arm_2_link','gripper_link'),
                 ('gripper_link','left_finger'), ('gripper_link','right_finger')]}
    names = [l.get('name') for l in r.findall('link')]
    for a, b in itertools.combinations(names, 2):
        if a not in coll or b not in coll or frozenset([a,b]) in adjacent:
            ET.SubElement(out, 'disable_collisions', link1=a, link2=b, reason='Adjacent' if a in coll and b in coll else 'NoCollisionGeometry')
    return ET.tostring(out, encoding='unicode')
