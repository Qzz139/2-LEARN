"""Deterministic tabletop scene and MoveIt semantic description."""
import itertools
import xml.etree.ElementTree as ET
from .model import fk

PICK = [1.1, -0.7]
LIFT = [0.8, -0.6]
CUBE_SIZE = 0.025
CUBE_START = fk(PICK)
TABLE_HEIGHT = CUBE_START[2] - CUBE_SIZE/2


def world_sdf():
    x, y, z = CUBE_START
    h = TABLE_HEIGHT
    return f'''<sdf version="1.6"><world name="ep_pick_world">
      <gravity>0 0 -9.81</gravity>
      <physics name="ode" type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate><ode><solver><iters>100</iters></solver></ode></physics>
      <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.82 0.86 0.91 1</background><shadows>true</shadows></scene>
      <light name="sun" type="directional"><pose>0 0 2 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.3 0.1 -1</direction></light>
      <gui><camera name="user_camera"><pose>0.9 -0.8 0.65 0 0.48 2.35</pose></camera></gui>
      <model name="ground"><static>true</static><link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry><material><ambient>0.65 0.69 0.73 1</ambient></material></visual>
      </link></model>
      <model name="pick_pedestal"><static>true</static><pose>0.305 {y} {h/2} 0 0 0</pose><link name="link">
        <collision name="collision"><geometry><box><size>0.08 0.12 {h}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.08 0.12 {h}</size></box></geometry><material><ambient>0.22 0.35 0.48 1</ambient></material></visual>
      </link></model>
      <model name="target_cube"><pose>{x} {y} {z+0.0001} 0 0 0</pose><link name="link">
        <inertial><mass>0.02</mass><inertia><ixx>0.0000020833</ixx><iyy>0.0000020833</iyy><izz>0.0000020833</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.025 0.025 0.025</size></box></geometry><surface><friction><ode><mu>2</mu><mu2>2</mu2></ode></friction><contact><ode><kp>100000</kp><kd>10</kd><min_depth>0.0005</min_depth><max_vel>0.05</max_vel></ode></contact></surface></collision>
        <visual name="visual"><geometry><box><size>0.025 0.025 0.025</size></box></geometry><material><ambient>0.95 0.28 0.06 1</ambient><diffuse>0.95 0.28 0.06 1</diffuse></material></visual>
      </link></model>
      <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>30</update_rate></plugin>
    </world></sdf>'''


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
