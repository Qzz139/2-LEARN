"""Adapt the pinned upstream EP geometry to a two-coordinate simulation tree.

The shoulder coordinate is the right servo angle; elbow is left minus right.
Zero-length counter-rotation joints encode the parallelogram without a loop.
The original gripper linkage is replaced by a documented parallel-jaw proxy.
"""
import math
import xml.etree.ElementTree as ET

ARM_JOINTS = ['arm_1_joint', 'arm_2_joint']
HOME = [0.35, -0.35]
BASE_HEIGHT = 0.0  # Upstream wheel centers are 0.05 m above base_link.
TOOL_OFFSET = (0.1056754, -0.000941, 0.1172202)


def element(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def inertial(link, mass=0.0001, diagonal=1e-7):
    i = element(link, 'inertial')
    element(i, 'mass', value=mass)
    element(i, 'inertia', ixx=diagonal, iyy=diagonal, izz=diagonal,
            ixy=0, ixz=0, iyz=0)


def joint(root, name, parent, child, kind='continuous', xyz='0 0 0',
          axis='0 1 0', mimic=None, multiplier=1):
    j = element(root, 'joint', name=name, type=kind)
    element(j, 'parent', link=parent)
    element(j, 'child', link=child)
    element(j, 'origin', xyz=xyz, rpy='0 0 0')
    if kind != 'fixed':
        element(j, 'axis', xyz=axis)
        element(j, 'limit', effort=20, velocity=1.0)
    if mimic:
        element(j, 'mimic', joint=mimic, multiplier=multiplier, offset=0)
    return j


def adapt(xml, controllers):
    root = ET.fromstring(xml)
    root.set('name', 'ep_fixed_arm')
    removed = {n.get('name') for n in root.findall('link')
               if 'gripper' in n.get('name') and n.get('name') != 'gripper_link'}
    for n in list(root):
        if n.tag == 'link' and n.get('name') in removed:
            root.remove(n)
        elif n.tag == 'joint' and (n.find('parent').get('link') in removed or
                                  n.find('child').get('link') in removed):
            root.remove(n)
    joints = {j.get('name'): j for j in root.findall('joint')}
    # All chassis, wheel and camera joints are fixed. Keep the arm's moving tree.
    for name, j in joints.items():
        if name not in ['arm_1_joint', 'arm_2_joint', 'endpoint_bracket_joint',
                        'rod_joint', 'rod_1_joint', 'rod_2_joint', 'rod_3_joint',
                        'triangle_joint']:
            j.set('type', 'fixed')
            for tag in ['mimic', 'limit', 'axis']:
                for n in j.findall(tag):
                    j.remove(n)
    elbow = joints['arm_2_joint']
    elbow.remove(elbow.find('mimic'))
    elbow.set('type', 'revolute')
    elbow.find('limit').set('lower', '-1.21475')
    elbow.find('limit').set('upper', '0.34732')
    # shoulder + elbow + (-shoulder) + (-elbow) = constant tool pitch.
    wrist = joints['endpoint_bracket_joint']
    original_origin = wrist.find('origin').get('xyz')
    wrist.find('parent').set('link', 'wrist_counter_link')
    wrist.find('origin').set('xyz', '0 0 0')
    wrist.find('mimic').set('joint', 'arm_2_joint')
    inertial(element(root, 'link', name='wrist_counter_link'))
    joint(root, 'wrist_counter_shoulder', 'arm_2_link', 'wrist_counter_link',
          xyz=original_origin, mimic='arm_1_joint', multiplier=-1)
    # The rod motor angle is shoulder + elbow; compose two coaxial rotations.
    rod = joints['rod_joint']
    rod_origin = rod.find('origin').get('xyz')
    rod.find('parent').set('link', 'rod_counter_link')
    rod.find('origin').set('xyz', '0 0 0')
    rod.set('type', 'continuous')
    element(rod, 'mimic', joint='arm_2_joint', multiplier=1, offset=0)
    inertial(element(root, 'link', name='rod_counter_link'))
    joint(root, 'rod_counter_shoulder', 'arm_base_link', 'rod_counter_link',
          xyz=rod_origin, mimic='arm_1_joint')
    element(joints['rod_3_joint'], 'mimic', joint='arm_2_joint', multiplier=-1, offset=0)
    # rod_1 needs the full rod angle too. Its original mimic only follows elbow now.
    rod1 = joints['rod_1_joint']
    origin1 = rod1.find('origin').attrib.copy()
    inertial(element(root, 'link', name='rod1_counter_link'))
    j = joint(root, 'rod1_counter_shoulder', 'endpoint_bracket_link',
              'rod1_counter_link', xyz=origin1['xyz'], mimic='arm_1_joint')
    j.find('origin').set('rpy', origin1.get('rpy', '0 0 0'))
    rod1.find('parent').set('link', 'rod1_counter_link')
    rod1.find('origin').set('xyz', '0 0 0')
    rod1.find('origin').set('rpy', '0 0 0')
    rod1.find('mimic').set('joint', 'arm_2_joint')
    element(root, 'link', name='world')
    joint(root, 'world_fixed', 'world', 'base_link', kind='fixed', xyz='0 0 '+str(BASE_HEIGHT))
    inertial(element(root, 'link', name='tool_link'))
    joint(root, 'tool_fixed', 'gripper_link', 'tool_link', kind='fixed', xyz='0.095 0 -0.039')
    for side, sign in [('left', 1), ('right', -1)]:
        link = element(root, 'link', name=side+'_finger')
        inertial(link, mass=0.025, diagonal=1e-5)
        for tag in ['visual', 'collision']:
            item = element(link, tag)
            element(element(item, 'geometry'), 'box', size='0.05 0.01 0.05')
            if tag == 'visual':
                element(element(item, 'material', name='finger_dark'), 'color', rgba='0.12 0.14 0.16 1')
        j = joint(root, side+'_finger_joint', 'tool_link', side+'_finger',
                  kind='prismatic', xyz='0 '+str(sign*0.005)+' 0',
                  axis='0 '+str(sign)+' 0',
                  mimic='left_finger_joint' if side == 'right' else None)
        j.find('limit').attrib.update(lower='0', upper='0.05', effort='20', velocity='0.04')
        g = element(root, 'gazebo', reference=side+'_finger')
        for key, value in [('mu1', 2), ('mu2', 2), ('kp', 100000), ('kd', 10),
                           ('minDepth', 0.0005), ('maxVel', 0.05)]:
            element(g, key).text = str(value)
    # Small convex collision proxies keep mesh detail out of the contact solver.
    collision_shapes = {
        'chassis_base_link': ('-0.01 0 0.027', '0 0 0', '0.31 0.18 0.065'),
        'arm_1_link': ('0.0065 0.0214 0.064', '0 0 0', '0.035 0.045 0.11'),
        'arm_2_link': ('0.053 0.003 -0.028', '0 0.487 0', '0.12 0.045 0.022'),
        'gripper_link': ('0.016 0 -0.022', '0 0 0', '0.04 0.06 0.03'),
    }
    for link in root.findall('link'):
        if link.get('name') in ('left_finger', 'right_finger'):
            continue
        for c in link.findall('collision'):
            link.remove(c)
        if link.get('name') in collision_shapes:
            xyz, rpy, size = collision_shapes[link.get('name')]
            c = element(link, 'collision')
            element(c, 'origin', xyz=xyz, rpy=rpy)
            element(element(c, 'geometry'), 'box', size=size)
    # Upstream tiny-link inertias are visualization values, not calibrated dynamics.
    # Use conservative positive diagonal approximations and disable gravity on
    # the arm's constrained tree; the world/bottle retain gravity and collisions.
    for link in root.findall('link'):
        if link.get('name') == 'world':
            continue
        if link.find('inertial') is None:
            inertial(link)
        i = link.find('inertial/inertia')
        if link.get('name') in ['wrist_counter_link', 'rod_counter_link', 'rod1_counter_link']:
            # Finite virtual-rotor inertia avoids near-singular coaxial ODE motors.
            # This is numerical regularization, not a measured EP rotor mass.
            link.find('inertial/mass').set('value', '0.05')
        mass = float(link.find('inertial/mass').get('value'))
        for k in ['ixx', 'iyy', 'izz']:
            i.set(k, str(max(1e-7, mass*0.002)))
        for k in ['ixy', 'ixz', 'iyz']:
            i.set(k, '0')
        g = element(root, 'gazebo', reference=link.get('name'))
        element(g, 'gravity').text = 'false'
        element(g, 'selfCollide').text = 'false'
    hardware = element(root, 'ros2_control', name='GazeboSystem', type='system')
    element(element(hardware, 'hardware'), 'plugin').text = 'ep_gazebo_control/PositionServo'
    moving = [j for j in root.findall('joint') if j.get('type') != 'fixed']
    # Put independent joints first so every mimic source is initialized first.
    moving.sort(key=lambda j: j.find('mimic') is not None)
    for j in moving:
        limit = j.find('limit')
        if limit is None:
            limit = element(j, 'limit', effort=20, velocity=1)
        if j.get('type') != 'prismatic':
            limit.set('effort', '20'); limit.set('velocity', '1')
        hw = element(hardware, 'joint', name=j.get('name'))
        finger = 'finger' in j.get('name')
        element(hw, 'param', name='servo_gain').text = '20'
        element(hw, 'param', name='max_velocity').text = '0.04' if finger else '0.5'
        element(hw, 'param', name='max_effort').text = '2.0' if finger else '20.0'
        m = j.find('mimic')
        if m is not None:
            element(hw, 'param', name='mimic').text = m.get('joint')
            element(hw, 'param', name='multiplier').text = m.get('multiplier', '1')
        element(hw, 'command_interface', name='position')
        element(hw, 'state_interface', name='position')
        element(hw, 'state_interface', name='velocity')
    plugin = element(element(root, 'gazebo'), 'plugin',
                     name='gazebo_ros2_control', filename='libgazebo_ros2_control.so')
    element(plugin, 'parameters').text = str(controllers)
    return ET.tostring(root, encoding='unicode')


def fk(q):
    """Tool center in world, meters, derived from the retained mesh joint origins."""
    a, b = q
    def ry(x, z, theta):
        return (math.cos(theta)*x + math.sin(theta)*z,
                -math.sin(theta)*x + math.cos(theta)*z)
    x1, z1 = ry(0.0018704, 0.1210238, a)
    x2, z2 = ry(0.1058557, -0.0561093, a+b)
    return [0.0103961+x1+x2+0.0002793+0.095,
            0.0010384-0.0255713+0.0438659+0.006752-0.027026,
            BASE_HEIGHT+0.03465+0.0906477+0.030741+z1+z2+0.0001815-0.039]


def valid(q):
    a, b = q
    return (-0.274 <= a <= 1.384 and -1.21475 <= b <= 0.34732
            and -0.79936 <= a+b <= 1.73137)


def ik(x, z):
    """Planar position IK for this pinned geometry; reject unreachable targets."""
    if not all(math.isfinite(v) for v in [x, z]):
        raise ValueError('Target coordinates must be finite')
    u, v = x-TOOL_OFFSET[0], z-TOOL_OFFSET[2]-BASE_HEIGHT
    l1, l2 = math.hypot(0.0018704, 0.1210238), math.hypot(0.1058557, -0.0561093)
    p1, p2 = math.atan2(0.1210238, 0.0018704), math.atan2(-0.0561093, 0.1058557)
    cosine = (u*u+v*v-l1*l1-l2*l2)/(2*l1*l2)
    if abs(cosine) > 1+1e-10:
        raise ValueError('Target lies outside the planar arm reach')
    for delta in [-math.acos(max(-1, min(1, cosine))), math.acos(max(-1, min(1, cosine)))]:
        a = p1-math.atan2(v, u)+math.atan2(l2*math.sin(delta), l1+l2*math.cos(delta))
        b = p2-p1-delta
        a = (a+math.pi) % (2*math.pi)-math.pi
        b = (b+math.pi) % (2*math.pi)-math.pi
        if valid([a, b]):
            return [a, b]
    raise ValueError('Target violates EP joint or coupled motor limits')
