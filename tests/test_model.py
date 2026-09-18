# 检查瓶子配置、正逆运动学和 URDF 拓扑；URDF 测试需要 xacro。
"""Run after building: python3 -m unittest discover -s tests -v."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ep_simulation.model import adapt, fk, ik, valid
from ep_simulation.scene import semantic, load_config, targets, bottle_start, bottle_goal, bottle_mass, world_sdf
CONFIG = Path(__file__).resolve().parents[1]/"src/ep_simulation/config/bottle.json"


class WorkspaceTests(unittest.TestCase):
    def test_demo_targets_inside_motor_limits(self):
        for q in list(targets(load_config(CONFIG)).values()) + [[0.35, -0.35]]:
            self.assertTrue(valid(q))
        self.assertFalse(valid([-0.274, -1.21475]))  # both individual limits pass; coupled motor fails
        self.assertFalse(valid([0.0, 0.4]))

    def test_floor_scene_and_clearance(self):
        cfg = load_config(CONFIG); qs = targets(cfg)
        self.assertAlmostEqual(bottle_start(cfg)[2], cfg['height_m']/2)
        self.assertAlmostEqual(fk(qs['lift'])[2]-fk(qs['pick'])[2], cfg['lift_height_m'])
        self.assertGreater(abs(bottle_start(cfg)[0]-bottle_goal(cfg)[0]), 0.04)
        root = ET.fromstring(world_sdf(cfg)); models = {m.get('name'):m for m in root.findall('world/model')}
        self.assertNotIn('pick_pedestal', models)
        # Avoid the closed jaw intersecting the bottle when the robot is spawned.
        self.assertGreater(cfg['pick_x_m']-cfg['diameter_m']/2-(fk([0,0])[0]+0.025), 0.003)
        self.assertIsNone(models['place_marker'].find('link/collision'))
        self.assertAlmostEqual(float(models['target_bottle'].find('link/inertial/mass').text), bottle_mass(cfg))

    def test_ik_roundtrip(self):
        for a in [0.0, 0.5, 0.9, 1.3]:
            for b in [-1.0, -0.6, 0.0]:
                if valid([a,b]):
                    xyz = fk([a,b]); reconstructed = fk(ik(xyz[0], xyz[2]))
                    for v,w in zip(xyz, reconstructed): self.assertAlmostEqual(v, w, places=8)

    def test_unreachable_and_nonfinite_targets_rejected(self):
        for x,z in [(1.0, 0.1), (float('nan'), 0.1), (0.1056754, 0.1172202)]:
            with self.assertRaises(ValueError): ik(x,z)
        xyz = fk([-0.4, 0.0])
        with self.assertRaises(ValueError): ik(xyz[0],xyz[2])



class URDFTests(unittest.TestCase):
    # 展开实际 Xacro，再对适配后的 XML 检查拓扑与运动学一致性。
    @classmethod
    def setUpClass(cls):
        try:
            import xacro
        except ImportError:
            raise unittest.SkipTest('URDF tests require the ROS xacro environment')
        root = Path(__file__).resolve().parents[1]
        raw = xacro.process_file(str(root/'src/robomaster_description/urdf/robomaster_ep.urdf.xacro')).toxml()
        cls.xml = adapt(raw, '/tmp/controllers.yaml')
        cls.robot = ET.fromstring(cls.xml)

    def test_fixed_root_and_independent_coordinates(self):
        joints = self.robot.findall('joint')
        independent = {j.get('name') for j in joints if j.get('type') != 'fixed' and j.find('mimic') is None}
        self.assertEqual(independent, {'arm_1_joint','arm_2_joint','left_finger_joint'})
        self.assertEqual(self.robot.find("joint[@name='world_fixed']").get('type'), 'fixed')
        children = [j.find('child').get('link') for j in joints]
        self.assertEqual(len(children), len(set(children)))

    def test_planning_group_contains_all_arm_mimics(self):
        srdf = ET.fromstring(semantic(self.xml))
        group = {j.get('name') for j in srdf.find("group[@name='arm']").findall('joint')}
        for joint in self.robot.findall('joint'):
            if joint.get('type') != 'fixed' and 'finger' not in joint.get('name'):
                self.assertIn(joint.get('name'), group)

    def test_tool_fk_matches_urdf_and_stays_level(self):
        joints = {j.get('name'):j for j in self.robot.findall('joint')}
        parents = {j.find('child').get('link'):j for j in joints.values()}
        for a in [-0.1, 0.35, 0.8, 1.1]:
            for b in [-0.7, -0.35, 0.1]:
                if not valid([a,b]): continue
                values={'arm_1_joint':a,'arm_2_joint':b,'left_finger_joint':0.04}
                def angle(j):
                    m=j.find('mimic')
                    if m is not None:
                        return float(m.get('multiplier','1'))*angle(joints[m.get('joint')])+float(m.get('offset','0'))
                    return values.get(j.get('name'),0)
                chain=[];link='tool_link'
                while link in parents:
                    j=parents[link];chain.append(j);link=j.find('parent').get('link')
                x=y=z=pitch=0.0
                for j in reversed(chain):
                    origin=j.find('origin');xyz=list(map(float,origin.get('xyz','0 0 0').split()))
                    self.assertEqual(origin.get('rpy','0 0 0'),'0 0 0')
                    x+=math.cos(pitch)*xyz[0]+math.sin(pitch)*xyz[2]
                    y+=xyz[1]
                    z+=-math.sin(pitch)*xyz[0]+math.cos(pitch)*xyz[2]
                    pitch+=angle(j)
                for actual, expected in zip([x,y,z],fk([a,b])):
                    self.assertAlmostEqual(actual,expected,places=9)
                self.assertAlmostEqual(pitch,0.0,places=9)


if __name__ == '__main__': unittest.main()
