"""Run after building: python3 -m unittest discover -s tests -v."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from ep_simulation.model import adapt, fk, valid
from ep_simulation.scene import semantic, CUBE_START, TABLE_HEIGHT, PICK, LIFT


class WorkspaceTests(unittest.TestCase):
    def test_demo_targets_inside_motor_limits(self):
        for q in [PICK, LIFT, [0.35, -0.35]]:
            self.assertTrue(valid(q))
        self.assertFalse(valid([-0.274, -1.21475]))  # both individual limits pass; coupled motor fails
        self.assertFalse(valid([0.0, 0.4]))

    def test_scene_clearance_and_lift(self):
        self.assertAlmostEqual(CUBE_START[2]-TABLE_HEIGHT, 0.0125)
        self.assertGreater(fk(LIFT)[2]-fk(PICK)[2], 0.04)
        self.assertAlmostEqual(fk(LIFT)[1], fk(PICK)[1])


class URDFTests(unittest.TestCase):
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
