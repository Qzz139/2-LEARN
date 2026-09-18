# 在 ROS 环境中验证非法任务在发送动作之前失败，并输出拒绝原因。
"""Exercise task rejection before any ROS action client can submit a goal."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires sourced ROS 2 environment')
class RejectedRequests(unittest.TestCase):
    def test_invalid_requests_produce_errors_without_motion(self):
        base = json.loads((ROOT/'src/ep_simulation/config/bottle.json').read_text())
        cases = {
            'unreachable': ({'pick_x_m': 1.0}, 'outside the planar arm reach'),
            'joint_limit': ({'pick_x_m': 0.1796188117642593,
                             'grasp_height_m': 0.21896093318057616,
                             'height_m': 0.32}, 'joint or coupled motor limits'),
            'nonfinite': ({'pick_x_m': float('nan')}, 'finite numeric'),
            'gripper_limit': ({'open_half_gap_m': 0.01}, 'opened 100 mm gripper'),
        }
        results = {}
        with tempfile.TemporaryDirectory() as directory:
            for name, (updates, expected) in cases.items():
                with self.subTest(case=name):
                    config = Path(directory)/(name+'.json')
                    output = Path(directory)/(name+'-result.json')
                    config.write_text(json.dumps(dict(base, **updates)))
                    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1')
                    run = subprocess.run([sys.executable,
                        str(ROOT/'src/ep_simulation/scripts/pick_demo.py'),
                        '--config', str(config), '--output', str(output)],
                        env=env, capture_output=True, text=True, timeout=15)
                    result = json.loads(output.read_text())
                    results[name] = {'return_code': run.returncode, 'result': result}
                    self.assertEqual(run.returncode, 1, run.stderr)
                    self.assertFalse(result['success'])
                    self.assertEqual(result['events'], [])
                    self.assertIn(expected, result['error'])
        dest = ROOT/'work/invalid-requests.json'
        dest.parent.mkdir(exist_ok=True)
        dest.write_text(json.dumps(results, indent=2)+'\n')
