# 通过假设备验证通信检查、坐标解码及清理过程，不连接真实机器人。
"""Verify the probe uses only communication/query methods and cleans up."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('ep_probe', Path(__file__).resolve().parents[1] / 'scripts/check_ep_connection.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


# 只暴露通信检查所需接口，调用列表用于核对订阅和关闭顺序。
class FakeRobot:
    def __init__(self, samples=(), connected=True):
        self.robotic_arm = self
        self.samples = samples
        self.connected = connected
        self.calls = []

    def initialize(self, **kwargs):
        self.calls.append('initialize')
        return self.connected

    def get_version(self):
        self.calls.append('get_version')
        return 'test-version'

    def sub_position(self, freq, callback):
        self.calls.append('sub_position')
        for sample in self.samples:
            callback(sample)
        return True

    def unsub_position(self):
        self.calls.append('unsub_position')

    def close(self):
        self.calls.append('close')


class ConnectionTests(unittest.TestCase):
    def test_valid_feedback_and_cleanup(self):
        bot, report = FakeRobot([(10, 20), (11, 20)]), {}
        probe.observe(bot, 0.01, report)
        self.assertTrue(report['success'])
        self.assertEqual(report['arm_position_mm'], {'x_mm': 11, 'y_mm': 20})
        self.assertEqual(bot.calls, ['initialize', 'get_version', 'sub_position', 'unsub_position', 'close'])

    def test_invalid_feedback_does_not_pass(self):
        bot = FakeRobot([(float('nan'), 0), (1, 2)])
        with self.assertRaisesRegex(RuntimeError, 'two valid'):
            probe.observe(bot, 0.01, {})
        self.assertEqual(bot.calls[-2:], ['unsub_position', 'close'])

    def test_connection_failure_closes(self):
        bot = FakeRobot(connected=False)
        with self.assertRaisesRegex(RuntimeError, 'connection failed'):
            probe.observe(bot, 0.01, {})
        self.assertEqual(bot.calls, ['initialize', 'close'])

    def test_unsigned_coordinate_is_not_accepted_for_motion(self):
        bot, report = FakeRobot([(159, 4294967293)] * 2), {}
        with self.assertRaisesRegex(RuntimeError, 'Implausible'):
            probe.observe(bot, 0.01, report)
        self.assertTrue(report['connection_ok'])
        self.assertFalse(report['position_valid'])
        self.assertEqual(report['arm_position_raw_sdk_mm']['y_mm'], 4294967293)
        self.assertEqual(report['signed_int32_candidate_mm']['y_mm'], -3)
        self.assertNotIn('arm_position_mm', report)
        self.assertEqual(bot.calls[-2:], ['unsub_position', 'close'])

    def test_signed_query_confirms_subscription_decoding(self):
        bot, report = FakeRobot([(159, 4294967293)] * 2), {}
        probe.observe(bot, 0.01, report, query_position=lambda _: {'x_mm': 159, 'y_mm': -3})
        self.assertTrue(report['success'])
        self.assertEqual(report['arm_position_mm']['y_mm'], -3)
        self.assertEqual(report['arm_position_raw_sdk_mm']['y_mm'], 4294967293)
        self.assertFalse(report['coordinate_calibrated'])

    def test_signed_query_disagreement_stops(self):
        bot, report = FakeRobot([(159, 4294967293)] * 2), {}
        with self.assertRaisesRegex(RuntimeError, 'disagree'):
            probe.observe(bot, 0.01, report, query_position=lambda _: {'x_mm': 159, 'y_mm': 50})
        self.assertFalse(report['position_valid'])
        self.assertNotIn('arm_position_mm', report)
        self.assertEqual(bot.calls[-2:], ['unsub_position', 'close'])


if __name__ == '__main__':
    unittest.main()
