import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('real_pick', ROOT / 'scripts/pick_real.py')
pick = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick)


def taught_config():
    cfg = pick.load_task(ROOT / 'config/real_pick.json')
    cfg.update(home_raw_sdk_mm=[139, 17], pick_raw_sdk_mm=[159, 4294967293],
               place_raw_sdk_mm=[189, 4294967293], safe_y_raw_sdk_mm=17)
    return cfg


class RealPickTests(unittest.TestCase):
    def test_relative_delta_across_unsigned_wrap(self):
        self.assertEqual(pick.modular_delta((159, 17), (159, 4294967293)), (0, 20))

    def exercise(self, fail_at=None, start=(159, 4294967293), release_feedback=True):
        calls, stages = [], []
        cfg = taught_config()
        bot = SimpleNamespace()
        task = pick.Pick(bot, cfg, lambda stage, **kw: stages.append(stage))
        task.feedback(start)
        def move(x, y):
            calls.append(('move', x, y))
            task.feedback(tuple((v + d) % 2**32 for v, d in zip(task.position, (x, y))))
            failed = stages[-1] == fail_at
            return SimpleNamespace(wait_for_completed=lambda timeout: True,
                                   has_succeeded=not failed, state='action_failed' if failed else 'action_succeeded')
        def opening(power):
            calls.append(('open', power))
            if release_feedback or stages[-1] == 'open_before_home_request':
                task.gripper_feedback('opened')
            else:
                task.gripper_feedback('normal')
            return True
        bot.robotic_arm = SimpleNamespace(move=move)
        bot.gripper = SimpleNamespace(
            close=lambda power: calls.append(('close', power)) or True,
            open=opening,
            pause=lambda: calls.append(('pause',)) or True)
        error = None
        with patch.object(pick.time, 'sleep'):
            try:
                task.run()
            except RuntimeError as exc:
                error = str(exc)
        return calls, stages, task, error

    def test_complete_sequence_releases_before_withdraw_and_returns_home(self):
        calls, stages, task, error = self.exercise()
        self.assertIsNone(error)
        self.assertEqual(task.relative_position(), (-20, 20))
        self.assertLess(stages.index('home_already_at_target'), stages.index('grip_request'))
        self.assertLess(stages.index('release_at_B_command_complete'), stages.index('withdraw_B_request'))
        self.assertEqual(stages[-1], 'returned_home')
        self.assertEqual(calls, [('open', 25), ('pause',), ('move', 0, 20), ('move', -20, 0),
                                ('move', 20, 0), ('move', 0, -20), ('close', 25), ('pause',),
                                ('move', 0, 20), ('move', 30, 0), ('move', 0, -17),
                                ('open', 25), ('pause',), ('move', 0, 17), ('move', -50, 0)])

    def test_failed_transfer_does_not_lower_release_or_return(self):
        calls, stages, task, error = self.exercise(fail_at='transfer_B_request')
        self.assertIn('SDK action failed', error)
        self.assertNotIn('lower_B_request', stages)
        self.assertNotIn('release_at_B_request', stages)
        self.assertNotIn('returned_home', stages)

    def test_release_without_open_feedback_does_not_return(self):
        calls, stages, task, error = self.exercise(release_feedback=False)
        self.assertIn('fully-open', error)
        self.assertNotIn('withdraw_B_request', stages)
        self.assertNotIn('returned_home', stages)

    def test_unexpected_start_has_no_commands(self):
        calls, stages, task, error = self.exercise(start=(259, 4294967293))
        self.assertIn('Startup pose', error)
        self.assertEqual(calls, [])

    def test_missing_teaching_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Missing taught poses'):
            pick.load_task(ROOT / 'config/real_pick.json', require_calibration=True)

    def test_unsafe_height_rejected(self):
        cfg = taught_config()
        cfg['safe_y_raw_sdk_mm'] = 0
        with self.assertRaises(ValueError):
            pick.layout(cfg)

    def test_returned_home_can_start_next_manually_reset_trial(self):
        calls, stages, task, error = self.exercise(start=(139, 17))
        self.assertIsNone(error)
        self.assertEqual(task.relative_position(), (-20, 20))


if __name__ == '__main__':
    unittest.main()
