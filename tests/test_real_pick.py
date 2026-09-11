import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('real_pick', ROOT / 'scripts/pick_real.py')
pick = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick)


class RealPickTests(unittest.TestCase):
    def test_relative_delta_across_unsigned_wrap(self):
        self.assertEqual(pick.modular_delta((159, 17), (159, 4294967293)), (0, 20))

    def exercise(self, fail=False):
        calls = []
        cfg = pick.load_task(ROOT / 'config/real_pick.json')
        bot = SimpleNamespace()
        task = pick.Pick(bot, cfg, lambda *args, **kwargs: None)
        task.feedback((159, 4294967293))
        def move(x, y):
            calls.append(('move', x, y))
            task.feedback(tuple((v + d) % 2**32 for v, d in zip(task.position, (x, y))))
            return SimpleNamespace(wait_for_completed=lambda timeout: True,
                                   has_succeeded=not fail, state='action_failed' if fail else 'action_succeeded')
        bot.robotic_arm = SimpleNamespace(move=move)
        bot.gripper = SimpleNamespace(
            close=lambda power: calls.append(('close', power)) or True,
            open=lambda power: calls.append(('open', power)) or True,
            pause=lambda: calls.append(('pause',)) or True)
        with patch.object(pick.time, 'sleep'):
            if fail:
                with self.assertRaisesRegex(RuntimeError, 'SDK action failed'):
                    task.run()
            else:
                task.run()
        return calls

    def test_complete_sequence(self):
        self.assertEqual(self.exercise(), [('close', 25), ('pause',), ('move', 0, 20),
                                          ('move', 30, 0), ('move', 0, -17), ('open', 25), ('pause',)])

    def test_failed_action_does_not_transfer_lower_or_release(self):
        self.assertEqual(self.exercise(True), [('close', 25), ('pause',), ('move', 0, 20)])


if __name__ == '__main__':
    unittest.main()
