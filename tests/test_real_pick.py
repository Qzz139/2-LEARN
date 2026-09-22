# 使用假 SDK 和可控时钟检查运动顺序、反馈超时、分段横移及失败中止。
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import json
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('real_pick', ROOT / 'scripts/pick_real.py')
pick = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick)


# 提供包含 uint32 回绕坐标的示教样例，检查负坐标的相对位移计算。
def taught_config():
    cfg = pick.load_task(ROOT / 'config/real_pick.json')
    cfg.update(home_raw_sdk_mm=[139, 17], pick_raw_sdk_mm=[159, 4294967293],
               place_raw_sdk_mm=[189, 4294967293], safe_y_raw_sdk_mm=17)
    return cfg


class RealPickTests(unittest.TestCase):
    def test_relative_delta_across_unsigned_wrap(self):
        self.assertEqual(pick.modular_delta((159, 17), (159, 4294967293)), (0, 20))

    # 记录假设备命令，用可控时钟持续注入反馈，并允许指定阶段失败。
    def exercise(self, fail_at=None, start=(159, 4294967293), release_feedback=True, home_only=False, cfg=None, check_path=False):
        calls, stages = [], []
        cfg = taught_config() if cfg is None else cfg
        bot = SimpleNamespace()
        task = pick.Pick(bot, cfg, lambda stage, **kw: stages.append(stage))
        task.feedback(start)
        def move(x, y):
            delta = pick.modular_delta((x, y), task.position)
            calls.append(('move', *delta))
            task.feedback((x % 2**32, y % 2**32))
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
        bot.robotic_arm = SimpleNamespace(moveto=move)
        bot.gripper = SimpleNamespace(
            close=lambda power: calls.append(('close', power)) or True,
            open=opening,
            pause=lambda: calls.append(('pause',)) or True)
        error = None
        clock = [pick.time.monotonic()]
        def sleep(dt):
            clock[0] += dt
            task.feedback(task.position)
            if task.gripper_status is not None:
                task.gripper_feedback(task.gripper_status)
        with patch.object(pick.time, 'sleep', sleep), patch.object(pick.time, 'monotonic', lambda: clock[0]):
            try:
                task.run(home_only=home_only, check_path=check_path)
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

    def retracted_config(self):
        cfg = taught_config()
        cfg.update(home_raw_sdk_mm=[146, 82], pick_raw_sdk_mm=[146, 62],
                   place_raw_sdk_mm=[236, 62], safe_y_raw_sdk_mm=82)
        return cfg

    def test_90_mm_cycle_segments_both_directions_and_releases(self):
        calls, stages, task, error = self.exercise(start=(146, 82), cfg=self.retracted_config())
        self.assertIsNone(error)
        horizontal = [c[1] for c in calls if c[0] == 'move' and c[1]]
        self.assertEqual(horizontal, [60, 30, -60, -30])
        self.assertEqual(task.relative_position(), (0, 20))
        self.assertLess(stages.index('release_at_B_command_complete'), stages.index('withdraw_B_request'))

    def test_empty_path_never_closes_gripper(self):
        calls, stages, task, error = self.exercise(start=(146, 82), cfg=self.retracted_config(), check_path=True)
        self.assertIsNone(error)
        self.assertFalse(any(c[0] == 'close' for c in calls))
        self.assertIn('empty_path_skip_grip', stages)
        self.assertEqual(stages[-1], 'returned_home')

    def test_segment_height_restored_before_second_horizontal_command(self):
        task = pick.Pick(SimpleNamespace(), self.retracted_config(), lambda *args, **kw: None)
        task.feedback((146, 80))
        calls = []
        def move(stage, x, y):
            calls.append((stage, x, y))
            actual_y = task.position[1] + y - (2 if len(calls)==1 else 0)
            task.feedback((task.position[0]+x, actual_y))
        task.move = move
        task.goto('transfer_B', x=90)
        self.assertEqual(calls, [('transfer_B_segment_1',60,0),
                                 ('transfer_B_restore_height',0,4),
                                 ('transfer_B_segment_2',30,0)])

    def test_height_restore_failure_prevents_second_segment(self):
        task = pick.Pick(SimpleNamespace(), self.retracted_config(), lambda *args, **kw: None)
        task.feedback((146,80))
        calls = []
        def move(stage,x,y):
            calls.append(stage)
            if stage.endswith('restore_height'):
                raise RuntimeError('height restoration failed')
            task.feedback((206,78))
        task.move = move
        with self.assertRaisesRegex(RuntimeError, 'height restoration failed'):
            task.goto('transfer_B', x=90)
        self.assertNotIn('transfer_B_segment_2', calls)

    def test_first_segment_failure_stops_before_second_segment_and_release(self):
        calls, stages, task, error = self.exercise(start=(146, 82), cfg=self.retracted_config(),
                                                  fail_at='transfer_B_segment_1_request')
        self.assertIn('SDK action failed', error)
        self.assertNotIn('transfer_B_segment_2_request', stages)
        self.assertNotIn('release_at_B_request', stages)

    def test_layout_beyond_requested_90_mm_rejected(self):
        cfg = self.retracted_config()
        cfg['place_raw_sdk_mm'][0] = 237
        with self.assertRaisesRegex(ValueError, 'Horizontal span'):
            pick.layout(cfg)

    def test_long_traverse_below_safe_height_has_no_motion(self):
        task = pick.Pick(SimpleNamespace(), self.retracted_config(), lambda *args, **kw: None)
        task.feedback((146, 62))
        with self.assertRaisesRegex(RuntimeError, 'safe height'):
            task.goto('transfer_B', x=90)

    def test_unsafe_height_rejected(self):
        cfg = taught_config()
        cfg['safe_y_raw_sdk_mm'] = 0
        with self.assertRaises(ValueError):
            pick.layout(cfg)

    def test_layout_allows_raised_home_above_ground_pick(self):
        cfg = taught_config()
        cfg.update(home_raw_sdk_mm=[159, 82], pick_raw_sdk_mm=[159, 4294967293],
                   place_raw_sdk_mm=[189, 4294967293], safe_y_raw_sdk_mm=82)
        points = pick.layout(cfg)
        self.assertEqual(points['home'][1], 85)
        self.assertEqual(points['safe_y'], 85)

    def test_layout_beyond_vertical_span_rejected(self):
        cfg = taught_config()
        cfg.update(home_raw_sdk_mm=[159, 150], pick_raw_sdk_mm=[159, 4294967293],
                   place_raw_sdk_mm=[189, 4294967293], safe_y_raw_sdk_mm=150)
        with self.assertRaisesRegex(ValueError, 'vertical workspace'):
            pick.layout(cfg)

    def test_vertical_lift_over_60_mm_is_segmented(self):
        cfg = taught_config()
        cfg.update(home_raw_sdk_mm=[159, 82], pick_raw_sdk_mm=[159, 4294967293],
                   place_raw_sdk_mm=[189, 4294967293], safe_y_raw_sdk_mm=82)
        task = pick.Pick(SimpleNamespace(), cfg, lambda *args, **kw: None)
        task.feedback((159, 4294967293))
        calls = []
        def move(stage, x, y):
            calls.append((stage, x, y))
            task.feedback((task.position[0] + x, (task.position[1] + y) % 2**32))
        task.move = move
        task.goto('lift', y=85)
        self.assertEqual(calls, [('lift_segment_1', 0, 60), ('lift_segment_2', 0, 25)])

    def test_returned_home_can_start_next_manually_reset_trial(self):
        calls, stages, task, error = self.exercise(start=(139, 17))
        self.assertIsNone(error)
        self.assertEqual(task.relative_position(), (-20, 20))

    def test_one_mm_correction_does_not_send_motion(self):
        task = pick.Pick(SimpleNamespace(), taught_config(), lambda *args, **kw: None)
        task.feedback((160, 17))
        task.goto('home_traverse', x=0)

    def test_home_only_never_grips_or_goes_to_B(self):
        calls, stages, task, error = self.exercise(home_only=True)
        self.assertIsNone(error)
        self.assertEqual(task.relative_position(), (-20, 20))
        self.assertNotIn('grip_request', stages)
        self.assertNotIn('transfer_B_request', stages)
        self.assertEqual(stages[-1], 'home_complete')

    def test_delayed_feedback_waits_instead_of_reissuing_command(self):
        self.check_feedback_timing(delay=0.9)

    def test_success_reply_without_new_feedback_is_not_completion(self):
        self.check_feedback_timing(delay=None, expected_error='stale')

    def test_stationary_axis_drop_is_rejected(self):
        self.check_feedback_timing(delay=0.1, displacement=(1, -7), expected_error='stationary axis')

    def test_sdk_success_with_fresh_unchanged_feedback_still_fails(self):
        self.check_feedback_timing(delay=0.1, displacement=(0, 0), expected_error='endpoint did not settle', tolerance=3)

    # 模拟应答与反馈到达时间不同，验证终点判断依赖新采样。
    def check_feedback_timing(self, delay, displacement=(20, 0), expected_error=None, tolerance=2):
        clock = [100.0]
        calls = []
        cfg = taught_config()
        cfg['position_tolerance_mm'] = tolerance
        task = pick.Pick(SimpleNamespace(), cfg, lambda *args, **kw: None)
        action = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=True, state='action_succeeded')
        task.bot.robotic_arm = SimpleNamespace(moveto=lambda x,y: calls.append((x,y)) or action)
        def sleep(dt):
            clock[0] += dt
            if delay is not None:
                delta = displacement if clock[0] >= 100 + delay else (0, 0)
                task.feedback((159 + delta[0], (4294967293 + delta[1]) % 2**32))
        with patch.object(pick.time, 'monotonic', lambda: clock[0]), patch.object(pick.time, 'sleep', sleep):
            task.feedback((159, 4294967293))
            if expected_error:
                with self.assertRaisesRegex(RuntimeError, expected_error):
                    task.move('test', 20, 0)
                self.assertIs(task.active, action)
            else:
                task.move('test', 20, 0)
                self.assertGreaterEqual(clock[0], 101.3)
                self.assertIsNone(task.active)
        self.assertEqual(calls, [(179, -3)])

    def test_stable_lift_shortfall_gets_only_one_upward_correction(self):
        self.check_lift_shortfall(2, [(159,16),(159,16)])

    def test_three_mm_tolerance_accepts_stable_shortfall_without_retry(self):
        self.check_lift_shortfall(3, [(159,16)])

    def test_larger_tolerance_rejected(self):
        cfg = taught_config()
        cfg['position_tolerance_mm'] = 4
        with self.assertRaisesRegex(ValueError, 'tolerance'):
            pick.Pick(SimpleNamespace(), cfg, lambda *args, **kw: None)

    def test_configured_tolerance_round_trip_and_bounds(self):
        cfg = taught_config()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            cfg['position_tolerance_mm'] = 3
            path.write_text(json.dumps(cfg))
            self.assertEqual(pick.load_task(path)['position_tolerance_mm'], 3)
            cfg['position_tolerance_mm'] = 4
            path.write_text(json.dumps(cfg))
            with self.assertRaisesRegex(ValueError, 'position_tolerance_mm'):
                pick.load_task(path)

    # 模拟稳定的抬升不足，检查仅允许一次向上补偿及配置容差。
    def check_lift_shortfall(self, tolerance, expected):
        clock, calls = [100.0], []
        cfg = taught_config()
        cfg['position_tolerance_mm'] = tolerance
        task = pick.Pick(SimpleNamespace(), cfg, lambda *args, **kw: None)
        def move(x,y):
            calls.append((x,y))
            actual_y = y-3 if len(calls)==1 else y
            task.feedback((x % 2**32, actual_y % 2**32))
            return SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=True, state='action_succeeded')
        task.bot.robotic_arm = SimpleNamespace(moveto=move)
        def sleep(dt):
            clock[0] += dt
            task.feedback(task.position)
        with patch.object(pick.time,'monotonic',lambda:clock[0]), patch.object(pick.time,'sleep',sleep):
            task.feedback((159,4294967293))
            task.move('lift',0,19)
        self.assertEqual(calls,expected)
        self.assertIsNone(task.active)

    def test_home_horizontal_command_preserves_measured_height(self):
        clock, calls = [100.0], []
        task = pick.Pick(SimpleNamespace(), taught_config(), lambda *args, **kw: None)
        def moveto(x, y):
            calls.append((x, y))
            task.feedback((x, y))
            return SimpleNamespace(wait_for_completed=lambda timeout: True,
                                   has_succeeded=True, state='action_succeeded')
        task.bot.robotic_arm = SimpleNamespace(moveto=moveto)
        def sleep(dt):
            clock[0] += dt
            task.feedback(task.position)
        with patch.object(pick.time, 'monotonic', lambda: clock[0]), patch.object(pick.time, 'sleep', sleep):
            task.feedback((182, 80))
            task.move('home_traverse', -6, 0)
        self.assertEqual(calls, [(176, 80)])


if __name__ == '__main__':
    unittest.main()
