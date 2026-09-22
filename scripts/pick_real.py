#!/usr/bin/env python3
"""Supervised taught HOME -> pick A -> place B -> HOME sequence.

HOME is a taught pose, not a mechanical origin search or SDK recenter command.
This SDK entry does not yet implement the course's ROS 2 trajectory interface.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import time

ROOT = Path(__file__).resolve().parent.parent
POSITION_TOLERANCE_MM = 2
MAX_HORIZONTAL_SPAN_MM = 90  # Requested layout, not a hardware reachability limit.
MAX_VERTICAL_SPAN_MM = 120  # Ground pick vs raised HOME; not a joint-limit proof.
MAX_COMMAND_MM = 60


# 验证任务参数类型、单位和范围；执行运动前要求示教完整。
def load_task(path, require_calibration=False):
    cfg = json.loads(Path(path).read_text())
    cfg.setdefault('position_tolerance_mm', POSITION_TOLERANCE_MM)
    limits = {'water_ml': (0, 150), 'position_tolerance_mm': (2, 3),
              'release_clearance_mm': (2, 5), 'grip_power': (1, 30),
              'open_power': (1, 30), 'grip_seconds': (0.2, 2),
              'open_seconds': (0.2, 2), 'action_timeout_s': (3, 15)}
    for key, (low, high) in limits.items():
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('%s must be within [%s, %s]' % (key, low, high))
        if key.endswith('_mm') or key.endswith('_power'):
            if value != int(value):
                raise ValueError(key + ' must be an integer')
            cfg[key] = int(value)
    for key in ('home_raw_sdk_mm', 'pick_raw_sdk_mm', 'place_raw_sdk_mm'):
        point = cfg.get(key)
        if point is not None:
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError(key + ' must contain exactly two SDK readings')
            if any(isinstance(v, bool) or not isinstance(v, int) or not -2**31 <= v < 2**32 for v in point):
                raise ValueError(key + ' contains an invalid SDK integer')
    height = cfg.get('safe_y_raw_sdk_mm')
    if height is not None and (isinstance(height, bool) or not isinstance(height, int) or not -2**31 <= height < 2**32):
        raise ValueError('safe_y_raw_sdk_mm must be an SDK integer')
    if require_calibration:
        layout(cfg)
    return cfg


# 按 32 位回绕计算相对位移，返回毫米值，不代表绝对坐标标定。
def modular_delta(new, old):
    """Relative encoder displacement across uint32 wrap, not absolute calibration."""
    return tuple((int(a) - int(b) + 2**31) % 2**32 - 2**31 for a, b in zip(new, old))


# 以 A 点为原点构建示教布局，验证跨度与安全高度。
def layout(cfg):
    required = ('home_raw_sdk_mm', 'pick_raw_sdk_mm', 'place_raw_sdk_mm', 'safe_y_raw_sdk_mm')
    missing = [key for key in required if cfg.get(key) is None]
    if missing:
        raise ValueError('Missing taught poses: ' + ', '.join(missing) + '; use --teach home/pick/place/safe')
    anchor = cfg['pick_raw_sdk_mm']
    home = modular_delta(cfg['home_raw_sdk_mm'], anchor)
    place = modular_delta(cfg['place_raw_sdk_mm'], anchor)
    safe = modular_delta((anchor[0], cfg['safe_y_raw_sdk_mm']), anchor)[1]
    clearance = cfg['release_clearance_mm']
    if max(abs(home[1]), abs(place[1])) > MAX_VERTICAL_SPAN_MM or not 5 <= safe <= MAX_VERTICAL_SPAN_MM:
        raise ValueError('Taught layout exceeds the %s mm vertical workspace' % MAX_VERTICAL_SPAN_MM)
    if max(home[0], place[0], 0) - min(home[0], place[0], 0) > MAX_HORIZONTAL_SPAN_MM:
        raise ValueError('Horizontal span exceeds %s mm' % MAX_HORIZONTAL_SPAN_MM)
    if safe < max(home[1], place[1] + clearance, 0) or safe - min(home[1], place[1], 0) > MAX_VERTICAL_SPAN_MM:
        raise ValueError('Safe height must be above HOME, A and the release pose, within %s mm vertical span' % MAX_VERTICAL_SPAN_MM)
    return {'home': home, 'pick': (0, 0), 'place': place, 'safe_y': safe}


# 真机取放状态机：运动是否完成由 SDK 应答和新鲜位置反馈共同判断。
class Pick:
    # 保存设备、配置和日志回调，初始化反馈时间与序号。
    def __init__(self, bot, cfg, record):
        self.bot, self.cfg, self.record = bot, cfg, record
        self.tolerance = cfg.get("position_tolerance_mm", POSITION_TOLERANCE_MM)
        if self.tolerance not in (2, 3):
            raise ValueError("Position tolerance must be 2 or 3 mm")
        self.position = None
        self.position_time = 0
        self.position_sequence = 0
        self.active = None
        self.gripper_status = None
        self.gripper_status_time = 0

    # 记录夹爪状态及接收时刻，避免把旧的打开状态当作本次结果。
    def gripper_feedback(self, status):
        self.gripper_status = status
        self.gripper_status_time = time.monotonic()

    # 只接受整数毫米反馈，递增序号以区分命令前后的采样。
    def feedback(self, value):
        if len(value) == 2 and all(isinstance(v, (int, float)) and math.isfinite(v) and v == int(v) for v in value):
            self.position = tuple(value)
            self.position_time = time.monotonic()
            self.position_sequence += 1

    # 超过一秒的反馈不再用于生成后续运动。
    def fresh_position(self):
        if self.position is None or time.monotonic() - self.position_time > 1:
            raise RuntimeError('Arm feedback missing or stale; subsequent motion stopped')
        return self.position

    # 发送绝对终点命令，并等待实测位移在容差内持续稳定。
    def move(self, stage, x, y, allow_upward_correction=True):
        before = self.fresh_position()
        sequence_before = self.position_sequence
        deadline = time.monotonic() + self.cfg['action_timeout_s']
        # DDS reports uint32; moveto requires signed Cartesian millimetres.
        # Use explicit endpoints so every command also fixes the other axis.
        signed_before = modular_delta(before, (0, 0))
        target = tuple(v + d for v, d in zip(signed_before, (x, y)))
        if any(not -2**31 <= v < 2**31 for v in target):
            raise RuntimeError(stage + ': absolute target exceeds signed SDK range')
        self.record(stage + '_request', relative_x_mm=x, relative_y_mm=y,
                    command_mode='absolute_moveto', absolute_target_sdk_mm=target)
        self.active = self.bot.robotic_arm.moveto(x=target[0], y=target[1])
        self.record(stage + '_dispatched', action_id=getattr(self.active, '_action_id', None),
                    raw_before=before, feedback_sequence=sequence_before)
        done = self.active.wait_for_completed(timeout=self.cfg['action_timeout_s'])
        self.record(stage + '_action', state=self.active.state,
                    action_id=getattr(self.active, '_action_id', None),
                    sdk_percent=getattr(self.active, '_percent', None),
                    sdk_action_xy=[getattr(self.active, '_x', None), getattr(self.active, '_y', None)])
        if not done or not self.active.has_succeeded:
            raise RuntimeError(stage + ': SDK action failed or timed out')
        # A success reply is not proof the arm has settled. Require new samples
        # near the endpoint for >=0.4 s, instead of one fixed-delay snapshot.
        last_sequence, stable_since, stable_reference, samples = sequence_before, None, None, 0
        short_since, short_reference, short_samples = None, None, 0
        dx, dy = 0, 0
        while time.monotonic() < deadline:
            after = self.fresh_position()
            if self.position_sequence != last_sequence:
                last_sequence = self.position_sequence
                dx, dy = modular_delta(after, before)
                if (x == 0 and abs(dx) > 5) or (y == 0 and abs(dy) > 5):
                    raise RuntimeError(stage + ': unexpected movement on stationary axis; measured (%s, %s) mm' % (dx, dy))
                # One small upward correction only after a successful action and
                # a stable, measured 3-5 mm shortfall. Never retry a failed action,
                # a horizontal move, a descent, or a correction itself.
                shortfall = y - dy
                if allow_upward_correction and x == 0 and y > 0 and dy > 0 and abs(dx) <= 2 and self.tolerance < shortfall <= 5:
                    measured = (dx, dy)
                    if short_reference is None or max(abs(a-b) for a,b in zip(measured, short_reference)) > 1:
                        short_since, short_reference, short_samples = self.position_time, measured, 1
                    else:
                        short_samples += 1
                    if short_samples >= 4 and self.position_time - short_since >= 1:
                        self.record(stage + '_upward_correction', shortfall_mm=shortfall, measured_mm=measured)
                        self.active = None
                        self.move(stage + '_correction', 0, shortfall, allow_upward_correction=False)
                        after = self.fresh_position()
                        total = modular_delta(after, before)
                        if max(abs(a-b) for a,b in zip(total, (x,y))) > self.tolerance:
                            raise RuntimeError(stage + ': still outside tolerance after one upward correction')
                        self.record(stage + '_feedback', raw_before=before, raw_after=after,
                                    dx_mm=total[0], dy_mm=total[1], corrected=True)
                        return
                else:
                    short_since, short_reference, short_samples = None, None, 0
                if abs(dx - x) <= self.tolerance and abs(dy - y) <= self.tolerance:
                    measured = (dx, dy)
                    if stable_reference is None or max(abs(a-b) for a,b in zip(measured, stable_reference)) > 1:
                        stable_since, stable_reference, samples = self.position_time, measured, 1
                    else:
                        samples += 1
                    if samples >= 3 and self.position_time - stable_since >= 0.4:
                        self.record(stage + '_feedback', raw_before=before, raw_after=after,
                                    dx_mm=dx, dy_mm=dy, settled_samples=samples)
                        self.active = None
                        return
                else:
                    stable_since, stable_reference, samples = None, None, 0
            time.sleep(0.05)
        self.record(stage + '_endpoint_timeout', requested_delta_mm=(x, y),
                    measured_delta_mm=(dx, dy), raw_before=before, raw_after=self.position,
                    absolute_target_sdk_mm=target, feedback_samples=self.position_sequence-sequence_before)
        raise RuntimeError(stage + ': endpoint did not settle; requested (%s, %s), measured (%s, %s) mm' % (x, y, dx, dy))

    # 启动前等待新鲜机械臂反馈，按任务需要同时等待夹爪反馈。
    def wait_ready(self, timeout=3, need_gripper=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            arm_ready = self.position is not None and now - self.position_time <= 1
            gripper_ready = self.gripper_status is not None and now - self.gripper_status_time <= 1
            if arm_ready and (not need_gripper or gripper_ready):
                return
            time.sleep(0.05)
        raise RuntimeError('Startup feedback timeout: arm or gripper status missing; no motion sent')

    # 限时驱动夹爪后暂停；释放还需本次命令之后的完全打开反馈。
    def jaws(self, opening, stage=None):
        stage = stage or ('release' if opening else 'grip')
        power = self.cfg['open_power' if opening else 'grip_power']
        self.record(stage + '_request', power=power)
        command = self.bot.gripper.open if opening else self.bot.gripper.close
        requested_at = time.monotonic()
        if not command(power=power):
            raise RuntimeError(stage + ': gripper rejected command')
        time.sleep(self.cfg['open_seconds' if opening else 'grip_seconds'])
        if not self.bot.gripper.pause():
            raise RuntimeError(stage + ': gripper pause not acknowledged')
        if opening and (self.gripper_status != 'opened' or
                        self.gripper_status_time < requested_at or
                        time.monotonic() - self.gripper_status_time > 1):
            raise RuntimeError(stage + ': fresh fully-open gripper feedback not received; no return motion')
        self.record(stage + '_command_complete')

    # 将当前原始 SDK 坐标换算为相对示教 A 点的毫米坐标。
    def relative_position(self):
        return modular_delta(self.fresh_position(), self.cfg['pick_raw_sdk_mm'])

    # 每次只移动一个轴；长横移拆段并在第二段前恢复安全高度。
    def goto(self, stage, x=None, y=None):
        current = self.relative_position()
        target = (current[0] if x is None else x, current[1] if y is None else y)
        delta = (target[0] - current[0], target[1] - current[1])
        if (delta[0] and delta[1]) or abs(delta[0]) > MAX_HORIZONTAL_SPAN_MM + self.tolerance or abs(delta[1]) > MAX_VERTICAL_SPAN_MM + self.tolerance:
            raise RuntimeError('Unsafe leg: require one axis, horizontal <=%s mm, vertical <=%s mm' % (
                MAX_HORIZONTAL_SPAN_MM, MAX_VERTICAL_SPAN_MM))
        if max(abs(v) for v in delta) <= self.tolerance:
            self.record(stage + '_already_at_target', target_mm=target, error_mm=delta)
            return
        if abs(delta[0]) > MAX_COMMAND_MM + self.tolerance:
            safe = layout(self.cfg)['safe_y']
            if current[1] < safe - self.tolerance:
                raise RuntimeError(stage + ': segmented traverse requires safe height')
            step = MAX_COMMAND_MM if delta[0] > 0 else -MAX_COMMAND_MM
            self.goto(stage + '_segment_1', x=current[0] + step)
            # Do not carry downward endpoint error into the next horizontal leg.
            # Restore the configured safe height first; failure aborts segment 2.
            self.goto(stage + '_restore_height', y=safe)
            self.goto(stage + '_segment_2', x=target[0])
        elif abs(delta[1]) > MAX_COMMAND_MM + self.tolerance:
            step = MAX_COMMAND_MM if delta[1] > 0 else -MAX_COMMAND_MM
            self.goto(stage + '_segment_1', y=current[1] + step)
            self.goto(stage + '_segment_2', y=target[1])
        else:
            # Cap boundary corrections but retain the taught endpoint check.
            command = tuple(max(-MAX_COMMAND_MM, min(MAX_COMMAND_MM, v)) for v in delta)
            self.move(stage, *command)
        actual = self.relative_position()
        if max(abs(a - b) for a, b in zip(actual, target)) > self.tolerance:
            raise RuntimeError(stage + ': taught endpoint error exceeds %s mm' % self.tolerance)

    # 依次回 HOME、抓取、搬运、释放、返回；任一步异常立即中断。
    def run(self, home_only=False, check_path=False):
        points = layout(self.cfg)
        home, place, safe = points['home'], points['place'], points['safe_y']
        current = self.relative_position()
        # Reject an unexpected startup pose before any jaw or arm command.
        xmin, xmax = min(home[0], place[0], 0), max(home[0], place[0], 0)
        ymin = min(home[1], place[1], 0)
        if not xmin - 5 <= current[0] <= xmax + 5 or not ymin - 5 <= current[1] <= safe + 5:
            raise RuntimeError('Startup pose outside taught area; no automatic homing')
        self.record('preflight', current_relative_to_A_mm=current, taught_layout=points,
                    position_tolerance_mm=self.tolerance)
        # Start empty. Bottle may be between the jaws but must rest on the floor.
        self.jaws(True, 'open_before_home')
        self.goto('home_raise', y=safe)
        self.goto('home_traverse', x=home[0])
        self.goto('home', y=home[1])
        if home_only:
            self.record('home_complete', relative_to_A_mm=self.relative_position())
            return
        self.goto('approach_raise', y=safe)
        self.goto('approach_A', x=0)
        self.goto('descend_A', y=0)
        if check_path:
            self.record('empty_path_skip_grip')
        else:
            self.jaws(False)
        self.goto('lift', y=safe)
        self.goto('transfer_B', x=place[0])
        self.goto('lower_B', y=place[1] + self.cfg['release_clearance_mm'])
        self.jaws(True, 'release_at_B')
        self.goto('withdraw_B', y=safe)
        self.goto('return_traverse', x=home[0])
        self.goto('return_HOME', y=home[1])
        self.record('returned_home', relative_to_A_mm=self.relative_position())

    # 尽力发送设备取消和夹爪暂停请求，日志不将其视为已确认物理停止。
    def stop_requests(self):
        # Request device cancellation, not SDK Action._abort(), which is local-only.
        # Pattern documented by jeguzzi/robomaster_ros; physical stop is unverified.
        if self.active is not None:
            try:
                message = self.bot.action_dispatcher.get_msg_by_action(self.active)
                message._proto._action_ctrl = 1
                self.bot.client.send_msg(message)
                self.record('cancel_requested', physical_stop_confirmed=False)
            except Exception as error:
                self.record('cancel_error', error=str(error))
        try:
            self.record('gripper_pause_requested', acknowledged=bool(self.bot.gripper.pause()))
        except Exception as error:
            self.record('gripper_pause_error', error=str(error))


# 默认只预览；显式示教或执行时才建立 SDK 连接。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    local_config = ROOT / 'config/real_pick.local.json'
    parser.add_argument('--config', type=Path, default=local_config if local_config.exists() else ROOT / 'config/real_pick.json')
    parser.add_argument('--connection', type=Path, default=ROOT / 'config/ep_connection.json')
    parser.add_argument('--execute', action='store_true', help='Run the full physical sequence; operator must be present')
    parser.add_argument('--task', choices=['pick', 'home', 'release', 'check-path'], default='pick', help='Full cycle, empty HOME check, supported release, or empty full path with jaws open')
    parser.add_argument('--object-supported', action='store_true', help='For release only: bottle is on floor or securely supported by the operator')
    parser.add_argument('--teach', choices=['home', 'pick', 'place', 'safe'], help='Read current arm pose into the task configuration; no movement')
    parser.add_argument('--position-tolerance-mm', type=int, choices=(2, 3), default=None,
                        help='Override configured endpoint tolerance (2 or 3 mm); old configs default to 2')
    args = parser.parse_args()
    if args.teach and args.execute:
        parser.error('--teach and --execute cannot be combined')
    if args.execute and args.task == 'release' and not args.object_supported:
        parser.error('Support the bottle first, then pass --object-supported; no commands sent')
    try:
        cfg = load_task(args.config, require_calibration=args.execute and args.task != 'release')
        if args.position_tolerance_mm is not None:
            cfg['position_tolerance_mm'] = args.position_tolerance_mm
    except (ValueError, KeyError) as error:
        parser.error(str(error))
    if not args.execute and not args.teach:
        print('预览，不连接机器人。HOME/A/B/安全高度均须记录并核对通路。')
        print(json.dumps(cfg, indent=2))
        print('回HOME → A上方 → 下降夹取 → 抬升 → B放置 → 撤离 → 返回HOME。执行需加 --execute。')
        print('本次任务:', args.task)
        try:
            print('已记录布局（相对A，毫米）:', layout(cfg))
        except ValueError as error:
            print('待标定:', error)
        return 0
    from check_ep_connection import load_config
    connection = load_config(args.connection)
    from robomaster import config, robot
    if connection.get('local_ip'):
        config.LOCAL_IP_STR = connection['local_ip']
    else:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect((connection['robot_ip'], 20020))
            config.LOCAL_IP_STR = route.getsockname()[0]
    config.ROBOT_IP_STR = connection['robot_ip']
    work = ROOT / 'work'
    work.mkdir(exist_ok=True)
    lock = (work / 'real-control.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    output = work / ('real-pick-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '.json')
    report = {'commands_completed': False, 'physical_grasp_success': None,
              'config': cfg, 'events': [], 'operator_observation_required': True,
              'mode': 'teach_' + args.teach if args.teach else args.task,
              'motion_commands_sent': False, 'teach_completed': False}

    def record(stage, **data):
        item = dict(stage=stage, time_utc=datetime.now(timezone.utc).isoformat(), **data)
        if stage.endswith('_request') and any(k in data for k in ('relative_x_mm', 'power')):
            report['motion_commands_sent'] = True
        report['events'].append(item)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('Operator or process supervisor requested stop')

    signal.signal(signal.SIGTERM, interrupted)
    bot, task, subscribed, gripper_subscribed = robot.Robot(), None, False, False
    record('starting')
    try:
        if not bot.initialize(conn_type='ap', proto_type='udp'):
            raise RuntimeError('SDK connection failed')
        task = Pick(bot, cfg, record)
        if args.task != 'release' or args.teach:
            subscribed = bool(bot.robotic_arm.sub_position(freq=5, callback=task.feedback))
            if not subscribed:
                raise RuntimeError('Arm feedback subscription failed')
        if not args.teach:
            gripper_subscribed = bool(bot.gripper.sub_status(freq=5, callback=task.gripper_feedback))
            if not gripper_subscribed:
                raise RuntimeError('Gripper status subscription failed')
        if args.task != 'release' or args.teach:
            task.wait_ready(need_gripper=not args.teach)
        if args.teach:
            first = task.fresh_position()
            time.sleep(0.6)
            raw = task.fresh_position()
            if max(abs(v) for v in modular_delta(raw, first)) > 2:
                raise RuntimeError('Arm moved while teaching; configuration was not saved')
            key = 'safe_y_raw_sdk_mm' if args.teach == 'safe' else args.teach + '_raw_sdk_mm'
            # Preserve other fields and leave a backup before changing taught points.
            saved = json.loads(args.config.read_text())
            destination = local_config if args.config.resolve() == (ROOT / 'config/real_pick.json').resolve() else args.config
            backup = destination.with_name(destination.name + '.bak')
            if destination.exists():
                backup.write_text(destination.read_text())
            saved[key] = int(raw[1]) if args.teach == 'safe' else [int(v) for v in raw]
            temporary = destination.with_name(destination.name + '.tmp')
            temporary.write_text(json.dumps(saved, ensure_ascii=False, indent=2) + '\n')
            temporary.replace(destination)
            report['teach_completed'] = True
            record('pose_saved', point=args.teach, raw_sdk_mm=raw, config_path=str(destination))
        else:
            if args.task == 'release':
                task.jaws(True, 'supported_release')
            else:
                task.run(home_only=args.task == 'home', check_path=args.task == 'check-path')
            report['commands_completed'] = True
            notices = {'release': 'Gripper opened; no arm move requested',
                       'home': 'HOME command complete; verify physical pose',
                       'check-path': 'Empty path complete; no close command sent; does not prove loaded grasp success',
                       'pick': 'Returned HOME; operator must verify bottle lifted, placed upright and undamaged'}
            record('commands_complete', notice=notices[args.task])
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error)
        record('stopped', error=str(error), notice='No automatic return or release. Cut power if motion continues.')
        if task and not args.teach and report['motion_commands_sent']:
            task.stop_requests()
    finally:
        cleanup = []
        if gripper_subscribed:
            cleanup.append(bot.gripper.unsub_status)
        if subscribed:
            cleanup.append(bot.robotic_arm.unsub_position)
        cleanup.append(bot.close)
        for close_resource in cleanup:
            try:
                close_resource()
            except Exception as error:
                record('cleanup_error', error=str(error))
        record('finished')
        print('结果文件:', output, flush=True)
    return 0 if report['commands_completed'] or report['teach_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
