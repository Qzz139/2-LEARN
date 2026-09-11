#!/usr/bin/env python3
"""Supervised SDK bring-up: bottle starts between open jaws, resting on floor.

No homing, chassis motion, repeated trials or automatic failure recovery.
This entry does not yet implement the course's ROS 2 trajectory interface.
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


def load_task(path):
    cfg = json.loads(Path(path).read_text())
    limits = {'water_ml': (0, 150), 'lift_mm': (5, 30), 'forward_mm': (0, 30),
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
    if cfg['release_clearance_mm'] >= cfg['lift_mm']:
        raise ValueError('Release clearance must be smaller than lift')
    return cfg


def modular_delta(new, old):
    """Relative encoder displacement across uint32 wrap, not absolute calibration."""
    return tuple((int(a) - int(b) + 2**31) % 2**32 - 2**31 for a, b in zip(new, old))


class Pick:
    def __init__(self, bot, cfg, record):
        self.bot, self.cfg, self.record = bot, cfg, record
        self.position = None
        self.position_time = 0
        self.active = None

    def feedback(self, value):
        if len(value) == 2 and all(isinstance(v, (int, float)) and math.isfinite(v) and v == int(v) for v in value):
            self.position = tuple(value)
            self.position_time = time.monotonic()

    def fresh_position(self):
        if self.position is None or time.monotonic() - self.position_time > 1:
            raise RuntimeError('Arm feedback missing or stale; subsequent motion stopped')
        return self.position

    def move(self, stage, x, y):
        before = self.fresh_position()
        self.record(stage + '_request', relative_x_mm=x, relative_y_mm=y)
        self.active = self.bot.robotic_arm.move(x=x, y=y)
        done = self.active.wait_for_completed(timeout=self.cfg['action_timeout_s'])
        self.record(stage + '_action', state=self.active.state)
        if not done or not self.active.has_succeeded:
            raise RuntimeError(stage + ': SDK action failed or timed out')
        self.active = None
        time.sleep(0.5)
        after = self.fresh_position()
        dx, dy = modular_delta(after, before)
        self.record(stage + '_feedback', raw_before=before, raw_after=after, dx_mm=dx, dy_mm=dy)
        if abs(dx - x) > 5 or abs(dy - y) > 5:
            raise RuntimeError(stage + ': arm displacement differs from request by more than 5 mm')

    def jaws(self, opening):
        stage = 'release' if opening else 'grip'
        power = self.cfg['open_power' if opening else 'grip_power']
        self.record(stage + '_request', power=power)
        command = self.bot.gripper.open if opening else self.bot.gripper.close
        if not command(power=power):
            raise RuntimeError(stage + ': gripper rejected command')
        time.sleep(self.cfg['open_seconds' if opening else 'grip_seconds'])
        if not self.bot.gripper.pause():
            raise RuntimeError(stage + ': gripper pause not acknowledged')
        self.record(stage + '_command_complete')

    def run(self):
        self.fresh_position()
        self.jaws(False)
        self.move('lift', 0, self.cfg['lift_mm'])
        if self.cfg['forward_mm']:
            self.move('transfer', self.cfg['forward_mm'], 0)
        self.move('lower', 0, -(self.cfg['lift_mm'] - self.cfg['release_clearance_mm']))
        self.jaws(True)

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/real_pick.json')
    parser.add_argument('--connection', type=Path, default=ROOT / 'config/ep_connection.json')
    parser.add_argument('--execute', action='store_true', help='Send physical commands; operator must be present')
    args = parser.parse_args()
    cfg = load_task(args.config)
    if not args.execute:
        print('预览，不连接机器人。瓶子须已在张开的夹爪之间、放稳地面。')
        print(json.dumps(cfg, indent=2))
        print('夹持 → 抬升 → 前移 → 下降并留间隙 → 松爪。执行需加 --execute。')
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
              'config': cfg, 'events': [], 'operator_observation_required': True}

    def record(stage, **data):
        item = dict(stage=stage, time_utc=datetime.now(timezone.utc).isoformat(), **data)
        report['events'].append(item)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def interrupted(signum, frame):
        raise KeyboardInterrupt('Operator or process supervisor requested stop')

    signal.signal(signal.SIGTERM, interrupted)
    bot, task, subscribed = robot.Robot(), None, False
    record('starting')
    try:
        if not bot.initialize(conn_type='ap', proto_type='udp'):
            raise RuntimeError('SDK connection failed')
        task = Pick(bot, cfg, record)
        subscribed = bool(bot.robotic_arm.sub_position(freq=5, callback=task.feedback))
        if not subscribed:
            raise RuntimeError('Arm feedback subscription failed')
        time.sleep(0.6)
        task.run()
        report['commands_completed'] = True
        record('commands_complete', notice='Operator must verify bottle lifted, placed upright and undamaged')
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error)
        record('stopped', notice='No automatic return or release. Cut power if motion continues.')
        if task:
            task.stop_requests()
    finally:
        try:
            if subscribed:
                bot.robotic_arm.unsub_position()
            bot.close()
        except Exception as error:
            record('cleanup_error', error=str(error))
        record('finished')
        print('结果文件:', output, flush=True)
    return 0 if report['commands_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
