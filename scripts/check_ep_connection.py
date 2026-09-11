#!/usr/bin/env python3
"""Check EP SDK communication and arm feedback without motion commands.

SDK initialization enables SDK mode, resets subscriptions and selects FREE mode.
This is a communication check, not a physical homing or movement test.
"""
import argparse
from datetime import datetime, timezone
import ipaddress
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parent.parent


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    for key in ('robot_ip', 'local_ip'):
        if key == 'local_ip' and cfg.get(key) is None:
            continue
        ipaddress.IPv4Address(cfg[key])
    for key in ('position_timeout_s', 'total_timeout_s'):
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + ' must be a positive finite number')
    if cfg['total_timeout_s'] < cfg['position_timeout_s'] + 10:
        raise ValueError('total_timeout_s must allow at least 10 seconds for connection and cleanup')
    return cfg


def observe(bot, timeout, report):
    """Only initialize/query/subscribe/unsubscribe/close; injectable for tests."""
    subscribed = False
    samples = []
    ready = threading.Event()

    def receive(position):
        try:
            x, y = map(float, position)
            if not all(math.isfinite(v) for v in (x, y)):
                return
            samples.append({'x_mm': x, 'y_mm': y})
            if len(samples) >= 2:
                ready.set()
        except (TypeError, ValueError):
            return

    try:
        report['stage'] = 'connect'
        if not bot.initialize(conn_type='ap', proto_type='udp'):
            raise RuntimeError('EP SDK connection failed')
        report['connection_ok'] = True
        report['stage'] = 'version'
        report['robot_version'] = bot.get_version()
        if not report['robot_version']:
            raise RuntimeError('Connected but no robot version received')
        report['stage'] = 'arm_feedback'
        subscribed = bool(bot.robotic_arm.sub_position(freq=5, callback=receive))
        if not subscribed:
            raise RuntimeError('EP rejected arm position subscription')
        if not ready.wait(timeout):
            raise RuntimeError('No two valid arm position samples received before timeout')
        report['arm_position_raw_sdk_mm'] = samples[-1]
        report['position_samples_received'] = len(samples)
        report['stage'] = 'arm_feedback_validation'
        # A coarse sanity check, NOT a calibrated workspace or motion limit.
        # SDK 0.1.1.62 decodes position with <II. Preserve suspicious raw values.
        if any(abs(v) > 1000 for v in samples[-1].values()):
            report['position_valid'] = False
            report['signed_int32_candidate_mm'] = {
                k: (v - 2**32 if v.is_integer() and 2**31 <= v < 2**32 else v)
                for k, v in samples[-1].items()}
            raise RuntimeError('Implausible raw arm coordinate; possible SDK unsigned decoding. Candidate is unverified; do not use it for motion.')
        report['arm_position_mm'] = samples[-1]
        report['position_valid'] = True
        report['stage'] = 'complete'
        report['success'] = True
    finally:
        try:
            if subscribed:
                bot.robotic_arm.unsub_position()
        finally:
            bot.close()


def probe(cfg, report):
    report['stage'] = 'sdk_import'
    from robomaster import config, robot, version
    report['sdk_version'] = version.__version__
    local_ip = cfg.get('local_ip')
    if not local_ip:
        # UDP connect here selects a local route; it does not send a packet.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect((cfg['robot_ip'], 20020))
            local_ip = route.getsockname()[0]
    config.LOCAL_IP_STR = local_ip
    config.ROBOT_IP_STR = cfg['robot_ip']
    report['local_ip'] = local_ip
    report['robot_ip'] = cfg['robot_ip']
    observe(robot.Robot(), cfg['position_timeout_s'], report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/ep_connection.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'work/ep-connection.json')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    report = {'success': False, 'checked_at_utc': datetime.now(timezone.utc).isoformat(),
              'motion_commands_sent': False, 'stage': 'config',
              'connection_ok': False, 'position_valid': False,
              'scope': 'SDK connection and position feedback only; no movement or homing'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        cfg = load_config(args.config)
        if args.worker:
            probe(cfg, report)
        else:
            # Bound even hangs in this older SDK's connection or close routines.
            with tempfile.TemporaryDirectory(prefix='ep-probe-') as temporary:
                result_path = Path(temporary) / 'result.json'
                command = [sys.executable, str(Path(__file__).resolve()), '--worker',
                           '--config', str(args.config.resolve()), '--output', str(result_path)]
                report['stage'] = 'sdk_probe'
                log_path = args.output.with_suffix('.log')
                with log_path.open('w') as log:
                    completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                               timeout=cfg['total_timeout_s'])
                if not result_path.exists():
                    raise RuntimeError('SDK process exited without a report; see ' + str(log_path))
                report = json.loads(result_path.read_text())
                if completed.returncode != 0:
                    report['success'] = False
    except subprocess.TimeoutExpired:
        report.update(success=False, error='SDK check timed out; subprocess terminated. Check EP power, Wi-Fi and other SDK sessions.')
    except Exception as error:
        report.update(success=False, error=str(error))
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    if not args.worker:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print('结果文件:', args.output)
    return 0 if report['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
