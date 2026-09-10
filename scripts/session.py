#!/usr/bin/env python3
"""Manage only the simulation process group launched from this checkout."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path(__file__).resolve().parent.parent
state = root/'work/session.json'


def alive(pid):
    try:
        os.kill(pid, 0)
        return (Path('/proc')/str(pid)/'stat').read_text().split(') ', 1)[1].split()[0] != 'Z'
    except (ProcessLookupError, FileNotFoundError):
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--run-task', action='store_true')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--display', default=os.environ.get('DISPLAY', ':2'))
    args = parser.parse_args()
    previous = json.loads(state.read_text()) if state.exists() else None
    if args.action == 'status':
        print({**(previous or {}), 'running': bool(previous and alive(previous['pid']))})
        return
    if previous and alive(previous['pid']):
        if args.action == 'start':
            raise SystemExit('This checkout already has a running session; stop it first.')
        # Avoid signalling a reused PID unrelated to this checkout.
        command = Path('/proc')/str(previous['pid'])/'cmdline'
        if ('ep_simulation' not in command.read_text().replace('\0', ' ') or
                (Path('/proc')/str(previous['pid'])/'cwd').resolve() != root):
            raise SystemExit('PID no longer belongs to this simulation; refusing to stop it.')
        os.killpg(previous['pid'], signal.SIGINT)
        for _ in range(100):
            if not alive(previous['pid']): break
            time.sleep(0.1)
        if alive(previous['pid']):
            raise SystemExit('Simulation has not stopped yet; keeping its session record.')
    if args.action == 'stop':
        if state.exists(): state.unlink()
        return
    (root/'work').mkdir(exist_ok=True)
    env = os.environ.copy()
    env['DISPLAY'] = args.display
    env.setdefault('XAUTHORITY', str(Path.home()/'.Xauthority'))
    log = root/'work/sim.log'
    with log.open('w') as handle:
        p = subprocess.Popen(['bash', str(root/'scripts/run_sim.sh'),
              'gui:='+str(not args.headless).lower(),
              'rviz:='+str(not args.headless).lower(),
              'run_task:='+str(args.run_task).lower(), 'cycles:='+str(args.cycles),
              'result_file:=work/bottle-five.json' if args.cycles == 5 else 'result_file:=work/pick_result.json'],
              cwd=root, env=env, stdout=handle, stderr=subprocess.STDOUT,
              start_new_session=True)
    state.write_text(json.dumps({'pid': p.pid, 'log': str(log), 'display': args.display}))
    print('Started simulation PID', p.pid, 'log:', log)


if __name__ == '__main__': main()
