#!/usr/bin/env python3
"""
Run every mode in several scenarios with one command.

Each scenario is a seed: the same seed gives the same starting poses, so the
three modes are compared in identical conditions. Results go to
metrics/<experiment>/seed<N>_<mode>/summary.csv.

Run from the repository root with the workspace sourced, for example:
    python3 scripts/run_experiments.py --scenarios 5 --duration 60
    python3 scripts/run_experiments.py --scenarios 3 --parallel --gui
"""

import argparse
import csv
from datetime import datetime
import os
import signal
import subprocess
import sys
import time

MODES = ['no_following', 'weak_turning', 'strong_turning']
BASE_PORT = 1234
# Parallel simulations get their own ROS domain so their topics never mix.
BASE_DOMAIN_ID = 40


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--scenarios', type=int, default=3,
        help='number of different starting positions (seeds 1..N)')
    parser.add_argument(
        '--seeds', type=int, nargs='+',
        help='exact seeds to run instead of 1..N (0 = poses of arena.wbt)')
    parser.add_argument(
        '--modes', nargs='+', choices=MODES, default=MODES,
        help='modes to compare (default: all three)')
    parser.add_argument(
        '--duration', type=float, default=60,
        help='simulated seconds per run (default: 60)')
    parser.add_argument(
        '--gui', action='store_true',
        help='show the Webots windows (default: no window, faster)')
    parser.add_argument(
        '--parallel', action='store_true',
        help='run the modes of each scenario at the same time')
    parser.add_argument(
        '--name', default=f'exp_{datetime.now():%Y%m%d-%H%M%S}',
        help='experiment folder inside metrics/')
    return parser.parse_args()


def start(mode, seed, slot, args, out_dir):
    """Start one ros2 launch in its own process group."""
    run_name = f'seed{seed}_{mode}'
    log = open(os.path.join(out_dir, f'{run_name}.log'), 'w')
    env = dict(os.environ)
    if args.parallel:
        env['ROS_DOMAIN_ID'] = str(BASE_DOMAIN_ID + slot)
    command = [
        'ros2', 'launch', 'swarm_follow', 'swarm_launch.py',
        f'mode:={mode}', f'seed:={seed}', f'duration:={args.duration:g}',
        f'gui:={"true" if args.gui else "false"}', 'webots_mode:=fast',
        f'port:={BASE_PORT + slot}', f'metrics_dir:={out_dir}',
        f'run_name:={run_name}',
    ]
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    return run_name, proc, log


def stop(proc):
    """Ask a launch to shut down, then force it if it does not."""
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)


def wait_all(runs, timeout):
    """Wait for the runs; stop any still running after timeout seconds."""
    deadline = time.monotonic() + timeout
    for _, proc, _ in runs:
        try:
            proc.wait(timeout=max(1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            stop(proc)
    for _, _, log in runs:
        log.close()


def finished(out_dir, name, duration):
    """Return True if the run's summary covers the whole duration."""
    path = os.path.join(out_dir, name, 'summary.csv')
    if not os.path.exists(path):
        return False
    with open(path, newline='') as file:
        rows = [r for r in csv.DictReader(file) if r['robot'] == 'all']
    return bool(rows) and float(rows[0]['duration_s']) >= duration - 0.1


def main():
    args = parse_args()
    # A wrong WEBOTS_HOME makes every launch wait for a Webots that never
    # starts, so stop here with a clear message instead.
    webots_home = os.environ.get('WEBOTS_HOME')
    if webots_home:
        with open('/proc/version') as file:
            wsl = 'microsoft' in file.read().lower()
        if wsl and not os.environ.get('WEBOTS_ROS2_LINUX_WEBOTS'):
            webots = os.path.join(
                webots_home, 'msys64', 'mingw64', 'bin', 'webots.exe')
        else:
            webots = os.path.join(webots_home, 'webots')
        if not os.path.exists(webots):
            sys.exit(f'Webots not found at {webots}. Fix WEBOTS_HOME '
                     f'(now {webots_home}, see README) and try again.')
    seeds = args.seeds or list(range(1, args.scenarios + 1))
    out_dir = os.path.abspath(os.path.join('metrics', args.name))
    os.makedirs(out_dir, exist_ok=True)
    # Generous wall-clock limit: Webots on WSL can run at ~0.5x real time.
    timeout = 120 + 10 * args.duration
    if args.parallel:
        batches = [[(mode, seed) for mode in args.modes] for seed in seeds]
    else:
        batches = [[(mode, seed)] for seed in seeds for mode in args.modes]
    total = len(seeds) * len(args.modes)
    print(f'{total} simulations ({len(seeds)} scenarios x '
          f'{len(args.modes)} modes), {args.duration:g} s each -> {out_dir}')

    done, failed, runs = 0, [], []
    try:
        for batch in batches:
            runs = [start(mode, seed, slot, args, out_dir)
                    for slot, (mode, seed) in enumerate(batch)]
            names = ', '.join(name for name, _, _ in runs)
            print(f'[{done + 1}-{done + len(runs)}/{total}] {names} ...',
                  flush=True)
            wait_all(runs, timeout)
            failed += [name for name, _, _ in runs
                       if not finished(out_dir, name, args.duration)]
            done += len(runs)
    except KeyboardInterrupt:
        print('\nInterrupted, closing the simulations...')
        for _, proc, _ in runs:
            stop(proc)
        sys.exit(1)

    print(f'Finished: {total - len(failed)}/{total} complete in {out_dir}')
    for name in failed:
        print(f'  FAILED {name}: see {os.path.join(out_dir, name)}.log')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
