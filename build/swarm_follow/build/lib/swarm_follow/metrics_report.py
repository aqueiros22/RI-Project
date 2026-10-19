"""Compute offline following and collision metrics from simulation samples."""

import argparse
import csv
import json
import math
from pathlib import Path


def read_samples(path):
    """Reject empty, truncated, reset or discontinuous recordings."""
    previous_end = None
    count = 0
    with Path(path).open(newline='') as stream:
        for row in csv.DictReader(stream):
            if None in row or any(value is None for value in row.values()):
                raise ValueError(f'Incomplete CSV row in {path}')
            time = float(row['time_s'])
            dt = float(row['dt_s'])
            if not math.isfinite(time) or not math.isfinite(dt) or dt <= 0:
                raise ValueError(f'Invalid simulation time in {path}')
            if previous_end is not None and not math.isclose(
                    time, previous_end, rel_tol=0, abs_tol=1e-6):
                raise ValueError(f'Time reset or missing samples in {path}')
            previous_end = time + dt
            count += 1
            yield row, time, dt
    if not count:
        raise ValueError(f'No samples in {path}')


def observation_metrics(samples, robot, mode):
    """Measure observed following episodes; do not infer target identity."""
    duration = visible_time = valid_time = clipped_time = 0.0
    following_time = error_abs = error_squared = 0.0
    losses = 0
    episodes = []
    active = None
    end = 0.0

    def close_episode(time, reason):
        nonlocal active
        episodes.append({
            'robot': robot, 'start_s': active, 'end_s': time,
            'duration_s': time - active, 'end_reason': reason,
            'right_censored': int(reason != 'target_loss'),
        })
        active = None

    for row, time, dt in samples:
        if row['robot'] != robot or row['mode'] != mode:
            raise ValueError('Robot or mode changed within a recording')
        if row['frame_valid'] not in ('0', '1') or row['visible'] not in (
                '0', '1'):
            raise ValueError('Invalid visibility flag')
        valid = row['frame_valid'] == '1'
        visible = row['visible'] == '1'
        if visible and not valid:
            raise ValueError('A missing frame cannot have a detected target')
        duration += dt
        valid_time += dt if valid else 0.0
        visible_time += dt if visible else 0.0
        end = time + dt
        following = mode != 'no_following' and visible
        if following:
            if active is None:
                active = time
            error = math.degrees(float(row['bearing_rad']))
            if not math.isfinite(error):
                raise ValueError('Non-finite angular error')
            following_time += dt
            error_abs += abs(error) * dt
            error_squared += error * error * dt
            clipped_time += dt if row['clipped'] == '1' else 0.0
        elif active is not None:
            reason = 'target_loss' if valid else 'missing_frame'
            losses += int(valid)
            close_episode(time, reason)
    if active is not None:
        close_episode(end, 'recording_end')
    lengths = [e['duration_s'] for e in episodes]
    return {
        'robot': robot, 'mode': mode, 'recorded_s': duration,
        'valid_camera_s': valid_time, 'visible_s': visible_time,
        'following_s': following_time,
        'following_percent': 100 * following_time / duration,
        'following_episodes': len(episodes),
        'following_mean_s': sum(lengths) / len(lengths) if lengths else '',
        'following_max_s': max(lengths) if lengths else '',
        'following_censored_episodes': sum(
            e['right_censored'] for e in episodes),
        'angular_mae_deg': (
            error_abs / following_time if following_time else ''),
        'angular_rmse_deg': (
            math.sqrt(error_squared / following_time)
            if following_time else ''),
        'following_clipped_s': clipped_time,
        'target_losses': losses,
        'losses_per_following_min': (
            60 * losses / following_time if following_time else ''),
    }, episodes


def collision_metrics(samples, robots):
    """Count episodes per unordered pair and avoid duplicate contact time."""
    stats = {name: {'robot': name, 'collision_count': 0, 'contact_s': 0.0,
                    'pair_contact_s': 0.0, 'collision_recorded_s': 0.0}
             for name in [*robots, 'ALL']}
    active = {}
    episodes = []
    end = 0.0

    def close_pair(pair, time, censored):
        start = active.pop(pair)
        episodes.append({
            'first': pair[0], 'second': pair[1], 'start_s': start,
            'end_s': time, 'duration_s': time - start,
            'right_censored': int(censored),
        })

    for row, time, dt in samples:
        pairs = set()
        for pair in json.loads(row['pairs_json']):
            if (len(pair) != 2 or pair[0] == pair[1]
                    or not any(name in robots for name in pair)):
                raise ValueError('Invalid collision pair')
            pairs.add(tuple(sorted(pair)))
        for pair in set(active) - pairs:
            close_pair(pair, time, False)
        for pair in pairs - set(active):
            active[pair] = time
            stats['ALL']['collision_count'] += 1
            for name in pair:
                if name in robots:
                    stats[name]['collision_count'] += 1
        involved = {name for pair in pairs for name in pair if name in robots}
        for name, values in stats.items():
            values['collision_recorded_s'] += dt
            values['contact_s'] += dt if (
                name in involved or name == 'ALL' and pairs) else 0.0
            values['pair_contact_s'] += dt * sum(
                name == 'ALL' or name in pair for pair in pairs)
        end = time + dt
    for pair in list(active):
        close_pair(pair, end, True)
    return stats, episodes


def write_csv(path, rows, fields):
    """Write a report even when there are no events."""
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def generate_report(directory):
    """Build reports only after all expected recordings validate."""
    directory = Path(directory)
    metadata = json.loads((directory / 'run.json').read_text())
    if metadata['schema_version'] != 1:
        raise ValueError('Unsupported recording schema')
    robots = metadata['robots']
    summaries, following_episodes = [], []
    for robot in robots:
        summary, episodes = observation_metrics(
            read_samples(directory / f'{robot}_observations.csv'),
            robot, metadata['mode'])
        summaries.append(summary)
        following_episodes.extend(episodes)
    stats, collision_episodes = collision_metrics(
        read_samples(directory / 'contacts.csv'), robots)
    for summary in summaries:
        summary.update(stats[summary['robot']])
    write_csv(directory / 'summary.csv', summaries, list(summaries[0]))
    write_csv(directory / 'following_episodes.csv', following_episodes, [
        'robot', 'start_s', 'end_s', 'duration_s', 'end_reason',
        'right_censored',
    ])
    write_csv(directory / 'collision_episodes.csv', collision_episodes, [
        'first', 'second', 'start_s', 'end_s', 'duration_s', 'right_censored',
    ])
    write_csv(directory / 'collision_summary.csv', stats.values(),
              list(stats['ALL']))
    return summaries


def main(args=None):
    """Generate reports after stopping a recorded simulation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path,
                        help='Run directory from launch')
    options = parser.parse_args(args)
    try:
        summaries = generate_report(options.directory)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(f'Reports written to {options.directory.resolve()}')
    for row in summaries:
        print(f"{row['robot']}: following={row['following_s']:.2f}s, "
              f"losses={row['target_losses']}, "
              f"collisions={row['collision_count']}")


if __name__ == '__main__':
    main()
