#!/usr/bin/env python3
"""
Build the article figures from the metrics CSVs.

Every folder in metrics/ with a timeseries.csv for the three modes is a
scenario. By default only two summary figures are written:
    trajectories_overview.pdf  robot paths, one row per scenario and one
                               column per mode
    summary_scenarios.pdf      following time, angular error and collisions
                               per scenario and mode
With --detail, each scenario also gets its own trajectories and bearing
figures.

Run from anywhere:
    python3 report/make_figures.py                     # every scenario
    python3 report/make_figures.py cenario_2 test      # only these
    python3 report/make_figures.py --detail
Figures go to report/figures/.
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
METRICS = os.path.join(ROOT, 'metrics')
OUT = os.path.join(ROOT, 'report', 'figures')

MODES = ['no_following', 'weak_turning', 'strong_turning']
TITLES = {'no_following': 'No following', 'weak_turning': 'Weak turning',
          'strong_turning': 'Strong turning'}
MODE_COLORS = {'no_following': '#7f7f7f', 'weak_turning': '#ff7f0e',
               'strong_turning': '#1f77b4'}
ROBOTS = [f'robot{i}' for i in range(1, 6)]
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd', '#d62728']


def run_dir(scenario, mode):
    return os.path.join(METRICS, scenario, f'seed0_{mode}')


def find_scenarios():
    """Return the folders of metrics/ with a timeseries for every mode."""
    return sorted(
        name for name in os.listdir(METRICS)
        if all(os.path.exists(os.path.join(run_dir(name, m),
                                           'timeseries.csv'))
               for m in MODES))


def load(scenario, mode):
    """Return {robot: {field: [values]}} from a timeseries.csv."""
    data = {r: {'t': [], 'x': [], 'y': [], 'bearing': []} for r in ROBOTS}
    path = os.path.join(run_dir(scenario, mode), 'timeseries.csv')
    with open(path, newline='') as file:
        for row in csv.DictReader(file):
            d = data[row['robot']]
            d['t'].append(float(row['t']))
            d['x'].append(float(row['x']))
            d['y'].append(float(row['y']))
            d['bearing'].append(
                float(row['bearing']) if row['following'] == '1'
                else math.nan)
    return data


def summary(scenario, mode):
    """Return {robot: row} from a summary.csv, including 'all'."""
    path = os.path.join(run_dir(scenario, mode), 'summary.csv')
    with open(path, newline='') as file:
        return {row['robot']: row for row in csv.DictReader(file)}


def draw_paths(ax, data):
    """Draw the arena and the five robot paths on one axis."""
    ax.plot([-1.5, 1.5, 1.5, -1.5, -1.5], [-1.5, -1.5, 1.5, 1.5, -1.5],
            color='grey', lw=0.8)
    for robot, color in zip(ROBOTS, COLORS):
        d = data[robot]
        ax.plot(d['x'], d['y'], color=color, lw=0.8,
                label=robot.replace('robot', 'R'))
        ax.plot(d['x'][0], d['y'][0], 'o', color=color, ms=3)
        ax.plot(d['x'][-1], d['y'][-1], 'x', color=color, ms=3)
    ax.set_aspect('equal')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.6, 1.6)


def trajectories_overview(scenarios):
    """All scenarios in one figure: a row per scenario, a column per mode."""
    rows = len(scenarios)
    fig, axes = plt.subplots(rows, 3, figsize=(5.4, 1.75 * rows + 0.5),
                             squeeze=False)
    for r, scenario in enumerate(scenarios):
        for c, mode in enumerate(MODES):
            ax = axes[r][c]
            draw_paths(ax, load(scenario, mode))
            ax.set_xticks([])
            ax.set_yticks([])
            if r == 0:
                ax.set_title(TITLES[mode], fontsize=8)
            if c == 0:
                ax.set_ylabel(scenario, fontsize=8)
    handles = [plt.Line2D([], [], color=c, lw=2) for c in COLORS]
    labels = [r.replace('robot', 'R') for r in ROBOTS]
    fig.legend(handles, labels, loc='lower center', ncol=5, fontsize=7,
               frameon=False)
    fig.tight_layout(rect=(0, 0.3 / (1.75 * rows + 0.5), 1, 1), h_pad=0.3,
                     w_pad=0.3)
    fig.savefig(os.path.join(OUT, 'trajectories_overview.pdf'))
    plt.close(fig)


def _collisions(row):
    return int(row['robot_collisions']) + int(row['wall_collisions'])


def summary_bars(scenarios):
    """Swarm following time, angular error and collisions per scenario."""
    fig, axes = plt.subplots(1,2, figsize=(7.2, 2.7))
    # Following time and bearing do not exist without following.
    panels = [
        (lambda r: float(r['following_pct']), 'following time (%)',
         MODES[1:]),
        (lambda r: float(r['mean_abs_bearing_deg'] or 0),
         'mean |bearing| (deg)', MODES[1:]),
      #  (_collisions, 'collisions (robot + wall)', MODES),
    ]
    for ax, (value, label, modes) in zip(axes, panels):
        width = 0.8 / len(modes)
        for k, mode in enumerate(modes):
            offset = (k - (len(modes) - 1) / 2) * width
            ax.bar([i + offset for i in range(len(scenarios))],
                   [value(summary(s, mode)['all']) for s in scenarios],
                   width, color=MODE_COLORS[mode], label=TITLES[mode])
        ax.set_xticks(range(len(scenarios)))
        ax.set_xticklabels(scenarios, fontsize=7, rotation=40, ha='right')
        ax.set_ylabel(label, fontsize=8)
        ax.tick_params(labelsize=7)
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=3, fontsize=7,
               frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(os.path.join(OUT, 'summary_scenarios.pdf'))
    plt.close(fig)


def trajectories(scenario):
    """Detail: one scenario, one panel per mode."""
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.9), sharey=True)
    for ax, mode in zip(axes, MODES):
        draw_paths(ax, load(scenario, mode))
        ax.set_title(TITLES[mode], fontsize=9)
        ax.set_xlabel('x (m)', fontsize=8)
        ax.tick_params(labelsize=7)
    axes[0].set_ylabel('y (m)', fontsize=8)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=5, fontsize=7,
               frameon=False)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(os.path.join(OUT, f'trajectories_{scenario}.pdf'))
    plt.close(fig)


def bearing(scenario):
    """Detail: angular error of the longest follower, weak vs strong."""
    rows = [summary(scenario, m) for m in ('weak_turning', 'strong_turning')]
    robot = max(ROBOTS, key=lambda r: sum(
        float(s[r]['following_time_s']) for s in rows))
    fig, ax = plt.subplots(figsize=(3.5, 2.0))
    for mode in ('weak_turning', 'strong_turning'):
        d = load(scenario, mode)[robot]
        ax.plot(d['t'], [math.degrees(b) for b in d['bearing']],
                color=MODE_COLORS[mode], lw=0.8, label=TITLES[mode])
    ax.axhline(0, color='grey', lw=0.5)
    ax.set_title(f'{scenario}, {robot.replace("robot", "R")}', fontsize=8)
    ax.set_xlabel('time (s)', fontsize=8)
    ax.set_ylabel('bearing (deg)', fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f'bearing_{scenario}.pdf'))
    plt.close(fig)


def main():
    args = sys.argv[1:]
    detail = '--detail' in args
    scenarios = [a for a in args if a != '--detail'] or find_scenarios()
    if not scenarios:
        sys.exit(f'No scenario with timeseries for all modes in {METRICS}')
    os.makedirs(OUT, exist_ok=True)
    trajectories_overview(scenarios)
    summary_bars(scenarios)
    if detail:
        for scenario in scenarios:
            trajectories(scenario)
            bearing(scenario)
    print(f'{len(scenarios)} scenarios -> {OUT}')


if __name__ == '__main__':
    main()
