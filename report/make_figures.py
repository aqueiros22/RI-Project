#!/usr/bin/env python3
"""
Build the article figures from the metrics CSVs.

Run from the repository root:
    python3 report/make_figures.py
Writes report/figures/*.pdf.
"""

import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

MODES = ['no_following', 'weak_turning', 'strong_turning']
TITLES = {'no_following': 'No following', 'weak_turning': 'Weak turning',
          'strong_turning': 'Strong turning'}
ROBOTS = [f'robot{i}' for i in range(1, 6)]
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd', '#d62728']
OUT = os.path.join('report', 'figures')


def load(path):
    """Return {robot: {field: [values]}} from a timeseries.csv."""
    data = {r: {'t': [], 'x': [], 'y': [], 'bearing': []} for r in ROBOTS}
    with open(path, newline='') as file:
        for row in csv.DictReader(file):
            d = data[row['robot']]
            d['t'].append(float(row['t']))
            d['x'].append(float(row['x']))
            d['y'].append(float(row['y']))
            d['bearing'].append(
                float(row['bearing']) if row['following'] == '1'
                else float('nan'))
    return data


def trajectories(scenario):
    """One panel per mode with the five robot paths."""
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.6), sharey=True)
    for ax, mode in zip(axes, MODES):
        data = load(os.path.join('metrics', scenario, f'seed0_{mode}',
                                 'timeseries.csv'))
        ax.plot([-1.5, 1.5, 1.5, -1.5, -1.5], [-1.5, -1.5, 1.5, 1.5, -1.5],
                color='grey', lw=1)
        for robot, color in zip(ROBOTS, COLORS):
            d = data[robot]
            ax.plot(d['x'], d['y'], color=color, lw=0.9,
                    label=robot.replace('robot', 'R'))
            ax.plot(d['x'][0], d['y'][0], 'o', color=color, ms=4)
            ax.plot(d['x'][-1], d['y'][-1], 'x', color=color, ms=4)
        ax.set_title(TITLES[mode], fontsize=9)
        ax.set_aspect('equal')
        ax.set_xlim(-1.6, 1.6)
        ax.set_ylim(-1.6, 1.6)
        ax.set_xlabel('x (m)', fontsize=8)
        ax.tick_params(labelsize=7)
    axes[0].set_ylabel('y (m)', fontsize=8)
    axes[2].legend(fontsize=6, loc='lower right', ncol=1)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f'trajectories_{scenario}.pdf'))
    plt.close(fig)


def bearing(scenario, robot):
    """Angular error of one robot over time, weak vs strong turning."""
    fig, ax = plt.subplots(figsize=(3.5, 2.0))
    for mode, color in [('weak_turning', '#ff7f0e'),
                        ('strong_turning', '#1f77b4')]:
        d = load(os.path.join('metrics', scenario, f'seed0_{mode}',
                              'timeseries.csv'))[robot]
        ax.plot(d['t'], [b * 57.29578 for b in d['bearing']], color=color,
                lw=0.8, label=TITLES[mode])
    ax.axhline(0, color='grey', lw=0.5)
    ax.set_xlabel('time (s)', fontsize=8)
    ax.set_ylabel('bearing (deg)', fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f'bearing_{scenario}_{robot}.pdf'))
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    trajectories('cenario_2')
    bearing('cenario_2', 'robot3')
    print(f'Figures written to {OUT}')


if __name__ == '__main__':
    main()
