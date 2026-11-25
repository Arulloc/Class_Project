#!/usr/bin/env python3
"""analyze_stageB.py: turn Stage B capture logs into a results table and figures.

Reads every results/stageB/stageB_*.csv written by scripts/capture.py (one file
per method and spin period, one row per capture attempt) and writes into the
same folder:
  summary.csv   - one row per method and spin period
  summary.md    - the same as a Markdown table
  fig_success.png  - capture success rate against spin speed
  fig_timing.png   - mean error along the handle's motion (timing) against
                     spin speed, with the 36 mm capture tolerance

Usage:  python3 scripts/analyze_stageB.py [results/stageB]
Needs:  matplotlib  (sudo apt install python3-matplotlib)
"""

import csv
import glob
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

METHODS = ['reactive', 'last_two', 'average', 'kalman']
LABELS = {'reactive': 'Reactive', 'last_two': 'Two-point',
          'average': 'Least-squares', 'kalman': 'Kalman filter'}
STYLE = {'reactive': ('#2a78d6', 'o'), 'last_two': ('#eb6834', 's'),
         'average': ('#1baf7a', '^'), 'kalman': ('#eda100', 'D')}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
TOLERANCE_MM = 36.0


def load(folder):
    runs = []
    for path in sorted(glob.glob(os.path.join(folder, 'stageB_*.csv'))):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        along = [float(r['err_along']) * 1000 for r in rows]
        across = [float(r['err_across']) * 1000 for r in rows]
        vert = [float(r['err_vertical']) * 1000 for r in rows]
        dist = [float(r['err_dist']) * 1000 for r in rows]
        ok = [int(r['success']) for r in rows]
        period = float(rows[0]['spin_period'])
        runs.append(dict(
            method=rows[0]['method'], period=period,
            speed=2 * math.pi / period, n=len(rows), captured=sum(ok),
            rate=100.0 * sum(ok) / len(rows),
            along_mean=statistics.mean(along),
            along_abs=statistics.mean(abs(a) for a in along),
            along_std=statistics.stdev(along) if len(along) > 1 else 0.0,
            across_abs=statistics.mean(abs(a) for a in across),
            vert_abs=statistics.mean(abs(v) for v in vert),
            dist_mean=statistics.mean(dist), dist_max=max(dist),
            file=os.path.basename(path)))
    return runs


def write_tables(runs, folder):
    order = sorted(runs, key=lambda r: (-r['period'], METHODS.index(r['method'])))
    cols = ['method', 'period', 'speed', 'n', 'captured', 'rate', 'along_mean',
            'along_abs', 'along_std', 'across_abs', 'vert_abs', 'dist_mean',
            'dist_max', 'file']
    with open(os.path.join(folder, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in order:
            w.writerow({k: (round(r[k], 3) if isinstance(r[k], float) else r[k])
                        for k in cols})
    with open(os.path.join(folder, 'summary.md'), 'w') as f:
        f.write('| Method | Spin period (s) | Captured | Success rate (%) '
                '| Timing error, mean (mm) | Timing error, spread (mm) '
                '| Aim error (mm) | Height error (mm) | Total error, mean (mm) '
                '| Total error, largest (mm) |\n')
        f.write('|' + ' --- |' * 10 + '\n')
        for r in order:
            f.write(f"| {LABELS[r['method']]} | {r['period']:g} "
                    f"| {r['captured']} of {r['n']} | {r['rate']:.0f} "
                    f"| {r['along_mean']:+.1f} | {r['along_std']:.1f} "
                    f"| {r['across_abs']:.1f} | {r['vert_abs']:.1f} "
                    f"| {r['dist_mean']:.1f} | {r['dist_max']:.1f} |\n")


def style(ax, ylabel, title):
    ax.set_xlabel('Target spin speed (rad/s)', color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.set_title(title, loc='left', color=INK, fontsize=11)
    ax.grid(axis='y', color=GRID, linewidth=1)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2)


def lines(ax, runs, key):
    for m in METHODS:
        pts = sorted((r['speed'], r[key]) for r in runs if r['method'] == m)
        if not pts:
            continue
        xs, ys = zip(*pts)
        color, marker = STYLE[m]
        ax.plot(xs, ys, color=color, marker=marker, markersize=6, linewidth=2,
                markeredgecolor='white', markeredgewidth=1.5, label=LABELS[m])
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, ncol=4,
              loc='upper center', bbox_to_anchor=(0.5, -0.2))


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else 'results/stageB'
    runs = load(folder)
    if not runs:
        sys.exit(f'No stageB_*.csv files in {folder}')
    write_tables(runs, folder)
    print(f'{len(runs)} runs -> summary.csv, summary.md')

    fig, ax = plt.subplots(figsize=(6.5, 4.0), dpi=200)
    lines(ax, runs, 'rate')
    style(ax, 'Capture success rate (%)', 'Capture success rate against target speed')
    ax.set_ylim(-5, 105)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, 'fig_success.png'))
    plt.close(fig)
    print('wrote fig_success.png')

    fig, ax = plt.subplots(figsize=(6.5, 4.0), dpi=200)
    lines(ax, runs, 'along_abs')
    style(ax, 'Timing error, mean size (mm)',
          'Timing error at the moment of capture against target speed')
    ax.set_ylim(bottom=0)
    ax.axhline(TOLERANCE_MM, color=INK2, linewidth=1, linestyle=(0, (4, 3)))
    ax.text(0.99, TOLERANCE_MM, f'capture tolerance {TOLERANCE_MM:g} mm ',
            transform=ax.get_yaxis_transform(), va='bottom', ha='right',
            color=INK2, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, 'fig_timing.png'))
    plt.close(fig)
    print('wrote fig_timing.png')


if __name__ == '__main__':
    main()
