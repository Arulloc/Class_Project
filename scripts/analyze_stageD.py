#!/usr/bin/env python3
"""analyze_stageD.py: turn Stage D capture logs into a results table and figures.

Reads every stageD_P<period>_<method>_d<delay>_c<1|0>.csv written by
scripts/run_stageD.sh (one file per method, added delay and correction setting,
one row per capture attempt). If the matching Stage C results exist (same spin
period, no added delay), they are included as the zero-delay point.

Writes into the Stage D folder:
  summary.md             - one row per method, delay and correction setting
  fig_delay_error.png    - mean distance from the handle against added delay
  fig_delay_success.png  - capture success rate against added delay

Usage:  python3 scripts/analyze_stageD.py [results/stageD_moveit] [results/stageC_moveit]
Needs:  matplotlib  (sudo apt install python3-matplotlib)
"""

import csv
import glob
import os
import re
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

METHODS = ['average', 'kalman']
LABELS = {'average': 'Least-squares', 'kalman': 'Kalman filter'}
COLORS = {'average': '#1baf7a', 'kalman': '#eda100'}
MARKERS = {'average': '^', 'kalman': 'D'}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
TOLERANCE_MM = 36.0
NAME = re.compile(r'stageD_P([\d.]+)_([a-z_]+)_d([\d.]+)_c([01])\.csv$')


def summarise(rows, method, period, delay, corrected):
    along = [float(r['err_along']) * 1000 for r in rows]
    dist = [float(r['err_dist']) * 1000 for r in rows]
    ok = [int(r['success']) for r in rows]
    return dict(method=method, period=period, delay=delay, corrected=corrected,
                n=len(rows), captured=sum(ok), rate=100.0 * sum(ok) / len(rows),
                along_mean=statistics.mean(along),
                dist_mean=statistics.mean(dist), dist_max=max(dist))


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def load(folder_d, folder_c):
    runs = []
    periods = set()
    for path in sorted(glob.glob(os.path.join(folder_d, 'stageD_*.csv'))):
        m = NAME.search(os.path.basename(path))
        rows = read(path)
        if not m or not rows:
            continue
        period, method = float(m.group(1)), m.group(2)
        periods.add(m.group(1))
        runs.append(summarise(rows, method, period, float(m.group(3)),
                              m.group(4) == '1'))
    # Zero added delay = Stage C, same spin period (counts for both settings,
    # since with no added delay there is nothing to correct)
    for p in periods:
        for method in METHODS:
            path = os.path.join(folder_c, f'stageC_P{p}_{method}.csv')
            if os.path.exists(path) and read(path):
                for corr in (True, False):
                    runs.append(summarise(read(path), method, float(p), 0.0, corr))
    return runs


def write_table(runs, folder):
    order = sorted(runs, key=lambda r: (r['delay'], not r['corrected'],
                                        METHODS.index(r['method'])))
    with open(os.path.join(folder, 'summary.md'), 'w') as f:
        f.write('| Added delay (s) | Delay correction | Method | Captured '
                '| Timing error, mean (mm) | Total error, mean (mm) '
                '| Total error, largest (mm) |\n')
        f.write('|' + ' --- |' * 7 + '\n')
        for r in order:
            if r['delay'] == 0 and not r['corrected']:
                continue        # the zero-delay row is printed once
            corr = ('none added' if r['delay'] == 0 else
                    'yes' if r['corrected'] else 'no')
            f.write(f"| {r['delay']:g} | {corr} | {LABELS[r['method']]} "
                    f"| {r['captured']} of {r['n']} | {r['along_mean']:+.1f} "
                    f"| {r['dist_mean']:.1f} | {r['dist_max']:.1f} |\n")


def style(ax, ylabel, title):
    ax.set_xlabel('Added measurement delay (s)', color=INK2)
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
    for method in METHODS:
        for corr in (True, False):
            pts = sorted((r['delay'], r[key]) for r in runs
                         if r['method'] == method and r['corrected'] == corr)
            if not pts:
                continue
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=COLORS[method], marker=MARKERS[method],
                    markersize=6, linewidth=2, markeredgecolor='white',
                    markeredgewidth=1.5, linestyle='-' if corr else (0, (4, 3)),
                    label=f"{LABELS[method]}, {'corrected' if corr else 'not corrected'}")
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, ncol=2,
              loc='upper center', bbox_to_anchor=(0.5, -0.2))


def main():
    folder_d = sys.argv[1] if len(sys.argv) > 1 else 'results/stageD_moveit'
    folder_c = (sys.argv[2] if len(sys.argv) > 2 else
                folder_d.replace('stageD', 'stageC'))
    runs = load(folder_d, folder_c)
    if not runs:
        sys.exit(f'No stageD_*.csv files in {folder_d}')
    write_table(runs, folder_d)
    print(f'{len(runs)} rows -> summary.md'
          + ('' if any(r['delay'] == 0 for r in runs)
             else f'  (no Stage C results found in {folder_c}; no zero-delay point)'))

    fig, ax = plt.subplots(figsize=(6.5, 4.2), dpi=200)
    lines(ax, runs, 'dist_mean')
    style(ax, 'Distance from the handle, mean (mm)',
          'Capture error against added measurement delay')
    ax.set_ylim(bottom=0)
    ax.axhline(TOLERANCE_MM, color=INK2, linewidth=1, linestyle=(0, (1, 2)))
    ax.text(0.01, TOLERANCE_MM, f' capture tolerance {TOLERANCE_MM:g} mm',
            transform=ax.get_yaxis_transform(), va='bottom', ha='left',
            color=INK2, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(folder_d, 'fig_delay_error.png'))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.5, 4.2), dpi=200)
    lines(ax, runs, 'rate')
    style(ax, 'Capture success rate (%)',
          'Capture success rate against added measurement delay')
    ax.set_ylim(-5, 105)
    fig.tight_layout()
    fig.savefig(os.path.join(folder_d, 'fig_delay_success.png'))
    plt.close(fig)
    print('wrote fig_delay_error.png, fig_delay_success.png')


if __name__ == '__main__':
    main()
