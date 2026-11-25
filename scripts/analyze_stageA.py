#!/usr/bin/env python3
"""analyze_stageA.py: turn Stage A prediction logs into a results table and figures.

Reads every results/stageA/*.csv written by the predictor (one file per run),
and writes into the same folder:
  summary.csv  - one row per run: mean, std, max error, speed estimate
  summary.md   - the same as a Markdown table, for the report
  fig_speed.png, fig_horizon.png, fig_noise.png, fig_delay.png

Usage:  python3 scripts/analyze_stageA.py [results/stageA]
Needs:  matplotlib  (sudo apt install python3-matplotlib)
"""

import csv
import glob
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use('Agg')            # draw to files, no window needed
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms

METHODS = ['reactive', 'last_two', 'average', 'kalman']
# Fixed colour per method (validated categorical palette, colour-blind safe
# with the markers and labels), plus a different marker shape for each.
STYLE = {
    'reactive': ('#2a78d6', 'o'),
    'last_two': ('#eb6834', 's'),
    'average':  ('#1baf7a', '^'),
    'kalman':   ('#eda100', 'D'),
}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'

# Baseline condition: every sweep changes one setting from here
BASE = dict(spin_period=10.0, horizon=1.0, noise_std=0.0, delay=0.0)


def load(folder):
    runs = []
    for path in sorted(glob.glob(os.path.join(folder, '*.csv'))):
        if os.path.basename(path) == 'summary.csv':
            continue
        with open(path) as fh:
            rows = list(csv.DictReader(fh))
        if not rows:
            print(f'skipped (empty): {path}')
            continue
        r0 = rows[0]
        err = [float(r['error_m']) * 1000 for r in rows]          # mm
        spd = [float(r['speed_est']) for r in rows]
        period = float(r0['spin_period'])
        runs.append(dict(
            file=os.path.basename(path), method=r0['method'],
            spin_period=period, horizon=float(r0['horizon']),
            noise_std=float(r0['noise_std']), delay=float(r0['delay']),
            compensate=int(r0['compensate']),
            n=len(err), mean=statistics.mean(err),
            std=statistics.stdev(err) if len(err) > 1 else 0.0,
            max=max(err), speed_set=2 * math.pi / period if period > 0 else 0.0,
            speed_est=statistics.mean(spd)))
    return runs


def matches(run, **fixed):
    return all(abs(run[k] - v) < 1e-9 for k, v in fixed.items())


def write_tables(runs, folder):
    cols = ['method', 'spin_period', 'horizon', 'noise_std', 'delay',
            'compensate', 'n', 'mean', 'std', 'max', 'speed_set',
            'speed_est', 'file']
    with open(os.path.join(folder, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in runs:
            w.writerow({k: (f'{r[k]:.4f}' if isinstance(r[k], float) else r[k])
                        for k in cols})
    with open(os.path.join(folder, 'summary.md'), 'w') as fh:
        fh.write('| Method | Spin period (s) | Horizon (s) | Noise (mm) | Delay (ms) '
                 '| Delay corrected | Mean error (mm) | Std dev (mm) | Max error (mm) '
                 '| Set speed (rad/s) | Estimated speed (rad/s) | Checks | File |\n')
        fh.write('|' + ' --- |' * 13 + '\n')
        for r in runs:
            fh.write(f"| {r['method']} | {r['spin_period']:g} | {r['horizon']:g} "
                     f"| {r['noise_std'] * 1000:g} | {r['delay'] * 1000:g} "
                     f"| {'yes' if r['compensate'] else 'no'} | {r['mean']:.1f} "
                     f"| {r['std']:.1f} | {r['max']:.1f} | {r['speed_set']:.3f} "
                     f"| {'n/a' if r['method'] == 'reactive' else format(r['speed_est'], '.3f')} "
                     f"| {r['n']} | {r['file']} |\n")


def style_axes(ax, xlabel, title):
    ax.set_xlabel(xlabel, color=INK2)
    ax.set_ylabel('Prediction error, mean ± std (mm)', color=INK2)
    ax.set_title(title, loc='left', color=INK, fontsize=11)
    ax.grid(axis='y', color=GRID, linewidth=1)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2)
    ax.set_ylim(bottom=0)


def label_ends(ax, ends):
    """Name each line at its right end. Lines ending at nearly the same value
    share one label; others are spaced so labels never overlap."""
    if not ends:
        return
    top = max(y for _, y, _ in ends) or 1.0
    gap = 0.06 * top
    ends = sorted(ends, key=lambda e: e[1])
    groups = []
    for x, y, name in ends:
        if groups and abs(y - groups[-1][1]) < gap / 2:
            groups[-1][2].append(name)
        else:
            groups.append([x, y, [name]])
    last = None
    for x, y, names in groups:
        ly = y if last is None else max(y, last + gap)
        last = ly
        # 8 points to the right of the line end, at the (possibly nudged) height
        ax.text(x, ly, ', '.join(names), va='center', color=INK2, fontsize=8,
                clip_on=False,
                transform=mtransforms.offset_copy(ax.transData, fig=ax.figure,
                                                  x=8, y=0, units='points'))


def plot_lines(ax, runs, xkey, xscale=1.0, linestyle='-'):
    ends = []
    for m in METHODS:
        pts = sorted((r[xkey] * xscale, r['mean'], r['std'])
                     for r in runs if r['method'] == m)
        if not pts:
            continue
        xs, ys, es = zip(*pts)
        color, marker = STYLE[m]
        ax.errorbar(xs, ys, yerr=es, color=color, marker=marker, markersize=6,
                    linewidth=2, linestyle=linestyle, capsize=3,
                    markeredgecolor='white', markeredgewidth=1.5, label=m)
        ends.append((xs[-1], ys[-1], m))
    label_ends(ax, ends)


def figure(folder, name, runs, xkey, xlabel, title, xscale=1.0):
    if len({r[xkey] for r in runs}) < 2:
        print(f'{name}: not enough conditions yet, skipped')
        return
    fig, ax = plt.subplots(figsize=(6.5, 4.0), dpi=200)
    plot_lines(ax, runs, xkey, xscale)
    style_axes(ax, xlabel, title)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, name))
    plt.close(fig)
    print(f'wrote {name}')


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else 'results/stageA'
    runs = load(folder)
    if not runs:
        sys.exit(f'No CSV files in {folder}')
    write_tables(runs, folder)
    print(f'{len(runs)} runs -> summary.csv, summary.md')

    b = BASE
    figure(folder, 'fig_speed.png',
           [r for r in runs if matches(r, horizon=b['horizon'], noise_std=0.0,
                                      delay=0.0)],
           'speed_set', 'Target spin speed (rad/s)',
           f"Prediction error vs target speed ({b['horizon']:g} s ahead, no noise or delay)")
    figure(folder, 'fig_horizon.png',
           [r for r in runs if matches(r, spin_period=b['spin_period'],
                                      noise_std=0.0, delay=0.0)],
           'horizon', 'Prediction horizon (s)',
           f"Prediction error vs horizon ({b['spin_period']:g} s spin period)")
    figure(folder, 'fig_noise.png',
           [r for r in runs if matches(r, spin_period=b['spin_period'],
                                      horizon=b['horizon'], delay=0.0)],
           'noise_std', 'Measurement noise, std per axis (mm)',
           f"Prediction error vs measurement noise ({b['spin_period']:g} s spin, "
           f"{b['horizon']:g} s ahead)", xscale=1000)

    # Delay: two panels, corrected and not corrected, same y scale
    dr = [r for r in runs if matches(r, spin_period=b['spin_period'],
                                     horizon=b['horizon'], noise_std=0.0)]
    if len({r['delay'] for r in dr}) >= 2:
        fig, axes = plt.subplots(1, 2, figsize=(9, 4.0), dpi=200, sharey=True)
        for ax, comp, label in ((axes[0], 1, 'Delay corrected'),
                                (axes[1], 0, 'Delay not corrected')):
            # zero delay belongs to both panels
            sel = [r for r in dr if r['compensate'] == comp or r['delay'] == 0]
            plot_lines(ax, sel, 'delay', 1000)
            style_axes(ax, 'Added measurement delay (ms)', label)
        axes[1].set_ylabel('')
        axes[0].legend(frameon=False, fontsize=8, labelcolor=INK2)
        fig.suptitle(f"Prediction error vs delay ({b['spin_period']:g} s spin, "
                     f"{b['horizon']:g} s ahead)", x=0.01, ha='left',
                     color=INK, fontsize=11)
        fig.tight_layout()
        fig.savefig(os.path.join(folder, 'fig_delay.png'))
        plt.close(fig)
        print('wrote fig_delay.png')
    else:
        print('fig_delay.png: not enough conditions yet, skipped')


if __name__ == '__main__':
    main()
