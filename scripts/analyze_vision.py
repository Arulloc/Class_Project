#!/usr/bin/env python3
"""analyze_vision.py: how accurate and how fast the camera measurement is.

Reads every results/stageC/vision_P*.csv written by scripts/tag_detector.py
(one row per camera image in which the tag was found, with the true handle
position at the instant the image was taken) and writes
results/stageC/vision_summary.md with, for each spin period:
  - measurements per second, and the share of the camera's 30 images per
    second in which the tag was found
  - handle position error: mean, standard deviation, largest (mm)
  - handle angle error: mean size (degrees)
  - processing delay from image to published measurement (ms)

Usage:  python3 scripts/analyze_vision.py [results/stageC]
"""

import csv
import glob
import os
import re
import statistics
import sys

CAMERA_RATE = 30.0


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else 'results/stageC'
    rows_out = []
    for path in sorted(glob.glob(os.path.join(folder, 'vision_P*.csv'))):
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if len(rows) < 2:
            continue
        period = float(re.search(r'vision_P([\d.]+)\.csv', path).group(1))
        t = [float(r['t_image']) for r in rows]
        err = [float(r['error_m']) * 1000 for r in rows]
        ang = [abs(float(r['angle_error_deg'])) for r in rows]
        lag = [float(r['latency']) * 1000 for r in rows]
        rate = (len(rows) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0
        rows_out.append((period, len(rows), rate, 100 * rate / CAMERA_RATE,
                         statistics.mean(err), statistics.stdev(err), max(err),
                         statistics.mean(ang), statistics.mean(lag)))
    if not rows_out:
        sys.exit(f'No vision_P*.csv files in {folder}')
    rows_out.sort(key=lambda r: -r[0])
    out = os.path.join(folder, 'vision_summary.md')
    with open(out, 'w') as f:
        f.write('| Spin period (s) | Measurements | Per second | Tag found (% of images) '
                '| Position error, mean (mm) | Spread (mm) | Largest (mm) '
                '| Angle error, mean (deg) | Processing delay (ms) |\n')
        f.write('|' + ' --- |' * 9 + '\n')
        for r in rows_out:
            f.write(f'| {r[0]:g} | {r[1]} | {r[2]:.1f} | {r[3]:.0f} | {r[4]:.1f} '
                    f'| {r[5]:.1f} | {r[6]:.1f} | {r[7]:.2f} | {r[8]:.0f} |\n')
    print(open(out).read())


if __name__ == '__main__':
    main()
