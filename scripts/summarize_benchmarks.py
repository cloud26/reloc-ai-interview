#!/usr/bin/env python3
"""Compare unchanged benchmark CSVs, counting success only within the time budget."""
import argparse
import csv
import math
from pathlib import Path
import statistics


def read(path):
    with path.open(newline='') as stream:
        return {row['filename']: row for row in csv.DictReader(stream)}


def percentile(values, fraction):
    values = sorted(values)
    index = (len(values) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    return values[low] + (values[high] - values[low]) * (index - low)


def stats(rows, budget):
    times = [float(row['time_ms']) for row in rows.values()]
    success = sum(row['success'] == '1' for row in rows.values())
    on_time = sum(row['success'] == '1' and float(row['time_ms']) <= budget
                  for row in rows.values())
    return [len(rows), success, on_time, statistics.median(times),
            percentile(times, 0.95), max(times), sum(t > budget for t in times)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True,
                        help='Directory containing scene/benchmark_result.csv')
    parser.add_argument('--optimized', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--budget-ms', type=float, default=3000)
    args = parser.parse_args()
    if args.budget_ms <= 0:
        parser.error('--budget-ms must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    summary, changes = [], []
    scenes = sorted(path.parent.name for path in args.baseline.glob('*/benchmark_result.csv'))
    for scene in scenes:
        old = read(args.baseline / scene / 'benchmark_result.csv')
        new = read(args.optimized / scene / 'benchmark_result.csv')
        if old.keys() != new.keys():
            raise ValueError(f'{scene}: query sets differ')
        summary.append([scene] + stats(old, args.budget_ms) + stats(new, args.budget_ms))
        for name in old:
            a, b = old[name], new[name]
            before = a['success'] == '1' and float(a['time_ms']) <= args.budget_ms
            after = b['success'] == '1' and float(b['time_ms']) <= args.budget_ms
            change = 'recovered' if after and not before else 'regressed' if before and not after else 'unchanged'
            changes.append([scene, name, change, int(before), int(after)] +
                           [a[key] for key in ('success', 'pos_err_m', 'yaw_err_deg', 'time_ms', 'fail_reason')] +
                           [b[key] for key in ('success', 'pos_err_m', 'yaw_err_deg', 'time_ms', 'fail_reason')])
    fields = ['count', 'success', 'success_under_budget', 'median_ms', 'p95_ms', 'max_ms', 'over_budget']
    with (args.output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['scene'] + ['baseline_' + f for f in fields] + ['optimized_' + f for f in fields])
        writer.writerows(summary)
    with (args.output / 'per_query_diff.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['scene', 'filename', 'change', 'baseline_within_budget', 'optimized_within_budget'] +
                        [prefix + key for prefix in ('baseline_', 'optimized_')
                         for key in ('success', 'pos_err_m', 'yaw_err_deg', 'time_ms', 'fail_reason')])
        writer.writerows(changes)
    for row in summary:
        print(f'{row[0]}: {row[3]}/{row[1]} -> {row[10]}/{row[8]} within {args.budget_ms:g} ms')


if __name__ == '__main__':
    main()
