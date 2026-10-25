#!/usr/bin/env python3
"""Run every dataset sequentially and summarize localization success within a time budget."""
import argparse
import csv
import os
from pathlib import Path
import re
import subprocess
from summarize_benchmarks import read, stats


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('datasets_root', type=Path)
    parser.add_argument('--experiment-id', required=True)
    parser.add_argument('--config', type=Path, default=project / 'config/localizer.yaml')
    parser.add_argument('--engine', choices=['native', 'docker'], default='native')
    parser.add_argument('--budget-ms', type=float, default=3000)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.experiment_id):
        parser.error('Use letters, digits, _ and - in the experiment ID')
    if args.budget_ms <= 0:
        parser.error('--budget-ms must be positive')
    environment = dict(os.environ, RELOC_EXPERIMENT_ID=args.experiment_id,
                       RELOC_CONFIG=str(args.config.resolve()))
    scenes = sorted(path for path in args.datasets_root.resolve().iterdir()
                    if path.is_dir() and (path / 'groundtruth.csv').is_file())
    if not scenes:
        parser.error('No datasets found')
    summary = []
    for dataset in scenes:
        output = project / 'results' / dataset.name / args.experiment_id
        if output.exists():
            raise FileExistsError(f'Use a new experiment ID: {output}')
        output.mkdir(parents=True)
        with (output / 'run.log').open('w') as log:
            subprocess.run([str(project / 'scripts/bench.sh'), str(dataset), args.engine],
                           env=environment, check=True, stdout=log, stderr=subprocess.STDOUT)
        values = stats(read(output / 'benchmark_result.csv'), args.budget_ms)
        summary.append([dataset.name] + values)
        print(f'{dataset.name}: {values[2]}/{values[0]} success within {args.budget_ms:g} ms; '
              f'median={values[3]:.1f}, P95={values[4]:.1f}, max={values[5]:.1f} ms', flush=True)
    output = project / 'results' / f'{args.experiment_id}_summary.csv'
    with output.open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['scene', 'count', 'success', 'success_under_budget',
                         'median_ms', 'p95_ms', 'max_ms', 'over_budget'])
        writer.writerows(summary)
    print(output)


if __name__ == '__main__':
    main()
