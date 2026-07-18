#!/usr/bin/env python3
"""Convert measured radio samples into the link-emulator CSV schema.

Input columns:
  time_s,throughput_mbps,rtt_ms,loss_percent[,jitter_ms]

The output uses one-way latency (RTT/2). Keep the raw measurement file as the
experimental record; this converter intentionally does not smooth it.
"""

import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()

    with args.input.open(newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))

    required = {'time_s', 'throughput_mbps', 'rtt_ms', 'loss_percent'}
    if not rows or not required.issubset(rows[0]):
        missing = ', '.join(sorted(required - (set(rows[0]) if rows else set())))
        raise SystemExit(f'missing required input columns: {missing}')

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='', encoding='utf-8') as destination:
        writer = csv.writer(destination)
        writer.writerow([
            '# time_s', 'bandwidth_mbps', 'one_way_latency_ms',
            'jitter_ms', 'loss_percent',
        ])
        for row in rows:
            writer.writerow([
                float(row['time_s']),
                max(0.0, float(row['throughput_mbps'])),
                max(0.0, float(row['rtt_ms']) / 2.0),
                max(0.0, float(row.get('jitter_ms') or 0.0)),
                min(100.0, max(0.0, float(row['loss_percent']))),
            ])


if __name__ == '__main__':
    main()
