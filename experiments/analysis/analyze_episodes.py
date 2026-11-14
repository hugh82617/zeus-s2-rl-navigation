#!/usr/bin/env python3
"""Recount the four archived 900-episode navigation evaluations.

Uses Python's standard library only. It preserves all outcomes, including
collisions and timeouts, and never rewrites source CSV files.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def analyze(path: Path, repo: Path, expected: int):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'episode_id', 'outcome', 'scenario_name', 'speed_tier', 'start_dist_m'}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f'{path.name}: missing columns: {sorted(missing)}')
        rows = list(reader)

    ids = [int(row['episode_id']) for row in rows]
    if len(rows) != expected or len(set(ids)) != len(rows) or set(ids) != set(range(expected)):
        raise ValueError(f'{path.name}: expected unique episode_id 0..{expected - 1}; got {len(rows)} rows, {len(set(ids))} unique IDs')

    outcomes = Counter(row['outcome'].strip() for row in rows)
    unknown = set(outcomes).difference({'success', 'collision', 'timeout'})
    if unknown:
        raise ValueError(f'{path.name}: unsupported outcome values: {sorted(unknown)}')

    groups = defaultdict(Counter)
    for row in rows:
        groups[(row['scenario_name'], row['speed_tier'])][row['outcome']] += 1

    result = {
        'path': path.relative_to(repo).as_posix(),
        'episodes': len(rows),
        'unique_episode_ids': len(set(ids)),
        'outcomes': {key: outcomes[key] for key in ['success', 'collision', 'timeout']},
        'success_rate': outcomes['success'] / len(rows),
        'groups': [
            {'scenario': scenario, 'speed': speed, 'episodes': sum(counts.values()),
             'outcomes': {key: counts[key] for key in ['success', 'collision', 'timeout']}}
            for (scenario, speed), counts in sorted(groups.items())
        ],
    }
    mapping = {int(row['episode_id']): tuple(row[field] for field in ['scenario_name', 'speed_tier', 'start_dist_m']) for row in rows}
    return result, mapping


def main():
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, default=repo / 'experiments' / 'datasets')
    parser.add_argument('--expected-episodes', type=int, default=900)
    parser.add_argument('--output', type=Path, help='Save JSON instead of printing it. Parent directory must already exist.')
    args = parser.parse_args()

    paths = sorted(path for path in args.data_dir.rglob('*.csv') if 'rollout_episode' in path.name)
    if len(paths) != 4:
        raise ValueError(f'Expected the four archived episode CSV files; found {len(paths)} in {args.data_dir}')

    results, maps = [], []
    for path in paths:
        result, mapping = analyze(path, repo, args.expected_episodes)
        results.append(result)
        maps.append(mapping)

    reference = maps[0]
    alignments = [
        {'path': result['path'], 'same_scenario_speed_start_distance_as_first_file': sum(mapping[key] == reference[key] for key in reference), 'compared_episodes': len(reference)}
        for result, mapping in zip(results, maps)
    ]
    output = {
        'method': 'All CSV rows; count outcome without filtering. Preserve both DWA settings separately.',
        'files': results,
        'alignment_fields': ['scenario_name', 'speed_tier', 'start_dist_m'],
        'alignment_reference': results[0]['path'],
        'alignments': alignments,
        'limitations': 'Matching these three fields does not verify all simulator, sensing, action, or termination settings. Real-robot records are a separate dataset.',
    }
    encoded = json.dumps(output, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.write_text(encoded, encoding='utf-8')
    else:
        print(encoded, end='')


if __name__ == '__main__':
    main()
