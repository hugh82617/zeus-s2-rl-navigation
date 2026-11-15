"""Summarize saved ROS topic rates; these are not hardware sampling rates."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics

REPO = Path(__file__).resolve().parents[2]
EXPERIMENT = REPO / 'experiments/real_robot_2026-04-28'
TOPICS = ['/scan', '/scan_aligned', '/imu/data', '/odom', '/odom_wheel',
          '/joint_states', '/motor_driverFeedback', '/zeus_speed',
          '/rl/lidar_181', '/rl/lidar_543', '/rl/obs_556', '/rl_cmd_vel']


def summarize():
    trials = {topic: [] for topic in TOPICS}
    for source in sorted((EXPERIMENT / '4-hz-message/01_frequency_information').glob('trial_*/frequency_summary.csv')):
        with source.open(encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                if row['topic'] not in trials or not row['avg_hz']:
                    continue
                rate = float(row['avg_hz'])
                if math.isfinite(rate):
                    trials[row['topic']].append({'trial': source.parent.name, 'avg_hz': rate,
                        'message_count': int(row['count']), 'source': source.relative_to(REPO).as_posix()})
    result = {'method': 'Median and range of saved per-trial avg_hz; no pooling of message streams.',
              'scope': 'ROS publication/receipt rate in saved windows, not device ADC or encoder sampling rate.',
              'topics': {}}
    for topic, rows in trials.items():
        rates = [row['avg_hz'] for row in rows]
        result['topics'][topic] = {'trials_with_valid_rate': len(rows),
            'median_hz': statistics.median(rates) if rates else None,
            'min_hz': min(rates) if rates else None, 'max_hz': max(rates) if rates else None,
            'measurements': rows}
    dedicated = {
        '/scan': '2-lidar-message/lidar-message/lidar-message/06_scan_hz.txt',
        '/imu/data': '3-other-message/imu-check-message/imu-check-message/08_imu_data_hz.txt',
        '/odom': '3-other-message/other-message/other-message/07_odom_hz.txt',
    }
    result['dedicated_rostopic_hz'] = {}
    for topic, relative in dedicated.items():
        source = EXPERIMENT / relative
        rates = [float(value) for value in re.findall(r'average rate:\s*([0-9.]+)', source.read_text(encoding='utf-8'))]
        result['dedicated_rostopic_hz'][topic] = {'final_reported_average_hz': rates[-1],
            'source': source.relative_to(REPO).as_posix()}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = summarize()
    if args.output:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({topic: {key: value for key, value in values.items() if key != 'measurements'}
                      for topic, values in report['topics'].items()}, ensure_ascii=False, indent=2))
    print(json.dumps(report['dedicated_rostopic_hz'], ensure_ascii=False))
