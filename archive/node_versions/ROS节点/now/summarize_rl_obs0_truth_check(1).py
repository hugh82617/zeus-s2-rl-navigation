#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import csv
import math
import os
import sys

DEFAULT_IN = os.path.expanduser('~/rl_obs0_truth_check.txt')
DEFAULT_OUT = os.path.expanduser('~/rl_obs0_truth_summary.txt')


def parse_bool(v):
    return str(v).strip().lower() == 'true'


def parse_float(v):
    s = str(v).strip().lower()
    if s in ('', 'nan', 'none'):
        return float('nan')
    return float(s)


def safe_pct(num, den):
    return 0.0 if den == 0 else 100.0 * float(num) / float(den)


def main():
    in_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IN
    out_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    if not os.path.exists(in_path):
        print('输入文件不存在: {}'.format(in_path))
        sys.exit(1)

    rows = []
    with open(in_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    total = len(rows)
    match_true = 0
    match_false = 0
    have543_false = 0
    havest_false = 0
    havehd_false = 0
    mono_false = 0
    frame_bad = 0
    srcstamp_bad = 0
    valid_diff_rows = 0
    max_abs_diff = -1.0
    max_abs_diff_seq = None
    max_header_delta = -1.0
    max_header_delta_seq = None
    mismatch_sum = 0

    bad_match_examples = []
    bad_stamp_examples = []
    bad_header_examples = []

    for row in rows:
        seq = row.get('seq', '')
        match = parse_bool(row.get('match', 'False'))
        have_543 = parse_bool(row.get('have_543', 'False'))
        have_stamps = parse_bool(row.get('have_stamps', 'False'))
        have_header = parse_bool(row.get('have_header', 'False'))
        mono = parse_bool(row.get('stamp_monotonic', 'False'))
        frame_id = row.get('frame_id', 'None')
        mad = parse_float(row.get('max_abs_diff', 'nan'))
        hds = parse_float(row.get('header_delta_sec', 'nan'))

        try:
            mc = int(row.get('mismatch_count', '-1'))
        except Exception:
            mc = -1

        if match:
            match_true += 1
        else:
            match_false += 1
            if len(bad_match_examples) < 10:
                bad_match_examples.append(
                    'seq={} max_abs_diff={} mismatch_count={} have_543={}'.format(seq, row.get('max_abs_diff'), row.get('mismatch_count'), row.get('have_543'))
                )

        if not have_543:
            have543_false += 1
        if not have_stamps:
            havest_false += 1
        if not have_header:
            havehd_false += 1

        if not mono:
            mono_false += 1
            if len(bad_stamp_examples) < 10:
                bad_stamp_examples.append(
                    'seq={} stamp_monotonic={} have_stamps={}'.format(seq, row.get('stamp_monotonic'), row.get('have_stamps'))
                )

        if frame_id != 'laser_link_aligned':
            frame_bad += 1
            if len(bad_header_examples) < 10:
                bad_header_examples.append(
                    'seq={} frame_id={} header_delta_sec={}'.format(seq, frame_id, row.get('header_delta_sec'))
                )

        if not math.isnan(hds):
            if hds > max_header_delta:
                max_header_delta = hds
                max_header_delta_seq = seq
            if hds > 0.20:
                srcstamp_bad += 1

        if mc >= 0:
            mismatch_sum += mc

        if not math.isnan(mad):
            valid_diff_rows += 1
            if mad > max_abs_diff:
                max_abs_diff = mad
                max_abs_diff_seq = seq

    lines = []
    lines.append('========================================')
    lines.append('RL obs0 真值一致性汇总报告')
    lines.append('输入文件: {}'.format(in_path))
    lines.append('总帧数: {}'.format(total))
    lines.append('========================================')
    lines.append('')

    lines.append('一、核心结论')
    if total == 0:
        lines.append('1) 输入文件没有任何数据行，无法判断。')
    else:
        if match_false == 0 and mono_false == 0 and frame_bad == 0 and have543_false == 0 and havest_false == 0 and havehd_false == 0:
            lines.append('1) 所有已记录帧均通过主数据一致性校验。')
            lines.append('2) /rl/debug/obs0_181 与 /rl/lidar_543 的前 181 维保持一致。')
            lines.append('3) 时间序列单调、frame_id 一致、配套话题齐全。')
        else:
            lines.append('1) 检测到异常，请重点查看“失败样例”和“统计明细”。')
            if match_false > 0:
                lines.append('2) 存在 obs0 与 lidar_543[:181] 不一致的帧。')
            if mono_false > 0:
                lines.append('3) 存在时间单调性异常的帧。')
            if frame_bad > 0:
                lines.append('4) 存在 frame_id 不是 laser_link_aligned 的帧。')
    lines.append('')

    lines.append('二、统计明细')
    lines.append('match=True                 : {} ({:.2f}%)'.format(match_true, safe_pct(match_true, total)))
    lines.append('match=False                : {} ({:.2f}%)'.format(match_false, safe_pct(match_false, total)))
    lines.append('have_543=False             : {} ({:.2f}%)'.format(have543_false, safe_pct(have543_false, total)))
    lines.append('have_stamps=False          : {} ({:.2f}%)'.format(havest_false, safe_pct(havest_false, total)))
    lines.append('have_header=False          : {} ({:.2f}%)'.format(havehd_false, safe_pct(havehd_false, total)))
    lines.append('stamp_monotonic=False      : {} ({:.2f}%)'.format(mono_false, safe_pct(mono_false, total)))
    lines.append('frame_id!=laser_link_aligned: {} ({:.2f}%)'.format(frame_bad, safe_pct(frame_bad, total)))
    lines.append('header_delta_sec>0.20      : {} ({:.2f}%)'.format(srcstamp_bad, safe_pct(srcstamp_bad, total)))
    lines.append('有效差值帧数               : {}'.format(valid_diff_rows))
    lines.append('mismatch_count 总和        : {}'.format(mismatch_sum))
    lines.append('')

    lines.append('三、极值')
    lines.append('max_abs_diff               : {} (seq={})'.format('nan' if max_abs_diff < 0 else '{:.9f}'.format(max_abs_diff), max_abs_diff_seq))
    lines.append('max_header_delta_sec       : {} (seq={})'.format('nan' if max_header_delta < 0 else '{:.6f}'.format(max_header_delta), max_header_delta_seq))
    lines.append('')

    lines.append('四、失败样例（最多 10 条）')
    if bad_match_examples:
        lines.append('[数据不一致]')
        lines.extend(bad_match_examples)
    else:
        lines.append('[数据不一致] 无')
    lines.append('')
    if bad_stamp_examples:
        lines.append('[时间单调性异常]')
        lines.extend(bad_stamp_examples)
    else:
        lines.append('[时间单调性异常] 无')
    lines.append('')
    if bad_header_examples:
        lines.append('[frame_id/header 异常]')
        lines.extend(bad_header_examples)
    else:
        lines.append('[frame_id/header 异常] 无')
    lines.append('')

    lines.append('五、判定建议')
    if total == 0:
        lines.append('当前没有可用样本，请先运行 rl_obs0_truth_validator。')
    else:
        if match_false == 0 and mono_false == 0 and frame_bad == 0 and have543_false == 0 and havest_false == 0 and havehd_false == 0:
            lines.append('可以认定：当前 RViz 节点显示的主数据来源，与 RL 推理节点实际吃进去的当前 181 维数据帧一致。')
        else:
            lines.append('暂不能完全认定来源严格一致，请先解决上述异常项后再下结论。')

    out_dir = os.path.dirname(out_path)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print('汇总完成: {}'.format(out_path))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
