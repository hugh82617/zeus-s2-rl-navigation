#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
bag_to_csv.py
------------------------------------------------------------
将 record_pipeline_v2.sh 录制的 bag 导出为 13 个 CSV 文件,
文件名与《总体修改意见》第四节"CSV 导出清单"完全一致。

用法:
    python bag_to_csv.py <input.bag> [output_dir]

依赖:
    - rosbag (ROS Python 绑定,Melodic 下用 python2)
    - 纯标准库 csv/os/sys,不依赖 pandas/numpy

输出目录布局:
    <output_dir>/
        scan_raw.csv                    <- /scan
        scan_aligned.csv                <- /scan_aligned
        scan_aligned_invalid_mask.csv   <- /scan_aligned_invalid_mask
        scan_aligned_invalid_stats.csv  <- /scan_aligned_invalid_stats
        lidar_181.csv                   <- /rl/lidar_181
        lidar_181_stamp.csv             <- /rl/lidar_181_stamp
        lidar_181_src_map.csv           <- /rl/lidar_181_src_map
        lidar_181_repair_mask.csv       <- /rl/lidar_181_repair_mask
        lidar_181_fill_stats.csv        <- /rl/lidar_181_fill_stats
        lidar_543.csv                   <- /rl/lidar_543
        lidar_543_ids.csv               <- /rl/lidar_543_ids
        lidar_543_ids_vis.csv           <- /rl/lidar_543_ids_vis
        lidar_543_stamps.csv            <- /rl/lidar_543_stamps

CSV 列约定:
    LaserScan 类型:
      bag_time, stamp_sec, frame_id,
      angle_min, angle_max, angle_increment,
      time_increment, scan_time, range_min, range_max,
      ranges_n, intensities_n,
      r_0 ... r_{N-1}, i_0 ... i_{M-1}

    Float32/64MultiArray 类型:
      bag_time, data_n, v_0, v_1, ..., v_{N-1}

    Header 类型(/rl/lidar_181_stamp):
      bag_time, stamp_sec, stamp_nsec, frame_id
============================================================
"""

from __future__ import division, print_function

import csv
import os
import sys
import traceback

try:
    import rosbag
except ImportError:
    sys.stderr.write(
        "ERROR: `import rosbag` failed. Please source your ROS environment "
        "(e.g. `source /opt/ros/melodic/setup.bash`) before running this script.\n"
    )
    sys.exit(1)


# ------------------------------------------------------------
# topic -> (out_filename, writer_tag)
# 顺序与文档第四节的清单一致(便于你肉眼 diff)。
# ------------------------------------------------------------
EXPORT_MAP = [
    ("/scan",                        "scan_raw.csv",                     "laserscan"),
    ("/scan_aligned",                "scan_aligned.csv",                 "laserscan"),
    ("/scan_aligned_invalid_mask",   "scan_aligned_invalid_mask.csv",    "multiarray"),
    ("/scan_aligned_invalid_stats",  "scan_aligned_invalid_stats.csv",   "multiarray"),
    ("/rl/lidar_181",                "lidar_181.csv",                    "multiarray"),
    ("/rl/lidar_181_stamp",          "lidar_181_stamp.csv",              "header"),
    ("/rl/lidar_181_src_map",        "lidar_181_src_map.csv",            "multiarray"),
    ("/rl/lidar_181_repair_mask",    "lidar_181_repair_mask.csv",        "multiarray"),
    ("/rl/lidar_181_fill_stats",     "lidar_181_fill_stats.csv",         "multiarray"),
    ("/rl/lidar_543",                "lidar_543.csv",                    "multiarray"),
    ("/rl/lidar_543_ids",            "lidar_543_ids.csv",                "multiarray"),
    ("/rl/lidar_543_ids_vis",        "lidar_543_ids_vis.csv",            "multiarray"),
    ("/rl/lidar_543_stamps",         "lidar_543_stamps.csv",             "multiarray"),
]


# ------------------------------------------------------------
# per-type writers
# ------------------------------------------------------------
def write_laserscan(bag, topic, out_path):
    count = 0
    with open(out_path, "w") as f:
        w = csv.writer(f)
        header_written = False
        for _, msg, t in bag.read_messages(topics=[topic]):
            n = len(msg.ranges)
            m = len(msg.intensities)
            if not header_written:
                cols = [
                    "bag_time", "stamp_sec", "frame_id",
                    "angle_min", "angle_max", "angle_increment",
                    "time_increment", "scan_time",
                    "range_min", "range_max",
                    "ranges_n", "intensities_n",
                ]
                cols += ["r_%d" % i for i in range(n)]
                cols += ["i_%d" % i for i in range(m)]
                w.writerow(cols)
                header_written = True
            row = [
                "%.9f" % t.to_sec(),
                "%.9f" % msg.header.stamp.to_sec(),
                msg.header.frame_id,
                msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.time_increment, msg.scan_time,
                msg.range_min, msg.range_max,
                n, m,
            ]
            row += list(msg.ranges)
            row += list(msg.intensities)
            w.writerow(row)
            count += 1
    return count


def write_multiarray(bag, topic, out_path):
    """Works for both Float32MultiArray and Float64MultiArray (layout identical)."""
    count = 0
    with open(out_path, "w") as f:
        w = csv.writer(f)
        header_written = False
        for _, msg, t in bag.read_messages(topics=[topic]):
            n = len(msg.data)
            if not header_written:
                cols = ["bag_time", "data_n"]
                cols += ["v_%d" % i for i in range(n)]
                w.writerow(cols)
                header_written = True
            row = ["%.9f" % t.to_sec(), n]
            row += list(msg.data)
            w.writerow(row)
            count += 1
    return count


def write_header(bag, topic, out_path):
    count = 0
    with open(out_path, "w") as f:
        w = csv.writer(f)
        w.writerow(["bag_time", "stamp_sec", "stamp_nsec", "frame_id"])
        for _, msg, t in bag.read_messages(topics=[topic]):
            w.writerow([
                "%.9f" % t.to_sec(),
                "%.9f" % msg.stamp.to_sec(),
                msg.stamp.nsecs,
                msg.frame_id,
            ])
            count += 1
    return count


WRITERS = {
    "laserscan":  write_laserscan,
    "multiarray": write_multiarray,
    "header":     write_header,
}


# ------------------------------------------------------------
# main
# ------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        sys.stderr.write("Usage: %s <input.bag> [output_dir]\n" % sys.argv[0])
        sys.exit(1)

    bag_path = sys.argv[1]
    if len(sys.argv) > 2:
        out_dir = sys.argv[2]
    else:
        out_dir = os.path.splitext(os.path.abspath(bag_path))[0] + "_csv"

    if not os.path.exists(bag_path):
        sys.stderr.write("ERROR: bag not found: %s\n" % bag_path)
        sys.exit(1)

    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    print("[bag_to_csv] input bag : %s" % bag_path)
    print("[bag_to_csv] output dir: %s" % out_dir)

    try:
        bag = rosbag.Bag(bag_path, "r")
    except Exception as e:
        sys.stderr.write("ERROR: failed to open bag: %s\n" % e)
        sys.exit(1)

    # 列出 bag 中实际出现的话题,供判断缺失
    try:
        topic_info = bag.get_type_and_topic_info()
        available = set(topic_info.topics.keys())
        print("[bag_to_csv] topics in bag (%d):" % len(available))
        for tp in sorted(available):
            nmsgs = topic_info.topics[tp].message_count
            ttype = topic_info.topics[tp].msg_type
            print("    %-40s  type=%-30s  msgs=%d" % (tp, ttype, nmsgs))
    except Exception:
        available = None
        print("[bag_to_csv] (cannot enumerate topic info; proceeding anyway)")

    print("")
    summary = []
    for topic, filename, tag in EXPORT_MAP:
        out_path = os.path.join(out_dir, filename)

        if available is not None and topic not in available:
            print("[skip] %s  (not in bag)" % topic)
            summary.append((topic, filename, 0, "not in bag"))
            continue

        writer = WRITERS.get(tag)
        if writer is None:
            print("[skip] %s  (unknown tag: %s)" % (topic, tag))
            summary.append((topic, filename, 0, "unknown tag"))
            continue

        try:
            n = writer(bag, topic, out_path)
            print("[ok]   %-40s -> %-40s  %7d msgs" % (topic, filename, n))
            summary.append((topic, filename, n, "ok"))
        except Exception as e:
            traceback.print_exc()
            print("[err]  %s: %s" % (topic, e))
            summary.append((topic, filename, 0, "error: " + str(e)))

    bag.close()

    print("\n--- summary ---")
    for topic, filename, n, status in summary:
        print("  %-40s -> %-40s  %7d  [%s]"
              % (topic, filename, n, status))


if __name__ == "__main__":
    main()
