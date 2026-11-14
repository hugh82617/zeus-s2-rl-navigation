#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import math
import json
import rospy
from sensor_msgs.msg import LaserScan


class ScanAlignedRecorder(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_aligned")
        self.record_dir = rospy.get_param("~record_dir", "/home/ros/lidar/record")
        self.record_file = rospy.get_param("~record_file", "scan_aligned_log.jsonl")
        self.record_full_ranges = bool(rospy.get_param("~record_full_ranges", False))
        self.record_full_intensities = bool(rospy.get_param("~record_full_intensities", False))
        self.log_every_n = int(rospy.get_param("~log_every_n", 1))
        self.print_summary = bool(rospy.get_param("~print_summary", True))

        self.msg_count = 0
        self.fp = None

        if not os.path.exists(self.record_dir):
            os.makedirs(self.record_dir)

        self.record_path = os.path.join(self.record_dir, self.record_file)
        self.fp = open(self.record_path, "a")

        rospy.logwarn("======================================")
        rospy.logwarn(" scan_aligned_recorder node started")
        rospy.logwarn(" scan_topic      : %s", self.scan_topic)
        rospy.logwarn(" record_path     : %s", self.record_path)
        rospy.logwarn(" full_ranges     : %s", str(self.record_full_ranges))
        rospy.logwarn(" full_intensites : %s", str(self.record_full_intensities))
        rospy.logwarn(" log_every_n     : %d", self.log_every_n)
        rospy.logwarn("======================================")

        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)

    @staticmethod
    def safe_value(value):
        try:
            if value is None:
                return None
            if math.isnan(value):
                return "nan"
            if math.isinf(value):
                return "inf" if value > 0 else "-inf"
            return float(value)
        except Exception:
            return str(value)

    @staticmethod
    def get_angle_value(ranges, angle_min, angle_increment, target_deg):
        if not ranges or angle_increment == 0.0:
            return None
        target_rad = math.radians(target_deg)
        idx = int(round((target_rad - angle_min) / angle_increment))
        idx = max(0, min(len(ranges) - 1, idx))
        return ranges[idx]

    def summarize_scan(self, msg):
        ranges = list(msg.ranges)
        intensities = list(msg.intensities)
        n = len(ranges)

        valid_count = 0
        inf_count = 0
        nan_count = 0
        neg_count = 0
        under_count = 0
        over_count = 0
        finite_values = []

        for r in ranges:
            if r is None:
                nan_count += 1
                continue
            if math.isnan(r):
                nan_count += 1
            elif math.isinf(r):
                inf_count += 1
            elif r < 0.0:
                neg_count += 1
            elif r < msg.range_min:
                under_count += 1
            elif r > msg.range_max:
                over_count += 1
            else:
                valid_count += 1
                finite_values.append(r)

        summary = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "count": n,
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment": msg.time_increment,
            "scan_time": msg.scan_time,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "valid_count": valid_count,
            "inf_count": inf_count,
            "nan_count": nan_count,
            "neg_count": neg_count,
            "under_count": under_count,
            "over_count": over_count,
            "min_valid": self.safe_value(min(finite_values) if finite_values else None),
            "max_valid": self.safe_value(max(finite_values) if finite_values else None),
            "intensities_count": len(intensities),
            "sample_deg": {
                "-180": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, -180)),
                "-135": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, -135)),
                "-90": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, -90)),
                "0": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, 0)),
                "90": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, 90)),
                "135": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, 135)),
                "180": self.safe_value(self.get_angle_value(ranges, msg.angle_min, msg.angle_increment, 180)),
            }
        }

        if self.record_full_ranges:
            summary["ranges"] = [self.safe_value(x) for x in ranges]
        if self.record_full_intensities:
            summary["intensities"] = [self.safe_value(x) for x in intensities]
        return summary

    def scan_callback(self, msg):
        self.msg_count += 1
        if self.log_every_n > 1 and (self.msg_count % self.log_every_n != 0):
            return

        summary = self.summarize_scan(msg)
        self.fp.write(json.dumps(summary, ensure_ascii=False) + "\n")
        self.fp.flush()

        if self.print_summary:
            rospy.logwarn_throttle(
                1.0,
                "[scan_aligned_recorder] seq=%d count=%d valid=%d inf=%d nan=%d neg=%d under=%d over=%d 0deg=%s",
                self.msg_count,
                summary["count"],
                summary["valid_count"],
                summary["inf_count"],
                summary["nan_count"],
                summary["neg_count"],
                summary["under_count"],
                summary["over_count"],
                str(summary["sample_deg"]["0"])
            )

    def shutdown(self):
        try:
            if self.fp is not None:
                self.fp.close()
                rospy.logwarn("[scan_aligned_recorder] file closed: %s", self.record_path)
        except Exception as e:
            rospy.logerr("[scan_aligned_recorder] close file failed: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("scan_aligned_recorder")
    node = ScanAlignedRecorder()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
