#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan


class ScanRealign180(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_scan_topic", "/scan")
        self.output_topic = rospy.get_param("~output_scan_topic", "/scan_aligned")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")  # 留空则沿用原 frame_id
        self.verbose = rospy.get_param("~verbose", True)

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback, queue_size=10)

        rospy.logwarn("======================================")
        rospy.logwarn(" scan_realign_180 node started")
        rospy.logwarn(" input topic : %s", self.input_topic)
        rospy.logwarn(" output topic: %s", self.output_topic)
        rospy.logwarn("======================================")

    def rotate_half_turn(self, data_list):
        """
        将长度为 N 的数组做半圈平移：
        1. 先保存前 N/2 段
        2. 再把后 N/2 段移到前面
        3. 最后把前 N/2 段接到后面
        这里只对数组顺序做重排，核心对象是 ranges。
        """
        n = len(data_list)
        if n == 0:
            return data_list

        half = n // 2

        if n % 2 != 0:
            rospy.logwarn_throttle(
                5.0,
                "[scan_realign_180] scan length is odd: %d, half-turn shift will have 1-sample mismatch.",
                n,
            )

        first_half = data_list[:half]
        new_list = data_list[half:]
        new_list.extend(first_half)
        return new_list

    def safe_value_str(self, value):
        if isinstance(value, float):
            if math.isinf(value):
                return "inf"
            if math.isnan(value):
                return "nan"
            return "%.3f" % value
        return str(value)

    def scan_callback(self, msg):
        n = len(msg.ranges)
        if n == 0:
            rospy.logwarn_throttle(2.0, "[scan_realign_180] Empty scan received.")
            return

        old_ranges = list(msg.ranges)
        new_ranges = self.rotate_half_turn(old_ranges)

        out = LaserScan()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = new_ranges

        # 如果你后续有模块依赖 intensities，也可以同步重排；
        # 当前按你的要求，核心改 ranges，intensities 原样带过去。
        out.intensities = msg.intensities

        self.pub.publish(out)

        if self.verbose:
            idx_0 = int(round((0.0 - msg.angle_min) / msg.angle_increment))
            idx_0 = max(0, min(n - 1, idx_0))
            idx_neg180 = 0
            idx_pos180 = n - 1

            raw_0 = old_ranges[idx_0]
            raw_neg180 = old_ranges[idx_neg180]
            raw_pos180 = old_ranges[idx_pos180]

            new_0 = new_ranges[idx_0]
            new_neg180 = new_ranges[idx_neg180]
            new_pos180 = new_ranges[idx_pos180]

            rospy.logwarn_throttle(
                1.0,
                "[scan_realign_180] raw: 0deg=%s, -180deg=%s, +180deg=%s | aligned: 0deg=%s, -180deg=%s, +180deg=%s",
                self.safe_value_str(raw_0),
                self.safe_value_str(raw_neg180),
                self.safe_value_str(raw_pos180),
                self.safe_value_str(new_0),
                self.safe_value_str(new_neg180),
                self.safe_value_str(new_pos180),
            )


if __name__ == "__main__":
    rospy.init_node("scan_realign_180")
    node = ScanRealign180()
    rospy.spin()
