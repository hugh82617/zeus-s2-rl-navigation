#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan


class LaserSwitch(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_scan_topic", "/scan")
        self.output_topic = rospy.get_param("~output_scan_topic", "/scan_aligned")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")  # 留空则沿用原 frame_id
        self.verbose = rospy.get_param("~verbose", True)

        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback, queue_size=10)

        rospy.logwarn("======================================")
        rospy.logwarn(" laser_switch node started")
        rospy.logwarn(" input topic : %s", self.input_topic)
        rospy.logwarn(" output topic: %s", self.output_topic)
        rospy.logwarn("======================================")

    def rotate_half_turn(self, data_list):
        """
        将长度为 N 的数组做半圈平移：
        1. 先保存前 N/2 段
        2. 再把后 N/2 段移到前面
        3. 最后把前 N/2 段接到后面
        """
        n = len(data_list)
        if n == 0:
            return data_list

        half = n // 2

        # 如果点数为奇数，严格来说无法精确一分为二
        # 这里仍然按 floor(N/2) 处理，并打印提醒
        if n % 2 != 0:
            rospy.logwarn_throttle(
                5.0,
                "[laser_switch] scan length is odd: %d, half-turn shift will have 1-sample mismatch.",
                n
            )

        # 1) 暂存前半段
        first_half = data_list[:half]

        # 2) 后半段移到前面
        new_list = data_list[half:]

        # 3) 前半段接到后面
        new_list.extend(first_half)

        return new_list

    def safe_value_str(self, value):
        """便于日志打印 inf / nan"""
        try:
            if value == float("inf"):
                return "inf"
            if value != value:
                return "nan"
            return "%.3f" % value
        except Exception:
            return str(value)

    def scan_callback(self, msg):
        n = len(msg.ranges)
        if n == 0:
            rospy.logwarn_throttle(2.0, "[laser_switch] Empty scan received.")
            return

        old_ranges = list(msg.ranges)
        old_intensities = list(msg.intensities)

        # 按“前半暂存 + 后半前移 + 前半接后”的办法进行 180 度重排
        new_ranges = self.rotate_half_turn(old_ranges)

        if len(old_intensities) == n:
            new_intensities = self.rotate_half_turn(old_intensities)
        else:
            new_intensities = old_intensities

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
        out.intensities = new_intensities

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
                "[laser_switch] raw: 0deg=%s, -180deg=%s, +180deg=%s | aligned: 0deg=%s, -180deg=%s, +180deg=%s",
                self.safe_value_str(raw_0),
                self.safe_value_str(raw_neg180),
                self.safe_value_str(raw_pos180),
                self.safe_value_str(new_0),
                self.safe_value_str(new_neg180),
                self.safe_value_str(new_pos180)
            )


if __name__ == "__main__":
    rospy.init_node("laser_switch")
    node = LaserSwitch()
    rospy.spin()
