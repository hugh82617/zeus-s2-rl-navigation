#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function

import math
import threading

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class LidarObsSingleFrameNode(object):
    """
    将 laser_link_aligned 输出的 LaserScan.ranges 转成模型单帧 LiDAR 输入格式。

    当前版本按你的新要求实现：
    1) 只转换一帧，不做 3 帧 stack
    2) 每次收到一帧 LaserScan，就立刻输出一帧 181 维结果
    3) 输出频率严格跟随 laser_link_aligned 的发布频率
    4) 视场裁剪到 [-135°, +135°]
    5) 重采样为 181 维
    6) inf / nan / 越界值 -> max_distance
    7) 最终归一化方式：value / 16.0

    注意：
    - 你当前要求的归一化上限是 16m，这与之前 obs_stage3.py 中 max_distance=12.0 的训练设定不同。
    - 这个节点按你“现在的部署要求”做，输出单帧 181 维归一化结果。
    """

    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_aligned")
        self.output_topic = rospy.get_param("~output_topic", "/rl/lidar_181")

        self.fov_min_deg = float(rospy.get_param("~fov_min_deg", -135.0))
        self.fov_max_deg = float(rospy.get_param("~fov_max_deg", 135.0))
        self.num_beams = int(rospy.get_param("~num_beams", 181))
        self.max_distance = float(rospy.get_param("~max_distance", 16.0))
        self.use_linear_interpolation = bool(rospy.get_param("~use_linear_interpolation", True))
        self.reverse_output = bool(rospy.get_param("~reverse_output", False))
        self.publish_debug_info = bool(rospy.get_param("~publish_debug_info", True))

        self.lock = threading.Lock()
        self.scan_count = 0

        self.target_angles = self._build_target_angles_rad(
            self.fov_min_deg, self.fov_max_deg, self.num_beams
        )

        self.pub = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)

        rospy.logwarn(
            "[lidar_obs_single_frame] started | scan_topic=%s | output_topic=%s | "
            "num_beams=%d | fov=[%.1f, %.1f] deg | max_distance=%.3f | interp=%s | reverse=%s",
            self.scan_topic,
            self.output_topic,
            self.num_beams,
            self.fov_min_deg,
            self.fov_max_deg,
            self.max_distance,
            str(self.use_linear_interpolation),
            str(self.reverse_output),
        )

    @staticmethod
    def _build_target_angles_rad(min_deg, max_deg, num_beams):
        if num_beams <= 1:
            return [math.radians(min_deg)]
        step = (max_deg - min_deg) / float(num_beams - 1)
        return [math.radians(min_deg + i * step) for i in range(num_beams)]

    def _sanitize_range(self, r):
        if r is None:
            return self.max_distance
        if math.isnan(r) or math.isinf(r):
            return self.max_distance
        if r < 0.0:
            return self.max_distance
        if r > self.max_distance:
            return self.max_distance
        return r

    def _get_range_nearest(self, scan, angle):
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        if idx < 0:
            idx = 0
        elif idx >= len(scan.ranges):
            idx = len(scan.ranges) - 1
        return self._sanitize_range(scan.ranges[idx])

    def _get_range_interp(self, scan, angle):
        pos = (angle - scan.angle_min) / scan.angle_increment

        if pos <= 0.0:
            return self._sanitize_range(scan.ranges[0])
        if pos >= len(scan.ranges) - 1:
            return self._sanitize_range(scan.ranges[-1])

        i0 = int(math.floor(pos))
        i1 = i0 + 1
        w = pos - i0

        r0 = self._sanitize_range(scan.ranges[i0])
        r1 = self._sanitize_range(scan.ranges[i1])
        return (1.0 - w) * r0 + w * r1

    def _resample_single_frame(self, scan):
        if not scan.ranges:
            return [1.0] * self.num_beams

        beams = []
        for angle in self.target_angles:
            if self.use_linear_interpolation:
                r = self._get_range_interp(scan, angle)
            else:
                r = self._get_range_nearest(scan, angle)

            # 归一化到 [0, 1]，按你的最新要求除以 16m
            beams.append(r / self.max_distance)

        if self.reverse_output:
            beams.reverse()
        return beams

    @staticmethod
    def _make_multiarray(data, dim_label):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label=dim_label, size=len(data), stride=len(data))
        ]
        msg.layout.data_offset = 0
        msg.data = data
        return msg

    def scan_callback(self, scan_msg):
        with self.lock:
            lidar_181 = self._resample_single_frame(scan_msg)
            self.pub.publish(self._make_multiarray(lidar_181, "lidar_181"))

            self.scan_count += 1
            if self.publish_debug_info and self.scan_count % 20 == 1:
                center_idx = len(lidar_181) // 2
                rospy.logwarn(
                    "[lidar_obs_single_frame] seq=%d raw_n=%d -> out_n=%d | min=%.4f max=%.4f center=%.4f",
                    self.scan_count,
                    len(scan_msg.ranges),
                    len(lidar_181),
                    min(lidar_181) if lidar_181 else -1.0,
                    max(lidar_181) if lidar_181 else -1.0,
                    lidar_181[center_idx] if lidar_181 else -1.0,
                )


def main():
    rospy.init_node("lidar_obs_single_frame", anonymous=False)
    LidarObsSingleFrameNode()
    rospy.spin()


if __name__ == "__main__":
    main()
