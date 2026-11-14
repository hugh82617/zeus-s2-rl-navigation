#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
============================================================
Pipeline Stage 1/3 : laser_switch  (v2)
============================================================
职责:原始激光数据方向校正入口。

输入 : /scan                (sensor_msgs/LaserScan)
输出 : /scan_aligned        (sensor_msgs/LaserScan)

处理:
  - 对 ranges / intensities 做 180° 半圈平移重排:
        new[i] = old[(i + N/2) % N]
    即:前半段暂存 -> 后半段前移 -> 前半段后接。
  - 不改变帧率、角度分辨率、angle_min/max、scan_time 等元数据,
    仅重排索引。
  - 可选地重写输出 frame_id 为 laser_link_aligned,用于下游 TF 对齐。

CHANGELOG v2(对齐性验证报告 C1.1 的"55 帧配对缺口"诊断):
  * 订阅/发布队列从 10 提到 50 —— 高峰期不再被 ROS 丢包
  * Subscriber 加 tcp_nodelay=True —— 压低端到端延迟,减少缓冲
  * 增加 recv/pub 计数统计日志,每 scan_stats_every 帧打印一次,
    便于将"raw -> aligned 丢帧"定位在节点侧还是录制/bag 侧:
      - 若 recv_count == pub_count 但 CSV 里 raw > aligned,
        那 55 帧丢失一定是 scan_aligned_recorder 或 rosbag 漏录;
      - 若 recv_count > pub_count,才是节点自身的问题。

管线定位:LiDAR 观测预处理链第 1 级,下游为 lidar_obs_single_frame。
============================================================
"""

import rospy
from sensor_msgs.msg import LaserScan


class LaserSwitch(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_scan_topic", "/scan")
        self.output_topic = rospy.get_param("~output_scan_topic", "/scan_aligned")
        self.output_frame_id = rospy.get_param("~output_frame_id", "laser_link_aligned")
        self.verbose = rospy.get_param("~verbose", True)
        # v2: 队列与统计参数
        self.sub_queue_size = int(rospy.get_param("~sub_queue_size", 50))
        self.pub_queue_size = int(rospy.get_param("~pub_queue_size", 50))
        self.tcp_nodelay    = bool(rospy.get_param("~tcp_nodelay", True))
        self.stats_every    = int(rospy.get_param("~stats_every", 200))

        # v2: 统计计数器
        self.recv_count = 0
        self.pub_count  = 0
        self.empty_count = 0

        self.pub = rospy.Publisher(self.output_topic, LaserScan,
                                   queue_size=self.pub_queue_size)
        self.sub = rospy.Subscriber(self.input_topic, LaserScan,
                                    self.scan_callback,
                                    queue_size=self.sub_queue_size,
                                    tcp_nodelay=self.tcp_nodelay)

        rospy.logwarn("======================================")
        rospy.logwarn("[laser_switch] node started (pipeline stage 1/3, v2)")
        rospy.logwarn("  input topic   : %s", self.input_topic)
        rospy.logwarn("  output topic  : %s", self.output_topic)
        rospy.logwarn("  output frame  : %s", self.output_frame_id)
        rospy.logwarn("  sub_queue     : %d   pub_queue: %d",
                      self.sub_queue_size, self.pub_queue_size)
        rospy.logwarn("  tcp_nodelay   : %s", str(self.tcp_nodelay))
        rospy.logwarn("  stats_every   : %d frames", self.stats_every)
        rospy.logwarn("======================================")

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    @staticmethod
    def rotate_half_turn(data_list):
        """对长度为 N 的数组做 180° 索引重排。"""
        n = len(data_list)
        if n == 0:
            return data_list

        half = n // 2
        if n % 2 != 0:
            rospy.logwarn_throttle(
                5.0,
                "[laser_switch] scan length is odd: %d, half-turn shift will have 1-sample mismatch.",
                n,
            )

        first_half = data_list[:half]
        new_list = data_list[half:]
        new_list.extend(first_half)
        return new_list

    @staticmethod
    def safe_value_str(value):
        """便于日志打印 inf / nan。"""
        try:
            if value == float("inf"):
                return "inf"
            if value != value:   # NaN
                return "nan"
            return "%.3f" % value
        except Exception:
            return str(value)

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        self.recv_count += 1
        n = len(msg.ranges)
        if n == 0:
            self.empty_count += 1
            rospy.logwarn_throttle(2.0, "[laser_switch] Empty scan received.")
            return

        old_ranges = list(msg.ranges)
        old_intensities = list(msg.intensities)

        new_ranges = self.rotate_half_turn(old_ranges)
        if len(old_intensities) == n:
            new_intensities = self.rotate_half_turn(old_intensities)
        else:
            new_intensities = old_intensities  # 长度不一致则原样透传

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
        self.pub_count += 1

        # v2: 周期性统计日志 —— 用来定位 C1.1 的 55 帧配对缺口发生在何处
        if self.stats_every > 0 and self.recv_count % self.stats_every == 0:
            dropped = self.recv_count - self.pub_count - self.empty_count
            rospy.logwarn(
                "[laser_switch][stats] recv=%d pub=%d empty=%d node_dropped=%d "
                "(若 pub 等于下游录到的行数则无节点丢帧)",
                self.recv_count, self.pub_count, self.empty_count, dropped,
            )

        if self.verbose:
            idx_0 = int(round((0.0 - msg.angle_min) / msg.angle_increment))
            idx_0 = max(0, min(n - 1, idx_0))
            idx_neg180 = 0
            idx_pos180 = n - 1

            rospy.logwarn_throttle(
                1.0,
                "[laser_switch] raw: 0deg=%s -180deg=%s +180deg=%s | aligned: 0deg=%s -180deg=%s +180deg=%s",
                self.safe_value_str(old_ranges[idx_0]),
                self.safe_value_str(old_ranges[idx_neg180]),
                self.safe_value_str(old_ranges[idx_pos180]),
                self.safe_value_str(new_ranges[idx_0]),
                self.safe_value_str(new_ranges[idx_neg180]),
                self.safe_value_str(new_ranges[idx_pos180]),
            )


if __name__ == "__main__":
    rospy.init_node("laser_switch")
    LaserSwitch()
    rospy.spin()
