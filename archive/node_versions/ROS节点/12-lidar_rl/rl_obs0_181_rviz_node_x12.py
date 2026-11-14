#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, print_function

import math
import threading

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_NUM_BEAMS = 181
DEFAULT_FOV_MIN_DEG = -135.0
DEFAULT_FOV_MAX_DEG = 135.0
DEFAULT_MAX_DISTANCE = 16.0
DEFAULT_RANGE_MIN = 0.05
DEFAULT_FRONT_FOV_DEG = 20.0
DEFAULT_RESTORE_SCALE = 12.0


class RlObs181RvizNode(object):
    def __init__(self):
        rospy.init_node("rl_obs0_181_rviz_node", anonymous=False)

        # ---------------- topics ----------------
        self.input_topic = rospy.get_param("~input_topic", "/rl/debug/obs0_181")
        self.stamp_topic = rospy.get_param("~stamp_topic", "/rl/lidar_181_stamp")
        self.repair_mask_topic = rospy.get_param("~repair_mask_topic", "/rl/lidar_181_repair_mask")
        self.fill_stats_topic = rospy.get_param("~fill_stats_topic", "/rl/lidar_181_fill_stats")

        self.output_scan_topic = rospy.get_param("~output_scan_topic", "/rl/debug/obs0_181_scan_vis")
        self.output_marker_topic = rospy.get_param("~output_marker_topic", "/rl/debug/obs0_181_markers_vis")

        # ---------------- geometry ----------------
        self.output_frame_id = rospy.get_param("~output_frame_id", "laser_link_aligned")
        self.num_beams = int(rospy.get_param("~num_beams", DEFAULT_NUM_BEAMS))
        self.fov_min_deg = float(rospy.get_param("~fov_min_deg", DEFAULT_FOV_MIN_DEG))
        self.fov_max_deg = float(rospy.get_param("~fov_max_deg", DEFAULT_FOV_MAX_DEG))
        self.max_distance = float(rospy.get_param("~max_distance", DEFAULT_MAX_DISTANCE))
        self.range_min = float(rospy.get_param("~range_min", DEFAULT_RANGE_MIN))
        self.reverse_output = bool(rospy.get_param("~reverse_output", False))
        self.front_fov_deg = float(rospy.get_param("~front_fov_deg", DEFAULT_FRONT_FOV_DEG))
        # 将输入的 181 维 RL 激光数据恢复为真实显示距离时使用的比例因子
        # 本次默认按你的要求设置为 12。
        self.restore_scale = float(rospy.get_param("~restore_scale", DEFAULT_RESTORE_SCALE))

        # ---------------- publish options ----------------
        self.publish_scan = bool(rospy.get_param("~publish_scan", True))
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))
        self.use_stamp_time = bool(rospy.get_param("~use_stamp_time", True))
        self.header_timeout_sec = float(rospy.get_param("~header_timeout_sec", 0.5))

        # ---------------- marker style ----------------
        self.points_scale = float(rospy.get_param("~points_scale", 0.05))
        self.highlight_scale = float(rospy.get_param("~highlight_scale", 0.10))
        self.text_scale = float(rospy.get_param("~text_scale", 0.18))
        self.text_z = float(rospy.get_param("~text_z", 0.35))
        self.show_text = bool(rospy.get_param("~show_text", True))

        self._lock = threading.Lock()
        self._latest_header = None
        self._latest_header_walltime = None
        self._latest_repair_mask = None
        self._latest_fill_stats = None

        self._angles = self._build_angles()
        self._center_idx = self.num_beams // 2
        self._front_half_idx = self._compute_front_half_idx()

        self.pub_scan = rospy.Publisher(self.output_scan_topic, LaserScan, queue_size=20)
        self.pub_markers = rospy.Publisher(self.output_marker_topic, MarkerArray, queue_size=20)

        rospy.Subscriber(self.stamp_topic, Header, self._stamp_cb, queue_size=50)
        rospy.Subscriber(self.repair_mask_topic, Float32MultiArray, self._repair_mask_cb, queue_size=20)
        rospy.Subscriber(self.fill_stats_topic, Float32MultiArray, self._fill_stats_cb, queue_size=20)
        rospy.Subscriber(self.input_topic, Float32MultiArray, self._obs_cb, queue_size=50)

        rospy.logwarn("======================================")
        rospy.logwarn("[rl_obs0_181_rviz] started")
        rospy.logwarn("  input_topic         = %s", self.input_topic)
        rospy.logwarn("  stamp_topic         = %s", self.stamp_topic)
        rospy.logwarn("  repair_mask_topic   = %s", self.repair_mask_topic)
        rospy.logwarn("  fill_stats_topic    = %s", self.fill_stats_topic)
        rospy.logwarn("  output_scan_topic   = %s", self.output_scan_topic)
        rospy.logwarn("  output_marker_topic = %s", self.output_marker_topic)
        rospy.logwarn("  output_frame_id     = %s", self.output_frame_id)
        rospy.logwarn("  num_beams           = %d", self.num_beams)
        rospy.logwarn("  fov_deg             = [%.1f, %.1f]", self.fov_min_deg, self.fov_max_deg)
        rospy.logwarn("  max_distance        = %.3f", self.max_distance)
        rospy.logwarn("  range_min           = %.3f", self.range_min)
        rospy.logwarn("  restore_scale       = %.3f", self.restore_scale)
        rospy.logwarn("  reverse_output      = %s", str(self.reverse_output))
        rospy.logwarn("  use_stamp_time      = %s", str(self.use_stamp_time))
        rospy.logwarn("======================================")

    def _build_angles(self):
        if self.num_beams <= 1:
            return [math.radians(self.fov_min_deg)]
        step = (self.fov_max_deg - self.fov_min_deg) / float(self.num_beams - 1)
        return [math.radians(self.fov_min_deg + i * step) for i in range(self.num_beams)]

    def _compute_front_half_idx(self):
        if self.num_beams <= 1:
            return 0
        deg_per_beam = (self.fov_max_deg - self.fov_min_deg) / float(self.num_beams - 1)
        if deg_per_beam <= 0.0:
            return 1
        return max(1, int(round(self.front_fov_deg / deg_per_beam)))

    def _stamp_cb(self, msg):
        with self._lock:
            self._latest_header = msg
            self._latest_header_walltime = rospy.Time.now()

    def _repair_mask_cb(self, msg):
        with self._lock:
            self._latest_repair_mask = list(msg.data)

    def _fill_stats_cb(self, msg):
        with self._lock:
            self._latest_fill_stats = list(msg.data)

    def _get_effective_header(self):
        now = rospy.Time.now()
        with self._lock:
            header_msg = self._latest_header
            header_wall = self._latest_header_walltime

        out = Header()
        out.frame_id = self.output_frame_id
        if self.use_stamp_time and header_msg is not None and header_wall is not None:
            age = (now - header_wall).to_sec()
            if age <= self.header_timeout_sec:
                out.stamp = header_msg.stamp
            else:
                out.stamp = now
        else:
            out.stamp = now
        return out

    def _sanitize_and_restore_ranges(self, data):
        ranges = []
        for v in data:
            try:
                # 原先这里是乘 max_distance；现在改为按固定恢复比例乘 12（可通过参数改）
                r = float(v) * self.restore_scale
            except Exception:
                r = self.max_distance
            if math.isnan(r) or math.isinf(r):
                r = self.max_distance
            if r < self.range_min:
                r = self.range_min
            if r > self.max_distance:
                r = self.max_distance
            ranges.append(r)
        return ranges

    def _compute_points(self, ranges):
        pts = []
        for i, r in enumerate(ranges):
            ang = self._angles[i]
            p = Point()
            p.x = r * math.cos(ang)
            p.y = r * math.sin(ang)
            p.z = 0.0
            pts.append(p)
        return pts

    def _find_min_idx(self, ranges, lo, hi):
        best_idx = lo
        best_val = ranges[lo]
        i = lo + 1
        while i < hi:
            if ranges[i] < best_val:
                best_val = ranges[i]
                best_idx = i
            i += 1
        return best_idx, best_val

    def _make_points_marker(self, header, marker_id, points):
        mk = Marker()
        mk.header = header
        mk.ns = "obs0_181_points"
        mk.id = marker_id
        mk.type = Marker.POINTS
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = self.points_scale
        mk.scale.y = self.points_scale
        mk.color.r = 0.10
        mk.color.g = 0.95
        mk.color.b = 0.10
        mk.color.a = 0.95
        mk.points = points
        return mk

    def _make_sphere_marker(self, header, marker_id, ns, point, r, g, b):
        mk = Marker()
        mk.header = header
        mk.ns = ns
        mk.id = marker_id
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position = point
        mk.pose.orientation.w = 1.0
        mk.scale.x = self.highlight_scale
        mk.scale.y = self.highlight_scale
        mk.scale.z = self.highlight_scale
        mk.color.r = r
        mk.color.g = g
        mk.color.b = b
        mk.color.a = 0.95
        return mk

    def _make_text_marker(self, header, marker_id, text):
        mk = Marker()
        mk.header = header
        mk.ns = "obs0_181_text"
        mk.id = marker_id
        mk.type = Marker.TEXT_VIEW_FACING
        mk.action = Marker.ADD
        mk.pose.position.x = 0.0
        mk.pose.position.y = 0.0
        mk.pose.position.z = self.text_z
        mk.pose.orientation.w = 1.0
        mk.scale.z = self.text_scale
        mk.color.r = 1.0
        mk.color.g = 1.0
        mk.color.b = 1.0
        mk.color.a = 0.95
        mk.text = text
        return mk

    def _build_marker_array(self, header, points, min_idx, min_val, front_idx, front_val):
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_array.markers.append(self._make_points_marker(header, 0, points))
        marker_array.markers.append(self._make_sphere_marker(header, 1, "closest_point", points[min_idx], 1.0, 0.15, 0.15))
        marker_array.markers.append(self._make_sphere_marker(header, 2, "front_min_point", points[front_idx], 1.0, 0.95, 0.10))

        if self.show_text:
            with self._lock:
                repair_mask = self._latest_repair_mask
                fill_stats = self._latest_fill_stats

            repair_count = -1
            if repair_mask is not None and len(repair_mask) == self.num_beams:
                repair_count = 0
                for v in repair_mask:
                    if v >= 0.5:
                        repair_count += 1

            txt = []
            txt.append("obs0_181")
            txt.append("min={:.3f}m idx={}".format(min_val, min_idx))
            txt.append("front_min={:.3f}m idx={}".format(front_val, front_idx))
            txt.append("restore_scale={:.3f}".format(self.restore_scale))
            if repair_count >= 0:
                txt.append("repair_count={}".format(repair_count))
            if fill_stats is not None and len(fill_stats) >= 4:
                txt.append("repair_ratio={:.3f} front_repair={} severe={:.3f}".format(
                    float(fill_stats[0]), int(fill_stats[2]), float(fill_stats[3])
                ))
            marker_array.markers.append(self._make_text_marker(header, 3, "\n".join(txt)))

        return marker_array

    def _obs_cb(self, msg):
        data = list(msg.data)
        if len(data) != self.num_beams:
            rospy.logwarn_throttle(1.0,
                                   "[rl_obs0_181_rviz] input length mismatch: got %d expected %d",
                                   len(data), self.num_beams)
            return

        if self.reverse_output:
            data.reverse()

        ranges = self._sanitize_and_restore_ranges(data)
        header = self._get_effective_header()

        if self.publish_scan:
            scan = LaserScan()
            scan.header = header
            scan.angle_min = math.radians(self.fov_min_deg)
            scan.angle_max = math.radians(self.fov_max_deg)
            if self.num_beams > 1:
                scan.angle_increment = (scan.angle_max - scan.angle_min) / float(self.num_beams - 1)
            else:
                scan.angle_increment = 0.0
            scan.time_increment = 0.0
            scan.scan_time = 0.0
            scan.range_min = self.range_min
            scan.range_max = self.max_distance
            scan.ranges = ranges
            scan.intensities = []
            self.pub_scan.publish(scan)

        if self.publish_markers:
            points = self._compute_points(ranges)
            min_idx, min_val = self._find_min_idx(ranges, 0, self.num_beams)
            lo = max(0, self._center_idx - self._front_half_idx)
            hi = min(self.num_beams, self._center_idx + self._front_half_idx + 1)
            front_idx, front_val = self._find_min_idx(ranges, lo, hi)
            marker_array = self._build_marker_array(header, points, min_idx, min_val, front_idx, front_val)
            self.pub_markers.publish(marker_array)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RlObs181RvizNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
