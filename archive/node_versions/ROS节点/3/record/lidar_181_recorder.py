#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import json
import math
import rospy
from std_msgs.msg import Float32MultiArray


class Lidar181Recorder(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/rl/lidar_181")
        self.record_dir = rospy.get_param("~record_dir", "/home/ros/lidar/record")
        self.record_file = rospy.get_param("~record_file", "lidar_181_log.jsonl")
        self.record_full_data = bool(rospy.get_param("~record_full_data", True))
        self.log_every_n = int(rospy.get_param("~log_every_n", 1))
        self.print_summary = bool(rospy.get_param("~print_summary", True))
        self.expected_len = int(rospy.get_param("~expected_len", 181))

        self.msg_count = 0
        self.fp = None

        if not os.path.exists(self.record_dir):
            os.makedirs(self.record_dir)

        self.record_path = os.path.join(self.record_dir, self.record_file)
        self.fp = open(self.record_path, "a")

        rospy.logwarn("======================================")
        rospy.logwarn(" lidar_181_recorder node started")
        rospy.logwarn(" input_topic    : %s", self.input_topic)
        rospy.logwarn(" record_path    : %s", self.record_path)
        rospy.logwarn(" full_data      : %s", str(self.record_full_data))
        rospy.logwarn(" expected_len   : %d", self.expected_len)
        rospy.logwarn(" log_every_n    : %d", self.log_every_n)
        rospy.logwarn("======================================")

        self.sub = rospy.Subscriber(self.input_topic, Float32MultiArray, self.callback, queue_size=1)

    @staticmethod
    def _safe_float(value):
        try:
            if math.isnan(value):
                return "nan"
            if math.isinf(value):
                return "inf" if value > 0 else "-inf"
            return float(value)
        except Exception:
            return str(value)

    def summarize(self, msg):
        data = list(msg.data)
        n = len(data)
        finite = [x for x in data if isinstance(x, (int, float)) and not math.isnan(x) and not math.isinf(x)]
        zero_count = sum(1 for x in data if x == 0.0)

        summary = {
            "stamp": rospy.Time.now().to_sec(),
            "length": n,
            "expected_len": self.expected_len,
            "length_ok": (n == self.expected_len),
            "min": self._safe_float(min(finite) if finite else None),
            "max": self._safe_float(max(finite) if finite else None),
            "mean": self._safe_float(sum(finite) / float(len(finite)) if finite else None),
            "zero_count": zero_count,
            "first": self._safe_float(data[0] if n > 0 else None),
            "center": self._safe_float(data[n // 2] if n > 0 else None),
            "last": self._safe_float(data[-1] if n > 0 else None),
            "layout_dim": [
                {
                    "label": d.label,
                    "size": d.size,
                    "stride": d.stride,
                }
                for d in msg.layout.dim
            ],
        }

        if self.record_full_data:
            summary["data"] = [self._safe_float(x) for x in data]
        return summary

    def callback(self, msg):
        self.msg_count += 1
        if self.log_every_n > 1 and (self.msg_count % self.log_every_n != 0):
            return

        summary = self.summarize(msg)
        self.fp.write(json.dumps(summary, ensure_ascii=False) + "\n")
        self.fp.flush()

        if self.print_summary:
            rospy.logwarn_throttle(
                1.0,
                "[lidar_181_recorder] seq=%d len=%d ok=%s min=%s max=%s zero=%d center=%s",
                self.msg_count,
                summary["length"],
                str(summary["length_ok"]),
                str(summary["min"]),
                str(summary["max"]),
                summary["zero_count"],
                str(summary["center"])
            )

    def shutdown(self):
        try:
            if self.fp is not None:
                self.fp.close()
                rospy.logwarn("[lidar_181_recorder] file closed: %s", self.record_path)
        except Exception as e:
            rospy.logerr("[lidar_181_recorder] close file failed: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("lidar_181_recorder")
    node = Lidar181Recorder()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
