#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class LidarStack543Node(object):
    """将 /rl/lidar_181 单帧观测堆叠为 [t0 | t1 | t2] 共 543 维。

    约定：
      - t0: 当前帧
      - t1: 上一帧
      - t2: 上上帧
    与 Stage3 观测定义一致。

    首次收到数据时，为避免虚假历史，使用当前帧填满三个槽：
      [cur | cur | cur]
    之后每来一帧滚动一次：
      t2 <- t1
      t1 <- t0
      t0 <- cur
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/rl/lidar_181")
        self.output_topic = rospy.get_param("~output_topic", "/rl/lidar_543")
        self.expected_len = int(rospy.get_param("~expected_len", 181))
        self.publish_debug = bool(rospy.get_param("~publish_debug_log", False))

        self.t0 = None
        self.t1 = None
        self.t2 = None
        self.msg_count = 0

        self.pub = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Float32MultiArray, self.cb, queue_size=10)

        rospy.logwarn("[lidar_stack_543] started")
        rospy.logwarn("[lidar_stack_543] input_topic  = %s", self.input_topic)
        rospy.logwarn("[lidar_stack_543] output_topic = %s", self.output_topic)
        rospy.logwarn("[lidar_stack_543] expected_len = %d", self.expected_len)

    def _validate_frame(self, data):
        if len(data) != self.expected_len:
            rospy.logerr_throttle(
                1.0,
                "[lidar_stack_543] input length error: got %d, expected %d",
                len(data), self.expected_len,
            )
            return False
        return True

    def _build_msg(self, stacked):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "lidar_543"
        dim.size = len(stacked)
        dim.stride = len(stacked)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = stacked
        return msg

    def reset(self):
        """episode_length_buf <= 1 时调用，重新用 t0 填满 t1/t2。
        与训练侧 obs_stage3.py 的 reset 语义保持一致。
        """
        if self.t0 is not None:
            self.t1 = list(self.t0)
            self.t2 = list(self.t0)
            rospy.logwarn("[lidar_stack_543] reset: refilled t1/t2 with current t0")

    def cb(self, msg):
        cur = list(msg.data)
        if not self._validate_frame(cur):
            return

        # 首帧：三槽全部填当前帧，和训练侧 reset 初始化语义一致
        if self.t0 is None:
            self.t0 = list(cur)
            self.t1 = list(cur)
            self.t2 = list(cur)
        else:
            # BUG FIX: 原代码直接引用赋值，t2/t1 指向同一对象，
            # 导致第3帧 t2（上上帧）与 t1 数据相同，不等于第1帧的当前帧。
            # 必须用 list() 创建独立副本，保证三帧数据完全隔离。
            self.t2 = list(self.t1)   # t2 ← t1（独立副本）
            self.t1 = list(self.t0)   # t1 ← t0（独立副本）
            self.t0 = list(cur)       # t0 ← 当前帧

        stacked = self.t0 + self.t1 + self.t2
        out = self._build_msg(stacked)
        self.pub.publish(out)

        self.msg_count += 1
        if self.publish_debug:
            rospy.logwarn_throttle(
                1.0,
                "[lidar_stack_543] count=%d len=%d | t0[0]=%.4f t1[0]=%.4f t2[0]=%.4f",
                self.msg_count, len(stacked), self.t0[0], self.t1[0], self.t2[0],
            )


if __name__ == "__main__":
    rospy.init_node("lidar_stack_543_node")
    LidarStack543Node()
    rospy.spin()
