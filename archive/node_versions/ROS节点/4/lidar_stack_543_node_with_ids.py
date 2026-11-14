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

    额外增加两个调试输出：
      1) /rl/lidar_543_ids
         长度为 3，内容为 [t0_id, t1_id, t2_id]
         可直接验证当前 543 三段分别来自第几帧。
      2) /rl/lidar_543_ids_vis
         长度为 543，将每一段 181 维都填成对应的编号：
         [t0_id * 181 | t1_id * 181 | t2_id * 181]
         这样 rostopic echo 时能直观看到三段编号边界。

    注意：
      当前真正发布到 /rl/lidar_543 的数据顺序仍然严格保持 [t0 | t1 | t2]，
      不改变模型输入语义。
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/rl/lidar_181")
        self.output_topic = rospy.get_param("~output_topic", "/rl/lidar_543")
        self.expected_len = int(rospy.get_param("~expected_len", 181))
        self.publish_debug = bool(rospy.get_param("~publish_debug_log", True))

        self.ids_topic = rospy.get_param("~ids_topic", "/rl/lidar_543_ids")
        self.ids_vis_topic = rospy.get_param("~ids_vis_topic", "/rl/lidar_543_ids_vis")

        self.t0 = None
        self.t1 = None
        self.t2 = None

        self.t0_id = None
        self.t1_id = None
        self.t2_id = None

        self.msg_count = 0
        self.frame_id_counter = 0

        self.pub = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=10)
        self.pub_ids = rospy.Publisher(self.ids_topic, Float32MultiArray, queue_size=10)
        self.pub_ids_vis = rospy.Publisher(self.ids_vis_topic, Float32MultiArray, queue_size=10)

        self.sub = rospy.Subscriber(self.input_topic, Float32MultiArray, self.cb, queue_size=10)

        rospy.logwarn("[lidar_stack_543] started")
        rospy.logwarn("[lidar_stack_543] input_topic    = %s", self.input_topic)
        rospy.logwarn("[lidar_stack_543] output_topic   = %s", self.output_topic)
        rospy.logwarn("[lidar_stack_543] ids_topic      = %s", self.ids_topic)
        rospy.logwarn("[lidar_stack_543] ids_vis_topic  = %s", self.ids_vis_topic)
        rospy.logwarn("[lidar_stack_543] expected_len   = %d", self.expected_len)

    def _validate_frame(self, data):
        if len(data) != self.expected_len:
            rospy.logerr_throttle(
                1.0,
                "[lidar_stack_543] input length error: got %d, expected %d",
                len(data), self.expected_len,
            )
            return False
        return True

    def _build_msg(self, data, label):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = len(data)
        dim.stride = len(data)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = data
        return msg

    def reset(self):
        if self.t0 is not None:
            self.t1 = list(self.t0)
            self.t2 = list(self.t0)
            self.t1_id = self.t0_id
            self.t2_id = self.t0_id
            rospy.logwarn("[lidar_stack_543] reset: refilled t1/t2 with current t0, ids=[%s,%s,%s]",
                          str(self.t0_id), str(self.t1_id), str(self.t2_id))

    def cb(self, msg):
        cur = list(msg.data)
        if not self._validate_frame(cur):
            return

        self.frame_id_counter += 1
        cur_id = self.frame_id_counter

        if self.t0 is None:
            self.t0 = list(cur)
            self.t1 = list(cur)
            self.t2 = list(cur)

            self.t0_id = cur_id
            self.t1_id = cur_id
            self.t2_id = cur_id
        else:
            self.t2 = list(self.t1)
            self.t1 = list(self.t0)
            self.t0 = list(cur)

            self.t2_id = self.t1_id
            self.t1_id = self.t0_id
            self.t0_id = cur_id

        stacked = self.t0 + self.t1 + self.t2
        self.pub.publish(self._build_msg(stacked, "lidar_543"))

        ids = [float(self.t0_id), float(self.t1_id), float(self.t2_id)]
        self.pub_ids.publish(self._build_msg(ids, "lidar_543_ids"))

        ids_vis = (
            [float(self.t0_id)] * self.expected_len +
            [float(self.t1_id)] * self.expected_len +
            [float(self.t2_id)] * self.expected_len
        )
        self.pub_ids_vis.publish(self._build_msg(ids_vis, "lidar_543_ids_vis"))

        self.msg_count += 1
        if self.publish_debug:
            rospy.logwarn(
                "[lidar_stack_543] recv #%d -> stack ids [t0,t1,t2]=[%d,%d,%d]",
                cur_id, self.t0_id, self.t1_id, self.t2_id
            )


if __name__ == "__main__":
    rospy.init_node("lidar_stack_543_node")
    LidarStack543Node()
    rospy.spin()
