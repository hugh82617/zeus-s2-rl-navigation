#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class OdomToPath(object):
    def __init__(self):
        rospy.init_node("odom_to_path", anonymous=False)

        # 参数
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.path_topic = rospy.get_param("~path_topic", "/trajectory")
        self.path_frame = rospy.get_param("~path_frame", "")   # 为空时自动使用 /odom 的 frame_id
        self.min_dist = rospy.get_param("~min_dist", 0.02)     # 两点最小间隔，单位 m
        self.max_points = rospy.get_param("~max_points", 5000) # 最多保留多少个轨迹点

        # 发布器与订阅器
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

        # Path 消息
        self.path_msg = Path()

        # 上一个记录点
        self.last_x = None
        self.last_y = None

        rospy.loginfo("odom_to_path 节点已启动")
        rospy.loginfo("订阅里程计话题: %s", self.odom_topic)
        rospy.loginfo("发布轨迹话题: %s", self.path_topic)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 设定轨迹坐标系
        if self.path_frame:
            self.path_msg.header.frame_id = self.path_frame
            pose_frame = self.path_frame
        else:
            self.path_msg.header.frame_id = msg.header.frame_id
            pose_frame = msg.header.frame_id

        # 第一个点直接加入
        if self.last_x is None or self.last_y is None:
            need_add = True
        else:
            dist = self.distance(x, y, self.last_x, self.last_y)
            need_add = dist >= self.min_dist

        if need_add:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = msg.header.stamp
            pose_stamped.header.frame_id = pose_frame
            pose_stamped.pose = msg.pose.pose

            self.path_msg.poses.append(pose_stamped)

            # 限制轨迹长度，防止无限增长
            if len(self.path_msg.poses) > self.max_points:
                self.path_msg.poses.pop(0)

            self.last_x = x
            self.last_y = y

        # 更新时间戳并发布
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)


if __name__ == "__main__":
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
