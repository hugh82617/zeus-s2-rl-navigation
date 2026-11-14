#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class OdomToPath(object):
    def __init__(self):
        rospy.init_node("odom_to_path", anonymous=False)

        # 参数
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.path_topic = rospy.get_param("~path_topic", "/trajectory")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.mode_topic = rospy.get_param("~mode_topic", "/cmd_vel_switch/active_mode")
        self.path_frame = rospy.get_param("~path_frame", "")
        self.min_dist = rospy.get_param("~min_dist", 0.02)
        self.max_points = rospy.get_param("~max_points", 5000)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.20)

        # 允许记录轨迹的实际控制模式
        self.allowed_modes = set(["dwa", "rl"])

        # 发布器
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)

        # 订阅器
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=10)
        self.mode_sub = rospy.Subscriber(self.mode_topic, String, self.mode_callback, queue_size=10)

        # 轨迹消息
        self.path_msg = Path()

        # 上一个轨迹点
        self.last_x = None
        self.last_y = None

        # 当前目标点
        self.goal_x = None
        self.goal_y = None
        self.goal_frame = None
        self.goal_active = False

        # 当前实际生效模式
        self.current_mode = ""
        self.record_enabled = False

        rospy.loginfo("odom_to_path 节点已启动")
        rospy.loginfo("订阅里程计话题: %s", self.odom_topic)
        rospy.loginfo("订阅目标点话题: %s", self.goal_topic)
        rospy.loginfo("订阅实际控制模式话题: %s", self.mode_topic)
        rospy.loginfo("发布轨迹话题: %s", self.path_topic)
        rospy.loginfo("允许记录轨迹的模式: %s", list(self.allowed_modes))

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def clear_path(self, frame_id="odom"):
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id
        self.path_msg.header.stamp = rospy.Time.now()
        self.last_x = None
        self.last_y = None
        self.path_pub.publish(self.path_msg)

    def mode_callback(self, msg):
        new_mode = msg.data.strip().lower()
        old_mode = self.current_mode
        self.current_mode = new_mode

        new_record_enabled = (new_mode in self.allowed_modes)

        if new_mode != old_mode:
            rospy.loginfo("实际控制模式切换: %s -> %s",
                          old_mode if old_mode else "none", new_mode)

        # 从允许记录模式切换到不允许记录模式（比如 teleop）时，立即清空轨迹
        if self.record_enabled and not new_record_enabled:
            rospy.loginfo("当前实际模式为 %s，不记录轨迹，清空当前轨迹", new_mode)
            frame_id = self.path_msg.header.frame_id if self.path_msg.header.frame_id else "odom"
            self.clear_path(frame_id=frame_id)

        self.record_enabled = new_record_enabled

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_frame = msg.header.frame_id if msg.header.frame_id else "odom"
        self.goal_active = True

        rospy.loginfo("收到新目标点: x=%.3f, y=%.3f, frame=%s",
                      self.goal_x, self.goal_y, self.goal_frame)

        # 收到新目标点时，清空旧轨迹，开始记录新轨迹
        self.clear_path(frame_id=self.goal_frame)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 轨迹坐标系
        if self.path_frame:
            current_frame = self.path_frame
        else:
            current_frame = msg.header.frame_id if msg.header.frame_id else "odom"

        self.path_msg.header.frame_id = current_frame

        # 没有目标点时不记录
        if not self.goal_active:
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path_msg)
            return

        # 当前实际控制来源不是 rl 或 dwa 时不记录
        if not self.record_enabled:
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path_msg)
            return

        # 到达目标点后清空轨迹
        # 注意：这里默认 /odom 与 /move_base/current_goal 在同一坐标系下可直接比较
        if self.goal_x is not None and self.goal_y is not None:
            dist_to_goal = self.distance(x, y, self.goal_x, self.goal_y)

            if dist_to_goal <= self.goal_tolerance:
                rospy.loginfo("已到达目标点，距离目标 %.3f m，清空轨迹", dist_to_goal)
                self.clear_path(frame_id=current_frame)
                self.goal_active = False
                self.goal_x = None
                self.goal_y = None
                self.goal_frame = None
                return

        # 轨迹点采样
        if self.last_x is None or self.last_y is None:
            need_add = True
        else:
            dist = self.distance(x, y, self.last_x, self.last_y)
            need_add = dist >= self.min_dist

        if need_add:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = msg.header.stamp
            pose_stamped.header.frame_id = current_frame
            pose_stamped.pose = msg.pose.pose

            self.path_msg.poses.append(pose_stamped)

            if len(self.path_msg.poses) > self.max_points:
                self.path_msg.poses.pop(0)

            self.last_x = x
            self.last_y = y

        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)


if __name__ == "__main__":
    try:
        OdomToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
