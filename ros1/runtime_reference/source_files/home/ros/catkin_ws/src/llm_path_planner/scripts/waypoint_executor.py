#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
import os
from geometry_msgs.msg import PoseStamped

WAYPOINT_FILE = "llm_waypoints.json"   # 路径点文件名
WAIT_TIME = 6  # 每两个路径点之间的间隔秒数（根据导航速度调）

def load_waypoints(path):
    if not os.path.exists(path):
        rospy.logerr(f"❌ 未找到路径点文件: {path}")
        return []
    with open(path, 'r') as f:
        try:
            waypoints = json.load(f)
            return waypoints
        except json.JSONDecodeError:
            rospy.logerr("❌ 路径点文件格式错误（不是合法 JSON）")
            return []

def publish_waypoints(waypoints):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(2)  # 等待 Publisher 建立连接
    for idx, point in enumerate(waypoints):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        pose.pose.position.z = 0.0

        pose.pose.orientation.w = 1.0  # 无方向旋转

        rospy.loginfo(f"📍 发布目标点 {idx + 1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        pub.publish(pose)

        rospy.sleep(WAIT_TIME)

if __name__ == '__main__':
    rospy.init_node('waypoint_executor', anonymous=True)
    rospy.loginfo("🚀 启动路径点执行器")

    waypoints = load_waypoints(WAYPOINT_FILE)
    if waypoints:
        publish_waypoints(waypoints)
        rospy.loginfo("✅ 所有路径点已发布完毕")
    else:
        rospy.logerr("❌ 路径点为空，未能执行导航")

