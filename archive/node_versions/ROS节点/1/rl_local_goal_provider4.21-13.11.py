#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import threading

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_norm(q):
    return math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)


class RLLocalGoalProviderTF(object):
    def __init__(self):
        rospy.init_node("rl_local_goal_provider_tf", log_level=rospy.INFO)

        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.clear_goal_topic = rospy.get_param("~clear_goal_topic", "/rl_local_goal_clear")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self.publish_array_topic = rospy.get_param("~publish_array_topic", "/rl_local_goal_body")
        self.publish_pose_topic = rospy.get_param("~publish_pose_topic", "/rl_local_goal_body_pose")
        self.publish_status_topic = rospy.get_param("~publish_status_topic", "/rl_local_goal_status")

        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))
        self.debug_log = bool(rospy.get_param("~debug_log", True))
        
        # 保持与推理节点一致的 0.50m 到达阈值
        self.goal_reached_dist_thresh = float(rospy.get_param("~goal_reached_dist_thresh", 0.50))
        self.goal_reached_clear = bool(rospy.get_param("~goal_reached_clear", True))

        self.listener = tf.TransformListener()

        self.body_pub = rospy.Publisher(self.publish_array_topic, Float32MultiArray, queue_size=1)
        self.body_pose_pub = rospy.Publisher(self.publish_pose_topic, PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(self.publish_status_topic, String, queue_size=1)

        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        self.clear_goal_sub = rospy.Subscriber(self.clear_goal_topic, String, self._clear_goal_cb, queue_size=1)

        self._lock = threading.Lock()
        self.latest_goal_global = None
        self.have_goal = False
        self._last_status = None

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("=" * 68)
        rospy.loginfo("[RL_GOAL_TF] improved node started")
        rospy.loginfo("[RL_GOAL_TF] subscribe goal       : %s", self.goal_topic)
        rospy.loginfo("[RL_GOAL_TF] clear goal topic    : %s", self.clear_goal_topic)
        rospy.loginfo("[RL_GOAL_TF] global_frame        : %s", self.global_frame)
        rospy.loginfo("[RL_GOAL_TF] base_frame          : %s", self.base_frame)
        rospy.loginfo("[RL_GOAL_TF] publish array       : %s", self.publish_array_topic)
        rospy.loginfo("[RL_GOAL_TF] publish pose        : %s", self.publish_pose_topic)
        rospy.loginfo("[RL_GOAL_TF] publish status      : %s", self.publish_status_topic)
        rospy.loginfo("[RL_GOAL_TF] publish rate        : %.2f Hz", self.publish_rate)
        rospy.loginfo("[RL_GOAL_TF] tf timeout          : %.3f s", self.tf_timeout)
        rospy.loginfo("[RL_GOAL_TF] dyaw mode           : atan2(dy, dx)")
        rospy.loginfo("[RL_GOAL_TF] goal_reached_thresh : %.3f m", self.goal_reached_dist_thresh)
        rospy.loginfo("[RL_GOAL_TF] goal_reached_clear  : %s", str(self.goal_reached_clear))
        rospy.loginfo("=" * 68)

    @staticmethod
    def _is_quat_valid(q, eps=1e-6):
        return quat_norm(q) > eps

    def _publish_status_once(self, text):
        if text != self._last_status:
            self.status_pub.publish(String(data=text))
            self._last_status = text

    def _goal_cb(self, msg):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.pose

        if goal.header.frame_id == "":
            goal.header.frame_id = self.global_frame

        with self._lock:
            self.latest_goal_global = goal
            self.have_goal = True

        self._publish_status_once("goal_received")
        rospy.loginfo(
            "[RL_GOAL_TF] received goal: frame=%s pos=(%.3f, %.3f, %.3f)",
            goal.header.frame_id,
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z
        )

    def _clear_goal_cb(self, _msg):
        with self._lock:
            self.latest_goal_global = None
            self.have_goal = False
        self._publish_status_once("goal_cleared")
        rospy.loginfo("[RL_GOAL_TF] current goal cleared by topic")

    def _clear_goal_internal(self, reason):
        with self._lock:
            self.latest_goal_global = None
            self.have_goal = False
        self._publish_status_once(reason)
        
        # 修改2：增强到达日志，明确区分到达和清除
        if reason == "goal_reached":
            rospy.loginfo("[RL_GOAL_TF] *** LOCAL GOAL REACHED: publishing status=goal_reached and clearing current goal ***")
        else:
            rospy.loginfo("[RL_GOAL_TF] current goal cleared: %s", reason)

    def _transform_goal_to_base(self, goal_msg):
        src_frame = goal_msg.header.frame_id.strip("/")

        if src_frame == self.base_frame.strip("/"):
            if not self._is_quat_valid(goal_msg.pose.orientation):
                rospy.logwarn("[RL_GOAL_TF] invalid quaternion in same-frame goal, skipping")
                return None

            out_pose = PoseStamped()
            out_pose.header.stamp = rospy.Time.now()
            out_pose.header.frame_id = self.base_frame
            out_pose.pose = goal_msg.pose
            return out_pose

        try:
            self.listener.waitForTransform(
                self.base_frame,
                goal_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout)
            )

            goal_for_tf = PoseStamped()
            goal_for_tf.header = goal_msg.header
            goal_for_tf.header.stamp = rospy.Time(0)
            goal_for_tf.pose = goal_msg.pose

            out_pose = self.listener.transformPose(self.base_frame, goal_for_tf)
            out_pose.header.stamp = rospy.Time.now()

            if not self._is_quat_valid(out_pose.pose.orientation):
                rospy.logwarn_throttle(1.0, "[RL_GOAL_TF] transformed pose has invalid quaternion, skipping")
                return None

            return out_pose

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(
                1.0,
                "[RL_GOAL_TF] transform failed: %s -> %s | %s",
                goal_msg.header.frame_id, self.base_frame, str(e)
            )
            self._publish_status_once("transform_failed")
            return None

    def _lookup_robot_in_global(self):
        try:
            self.listener.waitForTransform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout)
            )
            trans, rot = self.listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            yaw = normalize_angle(yaw)
            return trans[0], trans[1], trans[2], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _goal_in_global_for_log(self, goal_msg):
        try:
            src_frame = goal_msg.header.frame_id.strip("/")
            if src_frame == self.global_frame.strip("/"):
                p = goal_msg.pose.position
                q = goal_msg.pose.orientation
                yaw = 0.0
                if self._is_quat_valid(q):
                    yaw = normalize_angle(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
                return p.x, p.y, p.z, yaw

            self.listener.waitForTransform(
                self.global_frame,
                goal_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout)
            )
            goal_for_tf = PoseStamped()
            goal_for_tf.header = goal_msg.header
            goal_for_tf.header.stamp = rospy.Time(0)
            goal_for_tf.pose = goal_msg.pose
            goal_global = self.listener.transformPose(self.global_frame, goal_for_tf)
            p = goal_global.pose.position
            q = goal_global.pose.orientation
            yaw = 0.0
            if self._is_quat_valid(q):
                yaw = normalize_angle(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
            return p.x, p.y, p.z, yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def _publish_body_goal(self, pose_in_base, goal_snap):
        dx = float(pose_in_base.pose.position.x)
        dy = float(pose_in_base.pose.position.y)
        dz = float(pose_in_base.pose.position.z)
        dyaw = math.atan2(dy, dx)
        dist = math.sqrt(dx * dx + dy * dy)

        arr = Float32MultiArray()
        arr.data = [dx, dy, dz, dyaw]

        out_pose = PoseStamped()
        out_pose.header.stamp = rospy.Time.now()
        out_pose.header.frame_id = self.base_frame
        out_pose.pose = pose_in_base.pose

        self.body_pub.publish(arr)
        self.body_pose_pub.publish(out_pose)
        self._publish_status_once("goal_tracking")

        if self.debug_log:
            goal_map = self._goal_in_global_for_log(goal_snap)
            robot_map = self._lookup_robot_in_global()

            if goal_map is not None and robot_map is not None:
                rospy.loginfo_throttle(
                    0.5,
                    "[RL_GOAL_TF] goal_map=(x=%.4f, y=%.4f, z=%.4f, yaw=%.4f) | "
                    "robot_map=(x=%.4f, y=%.4f, z=%.4f, yaw=%.4f) | "
                    "body4=[dx=%.4f, dy=%.4f, dz=%.4f, dyaw=%.4f] dist=%.4f",
                    goal_map[0], goal_map[1], goal_map[2], goal_map[3],
                    robot_map[0], robot_map[1], robot_map[2], robot_map[3],
                    dx, dy, dz, dyaw, dist
                )
            else:
                rospy.loginfo_throttle(
                    0.5,
                    "[RL_GOAL_TF] body4=[dx=%.4f, dy=%.4f, dz=%.4f, dyaw=%.4f] dist=%.4f",
                    dx, dy, dz, dyaw, dist
                )

        # 修改2：到达判定时明确日志输出
        if dist < self.goal_reached_dist_thresh:
            rospy.loginfo_throttle(
                0.5,
                "[RL_GOAL_TF] local goal within reach: body4=[dx=%.4f, dy=%.4f, dz=%.4f, dyaw=%.4f] dist=%.4f < thresh=%.4f",
                dx, dy, dz, dyaw, dist, self.goal_reached_dist_thresh
            )
            if self.goal_reached_clear:
                # 明确日志：触发了 goal_reached 清除
                rospy.loginfo(
                    "[RL_GOAL_TF] *** REACHED THRESHOLD: dist=%.4f < %.4f, publishing status=goal_reached and clearing goal ***",
                    dist, self.goal_reached_dist_thresh
                )
                self._clear_goal_internal("goal_reached")

    def _on_shutdown(self):
        rospy.loginfo("[RL_GOAL_TF] shutting down")
        try:
            self.status_pub.publish(String(data="shutdown"))
        except Exception:
            pass

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        rospy.sleep(0.5)
        rospy.loginfo("[RL_GOAL_TF] waiting for goal ...")

        while not rospy.is_shutdown():
            with self._lock:
                ready = self.have_goal
                goal_snap = self.latest_goal_global

            if not ready or goal_snap is None:
                rate.sleep()
                continue

            pose_in_base = self._transform_goal_to_base(goal_snap)
            if pose_in_base is not None:
                self._publish_body_goal(pose_in_base, goal_snap)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = RLLocalGoalProviderTF()
        node.run()
    except rospy.ROSInterruptException:
        pass
