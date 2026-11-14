#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import threading

import numpy as np
import rospy
import onnxruntime as ort

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RLOnnxInferenceNode(object):
    def __init__(self):
        rospy.init_node("rl_onnx_inference_node", anonymous=False)

        # -------------------- parameters --------------------
        self.model_path = rospy.get_param("~model_path", "/home/ros/catkin_ws/src/rl_planner/models/model.onnx")
        self.input_name = rospy.get_param("~input_name", "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        self.topic_odom = rospy.get_param("~topic_odom", "/odom")
        self.topic_goal4 = rospy.get_param("~topic_goal4", "/rl_local_goal_body")
        self.topic_lidar543 = rospy.get_param("~topic_lidar543", "/rl/lidar_543")
        self.topic_cmd_vel = rospy.get_param("~topic_cmd_vel", "/cmd_vel")

        self.warn_goal_age_ms = float(rospy.get_param("~warn_goal_age_ms", 80.0))
        self.warn_speed_age_ms = float(rospy.get_param("~warn_speed_age_ms", 80.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.20))

        # 动作映射参数
        self.use_clip = bool(rospy.get_param("~use_clip", True))
        self.action_clip_low = float(rospy.get_param("~action_clip_low", -1.0))
        self.action_clip_high = float(rospy.get_param("~action_clip_high", 1.0))

        self.linear_min = float(rospy.get_param("~linear_min", -0.2))
        self.linear_max = float(rospy.get_param("~linear_max", 0.5))
        self.angular_min = float(rospy.get_param("~angular_min", -1.0))
        self.angular_max = float(rospy.get_param("~angular_max", 1.0))

        self.publish_zero_when_goal_reached = bool(rospy.get_param("~publish_zero_when_goal_reached", True))

        # -------------------- runtime state --------------------
        self.lock = threading.Lock()

        self.latest_lin3 = None
        self.latest_ang3 = None
        self.latest_goal4 = None
        self.latest_lidar543 = None

        self.latest_speed_time = None
        self.latest_goal_time = None
        self.latest_lidar_time = None

        self.has_lin3 = False
        self.has_ang3 = False
        self.has_goal4 = False
        self.has_lidar543 = False

        self.goal_reached = False
        self.goal_done_logged = False

        # -------------------- ONNX --------------------
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self._infer_input_name = self.input_name
        self._infer_output_name = self.output_name

        # 如果参数名不对，自动退回到第一个输入输出
        try:
            input_names = [x.name for x in self.session.get_inputs()]
            output_names = [x.name for x in self.session.get_outputs()]
            if self._infer_input_name not in input_names:
                self._infer_input_name = input_names[0]
            if self._infer_output_name not in output_names:
                self._infer_output_name = output_names[0]
        except Exception:
            pass

        # -------------------- publishers --------------------
        self.pub_cmd = rospy.Publisher(self.topic_cmd_vel, Twist, queue_size=10)

        self.pub_debug_lin3 = rospy.Publisher("/rl/debug/lin_vel3", Float32MultiArray, queue_size=10)
        self.pub_debug_ang3 = rospy.Publisher("/rl/debug/ang_vel3", Float32MultiArray, queue_size=10)
        self.pub_debug_goal4 = rospy.Publisher("/rl/debug/goal4", Float32MultiArray, queue_size=10)
        self.pub_debug_obs0 = rospy.Publisher("/rl/debug/obs0_181", Float32MultiArray, queue_size=10)
        self.pub_debug_obs1 = rospy.Publisher("/rl/debug/obs1_181", Float32MultiArray, queue_size=10)
        self.pub_debug_obs2 = rospy.Publisher("/rl/debug/obs2_181", Float32MultiArray, queue_size=10)
        self.pub_debug_obs556 = rospy.Publisher("/rl/obs_556", Float32MultiArray, queue_size=10)
        self.pub_debug_action = rospy.Publisher("/rl/debug/raw_action", Float32MultiArray, queue_size=10)

        # -------------------- subscribers --------------------
        rospy.Subscriber(self.topic_odom, Odometry, self.cb_odom, queue_size=50)
        rospy.Subscriber(self.topic_goal4, Float32MultiArray, self.cb_goal, queue_size=50)
        rospy.Subscriber(self.topic_lidar543, Float32MultiArray, self.cb_lidar, queue_size=50)

        rospy.loginfo("[RL] node started")
        rospy.loginfo("[RL] model_path=%s", self.model_path)
        rospy.loginfo("[RL] odom=%s goal4=%s lidar543=%s cmd_vel=%s",
                      self.topic_odom, self.topic_goal4, self.topic_lidar543, self.topic_cmd_vel)

    # -------------------- utils --------------------
    def _to_msg(self, arr):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in arr]
        return msg

    def safe_publish(self, pub, msg, name=""):
        if rospy.is_shutdown():
            return
        if pub is None:
            return
        try:
            pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logwarn_throttle(2.0, "[RL] publish failed on %s: %s", name, str(e))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[RL] unexpected publish error on %s: %s", name, str(e))

    def map_action_to_cmd(self, action):
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        if a.shape[0] < 2:
            raise ValueError("action dim < 2")

        raw_v = float(a[0])
        raw_w = float(a[1])

        if self.use_clip:
            raw_v = max(self.action_clip_low, min(self.action_clip_high, raw_v))
            raw_w = max(self.action_clip_low, min(self.action_clip_high, raw_w))

        # v: [-1,1] -> [linear_min, linear_max]
        v = (raw_v + 1.0) * 0.5 * (self.linear_max - self.linear_min) + self.linear_min
        # w: [-1,1] -> [angular_min, angular_max]
        w = (raw_w + 1.0) * 0.5 * (self.angular_max - self.angular_min) + self.angular_min

        return raw_v, raw_w, v, w

    def check_goal_reached(self):
        if self.latest_goal4 is None or len(self.latest_goal4) < 2:
            return False
        dx = float(self.latest_goal4[0])
        dy = float(self.latest_goal4[1])
        dist = math.sqrt(dx * dx + dy * dy)
        return dist <= self.goal_tolerance

    def build_obs_556(self):
        obs = []
        obs.extend(self.latest_lin3)      # 0:3
        obs.extend(self.latest_ang3)      # 3:6
        obs.extend([0.0, 0.0, -1.0])      # 6:9 gravity
        obs.extend(self.latest_goal4)     # 9:13
        obs.extend(self.latest_lidar543)  # 13:556
        if len(obs) != 556:
            raise ValueError("obs dim is %d, expected 556" % len(obs))
        return np.asarray(obs, dtype=np.float32).reshape(1, 556)

    # -------------------- callbacks --------------------
    def cb_odom(self, msg):
        with self.lock:
            self.latest_lin3 = [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ]
            self.latest_ang3 = [
                float(msg.twist.twist.angular.x),
                float(msg.twist.twist.angular.y),
                float(msg.twist.twist.angular.z),
            ]
            self.latest_speed_time = rospy.Time.now()
            self.has_lin3 = True
            self.has_ang3 = True

            self.safe_publish(self.pub_debug_lin3, self._to_msg(self.latest_lin3), "pub_debug_lin3")
            self.safe_publish(self.pub_debug_ang3, self._to_msg(self.latest_ang3), "pub_debug_ang3")

    def cb_goal(self, msg):
        data = list(msg.data)
        if len(data) < 4:
            rospy.logwarn_throttle(2.0, "[RL] goal4 len=%d < 4", len(data))
            return

        with self.lock:
            self.latest_goal4 = [float(data[0]), float(data[1]), float(data[2]), float(data[3])]
            self.latest_goal_time = rospy.Time.now()
            self.has_goal4 = True

            # 收到新目标后恢复运行
            self.goal_reached = False
            self.goal_done_logged = False

            self.safe_publish(self.pub_debug_goal4, self._to_msg(self.latest_goal4), "pub_debug_goal4")

    def cb_lidar(self, msg):
        data = list(msg.data)
        if len(data) < 543:
            rospy.logwarn_throttle(2.0, "[RL] lidar543 len=%d < 543", len(data))
            return

        with self.lock:
            # 已到目标后，直接静默返回，不再打印任何日志，不再推理
            if self.goal_reached:
                return

            self.latest_lidar543 = [float(x) for x in data[:543]]
            self.latest_lidar_time = rospy.Time.now()
            self.has_lidar543 = True

            if (not self.has_lin3) or (not self.has_ang3) or (not self.has_goal4) or (not self.has_lidar543):
                rospy.logwarn_throttle(
                    2.0,
                    "[RL] waiting topics... lidar-trigger  lin3=%s  ang3=%s  goal4=%s  lidar543=%s"
                    % (self.has_lin3, self.has_ang3, self.has_goal4, self.has_lidar543)
                )
                return

            # 先判断是否到达目标。到达后只打印一次，然后静默。
            if self.check_goal_reached():
                self.goal_reached = True
                if not self.goal_done_logged:
                    if self.publish_zero_when_goal_reached:
                        zero = Twist()
                        self.safe_publish(self.pub_cmd, zero, "pub_cmd_zero")
                    rospy.loginfo("[RL] 已到达目标点")
                    self.goal_done_logged = True
                return

            # 同步检查：只有未到达目标时才打印
            if self.latest_goal_time is not None:
                goal_age_ms = (self.latest_lidar_time - self.latest_goal_time).to_sec() * 1000.0
                if goal_age_ms > self.warn_goal_age_ms:
                    rospy.logwarn_throttle(
                        1.0,
                        "[RL][SYNC] goal_age_ms=%.1f > warn_goal_age_ms=%.1f"
                        % (goal_age_ms, self.warn_goal_age_ms)
                    )

            if self.latest_speed_time is not None:
                speed_age_ms = (self.latest_lidar_time - self.latest_speed_time).to_sec() * 1000.0
                if speed_age_ms > self.warn_speed_age_ms:
                    rospy.logwarn_throttle(
                        1.0,
                        "[RL][SYNC] speed_age_ms=%.1f > warn_speed_age_ms=%.1f"
                        % (speed_age_ms, self.warn_speed_age_ms)
                    )

            # 调试切片
            self.safe_publish(self.pub_debug_obs0, self._to_msg(self.latest_lidar543[0:181]), "pub_debug_obs0")
            self.safe_publish(self.pub_debug_obs1, self._to_msg(self.latest_lidar543[181:362]), "pub_debug_obs1")
            self.safe_publish(self.pub_debug_obs2, self._to_msg(self.latest_lidar543[362:543]), "pub_debug_obs2")

            # 组 obs
            try:
                obs = self.build_obs_556()
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] build_obs_556 failed: %s", str(e))
                return

            self.safe_publish(self.pub_debug_obs556, self._to_msg(obs.reshape(-1).tolist()), "pub_debug_obs556")

            # 推理
            try:
                outputs = self.session.run([self._infer_output_name], {self._infer_input_name: obs})
                raw_action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] onnx inference failed: %s", str(e))
                return

            self.safe_publish(self.pub_debug_action, self._to_msg(raw_action.tolist()), "pub_debug_action")

            # 动作映射
            try:
                raw_v, raw_w, v, w = self.map_action_to_cmd(raw_action)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[RL] map_action_to_cmd failed: %s", str(e))
                return

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.safe_publish(self.pub_cmd, cmd, "pub_cmd")

            rospy.loginfo_throttle(
                0.5,
                "[RL] raw_action=[%.6f, %.6f] -> cmd_vel=[%.6f, %.6f]"
                % (raw_v, raw_w, v, w)
            )


def main():
    try:
        RLOnnxInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
