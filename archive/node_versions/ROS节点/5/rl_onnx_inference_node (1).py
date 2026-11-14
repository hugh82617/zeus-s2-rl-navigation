#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

"""
真正的 RL ONNX 推理节点（Python 3 / Miniconda）

输入：
1) /raw_cmd_vel              geometry_msgs/Twist
   按话题原样读取 6 个速度分量：
   [0:3] = [linear.x, linear.y, linear.z]
   [3:6] = [angular.x, angular.y, angular.z]

2) /rl_local_goal_body       std_msgs/Float32MultiArray
   4 维：[dx_b, dy_b, dz_b, dyaw_b]

3) /rl/lidar_543             std_msgs/Float32MultiArray
   543 维：[lidar_t0 | lidar_t1 | lidar_t2]

固定项：
4) projected_gravity = [0, 0, -g]

拼接后 observation 共 556 维：
[0:3]    base_lin_vel
[3:6]    base_ang_vel
[6:9]    projected_gravity
[9:13]   pose_commands
[13:556] lidar_543

推理：
- 将 556 维 observation 输入 ONNX
- 输出 2 维动作 [v, w]
- 发布到 /rl_cmd_vel
"""

import math
import os
import threading

import numpy as np
import onnxruntime as ort
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class RlOnnxInferenceNode(object):
    def __init__(self):
        self.model_path = rospy.get_param("~model_path", "/mnt/data/model_1200_actor.onnx")
        self.input_name = rospy.get_param("~input_name", "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        self.cmd_vel_in_topic = rospy.get_param("~cmd_vel_in_topic", "/raw_cmd_vel")
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_local_goal_body")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/rl/lidar_543")
        self.cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic", "/rl_cmd_vel")
        self.obs_pub_topic = rospy.get_param("~obs_pub_topic", "/rl/obs_556")

        self.gravity_g = float(rospy.get_param("~gravity_g", 9.81))
        self.publish_obs = bool(rospy.get_param("~publish_obs", True))
        self.publish_debug_log = bool(rospy.get_param("~publish_debug_log", True))
        self.require_fresh_ms = float(rospy.get_param("~require_fresh_ms", 500.0))
        self.zero_on_stale = bool(rospy.get_param("~zero_on_stale", True))
        self.run_on_any_update = bool(rospy.get_param("~run_on_any_update", True))

        self.clip_v_min = float(rospy.get_param("~clip_v_min", float("nan")))
        self.clip_v_max = float(rospy.get_param("~clip_v_max", float("nan")))
        self.clip_w_min = float(rospy.get_param("~clip_w_min", float("nan")))
        self.clip_w_max = float(rospy.get_param("~clip_w_max", float("nan")))

        if not os.path.isfile(self.model_path):
            raise IOError("ONNX model not found: {}".format(self.model_path))

        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        model_inputs = self.session.get_inputs()
        model_outputs = self.session.get_outputs()
        if len(model_inputs) < 1 or len(model_outputs) < 1:
            raise RuntimeError("Invalid ONNX model I/O")

        real_input_name = model_inputs[0].name
        real_output_name = model_outputs[0].name
        if self.input_name != real_input_name:
            rospy.logwarn("[rl_onnx_inference] input_name param=%s, model input=%s, use model input.", self.input_name, real_input_name)
            self.input_name = real_input_name
        if self.output_name != real_output_name:
            rospy.logwarn("[rl_onnx_inference] output_name param=%s, model output=%s, use model output.", self.output_name, real_output_name)
            self.output_name = real_output_name

        self.lock = threading.Lock()

        self.latest_lin = [0.0, 0.0, 0.0]
        self.latest_ang = [0.0, 0.0, 0.0]
        self.latest_goal = None
        self.latest_lidar = None

        self.stamp_cmd = None
        self.stamp_goal = None
        self.stamp_lidar = None

        self.infer_count = 0

        self.pub_cmd = rospy.Publisher(self.cmd_vel_out_topic, Twist, queue_size=10)
        self.pub_obs = rospy.Publisher(self.obs_pub_topic, Float32MultiArray, queue_size=10) if self.publish_obs else None

        self.sub_cmd = rospy.Subscriber(self.cmd_vel_in_topic, Twist, self.cb_cmd, queue_size=10)
        self.sub_goal = rospy.Subscriber(self.goal_topic, Float32MultiArray, self.cb_goal, queue_size=10)
        self.sub_lidar = rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=10)

        rospy.logwarn("[rl_onnx_inference] started")
        rospy.logwarn("[rl_onnx_inference] model_path        = %s", self.model_path)
        rospy.logwarn("[rl_onnx_inference] cmd_vel_in_topic  = %s", self.cmd_vel_in_topic)
        rospy.logwarn("[rl_onnx_inference] goal_topic        = %s", self.goal_topic)
        rospy.logwarn("[rl_onnx_inference] lidar_topic       = %s", self.lidar_topic)
        rospy.logwarn("[rl_onnx_inference] cmd_vel_out_topic = %s", self.cmd_vel_out_topic)
        rospy.logwarn("[rl_onnx_inference] obs_pub_topic     = %s", self.obs_pub_topic)
        rospy.logwarn("[rl_onnx_inference] projected_gravity = [0.0, 0.0, -%.6f]", self.gravity_g)

    def cb_cmd(self, msg):
        with self.lock:
            self.latest_lin = [float(msg.linear.x), float(msg.linear.y), float(msg.linear.z)]
            self.latest_ang = [float(msg.angular.x), float(msg.angular.y), float(msg.angular.z)]
            self.stamp_cmd = rospy.Time.now()
        if self.run_on_any_update:
            self.try_infer_and_publish()

    def cb_goal(self, msg):
        data = list(msg.data)
        if len(data) != 4:
            rospy.logerr_throttle(1.0, "[rl_onnx_inference] /rl_local_goal_body length error: got %d, expected 4", len(data))
            return
        with self.lock:
            self.latest_goal = [float(x) for x in data]
            self.stamp_goal = rospy.Time.now()
        if self.run_on_any_update:
            self.try_infer_and_publish()

    def cb_lidar(self, msg):
        data = list(msg.data)
        if len(data) != 543:
            rospy.logerr_throttle(1.0, "[rl_onnx_inference] /rl/lidar_543 length error: got %d, expected 543", len(data))
            return
        with self.lock:
            self.latest_lidar = [float(x) for x in data]
            self.stamp_lidar = rospy.Time.now()
        self.try_infer_and_publish()

    def _is_fresh(self, stamp, now):
        if stamp is None:
            return False
        return ((now - stamp).to_sec() * 1000.0) <= self.require_fresh_ms

    def _build_obs(self, lin3, ang3, goal4, lidar543):
        projected_gravity = [0.0, 0.0, -self.gravity_g]
        obs = list(lin3) + list(ang3) + projected_gravity + list(goal4) + list(lidar543)
        if len(obs) != 556:
            raise ValueError("obs length error: got {}, expected 556".format(len(obs)))
        return obs

    def _clip_action(self, v, w):
        if not math.isnan(self.clip_v_min):
            v = max(v, self.clip_v_min)
        if not math.isnan(self.clip_v_max):
            v = min(v, self.clip_v_max)
        if not math.isnan(self.clip_w_min):
            w = max(w, self.clip_w_min)
        if not math.isnan(self.clip_w_max):
            w = min(w, self.clip_w_max)
        return v, w

    def _publish_obs(self, obs):
        if self.pub_obs is None:
            return
        msg = Float32MultiArray()
        msg.data = obs
        self.pub_obs.publish(msg)

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def try_infer_and_publish(self):
        now = rospy.Time.now()
        with self.lock:
            if self.latest_goal is None or self.latest_lidar is None:
                return

            cmd_fresh = self._is_fresh(self.stamp_cmd, now)
            goal_fresh = self._is_fresh(self.stamp_goal, now)
            lidar_fresh = self._is_fresh(self.stamp_lidar, now)

            if cmd_fresh:
                lin3 = list(self.latest_lin)
                ang3 = list(self.latest_ang)
            else:
                lin3 = [0.0, 0.0, 0.0]
                ang3 = [0.0, 0.0, 0.0]

            goal4 = list(self.latest_goal)
            lidar543 = list(self.latest_lidar)

        if (not goal_fresh) or (not lidar_fresh):
            rospy.logwarn_throttle(1.0, "[rl_onnx_inference] waiting fresh data... cmd=%s goal=%s lidar=%s", str(cmd_fresh), str(goal_fresh), str(lidar_fresh))
            if self.zero_on_stale:
                self._publish_cmd(0.0, 0.0)
            return

        try:
            obs = self._build_obs(lin3, ang3, goal4, lidar543)
            obs_np = np.asarray(obs, dtype=np.float32).reshape(1, 556)
            action = self.session.run([self.output_name], {self.input_name: obs_np})[0]
            action = np.asarray(action, dtype=np.float32).reshape(-1)
            if action.shape[0] < 2:
                raise ValueError("action shape invalid: {}".format(action.shape))

            v = float(action[0])
            w = float(action[1])
            v, w = self._clip_action(v, w)

            self.infer_count += 1
            self._publish_obs(obs)
            self._publish_cmd(v, w)

            if self.publish_debug_log:
                rospy.logwarn_throttle(
                    1.0,
                    "[rl_onnx_inference] infer=%d lin=[%.4f %.4f %.4f] ang=[%.4f %.4f %.4f] goal=[%.3f %.3f %.3f %.3f] -> act=[%.4f %.4f]",
                    self.infer_count,
                    lin3[0], lin3[1], lin3[2],
                    ang3[0], ang3[1], ang3[2],
                    goal4[0], goal4[1], goal4[2], goal4[3],
                    v, w,
                )
        except Exception as e:
            rospy.logerr_throttle(1.0, "[rl_onnx_inference] inference failed: %s", str(e))
            if self.zero_on_stale:
                self._publish_cmd(0.0, 0.0)


if __name__ == "__main__":
    rospy.init_node("rl_onnx_inference_node")
    RlOnnxInferenceNode()
    rospy.spin()
