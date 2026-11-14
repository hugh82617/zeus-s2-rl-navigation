#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
import onnxruntime as ort

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class RLOnnxInferenceNode(object):
    """
    RL ONNX 推理节点（联调/部署版）

    观测默认按以下顺序拼接（共 556 维）：
      [0:3]    lin_vel              -> [vx, vy, vz]
      [3:6]    ang_vel              -> [wx, wy, wz]
      [6:9]    projected_gravity    -> [gx, gy, gz]
      [9:13]   pose_commands        -> [dx_b, dy_b, dz_b, dyaw_b]
      [13:556] lidar_543            -> 543 维激光堆叠

    订阅：
      1) /raw_cmd_vel            (geometry_msgs/Twist)
      2) /rl_local_goal_body     (std_msgs/Float32MultiArray, 长度 4)
      3) /rl/lidar_543           (std_msgs/Float32MultiArray, 长度 543)

    发布：
      1) /rl_cmd_vel             (geometry_msgs/Twist)
      2) /rl/obs_556             (std_msgs/Float32MultiArray)

    注意：
      - 该版本已将 projected_gravity 改为三维显式参数：gravity_x/y/z
      - 建议先只让底盘不上线地监听 /rl_cmd_vel 做联调
    """

    def __init__(self):
        rospy.init_node("rl_onnx_inference_node", anonymous=False)

        # =========================
        # 参数
        # =========================
        default_model = "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx"
        self.model_path = rospy.get_param("~model_path", default_model)

        # 模型输入输出名（若与模型真实名字不同，会自动修正）
        self.input_name = rospy.get_param("~input_name", "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        # 推理频率
        self.rate_hz = float(rospy.get_param("~rate", 10.0))

        # 话题名
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/raw_cmd_vel")
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_local_goal_body")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/rl/lidar_543")
        self.out_cmd_topic = rospy.get_param("~out_cmd_topic", "/rl_cmd_vel")
        self.obs_topic = rospy.get_param("~obs_topic", "/rl/obs_556")

        # projected_gravity 三维显式参数
        self.gravity_x = float(rospy.get_param("~gravity_x", 0.0))
        self.gravity_y = float(rospy.get_param("~gravity_y", 0.0))
        self.gravity_z = float(rospy.get_param("~gravity_z", -1.0))

        # 输出限幅（若为 NaN，则表示不裁剪该端）
        self.clip_v_min = float(rospy.get_param("~clip_v_min", float("nan")))
        self.clip_v_max = float(rospy.get_param("~clip_v_max", float("nan")))
        self.clip_w_min = float(rospy.get_param("~clip_w_min", float("nan")))
        self.clip_w_max = float(rospy.get_param("~clip_w_max", float("nan")))

        # 调试开关
        self.publish_obs = bool(rospy.get_param("~publish_obs", True))
        self.debug_log = bool(rospy.get_param("~debug_log", True))
        self.debug_obs_log = bool(rospy.get_param("~debug_obs_log", False))

        # =========================
        # 缓存
        # =========================
        self.latest_lin3 = None   # [vx, vy, vz]
        self.latest_ang3 = None   # [wx, wy, wz]
        self.latest_goal4 = None  # [dx_b, dy_b, dz_b, dyaw_b]
        self.latest_lidar543 = None

        # =========================
        # 加载 ONNX
        # =========================
        rospy.loginfo("[RL] loading ONNX model: %s", self.model_path)
        self.session = ort.InferenceSession(
            self.model_path,
            providers=["CPUExecutionProvider"]
        )

        actual_input_name = self.session.get_inputs()[0].name
        actual_output_name = self.session.get_outputs()[0].name

        if self.input_name != actual_input_name:
            rospy.logwarn("[RL] input_name from param is '%s', but model expects '%s'. Use model value.",
                          self.input_name, actual_input_name)
            self.input_name = actual_input_name
        if self.output_name != actual_output_name:
            rospy.logwarn("[RL] output_name from param is '%s', but model outputs '%s'. Use model value.",
                          self.output_name, actual_output_name)
            self.output_name = actual_output_name

        rospy.loginfo("[RL] ONNX ready. input=%s output=%s", self.input_name, self.output_name)

        # =========================
        # ROS 通信
        # =========================
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cb_cmd_vel, queue_size=1)
        rospy.Subscriber(self.goal_topic, Float32MultiArray, self.cb_goal, queue_size=1)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=1)

        self.pub_cmd = rospy.Publisher(self.out_cmd_topic, Twist, queue_size=1)
        self.pub_obs = rospy.Publisher(self.obs_topic, Float32MultiArray, queue_size=1)

        rospy.loginfo("[RL] subscribed: cmd=%s, goal=%s, lidar=%s",
                      self.cmd_vel_topic, self.goal_topic, self.lidar_topic)
        rospy.loginfo("[RL] publish: cmd=%s, obs=%s", self.out_cmd_topic, self.obs_topic)
        rospy.loginfo("[RL] gravity = [%.6f, %.6f, %.6f]", self.gravity_x, self.gravity_y, self.gravity_z)

    # =========================
    # 回调
    # =========================
    def cb_cmd_vel(self, msg):
        self.latest_lin3 = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
        ], dtype=np.float32)

        self.latest_ang3 = np.array([
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ], dtype=np.float32)

    def cb_goal(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        if arr.size != 4:
            rospy.logwarn_throttle(1.0, "[RL] /rl_local_goal_body dim != 4, got %d", arr.size)
            return
        self.latest_goal4 = arr

    def cb_lidar(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        if arr.size != 543:
            rospy.logwarn_throttle(1.0, "[RL] /rl/lidar_543 dim != 543, got %d", arr.size)
            return
        self.latest_lidar543 = arr

    # =========================
    # 工具函数
    # =========================
    @staticmethod
    def _clip_if_valid(value, min_v, max_v):
        out = value
        if not math.isnan(min_v):
            out = max(out, min_v)
        if not math.isnan(max_v):
            out = min(out, max_v)
        return out

    def _ready(self):
        return (
            self.latest_lin3 is not None and
            self.latest_ang3 is not None and
            self.latest_goal4 is not None and
            self.latest_lidar543 is not None
        )

    def _build_obs(self):
        lin3 = self.latest_lin3.astype(np.float32)
        ang3 = self.latest_ang3.astype(np.float32)
        goal4 = self.latest_goal4.astype(np.float32)
        lidar543 = self.latest_lidar543.astype(np.float32)

        projected_gravity = np.array([
            self.gravity_x,
            self.gravity_y,
            self.gravity_z,
        ], dtype=np.float32)

        obs = np.concatenate([
            lin3,
            ang3,
            projected_gravity,
            goal4,
            lidar543,
        ], axis=0).astype(np.float32)

        if obs.size != 556:
            raise RuntimeError("obs dim error: expect 556, got {}".format(obs.size))

        return obs

    def _infer(self, obs_556):
        obs_batch = obs_556.reshape(1, 556).astype(np.float32)
        outputs = self.session.run([self.output_name], {self.input_name: obs_batch})
        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)

        if action.size != 2:
            raise RuntimeError("action dim error: expect 2, got {}".format(action.size))

        v = float(action[0])
        w = float(action[1])
        return v, w

    # =========================
    # 主循环
    # =========================
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self._ready():
                rospy.logwarn_throttle(2.0, "[RL] waiting topics... cmd/goal/lidar not all ready")
                rate.sleep()
                continue

            try:
                obs = self._build_obs()

                if self.publish_obs:
                    obs_msg = Float32MultiArray()
                    obs_msg.data = obs.tolist()
                    self.pub_obs.publish(obs_msg)

                if self.debug_obs_log:
                    rospy.loginfo_throttle(
                        1.0,
                        "[RL][OBS] lin=%s ang=%s gravity=%s goal=%s lidar0=%s lidar1=%s lidar2=%s",
                        np.array2string(obs[0:3], precision=4, separator=','),
                        np.array2string(obs[3:6], precision=4, separator=','),
                        np.array2string(obs[6:9], precision=4, separator=','),
                        np.array2string(obs[9:13], precision=4, separator=','),
                        np.array2string(obs[13:18], precision=4, separator=','),
                        np.array2string(obs[194:199], precision=4, separator=','),
                        np.array2string(obs[375:380], precision=4, separator=','),
                    )

                v, w = self._infer(obs)
                v_raw, w_raw = v, w

                v = self._clip_if_valid(v, self.clip_v_min, self.clip_v_max)
                w = self._clip_if_valid(w, self.clip_w_min, self.clip_w_max)

                cmd = Twist()
                cmd.linear.x = v
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = w
                self.pub_cmd.publish(cmd)

                if self.debug_log:
                    rospy.loginfo_throttle(
                        0.5,
                        "[RL] raw_action=[%.6f, %.6f] -> cmd_vel=[%.6f, %.6f]",
                        v_raw, w_raw, v, w
                    )

            except Exception as e:
                rospy.logerr_throttle(1.0, "[RL] inference failed: %s", str(e))

            rate.sleep()


if __name__ == "__main__":
    try:
        node = RLOnnxInferenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
