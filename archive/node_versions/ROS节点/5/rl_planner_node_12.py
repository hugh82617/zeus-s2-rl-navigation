#!/home/ros/miniconda3/envs/onnx_env/bin/python
# -*- coding: utf-8 -*-

import math
import threading

import numpy as np
import onnxruntime as ort
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray


LIDAR_FRAME_DIM = 181
LIDAR_FRAMES = 3
LIDAR_TOTAL = LIDAR_FRAME_DIM * LIDAR_FRAMES  # 543
OBS_DIM = 3 + 3 + 3 + 4 + LIDAR_TOTAL  # 556
ACTION_DIM = 2


class RLOnnxInferenceNode(object):
    def __init__(self):
        rospy.init_node("rl_onnx_inference_node", anonymous=False)
        self.initialized = False

        default_model = "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx"
        self.model_path = rospy.get_param("~model_path", default_model)
        self.input_name = rospy.get_param("~input_name", "obs")
        self.output_name = rospy.get_param("~output_name", "action")

        self.speed_topic = rospy.get_param("~speed_topic", "/zeus_speed")
        self.goal_topic = rospy.get_param("~goal_topic", "/rl_local_goal_body")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/rl/lidar_543")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu")
        self.out_cmd_topic = rospy.get_param("~out_cmd_topic", "/rl_cmd_vel")
        self.obs_topic = rospy.get_param("~obs_topic", "/rl/obs_556")

        self.debug_lin_topic = rospy.get_param("~debug_lin_topic", "/rl/debug/lin_vel3")
        self.debug_ang_topic = rospy.get_param("~debug_ang_topic", "/rl/debug/ang_vel3")
        self.debug_goal_topic = rospy.get_param("~debug_goal_topic", "/rl/debug/goal4")
        self.debug_obs0_topic = rospy.get_param("~debug_obs0_topic", "/rl/debug/obs0_181")
        self.debug_obs1_topic = rospy.get_param("~debug_obs1_topic", "/rl/debug/obs1_181")
        self.debug_obs2_topic = rospy.get_param("~debug_obs2_topic", "/rl/debug/obs2_181")
        self.debug_speed_age_topic = rospy.get_param("~debug_speed_age_topic", "/rl/debug/speed_age_ms")
        self.debug_goal_age_topic = rospy.get_param("~debug_goal_age_topic", "/rl/debug/goal_age_ms")
        self.debug_age_pair_topic = rospy.get_param("~debug_age_pair_topic", "/rl/debug/sync_age_pair_ms")

        self.use_static_gravity = bool(rospy.get_param("~use_static_gravity", False))
        self.gravity_x = float(rospy.get_param("~gravity_x", 0.0))
        self.gravity_y = float(rospy.get_param("~gravity_y", 0.0))
        self.gravity_z = float(rospy.get_param("~gravity_z", -1.0))

        self.goal_nonzero_eps = float(rospy.get_param("~goal_nonzero_eps", 1e-4))
        self.goal_pos_eps_enter = float(rospy.get_param("~goal_pos_eps_enter", 0.05))
        self.goal_pos_eps_exit = float(rospy.get_param("~goal_pos_eps_exit", 0.02))
        self.goal_yaw_eps_enter = float(rospy.get_param("~goal_yaw_eps_enter", 0.05))
        self.goal_yaw_eps_exit = float(rospy.get_param("~goal_yaw_eps_exit", 0.02))
        self.use_split_goal_gate = bool(rospy.get_param("~use_split_goal_gate", True))
        self._goal_active = False

        self.publish_obs_when_goal_invalid = bool(
            rospy.get_param("~publish_obs_when_goal_invalid", False)
        )

        self.clip_v_min = float(rospy.get_param("~clip_v_min", -0.2))
        self.clip_v_max = float(rospy.get_param("~clip_v_max", 1.5))
        self.clip_w_min = float(rospy.get_param("~clip_w_min", -3.0))
        self.clip_w_max = float(rospy.get_param("~clip_w_max", 3.0))

        self.publish_obs = bool(rospy.get_param("~publish_obs", True))
        self.debug_publish_inputs = bool(rospy.get_param("~debug_publish_inputs", True))
        self.debug_log = bool(rospy.get_param("~debug_log", True))
        self.debug_obs_log = bool(rospy.get_param("~debug_obs_log", False))
        self.debug_age_log = bool(rospy.get_param("~debug_age_log", True))
        self.zero_cmd_when_goal_invalid = bool(rospy.get_param("~zero_cmd_when_goal_invalid", True))

        self.warn_speed_age_ms = float(rospy.get_param("~warn_speed_age_ms", 60.0))
        self.warn_goal_age_ms = float(rospy.get_param("~warn_goal_age_ms", 80.0))
        self.drop_if_speed_too_old = bool(rospy.get_param("~drop_if_speed_too_old", False))
        self.drop_if_goal_too_old = bool(rospy.get_param("~drop_if_goal_too_old", False))

        self.latest_lin3 = None
        self.latest_ang3 = None
        self.latest_goal4 = None
        self.latest_lidar543 = None
        self.latest_gravity3 = np.array(
            [self.gravity_x, self.gravity_y, self.gravity_z], dtype=np.float32
        )

        self.latest_speed_time = None
        self.latest_goal_time = None
        self.latest_lidar_time = None

        self.lock = threading.Lock()

        rospy.loginfo("[RL] loading ONNX model: %s", self.model_path)
        self.session = ort.InferenceSession(
            self.model_path,
            providers=["CPUExecutionProvider"]
        )

        actual_input_name = self.session.get_inputs()[0].name
        actual_output_name = self.session.get_outputs()[0].name

        if self.input_name != actual_input_name:
            rospy.logwarn(
                "[RL] input_name from param is '%s', but model expects '%s'. Use model value.",
                self.input_name, actual_input_name
            )
            self.input_name = actual_input_name
        if self.output_name != actual_output_name:
            rospy.logwarn(
                "[RL] output_name from param is '%s', but model outputs '%s'. Use model value.",
                self.output_name, actual_output_name
            )
            self.output_name = actual_output_name

        rospy.loginfo("[RL] ONNX ready. input=%s output=%s", self.input_name, self.output_name)

        self.pub_cmd = rospy.Publisher(self.out_cmd_topic, Twist, queue_size=1)
        self.pub_obs = rospy.Publisher(self.obs_topic, Float32MultiArray, queue_size=1)

        self.pub_debug_lin = rospy.Publisher(self.debug_lin_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_ang = rospy.Publisher(self.debug_ang_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_goal = rospy.Publisher(self.debug_goal_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs0 = rospy.Publisher(self.debug_obs0_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs1 = rospy.Publisher(self.debug_obs1_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_obs2 = rospy.Publisher(self.debug_obs2_topic, Float32MultiArray, queue_size=1)
        self.pub_debug_speed_age = rospy.Publisher(self.debug_speed_age_topic, Float32, queue_size=1)
        self.pub_debug_goal_age = rospy.Publisher(self.debug_goal_age_topic, Float32, queue_size=1)
        self.pub_debug_age_pair = rospy.Publisher(self.debug_age_pair_topic, Float32MultiArray, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

        rospy.Subscriber(self.speed_topic, Twist, self.cb_speed, queue_size=1)
        rospy.Subscriber(self.goal_topic, Float32MultiArray, self.cb_goal, queue_size=1)
        rospy.Subscriber(self.lidar_topic, Float32MultiArray, self.cb_lidar, queue_size=1)

        if not self.use_static_gravity:
            rospy.Subscriber(self.imu_topic, Imu, self.cb_imu, queue_size=1)
            rospy.loginfo("[RL] IMU mode: subscribing %s for projected_gravity", self.imu_topic)
        else:
            rospy.loginfo(
                "[RL] Static gravity mode: gravity=[%.3f, %.3f, %.3f]",
                self.gravity_x, self.gravity_y, self.gravity_z
            )

        self.initialized = True

        rospy.loginfo("[RL] subscribed: speed=%s  goal=%s  lidar=%s",
                      self.speed_topic, self.goal_topic, self.lidar_topic)
        rospy.loginfo("[RL] publish: cmd=%s  obs=%s", self.out_cmd_topic, self.obs_topic)
        rospy.loginfo("[RL] mode = lidar-trigger inference + age monitor")
        rospy.loginfo("[RL] warn_speed_age_ms=%.1f warn_goal_age_ms=%.1f",
                      self.warn_speed_age_ms, self.warn_goal_age_ms)

    def cb_imu(self, msg):
        if not self.initialized:
            return
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            return
        grav = np.array([-ax / norm, -ay / norm, -az / norm], dtype=np.float32)
        with self.lock:
            self.latest_gravity3 = grav

    def cb_speed(self, msg):
        if not self.initialized:
            return
        debug_msgs = {}

        with self.lock:
            self.latest_lin3 = np.array(
                [msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float32
            )
            self.latest_ang3 = np.array(
                [msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float32
            )
            self.latest_speed_time = rospy.Time.now()
            if self.debug_publish_inputs:
                debug_msgs["lin"] = self._to_msg(self.latest_lin3)
                debug_msgs["ang"] = self._to_msg(self.latest_ang3)

        if debug_msgs:
            self.pub_debug_lin.publish(debug_msgs["lin"])
            self.pub_debug_ang.publish(debug_msgs["ang"])

    def cb_goal(self, msg):
        if not self.initialized:
            return
        arr = np.array(msg.data, dtype=np.float32)
        if arr.shape != (4,):
            rospy.logwarn_throttle(1.0, "[RL] %s shape != (4,), got %s", self.goal_topic, arr.shape)
            return

        debug_msgs = {}

        with self.lock:
            self.latest_goal4 = arr
            self.latest_goal_time = rospy.Time.now()
            if self.debug_publish_inputs:
                debug_msgs["goal"] = self._to_msg(self.latest_goal4)

        if debug_msgs:
            self.pub_debug_goal.publish(debug_msgs["goal"])

    def cb_lidar(self, msg):
        if not self.initialized:
            return
        arr = np.array(msg.data, dtype=np.float32)
        if arr.shape != (LIDAR_TOTAL,):
            rospy.logwarn_throttle(
                1.0, "[RL] %s shape != (%d,), got %s",
                self.lidar_topic, LIDAR_TOTAL, arr.shape
            )
            return

        debug_msgs = {}
        publish_tasks = []
        speed_age_ms = None
        goal_age_ms = None

        with self.lock:
            self.latest_lidar543 = arr
            self.latest_lidar_time = rospy.Time.now()
            if self.debug_publish_inputs:
                s = LIDAR_FRAME_DIM
                debug_msgs["obs0"] = self._to_msg(arr[0:s])
                debug_msgs["obs1"] = self._to_msg(arr[s:2 * s])
                debug_msgs["obs2"] = self._to_msg(arr[2 * s:3 * s])

            speed_age_ms = self._age_ms(self.latest_lidar_time, self.latest_speed_time)
            goal_age_ms = self._age_ms(self.latest_lidar_time, self.latest_goal_time)
            publish_tasks = self._process_lidar_event_locked(speed_age_ms, goal_age_ms)

        if debug_msgs:
            self.pub_debug_obs0.publish(debug_msgs["obs0"])
            self.pub_debug_obs1.publish(debug_msgs["obs1"])
            self.pub_debug_obs2.publish(debug_msgs["obs2"])

        self._publish_age_topics(speed_age_ms, goal_age_ms)
        self._execute_publish_tasks(publish_tasks)

    def _publish_age_topics(self, speed_age_ms, goal_age_ms):
        if speed_age_ms is not None:
            self.pub_debug_speed_age.publish(Float32(data=float(speed_age_ms)))
        if goal_age_ms is not None:
            self.pub_debug_goal_age.publish(Float32(data=float(goal_age_ms)))
        pair = Float32MultiArray()
        pair.data = [
            -1.0 if speed_age_ms is None else float(speed_age_ms),
            -1.0 if goal_age_ms is None else float(goal_age_ms),
        ]
        self.pub_debug_age_pair.publish(pair)

    @staticmethod
    def _clip_if_valid(value, min_v, max_v):
        out = value
        if min_v is not None and not math.isnan(min_v):
            out = max(out, min_v)
        if max_v is not None and not math.isnan(max_v):
            out = min(out, max_v)
        return out

    @staticmethod
    def _to_msg(arr):
        msg = Float32MultiArray()
        msg.data = np.asarray(arr, dtype=np.float32).tolist()
        return msg

    def _ready(self):
        return (
            self.latest_lin3 is not None and
            self.latest_ang3 is not None and
            self.latest_goal4 is not None and
            self.latest_lidar543 is not None
        )

    def _goal_valid(self):
        if self.latest_goal4 is None:
            self._goal_active = False
            return False

        goal4 = self.latest_goal4.astype(np.float32)

        if self.use_split_goal_gate:
            pos_norm = float(np.linalg.norm(goal4[:3]))
            yaw_abs = float(abs(goal4[3]))

            if self._goal_active:
                self._goal_active = (
                    (pos_norm > self.goal_pos_eps_exit) or
                    (yaw_abs > self.goal_yaw_eps_exit)
                )
            else:
                self._goal_active = (
                    (pos_norm > self.goal_pos_eps_enter) or
                    (yaw_abs > self.goal_yaw_eps_enter)
                )
        else:
            goal_norm = float(np.linalg.norm(goal4))
            if self._goal_active:
                self._goal_active = goal_norm > self.goal_nonzero_eps * 0.5
            else:
                self._goal_active = goal_norm > self.goal_nonzero_eps

        return self._goal_active

    def _build_obs(self):
        lin3 = self.latest_lin3.astype(np.float32)
        ang3 = self.latest_ang3.astype(np.float32)
        gravity3 = self.latest_gravity3.astype(np.float32)
        goal4 = self.latest_goal4.astype(np.float32)
        lidar543 = self.latest_lidar543.astype(np.float32)

        obs = np.concatenate([lin3, ang3, gravity3, goal4, lidar543]).astype(np.float32)

        if obs.shape != (OBS_DIM,):
            raise RuntimeError("obs dim error: expect {}, got {}".format(OBS_DIM, obs.shape))
        return obs

    def _infer(self, obs_556):
        obs_batch = obs_556[None, :].astype(np.float32)
        outputs = self.session.run([self.output_name], {self.input_name: obs_batch})
        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
        if action.shape != (ACTION_DIM,):
            raise RuntimeError("action dim error: expect {}, got {}".format(ACTION_DIM, action.shape))
        return float(action[0]), float(action[1])

    def _age_ms(self, newer, older):
        if newer is None or older is None:
            return None
        return (newer - older).to_sec() * 1000.0

    def _process_lidar_event_locked(self, speed_age_ms, goal_age_ms):
        tasks = []

        if not self._ready():
            rospy.logwarn_throttle(
                2.0,
                "[RL] waiting topics... lidar-trigger  lin3=%s  ang3=%s  goal4=%s  lidar543=%s",
                self.latest_lin3 is not None,
                self.latest_ang3 is not None,
                self.latest_goal4 is not None,
                self.latest_lidar543 is not None,
            )
            return tasks

        if self.debug_age_log:
            speed_str = "None" if speed_age_ms is None else "{:.1f}".format(speed_age_ms)
            goal_str = "None" if goal_age_ms is None else "{:.1f}".format(goal_age_ms)
            rospy.loginfo_throttle(
                0.5,
                "[RL][SYNC] lidar-trigger speed_age_ms=%s goal_age_ms=%s",
                speed_str, goal_str
            )

        if speed_age_ms is not None and speed_age_ms > self.warn_speed_age_ms:
            rospy.logwarn_throttle(
                0.5,
                "[RL][SYNC] speed_age_ms=%.1f > warn_speed_age_ms=%.1f",
                speed_age_ms, self.warn_speed_age_ms
            )
            if self.drop_if_speed_too_old:
                tasks.append((self.pub_cmd, Twist()))
                return tasks

        if goal_age_ms is not None and goal_age_ms > self.warn_goal_age_ms:
            rospy.logwarn_throttle(
                0.5,
                "[RL][SYNC] goal_age_ms=%.1f > warn_goal_age_ms=%.1f",
                goal_age_ms, self.warn_goal_age_ms
            )
            if self.drop_if_goal_too_old:
                tasks.append((self.pub_cmd, Twist()))
                return tasks

        goal_valid = self._goal_valid()

        if not goal_valid:
            if self.publish_obs and self.publish_obs_when_goal_invalid:
                obs = self._build_obs()
                tasks.append((self.pub_obs, self._to_msg(obs)))
            if self.zero_cmd_when_goal_invalid:
                tasks.append((self.pub_cmd, Twist()))
            return tasks

        obs = self._build_obs()

        if self.publish_obs:
            tasks.append((self.pub_obs, self._to_msg(obs)))

        if self.debug_obs_log:
            s = LIDAR_FRAME_DIM
            rospy.loginfo(
                "[RL][OBS][lidar] lin=%s ang=%s gravity=%s goal=%s lidar0[:5]=%s lidar1[:5]=%s lidar2[:5]=%s",
                np.array2string(obs[0:3], precision=4, separator=','),
                np.array2string(obs[3:6], precision=4, separator=','),
                np.array2string(obs[6:9], precision=4, separator=','),
                np.array2string(obs[9:13], precision=4, separator=','),
                np.array2string(obs[13:18], precision=4, separator=','),
                np.array2string(obs[13 + s:18 + s], precision=4, separator=','),
                np.array2string(obs[13 + 2 * s:18 + 2 * s], precision=4, separator=','),
            )

        v, w = self._infer(obs)
        v_raw, w_raw = v, w
        v = self._clip_if_valid(v, self.clip_v_min, self.clip_v_max)
        w = self._clip_if_valid(w, self.clip_w_min, self.clip_w_max)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        tasks.append((self.pub_cmd, cmd))

        if self.debug_log:
            rospy.loginfo_throttle(
                0.5,
                "[RL][lidar] raw=[%.6f, %.6f] -> clipped=[%.6f, %.6f] | lin=%s ang=%s",
                v_raw, w_raw, v, w,
                np.array2string(self.latest_lin3, precision=4, separator=','),
                np.array2string(self.latest_ang3, precision=4, separator=','),
            )

        return tasks

    @staticmethod
    def _execute_publish_tasks(tasks):
        for pub, msg in tasks:
            pub.publish(msg)

    def _on_shutdown(self):
        rospy.loginfo("[RL] shutting down — publishing zero cmd_vel...")
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass

    def spin(self):
        rospy.loginfo("[RL] lidar-driven + age-monitor mode running. waiting callbacks...")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RLOnnxInferenceNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
