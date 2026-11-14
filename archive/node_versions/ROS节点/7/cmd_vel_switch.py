#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
cmd_vel_switch — 速度指令切换节点（支持遥控器最高优先级抢占）

订阅:
  ~teleop_topic (Twist)   遥控器速度
  ~dwa_topic    (Twist)   DWA速度
  ~rl_topic     (Twist)   RL速度
  ~mode_topic   (String)  模式切换：dwa / rl

发布:
  ~out_topic    (Twist)   最终输出速度
  ~active_topic (String)  当前实际激活来源：teleop / dwa / rl

功能：
  1. 支持 teleop / dwa / rl 三路输入
  2. 遥控器 teleop 具有最高优先级，可随时中断 RL 和 DWA
  3. teleop 超时后，自动恢复到当前 mode 指定的 rl 或 dwa
  4. 超时保护：当前有效来源超时后自动输出零速
  5. 模式切换时主动发一帧零速，防止滑行
  6. 定频发布，保证下游始终有控制指令
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def zero_twist():
    return Twist()


class CmdVelSwitch(object):

    VALID_MODES = ("dwa", "rl")

    def __init__(self):
        rospy.init_node("cmd_vel_switch")

        # 话题参数
        self.teleop_topic = rospy.get_param("~teleop_topic", "/teleop_cmd_vel")
        self.dwa_topic = rospy.get_param("~dwa_topic", "/raw_cmd_vel_dwa")
        self.rl_topic = rospy.get_param("~rl_topic", "/rl_cmd_vel")
        self.out_topic = rospy.get_param("~out_topic", "/raw_cmd_vel")
        self.mode_topic = rospy.get_param("~mode_topic", "/cmd_vel_mode")
        self.active_topic = rospy.get_param("~active_topic", "/cmd_vel_switch/active_mode")

        # 控制参数
        self.pub_rate = float(rospy.get_param("~pub_rate", 20.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.teleop_timeout = float(rospy.get_param("~teleop_timeout", 0.3))

        # 初始基础模式（仅决定 teleop 不存在时用 rl 还是 dwa）
        init_mode = rospy.get_param("~mode", "rl").strip().lower()
        if init_mode not in self.VALID_MODES:
            rospy.logwarn("[Switch] 无效的初始模式 '%s'，回退到 'rl'", init_mode)
            init_mode = "rl"
        self._mode = init_mode

        # 状态缓存
        self._last_cmd = {
            "teleop": zero_twist(),
            "dwa": zero_twist(),
            "rl": zero_twist(),
        }
        self._last_time = {
            "teleop": rospy.Time(0),
            "dwa": rospy.Time(0),
            "rl": rospy.Time(0),
        }

        self._mode_dirty = False
        self._last_effective_source = None

        # 发布器
        self._cmd_pub = rospy.Publisher(self.out_topic, Twist, queue_size=1)
        self._active_pub = rospy.Publisher(self.active_topic, String, queue_size=1, latch=True)

        # 订阅器
        rospy.Subscriber(self.teleop_topic, Twist, self._teleop_cb, queue_size=1)
        rospy.Subscriber(self.dwa_topic, Twist, self._dwa_cb, queue_size=1)
        rospy.Subscriber(self.rl_topic, Twist, self._rl_cb, queue_size=1)
        rospy.Subscriber(self.mode_topic, String, self._mode_cb, queue_size=1)

        # 启动时广播当前基础模式
        self._publish_active_mode(self._mode)

        # 定时发布
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self._timer_cb)

        rospy.loginfo("[Switch] 节点启动")
        rospy.loginfo("[Switch] 初始基础模式 : %s", self._mode)
        rospy.loginfo("[Switch] teleop 输入  : %s", self.teleop_topic)
        rospy.loginfo("[Switch] dwa 输入     : %s", self.dwa_topic)
        rospy.loginfo("[Switch] rl 输入      : %s", self.rl_topic)
        rospy.loginfo("[Switch] 输出         : %s", self.out_topic)
        rospy.loginfo("[Switch] 模式切换     : %s", self.mode_topic)
        rospy.loginfo("[Switch] 激活模式回显 : %s", self.active_topic)
        rospy.loginfo("[Switch] 发布频率     : %.1f Hz", self.pub_rate)
        rospy.loginfo("[Switch] teleop超时   : %.2f s", self.teleop_timeout)
        rospy.loginfo("[Switch] rl/dwa超时   : %.2f s", self.cmd_timeout)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, new_mode):
        if new_mode == self._mode:
            return
        rospy.logwarn("[Switch] 基础模式切换: %s → %s", self._mode, new_mode)
        self._mode = new_mode
        self._mode_dirty = True

    def _teleop_cb(self, msg):
        self._last_cmd["teleop"] = msg
        self._last_time["teleop"] = rospy.Time.now()

    def _dwa_cb(self, msg):
        self._last_cmd["dwa"] = msg
        self._last_time["dwa"] = rospy.Time.now()

    def _rl_cb(self, msg):
        self._last_cmd["rl"] = msg
        self._last_time["rl"] = rospy.Time.now()

    def _mode_cb(self, msg):
        req = msg.data.strip().lower()
        if req in self.VALID_MODES:
            self.mode = req
        else:
            rospy.logwarn("[Switch] 收到无效模式指令: '%s'（合法值: dwa / rl）", msg.data)

    def _is_fresh(self, source, timeout):
        t = self._last_time[source]
        if t == rospy.Time(0):
            return False
        return (rospy.Time.now() - t).to_sec() < timeout

    def _publish_active_mode(self, source):
        self._active_pub.publish(String(data=source))

    def _select_effective_source(self):
        # 优先级：teleop > 当前mode(rl/dwa)
        if self._is_fresh("teleop", self.teleop_timeout):
            return "teleop"
        return self._mode

    def _timer_cb(self, event):
        effective_source = self._select_effective_source()

        # 实际控制来源变化时，先打一帧零速，避免突变
        if self._last_effective_source is None:
            self._last_effective_source = effective_source
            self._publish_active_mode(effective_source)
        elif effective_source != self._last_effective_source:
            rospy.logwarn("[Switch] 实际控制来源切换: %s → %s",
                          self._last_effective_source, effective_source)
            self._last_effective_source = effective_source
            self._publish_active_mode(effective_source)
            self._cmd_pub.publish(zero_twist())
            rospy.loginfo_throttle(1.0, "[Switch] 来源切换缓冲零速帧已发出")
            return

        # 基础模式切换后，如果当前不是 teleop 抢占，则插入一帧零速
        if self._mode_dirty and effective_source != "teleop":
            self._cmd_pub.publish(zero_twist())
            self._mode_dirty = False
            rospy.loginfo_throttle(1.0, "[Switch] 基础模式切换缓冲零速帧已发出")
            return
        elif self._mode_dirty and effective_source == "teleop":
            # teleop 抢占期间不处理，等退出 teleop 后由来源切换逻辑处理
            pass

        # 当前实际来源输出
        if effective_source == "teleop":
            if self._is_fresh("teleop", self.teleop_timeout):
                out = self._last_cmd["teleop"]
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "[Switch] teleop 指令超时（> %.2f s），输出零速",
                    self.teleop_timeout
                )
                out = zero_twist()
        else:
            if self._is_fresh(effective_source, self.cmd_timeout):
                out = self._last_cmd[effective_source]
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "[Switch] 当前模式 '%s' 指令超时（> %.2f s），输出零速",
                    effective_source, self.cmd_timeout
                )
                out = zero_twist()

        self._cmd_pub.publish(out)


if __name__ == "__main__":
    try:
        CmdVelSwitch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
