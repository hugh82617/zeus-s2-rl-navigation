# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
commands_V36.py  —  Stage-1 自定义目标点命令
=============================================
功能：
  - MinDistPose2dCommand：带 reject sampling 的 2D 目标点采样
      保证每次 episode 起始距离在 [min_start_dist, max_start_dist] 范围内
  - heading_command_w 强制指向 robot→goal 方向（朝外），
      防止 simple_heading=True 时父类选"朝内"方向导致陀螺旋转
  - self.metrics 提供所有任务级监控指标（不走奖励项，真实无偏）

监控指标（TensorBoard / 控制台均可见）：
  Metrics/pose_command/start_dist          episode 起始距离（m）
  Metrics/pose_command/error_heading       当前朝向与 robot→goal 方位角误差（rad）
  Metrics/pose_command/lidar_min_range_m   最近 lidar 距离（m）
  Metrics/pose_command/near_wall_ratio     是否进入 near_wall_threshold 内（0/1）
  Metrics/pose_command/collision_rate      当步碰撞率（0/1，与 illegal_contact 同口径）
  Metrics/pose_command/lin_speed_abs_mean  前向线速度绝对值（m/s）
  Metrics/pose_command/ang_speed_abs_mean  yaw 角速度绝对值（rad/s）
  Metrics/pose_command/toward_goal_speed   速度在 robot→goal 方向的投影（m/s，≥0）
  Metrics/pose_command/path_efficiency     start_dist / 累计路径长度，越近1越直
"""

from __future__ import annotations

from dataclasses import MISSING
from typing import TYPE_CHECKING, Sequence

import torch

from isaaclab.envs.mdp.commands import UniformPose2dCommand
from isaaclab.envs.mdp.commands.commands_cfg import UniformPose2dCommandCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


@configclass
class MinDistPose2dCommandCfg(UniformPose2dCommandCfg):
    """带距离约束 + 监控 metrics 的 2D 目标点命令配置。"""

    class_type: type = MISSING

    # reject sampling 参数
    min_start_dist: float = 1.5
    """目标与机器人的最小起始距离（m）。"""

    max_start_dist: float = float("inf")
    """目标与机器人的最大起始距离（m），默认不限制。"""

    max_resample_attempts: int = 50
    """reject sampling 最大重试次数。"""

    # lidar 监控参数
    lidar_sensor_name: str = "lidar"
    """scene 中 lidar 传感器的名字。"""

    near_wall_threshold: float = 0.7
    """near_wall_ratio 的距离阈值（m）。"""

    lidar_max_distance: float = 12.0
    """lidar 射线最大有效距离（m），inf/nan 用此值填充。"""

    # 碰撞监控参数
    contact_sensor_name: str = "contact_sensor"
    """scene 中接触传感器的名字。"""

    contact_threshold: float = 500.0
    """碰撞判定阈值（N），与 illegal_contact_penalty 保持一致。"""

    contact_body_names: tuple = ("base_footprint",)
    """只统计这些 body 的接触力，排除轮地正常接触误报。"""


class MinDistPose2dCommand(UniformPose2dCommand):
    """带距离约束 + 监控 metrics 的 2D 目标点命令。"""

    cfg: MinDistPose2dCommandCfg

    def __init__(self, cfg: MinDistPose2dCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        # 监控 metrics 初始化（父类已有 error_pos_2d / error_heading）
        self.metrics["start_dist"]          = torch.zeros(self.num_envs, device=self.device)
        self.metrics["lidar_min_range_m"]   = torch.zeros(self.num_envs, device=self.device)
        self.metrics["near_wall_ratio"]     = torch.zeros(self.num_envs, device=self.device)
        self.metrics["collision_rate"]      = torch.zeros(self.num_envs, device=self.device)
        self.metrics["lin_speed_abs_mean"]  = torch.zeros(self.num_envs, device=self.device)
        self.metrics["ang_speed_abs_mean"]  = torch.zeros(self.num_envs, device=self.device)
        self.metrics["toward_goal_speed"]   = torch.zeros(self.num_envs, device=self.device)
        self.metrics["path_efficiency"]     = torch.zeros(self.num_envs, device=self.device)
        # 路径长度累计（episode 内累加，reset 时清零）
        self._path_length = torch.zeros(self.num_envs, device=self.device)
        self._prev_pos    = torch.zeros(self.num_envs, 2, device=self.device)

    # ── 内部工具 ─────────────────────────────────────────────────────────

    def _lidar_min_dist(self) -> torch.Tensor:
        """每个 env 的最小 lidar 距离（m）。

        使用逐射线真实起点 sensor._ray_starts_w，
        过滤 dist < 0.1m 的自击中/无效射线。
        """
        sensor     = self._env.scene.sensors[self.cfg.lidar_sensor_name]
        ray_hits   = sensor.data.ray_hits_w                          # (N, R, 3)
        if hasattr(sensor, '_ray_starts_w'):
            ray_origins = sensor._ray_starts_w                       # (N, R, 3) 逐射线起点
        else:
            ray_origins = sensor.data.pos_w.unsqueeze(1)             # (N, 1, 3) fallback
        dists = torch.norm(ray_hits - ray_origins, dim=-1)           # (N, R)
        invalid = torch.isinf(dists) | torch.isnan(dists) | (dists < 0.1)
        dists = torch.where(
            invalid,
            torch.full_like(dists, self.cfg.lidar_max_distance),
            dists,
        )
        return dists.min(dim=1).values                                # (N,)

    def _collision_triggered(self) -> torch.Tensor:
        """每个 env 当步是否触发碰撞（float 0/1）。

        只看 cfg.contact_body_names 指定的 body（默认 base_footprint），
        与 illegal_contact_penalty 完全同口径。
        """
        sensor     = self._env.scene.sensors[self.cfg.contact_sensor_name]
        body_ids, _ = sensor.find_bodies(self.cfg.contact_body_names)
        forces      = sensor.data.net_forces_w_history[:, -1, :, :]  # (N, B, 3) 当前帧
        forces      = forces[:, body_ids, :]                          # (N, len, 3)
        force_norm  = torch.norm(forces, dim=-1)                      # (N, len)
        return (force_norm.max(dim=1).values > self.cfg.contact_threshold).float()

    # ── 每步更新 metrics ─────────────────────────────────────────────────

    def _update_metrics(self):
        """每步（compute() 内部调用）更新所有 metrics。"""
        super()._update_metrics()

        import torch as _torch
        robot   = self._env.scene["robot"]
        goal_w  = self.pos_command_w[:, :2]               # (N, 2)
        robot_w = robot.data.root_pos_w[:, :2]            # (N, 2)
        diff    = goal_w - robot_w                        # (N, 2)

        # error_heading：当前朝向与 robot→goal 方位角的误差
        # 覆盖父类的值，避免 simple_heading 角度定义差异导致 >π 异常值
        from isaaclab.utils.math import wrap_to_pi
        heading_now     = robot.data.heading_w            # (N,)
        heading_to_goal = _torch.atan2(diff[:, 1], diff[:, 0])
        self.metrics["error_heading"] = wrap_to_pi(heading_to_goal - heading_now).abs()

        # lidar 监控
        min_dist = self._lidar_min_dist()
        self.metrics["lidar_min_range_m"] = min_dist
        self.metrics["near_wall_ratio"]   = (min_dist < self.cfg.near_wall_threshold).float()

        # 碰撞监控（与 illegal_contact_penalty 同口径）
        self.metrics["collision_rate"] = self._collision_triggered()

        # 速度监控
        lin_vel = robot.data.root_lin_vel_b[:, 0].abs()   # 前向线速度（m/s）
        ang_vel = robot.data.root_ang_vel_b[:, 2].abs()   # yaw 角速度（rad/s）
        self.metrics["lin_speed_abs_mean"] = lin_vel
        self.metrics["ang_speed_abs_mean"] = ang_vel

        # 朝目标速度：世界系速度在 robot→goal 方向的投影（≥0）
        vel_w        = robot.data.root_lin_vel_w[:, :2]   # (N, 2) 世界系线速度
        dist_now     = _torch.norm(diff, dim=1, keepdim=True).clamp(min=1e-3)
        unit_to_goal = diff / dist_now                    # (N, 2) 单位向量
        self.metrics["toward_goal_speed"] = (vel_w * unit_to_goal).sum(dim=1).clamp(min=0.0)

        # 路径效率：start_dist / 累计路径长度
        cur_pos    = robot.data.root_pos_w[:, :2]
        step_dist  = _torch.norm(cur_pos - self._prev_pos, dim=1)
        self._path_length += step_dist
        self._prev_pos     = cur_pos.clone()
        self.metrics["path_efficiency"] = (
            self.metrics["start_dist"] / self._path_length.clamp(min=1e-3)
        ).clamp(max=2.0)

    # ── episode 重置：目标重采样 ─────────────────────────────────────────

    def _resample_command(self, env_ids: Sequence[int]):
        """带 reject sampling 的目标点重采样，更新 start_dist 和 heading。"""
        # 统一转成 Python list[int]
        if hasattr(env_ids, 'tolist'):
            remaining_ids = env_ids.tolist()
        else:
            remaining_ids = [int(i) for i in env_ids]

        # reject sampling：重采样直到距离满足 [min_start_dist, max_start_dist]
        for _ in range(self.cfg.max_resample_attempts):
            super()._resample_command(remaining_ids)
            ids_tensor = torch.tensor(remaining_ids, device=self.device, dtype=torch.long)
            goal_w  = self.pos_command_w[ids_tensor, :2]
            robot_w = self.robot.data.root_pos_w[ids_tensor, :2]
            dist    = torch.norm(goal_w - robot_w, dim=1)
            need_resample = (dist < self.cfg.min_start_dist) | (dist > self.cfg.max_start_dist)
            if not need_resample.any():
                break
            remaining_ids = [
                remaining_ids[i] for i in range(len(remaining_ids))
                if need_resample[i].item()
            ]

        all_ids = torch.tensor(
            env_ids.tolist() if hasattr(env_ids, 'tolist') else [int(i) for i in env_ids],
            device=self.device, dtype=torch.long,
        )
        goal_w  = self.pos_command_w[all_ids, :2]
        robot_w = self.robot.data.root_pos_w[all_ids, :2]

        # 记录起始距离
        self.metrics["start_dist"][all_ids] = torch.norm(goal_w - robot_w, dim=1)

        # 强制 heading_command_w 指向 robot→goal（朝外）
        # 防止父类 simple_heading=True 选"最近车头方向"（可能朝内）导致陀螺旋转
        diff = goal_w - robot_w
        self.heading_command_w[all_ids] = torch.atan2(diff[:, 1], diff[:, 0])

        # 路径统计清零
        self._path_length[all_ids] = 0.0
        self._prev_pos[all_ids]    = self._env.scene["robot"].data.root_pos_w[all_ids, :2].clone()


# class_type 默认值在类定义之后填入
MinDistPose2dCommandCfg.__dataclass_fields__["class_type"].default = MinDistPose2dCommand
