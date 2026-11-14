# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
obs_stage3.py  —  Stage-3 专属观测函数
=======================================
部署文件名：obs_stage3.py

新增函数：
  lidar_stack_obs：输出 3 帧 LiDAR stack，shape (N, 543)

obs 维度布局（共 556 维）：
  [0:3]    base_lin_vel           当前帧线速度
  [3:6]    base_ang_vel           当前帧角速度
  [6:9]    projected_gravity      当前帧重力投影
  [9:13]   pose_commands          当前帧目标相对量 [dx_b, dy_b, dz_b, dyaw_b]
  [13:194] lidar_t0               当前帧 LiDAR（181 条射线）
  [194:375] lidar_t1              上一帧 LiDAR（181 条射线）
  [375:556] lidar_t2              上上帧 LiDAR（181 条射线）

LiDAR 历史缓冲区管理：
  缓冲区形状 (N, 3, 181)，顺序 [t0, t1, t2]，t0 最新。
  Reset 检测：episode_length_buf <= 1 时用当前帧填满全部三个槽，
  与 rewards_stage3.update_d_front_dot 的 reset 检测模式完全一致。
  不依赖额外回调，逻辑全部收敛在此函数内部。
"""

from __future__ import annotations
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lidar_stack_obs(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    max_distance: float = 12.0,
) -> torch.Tensor:
    """输出 3 帧 LiDAR stack，shape (N, 543)。

    每帧归一化到 [0,1]（原始距离 / max_distance）。
    帧顺序：[lidar_t0 | lidar_t1 | lidar_t2]，t0 最新。

    LiDAR 历史缓冲区在此函数内部维护：
      - 首次调用时初始化（3 帧全用当前帧填充）
      - episode_length_buf <= 1 时对发生 reset 的 env 重置历史
      - 每步滚动更新：t2 ← t1，t1 ← t0，t0 ← 当前帧

    Returns:
        (N, 543) 张量，值域 [0, 1]
    """
    sensor = env.scene.sensors[sensor_cfg.name]

    # ── 计算当前帧 LiDAR 距离（归一化）─────────────────────────────────
    hits_w  = sensor.data.ray_hits_w          # (N, 181, 3)，无命中为 inf
    pos_w   = sensor.data.pos_w               # (N, 3)

    dists = torch.norm(hits_w - pos_w.unsqueeze(1), dim=-1)  # (N, 181)
    # inf（无命中）替换为 max_distance，归一化到 [0, 1]
    dists = torch.clamp(
        torch.where(torch.isinf(dists), torch.full_like(dists, max_distance), dists),
        0.0, max_distance
    ) / max_distance   # (N, 181)

    # ── 历史缓冲区初始化（首次调用）────────────────────────────────────
    if not hasattr(env, '_s3_lidar_history') or env._s3_lidar_history is None:
        # 首次调用：三帧全用当前帧填充，不产生虚假差分信号
        env._s3_lidar_history = dists.unsqueeze(1).expand(-1, 3, -1).clone()

    # ── 滚动更新：t2 ← t1，t1 ← t0，t0 ← 当前帧 ──────────────────────
    # 必须先滚动更新，再做 reset 覆盖。
    # 顺序不能颠倒：若先 reset 再 roll，刚填好的三帧初始状态会被 roll 打乱，
    # t1/t2 会混入 roll 之前缓冲区里的旧 episode 数据。
    env._s3_lidar_history = torch.roll(env._s3_lidar_history, shifts=1, dims=1)
    env._s3_lidar_history[:, 0, :] = dists

    # ── Episode reset 覆盖（必须在滚动更新之后）──────────────────────────
    # 对发生 reset 的 env，用当前帧填满三个槽，消除上一 episode 的残留历史。
    # 与 rewards_stage3.update_d_front_dot 的 reset 检测模式完全一致。
    reset_mask = env.episode_length_buf <= 1   # (N,) bool
    if reset_mask.any():
        env._s3_lidar_history[reset_mask] = (
            dists[reset_mask].unsqueeze(1).expand(-1, 3, -1).clone()
        )

    # ── 拼接三帧输出 ─────────────────────────────────────────────────────
    t0 = env._s3_lidar_history[:, 0, :]   # (N, 181) 当前帧
    t1 = env._s3_lidar_history[:, 1, :]   # (N, 181) 上一帧
    t2 = env._s3_lidar_history[:, 2, :]   # (N, 181) 上上帧

    return torch.cat([t0, t1, t2], dim=-1)   # (N, 543)
