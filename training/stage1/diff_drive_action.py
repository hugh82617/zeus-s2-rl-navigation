# Copyright (c) 2022-2026, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

# Low-level joint velocity action
from isaaclab.envs.mdp.actions.actions_cfg import JointVelocityActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointVelocityAction

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class DiffDriveAction(ActionTerm):
    """Diff-drive action term.

    Policy outputs: [v, w] in base frame.
      v: linear velocity (m/s)
      w: yaw rate (rad/s)

    Internal mapping (standard differential drive):
      v_left  = v - w * L/2
      v_right = v + w * L/2
      omega_left  = v_left / r
      omega_right = v_right / r

    Wheel order enforced as: [fl, fr, rl, rr]
    """

    cfg: "DiffDriveActionCfg"

    def __init__(self, cfg: "DiffDriveActionCfg", env: "ManagerBasedEnv") -> None:
        super().__init__(cfg, env)

        # cache params
        self._r = float(cfg.wheel_radius)
        self._L = float(cfg.wheel_separation)
        if self._r <= 0.0:
            raise ValueError(f"wheel_radius must be > 0, got {self._r}")
        if self._L <= 0.0:
            raise ValueError(f"wheel_separation must be > 0, got {self._L}")

        # enforce wheel joint order
        self._wheel_joint_names = [
            cfg.wheel_fl_joint,
            cfg.wheel_fr_joint,
            cfg.wheel_rl_joint,
            cfg.wheel_rr_joint,
        ]

        # raw action: [v, w]
        self._raw_actions = torch.zeros(self.num_envs, 2, device=self.device)

        # wheel targets: [fl, fr, rl, rr] in rad/s
        self._wheel_targets = torch.zeros(self.num_envs, 4, device=self.device)

        # build a safe low-level joint velocity action term
        # IMPORTANT: force offset to zero and disable default offset
        self._ll_cfg = JointVelocityActionCfg(
            asset_name=cfg.asset_name,
            joint_names=self._wheel_joint_names,
            scale=1.0,
            offset=0.0,
            use_default_offset=False,
            preserve_order=True,
        )
        # instantiate the low-level action term
        # (we rely on our own safety clamp; ll_cfg.clip is optional and not required here)
        self._ll_term: ActionTerm = JointVelocityAction(self._ll_cfg, env)

    # --- properties expected by ActionTerm ---
    @property
    def action_dim(self) -> int:
        # [v, w]
        return 2

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        # for this term, we treat raw_actions as processed (we do our own clamps in apply_actions)
        return self._raw_actions

    # --- operations ---
    def process_actions(self, actions: torch.Tensor):
        # store raw [v, w]
        # (we do NOT clamp here to keep a single control point in apply_actions)
        self._raw_actions[:] = actions

    def apply_actions(self):
        # unpack v, w
        v = self._raw_actions[:, 0]
        w = self._raw_actions[:, 1]

        # ---------------------------
        # SAFETY CLIP 1: clip v and w
        # ---------------------------
        if self.cfg.max_linear_speed is not None:
            v = torch.clamp(v, self.cfg.min_linear_speed, self.cfg.max_linear_speed)
        if self.cfg.max_angular_speed is not None:
            w = torch.clamp(w, -self.cfg.max_angular_speed, self.cfg.max_angular_speed)

        # diff-drive mapping
        half_L = 0.5 * self._L
        omega_left = (v - w * half_L) / self._r
        omega_right = (v + w * half_L) / self._r

        # order: [fl, fr, rl, rr]
        self._wheel_targets[:, 0] = omega_left
        self._wheel_targets[:, 1] = omega_right
        self._wheel_targets[:, 2] = omega_left
        self._wheel_targets[:, 3] = omega_right

        # -------------------------------
        # SAFETY CLIP 2: clip wheel omega
        # -------------------------------
        if self.cfg.max_wheel_omega is not None:
            self._wheel_targets[:] = torch.clamp(
                self._wheel_targets,
                -self.cfg.max_wheel_omega,
                self.cfg.max_wheel_omega,
            )

        # send to low-level joint velocity controller
        self._ll_term.process_actions(self._wheel_targets)
        self._ll_term.apply_actions()


@configclass
class DiffDriveActionCfg(ActionTermCfg):
    """Config for DiffDriveAction."""

    class_type: type[ActionTerm] = DiffDriveAction

    # articulation asset name in scene
    asset_name: str = "robot"

    # wheel joints (must exist)
    wheel_fl_joint: str = MISSING
    wheel_fr_joint: str = MISSING
    wheel_rl_joint: str = MISSING
    wheel_rr_joint: str = MISSING

    # kinematics
    wheel_radius: float = MISSING       # meters
    wheel_separation: float = MISSING   # meters (left-right distance)

    # safety limits
    # Set these explicitly for your robot; defaults below are "indoor safe-ish"
    max_linear_speed: float | None = 1.0    # m/s
    min_linear_speed: float = 0.0           # m/s  (0.0 = no reverse)
    max_angular_speed: float | None = 2.0   # rad/s
    max_wheel_omega: float | None = 20.0    # rad/s
