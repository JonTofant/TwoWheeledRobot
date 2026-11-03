"""Shared env construction and rollout helpers for the payload-mass scripts.

Import only after AppLauncher has started (needs isaaclab + the task package).
"""

from __future__ import annotations

import random

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from estimator import FEATURE_GROUPS  # noqa: F401  (re-exported)

from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

TASK = "Template-Twowheeledrobot-NNDrivePayload-v0"


def make_env(num_envs: int, seed: int, **overrides):
    random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    cfg = load_cfg_from_registry(TASK, "env_cfg_entry_point")
    cfg.seed = seed
    cfg.scene.num_envs = num_envs
    cfg.terrain_mode = "flat"
    for key, value in overrides.items():
        if not hasattr(cfg, key):
            raise AttributeError(f"unknown cfg field {key}")
        setattr(cfg, key, value)
    return gym.make(TASK, cfg=cfg)


def estimator_inputs(env, obs: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
    """Per-step estimator input, (n, 23); everything is readable on the robot.

    [0:13]  the balance policy's own 13-value observation (as the policy saw it)
    [13:15] the policy's wheel action this step, as commanded current (A)
    [15:19] CyberGear joint angles (rad), fl fr bl br
    [19:23] CyberGear torque estimates (Nm), fl fr bl br
    """
    u = env.unwrapped
    current = torch.tanh(action) * u.cfg.i_max_a
    return torch.cat([obs, current, u.leg_features()], dim=1)

