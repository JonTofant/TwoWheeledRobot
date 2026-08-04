# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Small RSL-RL runner extensions."""

from collections.abc import Sequence
from typing import Any

import torch


def enforce_action_std_floor(runner: Any, action_std_floor: Sequence[float] | None) -> None:
    """Keep selected action exploration standard deviations above a fixed floor."""
    if action_std_floor is None:
        return

    policy = runner.alg.policy
    if hasattr(policy, "log_std"):
        std_parameter = policy.log_std
        floor = torch.as_tensor(action_std_floor, device=std_parameter.device, dtype=std_parameter.dtype).log()
    elif hasattr(policy, "std"):
        std_parameter = policy.std
        floor = torch.as_tensor(action_std_floor, device=std_parameter.device, dtype=std_parameter.dtype)
    else:
        raise ValueError("Action std floors require a policy with state-independent action noise.")

    if floor.shape != std_parameter.shape:
        raise ValueError(f"Expected {std_parameter.numel()} action std floors, got {floor.numel()}.")

    def clamp_std() -> None:
        with torch.no_grad():
            std_parameter.copy_(torch.maximum(std_parameter, floor))

    clamp_std()
    runner.alg.optimizer.register_step_post_hook(lambda *_args, **_kwargs: clamp_std())
