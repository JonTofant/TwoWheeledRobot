# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register the stand-up task.
##

gym.register(
    id="Template-Twowheeledrobot-Standup-v0",
    entry_point=f"{__name__}.standup_env:StandupEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point":    f"{__name__}.standup_env_cfg:StandupEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_standup_cfg:StandupPPORunnerCfg",
    },
)
