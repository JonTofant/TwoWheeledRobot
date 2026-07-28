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

gym.register(
    id="Template-Twowheeledrobot-ResidualLQR-v0",
    entry_point=f"{__name__}.residual_lqr_env:ResidualLqrEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point":    f"{__name__}.residual_lqr_env_cfg:ResidualLqrEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_residual_lqr_cfg:ResidualLqrPPORunnerCfg",
    },
)

gym.register(
    id="Template-Twowheeledrobot-PureNNBalance-v0",
    entry_point=f"{__name__}.pure_nn_balance_env:PureNNBalanceEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pure_nn_balance_env_cfg:PureNNBalanceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_pure_nn_balance_cfg:PureNNBalancePPORunnerCfg",
    },
)

gym.register(
    id="Template-Twowheeledrobot-NNDrive-v0",
    entry_point=f"{__name__}.nn_drive_env:NNDriveEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.nn_drive_env_cfg:NNDriveEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_nn_drive_cfg:NNDrivePPORunnerCfg",
    },
)

# Presentation-only variant of the NN drive task: identical policy contract, but
# a hand-built training-ground scene (pads + drop ledge) for video/screenshots.
# Used by scripts/record_isaac_demo.py; never trained against.
gym.register(
    id="Template-Twowheeledrobot-NNDriveDemo-v0",
    entry_point=f"{__name__}.nn_drive_demo_env:NNDriveDemoEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.nn_drive_demo_env_cfg:NNDriveDemoEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_nn_drive_cfg:NNDrivePPORunnerCfg",
    },
)
