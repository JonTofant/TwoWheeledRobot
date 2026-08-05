# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

# The joystick-commanded drive task is the only trainable task here. The
# Standup, ResidualLQR and PureNNBalance registrations were removed 2026-08-05
# when the repository was narrowed to the MDPI actuators work.
#
# standup_env.py and pure_nn_balance_env.py remain on disk because NNDriveEnv
# derives from them (NNDriveEnv -> PureNNBalanceEnv -> StandupEnv). They are
# base classes now, not tasks, and are no longer reachable through gym.

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
