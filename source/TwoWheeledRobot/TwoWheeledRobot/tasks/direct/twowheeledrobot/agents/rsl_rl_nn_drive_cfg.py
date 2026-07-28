"""RSL-RL PPO configuration for the joystick NN drive controller.

The [64, 64] actor is still comfortably STM32F446RE-sized: with 18 inputs and
6 outputs it is ~5.8k float32 parameters (~23 KB flash, ~21k MACs per inference
— well under 1 ms at 180 MHz with the CMSIS FPU).
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class NNDrivePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 64
    max_iterations = 2000
    save_interval = 100
    experiment_name = "nn_drive_two_wheel"

    policy: RslRlPpoActorCriticCfg = RslRlPpoActorCriticCfg(
        init_noise_std=0.3,
        noise_std_type="log",
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[128, 128],
        activation="relu",
    )

    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # 0.004 made the exploration std run away: across the 2026-07-09 five-stage
        # curriculum Policy/mean_noise_std grew 0.17 -> 2.04 (tanh-saturating,
        # effectively bang-bang exploration), and pos_err/pitch/roll flatlined at
        # 0.23 m / 6 deg / 6 deg from stage 2 on. With ~1.3 reward/step the entropy
        # bonus outweighed the shaping gradient on log_std. Keep this low enough
        # that precision pays; watch Policy/mean_noise_std stays under ~0.4.
        entropy_coef=0.0005,
        num_learning_epochs=4,
        num_mini_batches=4,
        learning_rate=5.0e-4,
        schedule="adaptive",
        # 0.995 = 3 s effective horizon at 66.7 Hz, too short to value slow
        # station-keeping drift. 0.998 = 7.5 s, matching the timescale on which
        # the hardware robot walks away from its start point.
        gamma=0.998,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=0.5,
        normalize_advantage_per_mini_batch=True,
    )
