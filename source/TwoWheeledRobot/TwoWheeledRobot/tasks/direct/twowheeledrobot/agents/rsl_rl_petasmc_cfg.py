"""
RSL-RL PPO configuration for the PETASMC gain-scheduling policy.

Network is kept deliberately small ([32, 32]) so that the trained weights
can be exported and run on the STM32F446RE microcontroller:

    10 inputs → 32 → 32 → 4 outputs
    Total parameters: ~1 540 float32  ≈  6 KB

Export after training:
    The actor weights are saved in logs/rsl_rl/petasmc_gain_scheduler/.
    Copy the mean_net weights to a C header and run a 3-layer matmul + ELU
    at 50 Hz alongside the PETASMC firmware loop.
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class PetasmcPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env  = 24       # rollout steps per environment before update
    max_iterations     = 2000
    save_interval      = 200
    experiment_name    = "petasmc_gain_scheduler"

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,              # moderate initial exploration in gain space
        actor_obs_normalization=False,   # inputs are already bounded — skip running stats
        critic_obs_normalization=False,
        actor_hidden_dims=[32, 32],      # STM32-deployable size
        critic_hidden_dims=[64, 64],     # critic can be larger — runs on GPU only
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,        # encourage gain-space exploration
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
