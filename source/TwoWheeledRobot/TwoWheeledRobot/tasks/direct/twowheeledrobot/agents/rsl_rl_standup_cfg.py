"""
RSL-RL PPO configuration for the StandupEnv self-righting task.

The policy is designed for deployment on the STM32F446RE (ARM Cortex-M4,
FPU, 128 KB SRAM, 512 KB Flash).  The [32, 32] hidden-layer constraint is
hard:

    Network parameters:
        18×32  +  32×32  +  32×6  = 576 + 1024 + 192 = 1792 weights
        + 32 + 32 + 6 = 70 biases
        Total: 1862 float32 parameters → 7.4 KB Flash  ✓

    SRAM for inference:
        Activations: max(18, 32, 32, 6) = 32 floats per layer → < 1 KB  ✓

Policy constraints:
    - Small [32, 32] actor/critic network due to STM32 constraints.
    - Higher init_noise_std (0.5): standup requires exploring diverse recovery
      strategies across 6 distinct fallen orientations.
    - Lower learning_rate (5e-4): small networks oscillate with high LR.
    - More iterations (5000): [32,32] has less capacity — needs more steps.
    - entropy_coef=0.001: keeps exploration without letting log_std diverge.
    - No obs normalizer: manual normalisation in standup_env.py matches
      the fixed-point firmware implementation on STM32.

Training command (from TwoWheeledRobot/ directory):
    python scripts/rsl_rl/train.py \\
        --task Template-Twowheeledrobot-Standup-v0 \\
        --headless --num_envs 4096
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class StandupPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    # ── Runner ────────────────────────────────────────────────────────────────
    num_steps_per_env = 48        # ≈10 % of 500-step episode; good balance of
                                  # advantage accuracy vs transition diversity
    max_iterations    = 5000      # small net + hard multi-scenario task
    save_interval     = 200
    experiment_name   = "standup_two_wheel"

    # ── Policy network ────────────────────────────────────────────────────────
    # [32, 32] is a hard constraint — see module docstring for Flash/SRAM budget.
    # actor_obs_normalization=False: observations are manually normalised in
    # standup_env.py so the same fixed formula can be replicated on STM32.
    policy: RslRlPpoActorCriticCfg = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        noise_std_type="log",
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[32, 32],
        critic_hidden_dims=[32, 32],
        activation="elu",
    )

    # ── PPO algorithm ─────────────────────────────────────────────────────────
    algorithm: RslRlPpoAlgorithmCfg = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # 0.001 (not 0.01) — entropy_coef=0.01 caused log_std to diverge to
        # infinity.  With very negative early rewards (policy not yet learning),
        # policy gradient ≈ 0, so the entropy gradient dominates and pushes
        # log_std up each update without bound → NaN std → crash.
        # At 0.001 the entropy bonus is 10× weaker and cannot overpower the
        # uprighting signal once the robot starts learning.
        entropy_coef=0.001,
        num_learning_epochs=4,
        num_mini_batches=4,   # 4096 × 48 / 4 = 49 152 samples per mini-batch
        learning_rate=5.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=0.5,
        normalize_advantage_per_mini_batch=True,
    )
