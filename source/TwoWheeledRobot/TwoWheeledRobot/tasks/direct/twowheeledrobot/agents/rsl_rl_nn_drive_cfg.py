"""RSL-RL PPO configuration for the joystick NN drive controller.

The [64, 64] actor is still comfortably STM32F446RE-sized: with 20 inputs and
6 outputs it is ~5.8k float32 parameters (~23 KB flash, ~21k MACs per inference
— well under 1 ms at 180 MHz with the CMSIS FPU).
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class NNDrivePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 64
    max_iterations = 2000
    # Selection benchmarks a reward-ranked shortlist of saved checkpoints.
    # A 25-iteration interval prevents a short-lived optimum (the previous run
    # peaked 17 iterations after a save) from disappearing between snapshots.
    save_interval = 25
    experiment_name = "nn_drive_two_wheel"
    # Actions 4/5 drive the wheels. Below 0.15 raw std their exploration is
    # largely swallowed by the randomized motor deadzone.
    # NOTE: 0.15 was chosen against the old 0.03-0.20 A deadzone. That range is
    # now 0.031-0.078 A (measured, EMB-18), i.e. ~2.5x smaller, so less
    # exploration is swallowed and this floor is now conservative rather than
    # tight. Kept as-is because it is a floor and it fixed the exploration
    # collapse; revisit only if mean_noise_std pins to it for a whole stage.
    action_std_floor: list[float] = [0.0, 0.0, 0.0, 0.0, 0.15, 0.15]

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
        entropy_coef=0.002,
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


@configclass
class NNDriveFixedStancePPORunnerCfg(NNDrivePPORunnerCfg):
    """Matched PPO config for the two-action fixed-stance diagnostic task."""

    experiment_name = "nn_drive_fixed_stance"
    action_std_floor: list[float] = [0.15, 0.15]
