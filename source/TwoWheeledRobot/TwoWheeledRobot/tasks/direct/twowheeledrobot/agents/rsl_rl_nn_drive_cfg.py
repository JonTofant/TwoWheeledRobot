"""RSL-RL PPO configuration for the joystick NN drive controller.

The [64, 64] actor is still comfortably STM32F446RE-sized: with 20 inputs and
6 outputs it is ~5.8k float32 parameters (~23 KB flash, ~21k MACs per inference
— well under 1 ms at 180 MHz with the CMSIS FPU).
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoActorCriticRecurrentCfg,
    RslRlPpoAlgorithmCfg,
)


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
    # Restore the hardware-proven August exploration floor (2026-10-06).
    # The r2 0.05 floor improved simulated tracking, but C43 regressed on the
    # physical robot. Its causal contribution is unproven: r2 also changed the
    # plant and checkpoint selection. This is training exploration, not an
    # inference current limit. Keep the same floor for all comparison arms.
    action_std_floor: list[float] = [0.15, 0.15]


@configclass
class NNDriveFixedStanceGRUPPORunnerCfg(NNDriveFixedStancePPORunnerCfg):
    """Recurrent (GRU) counterpart of NNDriveFixedStancePPORunnerCfg.

    For the point-vs-range-vs-recurrent-under-range comparison: this must be
    registered against the SAME env_cfg_entry_point (NNDriveFixedStanceEnvCfg)
    as the plain MLP arm, so observation, reward, curriculum and dynamics stay
    byte-identical — the only difference between the two trained policies is
    this runner cfg. See __init__.py's Template-Twowheeledrobot-
    NNDriveFixedStanceGRU-v0 registration.

    actor_hidden_dims/critic_hidden_dims are inherited unchanged from the MLP
    config: ActorCriticRecurrent feeds obs through the RNN first and then the
    SAME [64, 64]/[128, 128] MLP head (see rsl_rl.modules.ActorCriticRecurrent
    .__init__: self.actor = MLP(rnn_hidden_dim, ..., actor_hidden_dims, ...)),
    so this is the GRU added as a memory front-end, not a differently-shaped
    network -- the comparison this is for is about recurrence, not capacity.

    rnn_hidden_dim=64 matches the MLP width for the same reason. Untuned
    starting point, not a claim that it's optimal.
    """

    policy: RslRlPpoActorCriticRecurrentCfg = RslRlPpoActorCriticRecurrentCfg(
        init_noise_std=0.3,
        noise_std_type="log",
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[128, 128],
        activation="relu",
        rnn_type="gru",
        rnn_hidden_dim=64,
        rnn_num_layers=1,
    )


@configclass
class NNDriveFixedStanceWidePPORunnerCfg(NNDriveFixedStancePPORunnerCfg):
    """Capacity-matched feedforward control for the GRU arm.

    Same task, observation and training budget as the [64, 64] MLP; only the
    actor is wider. [145, 145] on 13 inputs is 23,492 actor parameters against
    the GRU actor's 23,618 (13->64 GRU + [64, 64] head), so the
    MLP-vs-GRU comparison can separate recurrence from network size.
    """

    policy: RslRlPpoActorCriticCfg = RslRlPpoActorCriticCfg(
        init_noise_std=0.3,
        noise_std_type="log",
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[145, 145],
        critic_hidden_dims=[128, 128],
        activation="relu",
    )
