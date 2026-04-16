"""
Configuration for the PETASMC RL gain-scheduling environment.

The RL policy outputs four gains each control step:
    action[0] → T_min_bal  (periodic floor, balance trigger)
    action[1] → T_min_yaw  (periodic floor, heading trigger)
    action[2] → K_max_bal  (adaptive gain ceiling, balance)
    action[3] → K_max_yaw  (adaptive gain ceiling, heading)

Actions are normalised to [-1, 1] and linearly mapped to the ranges defined
by the *_lo / *_hi fields below.
"""

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import TWO_WHEELED_ROBOT_CFG
from .sim_params import PHYSICS_DT, CONTROL_DECIMATION


@configclass
class PetasmcRlEnvCfg(DirectRLEnvCfg):
    # ── Timing ────────────────────────────────────────────────────────────────
    # Same physics / control rate as the manual env: 200 Hz physics, 50 Hz control.
    decimation: int       = CONTROL_DECIMATION   # 4  →  20 ms control step
    episode_length_s: float = 30.0

    # ── RL interface ──────────────────────────────────────────────────────────
    # obs: [θ, θ̇, ψ̇, v, v_cmd, s_bal, s_yaw, iet_bal_norm, iet_yaw_norm, T_min_bal_norm]
    observation_space: int = 10
    # act: [T_min_bal, T_min_yaw, K_max_bal, K_max_yaw]  ∈ [-1, 1]
    action_space: int = 4
    state_space: int = 0

    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=CONTROL_DECIMATION)

    # ── Robot ─────────────────────────────────────────────────────────────────
    robot_cfg = TWO_WHEELED_ROBOT_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # ── Scene ─────────────────────────────────────────────────────────────────
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=2048,
        env_spacing=2.5,
        replicate_physics=True,
    )

    # ── IMU sensor ────────────────────────────────────────────────────────────
    bno080: ImuCfg = ImuCfg(
        prim_path=(
            "/World/envs/env_.*/Robot"
            "/SimplifiedBipedMainAssembly"
            "/SimplifiedBipedMainAssembly"
            "/Platform_Group/BNO080"
        ),
        visualizer_cfg=None,
    )

    # ── Termination ───────────────────────────────────────────────────────────
    max_pitch_deg: float = 44.0   # episode ends if tilt exceeds this

    # ── Velocity commands ─────────────────────────────────────────────────────
    # Sampled uniformly at each episode reset.
    vel_cmd_max:      float = 0.3    # m/s
    yaw_rate_cmd_max: float = 0.0    # rad/s  (keep 0 — balance-only curriculum first)

    # ── Domain randomisation ──────────────────────────────────────────────────
    mass_scale_min: float = 0.85     # ±15 % total mass variation
    mass_scale_max: float = 1.15

    push_interval_s: float = 4.0    # average seconds between random velocity impulses
    push_vel_min:    float = 0.3    # m/s  — impulse magnitude range
    push_vel_max:    float = 1.2    # m/s

    init_tilt_std: float = 0.04     # rad  — random initial tilt (≈ 2.3°)

    # ── Observation noise (IMU-level std devs) ────────────────────────────────
    noise_theta:     float = 0.005   # rad
    noise_theta_dot: float = 0.020   # rad/s
    noise_psi_dot:   float = 0.020   # rad/s
    noise_vel:       float = 0.020   # m/s

    # ── RL-controlled gain ranges ─────────────────────────────────────────────
    # action[i] ∈ [-1, 1] → linearly mapped to [lo, hi].
    t_min_bal_lo: float = 0.005   # s   (= sensor period, tightest possible floor)
    t_min_bal_hi: float = 0.040   # s   (= 2× nominal 20 ms)
    t_min_yaw_lo: float = 0.005   # s
    t_min_yaw_hi: float = 0.040   # s
    k_max_bal_lo: float = 0.80    # A
    k_max_bal_hi: float = 2.50    # A
    k_max_yaw_lo: float = 0.05    # A
    k_max_yaw_hi: float = 0.30    # A
