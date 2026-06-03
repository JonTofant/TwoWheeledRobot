"""
Standup RL Environment configuration for the Two-Wheeled Leg Robot.

The policy learns to self-right the robot from any fallen orientation (side,
forward, backward, upside down) to the upright balancing position.  It is
designed for deployment on the STM32F446RE microcontroller, so the network
is constrained to [32, 32] hidden layers (~1862 float32 parameters, 7.4 KB).

Observation (18 dims) — all manually normalised to [-1, 1], no running stats:
    [0–2]   projected_gravity_b (3D)    full gravity vector in body frame
    [3–5]   ang_vel_b / 10.0           body angular velocities / 10 rad/s
    [6–9]   CyberGear position         zero-centred extension fraction
    [10–11] DDSM115 wheel velocity      left/right wheel velocity / no-load speed
    [12–17] prev_actions (6D)           leg + wheel actions from last step

Action (6 dims):
    [0–3]   CyberGear absolute angles [fl, fr, bl, br]
    [4]     Left  wheel current command
    [5]     Right wheel current command

Spawn scenarios (sampled each episode reset):
    right side down   15 % — roll ≈ −π/2
    left  side down   15 % — roll ≈ +π/2
    fallen forward    15 % — pitch ≈ +π/2
    fallen backward   15 % — pitch ≈ −π/2
    upside down       15 % — roll ≈ π
    partial fall      25 % — random angle ∈ [30°, 180°]

Training command (from TwoWheeledRobot/ directory):
    python scripts/rsl_rl/train.py \\
        --task Template-Twowheeledrobot-Standup-v0 \\
        --headless --num_envs 4096
"""

import math

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, ImuCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .robot_cfg import TWO_WHEELED_ROBOT_CFG
from .sim_params import (
    CONTROL_DECIMATION,
    DDSM115_KT,
    DDSM115_NO_LOAD_SPEED,
    DDSM115_TAU_RATED,
    PHYSICS_DT,
)


@configclass
class StandupEnvCfg(DirectRLEnvCfg):
    # ── Timing ────────────────────────────────────────────────────────────────
    decimation: int = CONTROL_DECIMATION      # 4 → 20 ms control step (50 Hz)
    episode_length_s: float = 10.0            # 500 steps — generous budget

    # ── RL interface ──────────────────────────────────────────────────────────
    observation_space: int = 18
    action_space: int = 6
    state_space: int = 0

    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=CONTROL_DECIMATION)

    # ── Robot ─────────────────────────────────────────────────────────────────
    robot_cfg = TWO_WHEELED_ROBOT_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # ── Scene ─────────────────────────────────────────────────────────────────
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,
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

    enable_wheel_contacts: bool = False
    wheel_contacts: ContactSensorCfg = ContactSensorCfg(
        prim_path=(
            "/World/envs/env_.*/Robot"
            "/SimplifiedBipedMainAssembly"
            "/SimplifiedBipedMainAssembly"
            "/DDSM115_Simplified.*"
        ),
        update_period=0.0,
        history_length=1,
        debug_vis=False,
    )

    # ── Fallen spawn configuration ────────────────────────────────────────────
    # Geometric spawn height constants — robot CoM rests close to ground.
    # spawn_z is computed per-episode from tilt angle via:
    #   z = SPAWN_UPRIGHT_Z * |cos(tilt)| + SPAWN_LATERAL_H * |sin(tilt)| + SPAWN_CLEARANCE
    # This gives ~0.09 m when upright, ~0.15 m when on side, ~0.09 m when inverted.
    spawn_upright_z:  float = 0.06859   # m — nominal CoM height (wheel-centre height)
    spawn_lateral_h:  float = 0.13      # m — estimated half body width at CoM level
    spawn_clearance:  float = 0.04      # m — safety gap above computed contact height

    # Gaussian noise added to each canonical fallen angle (except partial, which
    # is already fully random).
    spawn_tilt_noise_std: float = 0.175   # rad ≈ ±10°

    # Per-episode scenario probabilities — must sum to 1.0.
    spawn_weight_right_side: float  = 0.15
    spawn_weight_left_side: float   = 0.15
    spawn_weight_forward: float     = 0.15
    spawn_weight_backward: float    = 0.15
    spawn_weight_upside_down: float = 0.15
    spawn_weight_partial: float     = 0.25   # random angle ∈ [30°, 180°]

    # ── Success detection ─────────────────────────────────────────────────────
    # Episode terminates with success when the robot holds the upright cone for
    # `success_steps_required` consecutive control steps.
    # Success cone:
    #   pitch uses the forward/back tilt axis (projected_gravity_b[:, 1])
    #   roll uses the sideways tilt axis     (projected_gravity_b[:, 0])
    # Forward/back self-righting can look visibly upright while still carrying
    # more residual tilt than the old 8.6° limit, so allow a wider pitch cone.
    # Keep roll a bit stricter so the robot must truly settle laterally.
    success_pitch_threshold: float = math.sin(math.radians(15.0))  # |proj_grav[1]| < 0.259
    success_roll_threshold: float  = math.sin(math.radians(8.0))   # |proj_grav[0]| < 0.139
    success_steps_required: int    = 2      # consecutive steps ≈ 40 ms at 50 Hz

    # ── Action parameterisation ───────────────────────────────────────────────
    # Use rated torque for conservative training. The environment still models
    # the 2.0 Nm short-term peak and the torque-speed envelope.
    wheel_torque_command_limit: float = DDSM115_TAU_RATED
    wheel_current_max: float = wheel_torque_command_limit / DDSM115_KT
    wheel_velocity_norm: float = DDSM115_NO_LOAD_SPEED

    # Standup is a high-impulse maneuver, so use fixed strong CyberGear gains
    # instead of reset-time randomisation.  This makes the legs reliably push
    # into the ground/body and lever the robot upright instead of collapsing
    # under softer sampled gains.
    cg_use_fixed_gains: bool = True
    cg_fixed_kp: float = 60.0        # Nm/rad — intentionally stiffer than walk defaults for self-righting
    cg_fixed_kd: float = 4.0         # Nm·s/rad — high damping to keep the kick controlled

    # ── Reward weights ────────────────────────────────────────────────────────
    # Primary uprighting: exp(-(proj_grav_z + 1)^2 / sigma).
    # Value = 1.0 when upright (proj_grav_z = -1), ≈ 0 when fallen on side or back.
    rew_uprighting_scale: float = 1.0
    rew_uprighting_sigma: float = 0.3

    # Sparse success bonus paid once per episode on first sustained upright
    # near-zero leg stance.  Keep this large so the task strongly prefers
    # finishing quickly instead of farming dense upright reward.
    rew_success_bonus: float = 15.0
    rew_fast_success_scale: float = 10.0   # extra bonus scaled by remaining episode fraction at success

    # Once the robot is already upright, encourage the final CyberGear pose to
    # settle near mechanical zero on all four joints.  This matches the desired
    # balanced standing stance better than mere left/right symmetry.
    rew_final_pose_scale: float = 0.35
    final_pose_deg_threshold: float = 12.0   # each CyberGear should be within ±12° of 0 for success

    # XY displacement from spawn position — penalises drifting sideways.
    # Keep small: a random policy drifts 1–2 m, so at -0.5 this term would
    # dominate the uprighting signal (~0.04) by 10–50×.  At -0.02 it is a
    # light shaping penalty that does not swamp learning.
    rew_displacement_scale: float = -0.02

    # Energy and smoothness regularisers — kept small so they shape but don't dominate.
    rew_wheel_energy_scale: float = -2.0e-4
    rew_cg_energy_scale: float    = -2.0e-5
    rew_action_rate_scale: float  = -0.002

    # Per-step time penalty to encourage fast recovery.
    rew_timeout_penalty: float = -0.03

    # ── Domain randomisation ──────────────────────────────────────────────────
    cg_kp_scale_range:         tuple = (1.00, 1.00)   # unused when cg_use_fixed_gains=True
    cg_kd_scale_range:         tuple = (1.00, 1.00)   # unused when cg_use_fixed_gains=True
    wheel_damping_scale_range: tuple = (0.50, 1.50)   # ±50 % wheel friction

    # ── Observation noise ─────────────────────────────────────────────────────
    noise_proj_grav_std: float = 0.010   # BNO080 gravity projection noise
    noise_ang_vel_std:   float = 0.020   # rad/s
    noise_cg_pos_std:    float = 0.005   # rad — CyberGear encoder noise
