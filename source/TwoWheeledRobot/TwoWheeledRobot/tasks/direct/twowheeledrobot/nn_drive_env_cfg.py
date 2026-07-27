"""NN drive task configuration: joystick-commanded balancing/driving with terrain.

Extends the pure NN balance task with:
  - velocity + yaw-rate commands (joystick contract, integrated pos/yaw references),
  - 6-dim actions (4 CyberGear stance targets + 2 DDSM115 wheel currents),
  - generated terrain (flat / small bumps / inclines) for sim2real driving,
  - much wider domain randomization (mass, COM, odometry scale, gyro bias,
    IMU mounting bias, CyberGear gains).

Observation and action layout are documented in DriveObservationBuilder in
pure_nn_components.py and in STM32_DEPLOYMENT.md — keep them aligned with the
STM32 firmware when editing.
"""

import math

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.terrains import TerrainGeneratorCfg, TerrainImporterCfg
from isaaclab.utils import configclass

from .pure_nn_balance_env_cfg import PureNNBalanceEnvCfg
from .sim_params import GROUND_DYNAMIC_FRICTION, GROUND_RESTITUTION, GROUND_STATIC_FRICTION

# Flat / bumps / slopes mix sized for a 10 cm wheel robot: bumps are 1-2.5 cm
# noise (tile joints, carpet edges, cables) and slopes go up to ~9 deg.
DRIVE_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    size=(8.0, 8.0),
    # Border must exceed the farthest a robot can drive in one episode
    # (0.55 m/s * 20 s = 11 m) so nobody drives off the terrain mesh.
    border_width=12.0,
    num_rows=6,
    num_cols=6,
    horizontal_scale=0.05,
    vertical_scale=0.002,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.35),
        "bumps_small": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.15,
            noise_range=(0.004, 0.010),
            noise_step=0.002,
            downsampled_scale=0.2,
            border_width=0.25,
        ),
        "bumps_large": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.10,
            noise_range=(0.010, 0.022),
            noise_step=0.002,
            downsampled_scale=0.3,
            border_width=0.25,
        ),
        "slope_up": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.20,
            slope_range=(0.05, 0.16),  # ~3-9 deg
            platform_width=1.5,
            border_width=0.25,
        ),
        "slope_down": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.20,
            slope_range=(0.05, 0.16),
            platform_width=1.5,
            border_width=0.25,
        ),
    },
)


@configclass
class NNDriveEnvCfg(PureNNBalanceEnvCfg):
    observation_space: int = 18
    action_space: int = 6
    state_space: int = 0

    # Longer horizon than the 8 s balance task: the ~10 s hardware falls were
    # exactly at the edge of what the old policy had ever experienced.
    episode_length_s: float = 20.0

    # ── Terrain ───────────────────────────────────────────────────────────────
    # "flat"  → plane importer (fast, early curriculum stages)
    # "generator" → DRIVE_TERRAINS_CFG mix (bumps + inclines)
    terrain_mode: str = "generator"
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=DRIVE_TERRAINS_CFG,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=GROUND_STATIC_FRICTION,
            dynamic_friction=GROUND_DYNAMIC_FRICTION,
            restitution=GROUND_RESTITUTION,
        ),
        debug_vis=False,
    )
    spawn_extra_clearance_m: float = 0.01  # extra drop height on bumpy tiles

    # ── Policy interface ─────────────────────────────────────────────────────
    # Divisors for the 18 observation values; see DriveObservationBuilder.
    drive_observation_scale: tuple = (
        0.5,                 # pos_err (m), clamped to +-cmd_pos_err_clamp_m
        1.0,                 # velocity (m/s)
        math.radians(25.0),  # pitch (rad)
        4.0,                 # pitch_rate (rad/s)
        1.5,                 # yaw_err (rad)
        4.0,                 # yaw_rate (rad/s)
        1.0,                 # velocity_cmd (m/s)
        2.0,                 # yaw_rate_cmd (rad/s)
        1.0, 1.0, 1.0, 1.0,  # cg extension fraction (already [-1, 1])
        2.0, 2.0,            # previous wheel current (A)
        1.0, 1.0, 1.0, 1.0,  # previous cg tanh action (already [-1, 1])
    )

    # ── Commands (joystick contract) ─────────────────────────────────────────
    # Per-stage max |velocity| (m/s) and |yaw rate| (rad/s), indexed by
    # curriculum_stage 1..5. DDSM115 rated speed ≈ 0.61 m/s at the wheel.
    cmd_stage_velocity_max_mps: tuple = (0.0, 0.20, 0.35, 0.45, 0.55)
    cmd_stage_yaw_rate_max_radps: tuple = (0.0, 0.6, 1.2, 1.6, 2.0)
    cmd_resample_s_range: tuple = (2.5, 5.0)
    cmd_still_episode_prob: float = 0.25   # whole episode zero-command (station keeping, incl. on slopes)
    cmd_zero_axis_prob: float = 0.30       # per resample, chance each axis is zeroed
    cmd_velocity_slew_mps2: float = 1.0
    cmd_yaw_slew_radps2: float = 4.0
    cmd_settle_s: float = 1.0              # zero commands right after reset
    cmd_pos_err_clamp_m: float = 0.5       # anti-windup for odometry drift, must match firmware
    # Benchmark hooks: forced_command_mode="fixed" pins commands for evaluation.
    forced_command_mode: str = ""
    forced_velocity_cmd_mps: float = 0.0
    forced_yaw_rate_cmd_radps: float = 0.0

    # ── CyberGear stance action ──────────────────────────────────────────────
    cg_action_authority_rad: float = 0.45      # ~26 deg of stance authority around zero
    cg_target_slew_radps: float = 3.0          # firmware-side target slew limit
    cg_calib_bias_rad_range: tuple = (math.radians(-1.0), math.radians(1.0))
    cg_kp_range: tuple = (21.0, 39.0)          # Nm/rad, +-30% around sim nominal 30
    cg_kd_range: tuple = (2.1, 3.9)            # Nm*s/rad, +-30% around sim nominal 3
    noise_cg_pos_std: float = 0.005            # rad, CyberGear encoder noise (obs)

    # ── Reward ───────────────────────────────────────────────────────────────
    rew_alive: float = 1.0
    rew_vel_track: float = 0.8
    vel_track_sigma: float = 0.25              # m/s
    rew_yaw_rate_track: float = 0.5
    yaw_rate_track_sigma: float = 0.6          # rad/s
    rew_position: float = 1.5                  # on clamped pos_err (max 0.375 at the 0.5 m clamp)
    rew_yaw_error: float = 0.5
    rew_pitch: float = 4.0                     # lower than balance task: slopes need lean
    rew_pitch_rate: float = 0.3
    rew_roll: float = 8.0                      # no legitimate reason to lean sideways, unlike pitch
    rew_roll_rate: float = 0.3
    rew_current: float = 0.01
    rew_delta_current: float = 0.05            # actuation smoothness — matters on hardware
    rew_cg_pos: float = 0.05
    rew_cg_rate: float = 0.5

    # ── Reset state ──────────────────────────────────────────────────────────
    reset_pitch_range_deg: float = 12.0
    reset_pitch_rate_range_radps: float = 0.5
    reset_velocity_range_mps: float = 0.15
    reset_yaw_random: bool = True

    # ── Extra domain randomization (all per-episode unless noted) ────────────
    body_mass_scale_range: tuple = (0.85, 1.15)         # all bodies, inertia scaled alike
    com_offset_x_range_m: tuple = (-0.015, 0.015)       # platform COM shift (payload model)
    com_offset_z_range_m: tuple = (-0.010, 0.015)
    odometry_scale_range: tuple = (0.97, 1.03)          # wheel-radius error seen by obs only
    pitch_bias_rad_range: tuple = (math.radians(-1.2), math.radians(1.2))  # IMU mounting error
    pitch_rate_bias_radps_range: tuple = (-0.03, 0.03)  # gyro bias
    yaw_rate_bias_radps_range: tuple = (-0.03, 0.03)
    yaw_rate_noise_std: float = 0.02

    # Continuous low-amplitude force noise (floor texture / debris proxy),
    # first-order-filtered white noise on the platform, in addition to the
    # inherited discrete push/payload/slope disturbance curriculum.
    enable_force_noise: bool = True
    force_noise_amp_n_range: tuple = (0.0, 0.6)
    force_noise_tau_s: float = 0.3

    # Stronger pushes than the balance task — the drive robot must shrug off
    # real human shoves while moving.
    human_push_fx_n_range: tuple = (-8.0, 8.0)
    human_push_fy_n_range: tuple = (-2.5, 2.5)
    human_push_yaw_torque_nm_range: tuple = (-0.25, 0.25)
    # Spread events over the full 20 s episode (the balance task's 1-6 s window
    # was sized for 8 s episodes and would teach "nothing happens after 6 s").
    human_push_start_s_range: tuple = (1.5, 16.0)
    payload_start_s_range: tuple = (1.0, 12.0)
    enable_slope_disturbance: bool = False  # real slopes come from terrain now
