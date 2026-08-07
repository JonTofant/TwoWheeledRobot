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
from isaaclab.sensors import ContactSensorCfg
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
    observation_space: int = 20
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

    # ── Falling ──────────────────────────────────────────────────────────────
    # "tilt"    → inherited rule: |pitch| or total tilt past fall_*_threshold_deg
    #             held for fall_consecutive_steps.
    # "contact" → any body other than the two wheels touching the ground.
    #
    # NNDrive uses "contact" (2026-08-07). The tilt rule was not measuring falling:
    # with the COM range fixed, relaxing it from 25 deg to 80 deg took stage-1
    # termination from 14-20% to exactly 0% across all three scenarios with full
    # 15 s survival — i.e. every "fall" was a recovery swing overshooting 25 deg,
    # not the robot going down. A leaning robot is not a fallen robot, and 0 deg
    # pitch is not even a well-defined target when the IMU carries a mounting
    # bias, so attitude is no longer a termination criterion at all. It is shaped
    # by reward only (see rew_pitch_band / rew_roll_band).
    #
    # The sensor deliberately covers ALL bodies, not just the non-wheel ones: the
    # wheel channels are the runtime proof that contact reporting is actually on
    # (robot_cfg.py activate_contact_sensors). Non-wheel bodies are selected in
    # code via find_bodies, not by excluding wheels in this regex.
    fall_mode: str = "contact"
    contact_force_threshold_n: float = 1.0
    ground_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path=(
            "/World/envs/env_.*/Robot/SimplifiedBipedMainAssembly/SimplifiedBipedMainAssembly/.*"
        ),
        update_period=0.0,
        history_length=1,
        debug_vis=False,
    )

    # ── Policy interface ─────────────────────────────────────────────────────
    # Divisors for the 20 observation values; see DriveObservationBuilder.
    drive_observation_scale: tuple = (
        0.5,  # pos_err (m), clamped to +-cmd_pos_err_clamp_m
        1.0,  # velocity (m/s)
        math.radians(25.0),  # pitch (rad)
        4.0,  # pitch_rate (rad/s)
        1.5,  # yaw_err (rad)
        4.0,  # yaw_rate (rad/s)
        1.0,  # velocity_cmd (m/s)
        2.0,  # yaw_rate_cmd (rad/s)
        1.0,
        1.0,
        1.0,
        1.0,  # cg joint angle / 90 deg (already within [-1, 1])
        2.0,
        2.0,  # previous wheel current (A)
        1.0,
        1.0,
        1.0,
        1.0,  # previous cg tanh action (already [-1, 1])
        math.radians(25.0),  # roll (rad) — same scale as pitch
        4.0,  # roll_rate (rad/s) — same scale as pitch_rate
    )

    # ── Commands (joystick contract) ─────────────────────────────────────────
    # Per-stage max |velocity| (m/s) and |yaw rate| (rad/s), indexed by
    # curriculum_stage 1..5. DDSM115 rated speed ≈ 0.61 m/s at the wheel.
    # Stage 1 must NOT be zero. With v_max=0.0 the policy spent its entire first
    # stage learning that the correct answer is "never move", and once the
    # exploration std settled (~0.08) no later stage could escape that attractor:
    # a velocity sweep showed 0.004 m/s achieved against a 0.55 m/s command, with
    # current at 0.24 A of a 2.0 A budget and zero torque-derate clipping — the
    # robot was not failing to drive, it was not trying. Standing still under a
    # 0.40 m/s command already forfeits ~1.65 reward/step, so this is an
    # exploration failure, not a shaping one; the cure is to never create the
    # standstill-only regime. cmd_still_episode_prob still gives station-keeping
    # practice, mixed in rather than as a whole stage.
    cmd_stage_velocity_max_mps: tuple = (0.15, 0.30, 0.40, 0.50, 0.55)
    # Measured yaw-rate ceiling is ~0.75 rad/s (scripts/diagnose_turn_failure.py):
    # commanding 0.8/1.0/1.2/1.6/2.0 achieves 0.74/0.75/0.69/0.58/0.47, and it is
    # not motor-limited (0.86 A of a 2.0 A budget, torque derate clipping <1% of
    # steps). The old stage-5 max of 2.0 was 2.7x what the robot can do, so most
    # late-stage yaw commands were unachievable by construction and guaranteed
    # reference windup. Raise these again once a policy demonstrably turns faster.
    cmd_stage_yaw_rate_max_radps: tuple = (0.0, 0.4, 0.6, 0.8, 1.0)
    cmd_resample_s_range: tuple = (2.5, 5.0)
    cmd_still_episode_prob: float = 0.35  # whole episode zero-command (station keeping, incl. on slopes)
    cmd_zero_axis_prob: float = 0.30  # per resample, chance each axis is zeroed
    cmd_velocity_slew_mps2: float = 1.0
    cmd_yaw_slew_radps2: float = 4.0
    cmd_settle_s: float = 1.0  # zero commands right after reset
    # Reference anti-windup limits — the reference is back-calculated so it can
    # never run further than these ahead of the robot. Both must match firmware.
    cmd_pos_err_clamp_m: float = 0.5  # anti-windup for odometry drift
    cmd_yaw_err_clamp_rad: float = 1.0  # keeps yaw_err far from the +-pi wrap
    # Benchmark hooks: forced_command_mode="fixed" pins commands for evaluation.
    forced_command_mode: str = ""
    forced_velocity_cmd_mps: float = 0.0
    forced_yaw_rate_cmd_radps: float = 0.0

    # ── CyberGear stance action ──────────────────────────────────────────────
    # ``policy`` is the deployable 6-action interface. ``fixed`` is reserved
    # for the focused action-interface A/B task registered alongside it: the
    # policy then emits only the two wheel actions while all four CyberGears are
    # held at fixed_leg_stance_rad. Everything else in the task stays shared.
    leg_action_mode: str = "policy"
    fixed_leg_stance_rad: tuple = (0.0, 0.0, 0.0, 0.0)
    # Each tanh action is mapped piecewise onto its confirmed joint limits while
    # preserving the hardware failsafe: action 0 -> joint angle 0. Thus -1 maps
    # to that joint's lower limit and +1 to its upper limit, with no unreachable
    # request subsequently flattened by a clamp. Physical joint position remains
    # normalized by 90 deg in the observation for firmware compatibility.
    cg_position_scale_rad: float = math.pi / 2
    cg_target_slew_radps: float = 3.0  # firmware-side target slew limit
    cg_calib_bias_rad_range: tuple = (math.radians(-1.0), math.radians(1.0))
    # CyberGear kp/kd are *commanded* over the bus in MIT mode (they match the
    # kp/kd in cybergear.c, sim nominal CYBERGEAR_STIFFNESS/CYBERGEAR_DAMPING =
    # 30 / 3), so unlike the DDSM115 current gain these are set values, not
    # manufacturing draws. Only the tracking of the commanded gain varies between
    # units, so randomize tightly around nominal (+-5%) rather than the previous
    # +-30%, which trained over leg stiffnesses that are never actually commanded.
    cg_kp_range: tuple = (28.5, 31.5)  # Nm/rad, +-5% around sim nominal 30
    cg_kd_range: tuple = (2.85, 3.15)  # Nm*s/rad, +-5% around sim nominal 3
    noise_cg_pos_std: float = 0.005  # rad, CyberGear encoder noise (obs)

    # ── Reward ───────────────────────────────────────────────────────────────
    # Every weight below can be zeroed from the CLI (e.g. env.rew_hold_velocity=0.0)
    # to bisect which term is responsible for a behaviour change.
    #
    # STRUCTURAL INVARIANT (2026-08-07): per-step reward is >= 0 in every reachable
    # state. Every goal term is a bounded NON-NEGATIVE bonus; only the actuation
    # costs are penalties, and their worst case sums to 0.706 -- below rew_alive.
    # With all rewards >= 0 and gamma < 1, a longer episode weakly dominates a
    # shorter one, so ending the episode early can never pay. That makes "diving
    # for the floor is never optimal" a property of the reward's shape rather than
    # of a clamp calibration, which is what previously failed: see
    # docs/experiments/2026-07-28 (unbounded yaw penalty -> 98% turn-in-place
    # falls). The old "keep every penalty below the ~2.3/step forfeit" rule and
    # the clamps justifying themselves against it are gone with the penalties.
    #
    # PRIORITY ORDER, expressed as weight mass: falling (a hard termination, not a
    # reward term) >> drift/position > velocity tracking > attitude. This inverts
    # the previous ordering, where rew_roll=12 and rew_pitch=6 dominated
    # everything and attitude outweighed every tracking term individually.
    #
    # Two shapes are used, both defined in pure_nn_components.py:
    #   tent_bonus(e, e_max, w)  = w * (1 - |e|/e_max), floored at 0.
    #       Steepest at e = 0, which is what closes out the last few cm/s -- the
    #       job the old quadratic partners did, now as a bonus.
    #   flat_top_bonus(x, flat, sigma, w) = w * exp(-max(0,|x|-flat)^2 / sigma^2).
    #       Exactly w inside +-flat with ZERO gradient there, decaying outside.
    rew_alive: float = 1.0

    # Drift / position — the largest bonus mass after alive.
    # Tent spans the observation clamp plus the far range (0.5 + 1.5 = 2.0 m), so
    # the old separate rew_position_far linear term is subsumed: the tent already
    # reaches zero at 2.0 m and pulls home the whole way.
    rew_position: float = 3.0
    pos_err_far_max_m: float = 1.5
    # Station keeping: only active while the joystick is centred, so it cannot
    # fight command tracking.
    rew_hold_velocity: float = 2.0
    rew_hold_position: float = 2.0
    hold_velocity_tent_max_mps: float = 0.5
    hold_position_tent_max_m: float = 0.5
    hold_velocity_cmd_threshold_mps: float = 0.03
    hold_yaw_rate_cmd_threshold_radps: float = 0.05

    # Command tracking. The exp kernels are kept as-is; the tents replace the
    # former quadratic penalties and serve the same "steep at zero" purpose.
    # Raised 0.8/1.0 -> 1.5/2.0 on 2026-08-07. At the original weights tracking
    # carried 1.8 of weight mass against 7.0 for position+hold, and the first
    # contact-termination run tracked +0.006 m/s against a +0.10 m/s command
    # (rms_vel_err 0.115 against a 0.12 limit) while drift improved to 0.14 m.
    # Position alone cannot substitute: reference anti-windup bounds pos_err by
    # construction, so the position bonus stays near maximum whether or not the
    # robot is actually moving, and it exerts little pull on velocity.
    rew_vel_track: float = 1.5
    vel_track_sigma: float = 0.25  # m/s
    rew_yaw_rate_track: float = 0.5
    yaw_rate_track_sigma: float = 0.6  # rad/s
    rew_vel_err: float = 2.0
    vel_err_tent_max_mps: float = 1.0
    rew_yaw_rate_err: float = 0.15
    yaw_rate_err_tent_max_radps: float = 3.0
    rew_yaw_error: float = 0.5
    yaw_error_tent_max_rad: float = 1.0

    # Torso attitude — now the SMALLEST bonus mass, and shaped as a band.
    #
    # The reward reads TRUE pitch while the policy only ever observes
    # pitch + _pitch_bias (+-3 deg per episode, ~+-1.5 deg on real hardware). So
    # "hold theta = 0" asks the policy to zero a quantity it cannot measure, and
    # on hardware you can never do better than your IMU calibration. A flat top
    # wider than the bias makes that honest: inside +-3 deg there is no gradient
    # at all, so the policy is never pushed to resolve an angle its sensor cannot
    # resolve, and the ~+-1.8 deg trim lean induced by com_offset_y_range_m sits
    # inside the band for free instead of being fought.
    #
    # Sigma is sized so the bonus is nearly gone by ~15 deg, past which recovery
    # is unlikely anyway: at 15 deg the pitch bonus is exp(-4) = 1.8% of weight.
    # Between 3 and 15 deg the gradient still pulls upright -- necessary, because
    # attitude is no longer a termination criterion and nothing else keeps the
    # robot vertical.
    rew_pitch: float = 0.5
    pitch_flat_deg: float = 3.0
    pitch_sigma_deg: float = 6.0
    # Roll gets a much narrower flat top: its mounting bias is only +-1 deg and,
    # unlike pitch, there is no legitimate reason to lean sideways (no trim, no
    # acceleration term). The robot is symmetric to 0.003 deg measured airborne.
    #
    # Raised 0.5 -> 1.5 with the shoulder tightened 4.0 -> 2.5 deg on 2026-08-07.
    # At 0.5/4.0 roll degraded an order of magnitude versus the old quadratic
    # design (0.2-0.36 deg -> 2.5-3.1 deg at benchmark) and drifted upward across
    # training (0.99 -> 2.18 deg), i.e. the band was wide enough and cheap enough
    # that parking at an arbitrary lean was nearly free — the same failure the
    # old rew_roll_abs linear term was added to fix. Roll is weighted above pitch
    # on purpose: pitch has a legitimate nonzero equilibrium, roll does not.
    rew_roll: float = 1.5
    roll_flat_deg: float = 1.0
    roll_sigma_deg: float = 2.5
    # Rate damping, as tents so they stay non-negative. 2 rad/s (115 deg/s) is far
    # faster than a carrying surface should move, so the bonus is zero beyond it.
    rew_pitch_rate: float = 0.25
    rew_roll_rate: float = 0.25
    attitude_rate_tent_max_radps: float = 2.0

    # ── Actuation costs: the only remaining penalties ────────────────────────
    # Worst case must stay below rew_alive or the >= 0 invariant breaks.
    # current 0.08 + delta_current 0.10 + cg_pos 0.494 + cg_rate 0.032 = 0.706.
    # NEVER cap a penalty's output here. A capped penalty saturates and its
    # gradient dies exactly where pressure is most needed, which is invisible in
    # the logs (the term just reads as a constant). Both caps tried on 2026-08-07
    # failed this way within one run: cg_rate pinned at -0.0977/-0.10 for a whole
    # run and stopped discouraging flapping; cg_pos then pinned at -0.1966/-0.20,
    # the legs parked at cg_action_abs 0.67, and the resulting COM shift put the
    # robot into a 15-24 deg lean and a -0.16 m/s drift. Bound the INPUT to what
    # is physically meaningful, or lower the weight until the uncapped worst case
    # fits the budget. Both are done below.
    rew_current: float = 0.01  # worst case 0.01 * 2 * i_max^2 = 0.08
    rew_delta_current: float = 0.05  # actuation smoothness — matters on hardware
    # Bounded on the input: the current-loop lag means a step beyond ~1 A per
    # control step cannot be realized anyway. Worst case 0.05 * 2 * 1.0^2 = 0.10.
    delta_current_clamp_a: float = 1.0
    # Centring the legs is now cheap: hip fore/aft is the actuator that levels
    # the platform and shifts the contact point under the COM without driving.
    # Units are per rad^2 of the mapped physical target, not per tanh unit. The
    # weights retain the physical cost calibrated from the earlier 0.45 rad
    # controller: 0.02 / 0.45^2 and 0.8 / 0.45^2 respectively.
    #
    # cg_pos is UNCAPPED and its weight lowered 0.0988 -> 0.05 instead, so the
    # restoring gradient stays live at every deflection. At 0.0988 with a 0.20
    # cap the term saturated once the legs passed ~0.71 rad RMS and stopped
    # centring them at all — the direct cause of the 2026-08-07 v2 regression.
    # Uncapped worst case is now 0.05 * 4 * (pi/2)^2 = 0.494, which fits the
    # budget: 0.08 + 0.10 + 0.494 + 0.032 = 0.706 < rew_alive.
    rew_cg_pos: float = 0.05
    rew_cg_rate: float = 3.951  # 0.8 / 0.45^2, discourages flapping
    # cg_rate was UNBOUNDED before 2026-08-07 and the bound is not obvious: it is
    # computed on target_angle, which is NOT slew-limited (cg_target_slew_radps
    # limits applied_target, one stage later). So a single-step action reversal
    # can move all four targets across their full range at once — sum(dtheta^2)
    # up to ~39 rad^2, i.e. a -156/step penalty, against the ~2.3/step a fall
    # forfeited under the old economics. A trained policy never jumps that far,
    # so the logged value sat at -0.006 and it never surfaced; random-action
    # probing hits -32/step immediately (scripts/verify_contact_and_reward.py).
    # Same class as the 2026-07-28 wrapped-yaw penalty, found by the >= 0 check.
    #
    # Bounded by clamping the DELTA, not the resulting penalty. Capping the
    # penalty at 0.10 was tried first and saturated at -0.0977 from iteration 0
    # for a whole run: a saturated penalty has zero gradient, so the term stopped
    # discouraging flapping entirely and only subtracted a constant. Clamping the
    # delta instead keeps full gradient across the range that can physically be
    # applied and flattens only beyond it. 3.0 rad/s * 0.015 s = 0.045 rad is the
    # most cg_target_slew_radps can move applied_target in one control step, so
    # requesting more than that is already a no-op at the actuator.
    # Worst case: 4 * 0.045^2 * 3.951 = 0.032.
    cg_rate_delta_clamp_rad: float = 0.045

    # Upper clamp on the summed reward. MUST exceed the analytic maximum or the
    # bonuses are silently truncated whenever the robot is doing well — the old
    # value was 3.0 against a 2.3 maximum, i.e. 0.7 of headroom.
    # Current maximum: alive 1.0 + position 3.0 + hold 2.0 + 2.0 + vel_track 1.5
    # + vel_err 2.0 + yaw_rate_track 0.5 + yaw_rate_err 0.15 + yaw_error 0.5
    # + pitch 0.5 + roll 1.5 + pitch_rate 0.25 + roll_rate 0.25 = 15.15.
    # verify_contact_and_reward.py asserts the observed maximum stays below this.
    reward_total_max: float = 20.0

    # ── Reset state ──────────────────────────────────────────────────────────
    reset_pitch_range_deg: float = 12.0
    # Per-stage multiplier on the above: 9.0 / 10.0 / 11.0 / 12.0 / 12.0 deg.
    # Stage 1 sits inside the ~9 deg recoverable envelope measured 2026-08-07;
    # see CurriculumSampler.reset_ranges for the dose-response behind it. The
    # hard spawns are not removed, only deferred to the stages that also carry
    # pushes and payloads (disturbances start at stage 3).
    reset_stage_pitch_scale: tuple = (0.75, 0.833, 0.917, 1.0, 1.0)
    reset_pitch_rate_range_radps: float = 0.5
    reset_velocity_range_mps: float = 0.15
    reset_yaw_random: bool = True

    # ── Extra domain randomization (all per-episode unless noted) ────────────
    body_mass_scale_range: tuple = (0.85, 1.15)  # all bodies, inertia scaled alike
    # Persistent trim errors are the direct cause of "drives away": a constant
    # offset between the measured zero-pitch and the true balance point makes a
    # pitch-servoing policy accelerate forever. The only cure is for the policy
    # to learn to re-trim from pos_err/velocity, which it only learns if the
    # offsets in training are big enough to matter. Widened from +-1.5 cm /
    # +-1.2 deg, which real COM tolerance and BNO080 mounting easily exceed.
    #
    # RENAMED FROM com_offset_x_range_m 2026-08-05. It was applied to coms[..., 0]
    # = X, which is the LATERAL axis (confirmed against the USD world frame:
    # Y is fore/aft, X is sideways lean, Z is yaw). The trim error described
    # above is a fore/aft quantity — it moves the pitch balance point — so it
    # belongs on Y. Consequences of the transposition: the "drives away" trim
    # randomization this comment justifies had never actually trained, and a
    # +-3 cm sideways COM offset was instead perturbing roll, on an axis the
    # policy could not even observe before 1e20e5f. Same class of bug as the
    # transposed disturbance-force axes fixed in e518082, missed in that pass.
    # The lateral offset is dropped rather than kept: it was never intended, and
    # +-3 cm off-centre is a large payload asymmetry to demand.
    #
    # NARROWED +-30 mm -> +-5 mm on 2026-08-07 (docs/experiments/2026-08-07). Once
    # the axis was fixed, +-30 mm became the single dominant cause of stage-1
    # falls: separation analysis over 26 randomized parameters put
    # abs(com_offset_y) at Cohen's d = 0.999 / AUC = 0.758 with nothing else above
    # d = 0.29, and fall rate rose monotonically 13% -> 79% across its range.
    # The arithmetic: Platform_Group is 59.7% of the robot's 3.80 kg, so a
    # platform COM shift d moves the whole-robot COM by 0.597*d, and the measured
    # trim sensitivity is ~0.6 deg of permanent lean per mm of whole-robot offset
    # (effective pendulum height ~93 mm, backed out from observed trim). So
    # +-30 mm demanded +-11 deg of permanent lean against a 25 deg fall threshold.
    # +-5 mm -> +-3.0 mm whole-robot -> ~+-1.8 deg, which sits just below the
    # +-3 deg IMU mounting bias -- the right ordering, since that bias is
    # deliberately exaggerated (see pitch_bias_rad_range).
    #
    # This models BUILD TOLERANCE only: cable routing, battery seating, PCB and
    # IMU placement on a 2.27 kg deck. Objects placed on the platform are a
    # different physical event and are modelled by DIST_PAYLOAD (downward force
    # plus payload_torque) at stages 4-5, not by this range.
    # ASSUMED, not IDENTIFIED, for paper section 2.4: this is a reasoned bound
    # from the trim-sensitivity measurement above, not a bench measurement.
    com_offset_y_range_m: tuple = (-0.005, 0.005)  # fore/aft platform COM shift (trim error)
    com_offset_z_range_m: tuple = (-0.010, 0.015)  # vertical COM shift
    odometry_scale_range: tuple = (0.97, 1.03)  # wheel-radius error seen by obs only
    pitch_bias_rad_range: tuple = (math.radians(-3.0), math.radians(3.0))  # IMU mounting error
    payload_pitch_torque_nm_range: tuple = (-0.4, 0.4)  # persistent COM-shift torque
    pitch_rate_bias_radps_range: tuple = (-0.03, 0.03)  # gyro bias
    yaw_rate_bias_radps_range: tuple = (-0.03, 0.03)
    yaw_rate_noise_std: float = 0.02
    # Roll axis of the same BNO080 mount, added with the roll observation
    # 2026-08-04. Same magnitudes as the pitch equivalents but sampled
    # separately — two axes of one mount are independent errors, not a shared
    # one. Roll uses pitch_noise_std / pitch_rate_noise_std for its white noise.
    # NOTE: pitch_bias_rad_range is deliberately +-3 deg, wider than a real
    # mounting error, to force the policy to re-trim from pos_err (see its
    # comment). That argument is fore/aft-specific: there is no "drives away"
    # failure on the roll axis, so roll bias is set to a realistic +-1 deg.
    roll_bias_rad_range: tuple = (math.radians(-1.0), math.radians(1.0))
    roll_rate_bias_radps_range: tuple = (-0.03, 0.03)

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


@configclass
class NNDriveFixedStanceEnvCfg(NNDriveEnvCfg):
    """Diagnostic NNDrive variant with fixed legs and wheel-only actions."""

    action_space: int = 2
    leg_action_mode: str = "fixed"
