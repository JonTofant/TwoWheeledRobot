"""Configurable components for pure neural-network balance control."""

from __future__ import annotations

import math

import torch


def pitch_from_projected_gravity(projected_gravity_body: torch.Tensor) -> torch.Tensor:
    return torch.atan2(projected_gravity_body[:, 1], -projected_gravity_body[:, 2])


def roll_from_projected_gravity(projected_gravity_body: torch.Tensor) -> torch.Tensor:
    return torch.atan2(projected_gravity_body[:, 0], -projected_gravity_body[:, 2])


def yaw_from_quat_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    w, x, y, z = quat_wxyz.unbind(dim=1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle_rad(value: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(value), torch.cos(value))


def tent_bonus(error: torch.Tensor, max_error: float, weight: float) -> torch.Tensor:
    """Bounded non-negative bonus, maximal and steepest at zero error.

    ``weight`` at ``error == 0``, falling linearly to 0 at ``|error| == max_error``
    and staying there. This is the non-negative replacement for the old ``-w*e^2``
    penalties: it keeps the constant restoring gradient that closes out the last
    few cm/s (a pure exp kernel is flat at zero and cannot), while never going
    negative, which is what makes ending an episode early unable to pay.
    """
    return weight * (1.0 - error.abs() / max_error).clamp(min=0.0)


def flat_top_bonus(value: torch.Tensor, flat: float, sigma: float, weight: float) -> torch.Tensor:
    """Full ``weight`` inside ``+-flat`` with zero gradient there, decaying outside.

    Used for attitude. The flat region is deliberately sized to the sensor's
    mounting-bias range: the reward is computed on true pitch/roll but the policy
    only observes a biased copy, so demanding a specific angle inside the bias
    band would be asking it to resolve something it cannot measure. Outside the
    band the Gaussian shoulder still pulls upright.
    """
    excess = (value.abs() - flat).clamp(min=0.0)
    return weight * torch.exp(-excess.pow(2) / (sigma**2))


def map_cybergear_tanh_to_joint_target(
    tanh_action: torch.Tensor, joint_lo: torch.Tensor, joint_hi: torch.Tensor
) -> torch.Tensor:
    """Map each zero-centred action onto its complete asymmetric joint range.

    For every joint, ``-1 -> joint_lo``, ``0 -> 0`` and ``+1 -> joint_hi``.
    The two halves are scaled independently so the policy never requests an
    angle outside the confirmed mechanical range and no action interval is
    discarded by a subsequent clamp.
    """
    return torch.where(tanh_action >= 0.0, tanh_action * joint_hi, (-tanh_action) * joint_lo)


class NormalizedObservationBuilder:
    """Build the compact 8-value balance observation with fixed normalization."""

    def __init__(self, cfg, device: torch.device):
        self.cfg = cfg
        self.scale = torch.tensor(cfg.observation_scale, device=device, dtype=torch.float32).view(1, -1)
        if len(cfg.observation_scale) != cfg.observation_space:
            raise ValueError("observation_scale length must match observation_space")

    def build(
        self,
        x_rel: torch.Tensor,
        linear_velocity: torch.Tensor,
        pitch: torch.Tensor,
        pitch_rate: torch.Tensor,
        yaw_error: torch.Tensor,
        yaw_rate: torch.Tensor,
        previous_current: torch.Tensor,
    ) -> torch.Tensor:
        values = [x_rel, linear_velocity, pitch, pitch_rate, yaw_error]
        if self.cfg.include_yaw_rate:
            values.append(yaw_rate)
        obs = torch.cat([torch.stack(values, dim=1), previous_current], dim=1)
        return torch.nan_to_num(obs / self.scale, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)


class CurrentActionProcessor:
    """Apply tanh current scaling, per-episode motor randomization, delay, and current-loop lag."""

    def __init__(self, cfg, num_envs: int, device: torch.device):
        self.cfg = cfg
        self.device = device
        self.net_current = torch.zeros(num_envs, 2, device=device)
        self.filtered_current = torch.zeros(num_envs, 2, device=device)
        self.command_current = torch.zeros(num_envs, 2, device=device)
        self.prev_command_current = torch.zeros(num_envs, 2, device=device)
        self.action_delay_current = torch.zeros(num_envs, 2, device=device)
        self.action_delay_samples = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.left_gain = torch.ones(num_envs, device=device)
        self.right_gain = torch.ones(num_envs, device=device)
        self.deadzone = torch.zeros(num_envs, 2, device=device)
        self.breakaway_mult = torch.ones(num_envs, 2, device=device)
        self.bias = torch.zeros(num_envs, 2, device=device)
        self.tau_s = torch.full((num_envs, 2), cfg.motor_tau_s_range[0], device=device)
        self.current_limit = torch.full((num_envs, 2), cfg.i_max_a, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        n = len(env_ids)
        self.net_current[env_ids] = 0.0
        self.filtered_current[env_ids] = 0.0
        self.command_current[env_ids] = 0.0
        self.prev_command_current[env_ids] = 0.0
        self.action_delay_current[env_ids] = 0.0
        self.action_delay_samples[env_ids] = torch.randint(0, 2, (n,), device=self.device)

        gain_lo, gain_hi = self.cfg.motor_gain_range
        dz_lo, dz_hi = self.cfg.motor_deadzone_a_range
        bias_lo, bias_hi = self.cfg.motor_bias_a_range
        tau_lo, tau_hi = self.cfg.motor_tau_s_range
        limit_lo, limit_hi = self.cfg.motor_current_limit_a_range
        self.left_gain[env_ids] = torch.empty(n, device=self.device).uniform_(gain_lo, gain_hi)
        self.right_gain[env_ids] = torch.empty(n, device=self.device).uniform_(gain_lo, gain_hi)
        # One deadzone draw per wheel (columns are left/right, not POS/NEG), applied
        # symmetrically to both directions in process(). Measured POS/NEG deadzone
        # correlates at r = +0.989 across units (EMB-18), so per-direction sampling
        # would train over motors that do not exist. Do not split this into 4 draws.
        self.deadzone[env_ids] = torch.empty(n, 2, device=self.device).uniform_(dz_lo, dz_hi)
        # Per-wheel breakaway draw. Independent of the deadzone draw: the
        # kinetic value is a measured per-unit constant, the breakaway factor is
        # an assumed bound (see cfg), and pretending one predicts the other
        # would invent a correlation nobody measured.
        bk_lo, bk_hi = self.cfg.motor_breakaway_multiplier_range
        self.breakaway_mult[env_ids] = torch.empty(n, 2, device=self.device).uniform_(bk_lo, bk_hi)
        self.bias[env_ids] = torch.empty(n, 2, device=self.device).uniform_(bias_lo, bias_hi)
        self.tau_s[env_ids] = torch.empty(n, 2, device=self.device).uniform_(tau_lo, tau_hi)
        self.current_limit[env_ids] = torch.empty(n, 2, device=self.device).uniform_(limit_lo, limit_hi)

    def process(self, raw_actions: torch.Tensor, dt: float, wheel_omega: torch.Tensor | None = None) -> torch.Tensor:
        """Map policy actions to commanded wheel current.

        ``wheel_omega`` (rad/s, per wheel) enables the Stribeck breakaway term.
        Optional so callers that do not model it keep the previous behaviour;
        with the default multiplier range of (1.0, 1.0) passing it changes
        nothing either, so this is inert until the cfg is raised.
        """
        self.prev_command_current = self.command_current.clone()
        self.net_current = torch.tanh(raw_actions[:, :2]) * self.cfg.i_max_a
        delayed = torch.where(self.action_delay_samples.view(-1, 1) > 0, self.action_delay_current, self.net_current)
        self.action_delay_current = self.net_current.clone()

        alpha = self.cfg.action_smoothing_alpha
        target = alpha * self.filtered_current + (1.0 - alpha) * delayed
        delta = target - self.filtered_current
        if self.cfg.enable_current_slew_limit or self.cfg.hardware_safe_current_slew_limit:
            delta = delta.clamp(-self.cfg.current_slew_limit_a, self.cfg.current_slew_limit_a)
        self.filtered_current = self.filtered_current + delta

        gain = torch.stack([self.left_gain, self.right_gain], dim=1)
        motor_target = self.filtered_current * gain + self.bias
        # Stribeck: the deadzone is largest from rest and decays toward the
        # measured kinetic value as the wheel spins up. exp(-|w|/w_s) rather
        # than a threshold switch -- a hard switch is a discontinuity in the
        # plant exactly where station-keeping operates, which is both physically
        # wrong and a poor thing to ask a policy to learn across.
        deadzone = self.deadzone
        if wheel_omega is not None:
            excess = (self.breakaway_mult - 1.0).clamp(min=0.0)
            stribeck = torch.exp(-wheel_omega.abs() / max(self.cfg.motor_breakaway_speed_radps, 1.0e-6))
            deadzone = self.deadzone * (1.0 + excess * stribeck)
        motor_target = torch.sign(motor_target) * torch.clamp(motor_target.abs() - deadzone, min=0.0)
        motor_target = torch.maximum(-self.current_limit, torch.minimum(motor_target, self.current_limit))

        lag_alpha = (dt / torch.clamp(self.tau_s, min=dt)).clamp(0.0, 1.0)
        self.command_current = self.command_current + lag_alpha * (motor_target - self.command_current)
        return self.command_current

    def delta_current(self) -> torch.Tensor:
        return self.command_current - self.prev_command_current


class CurriculumSampler:
    def __init__(self, cfg, device: torch.device):
        self.cfg = cfg
        self.device = device

    @property
    def stage(self) -> int:
        return int(self.cfg.curriculum_stage)

    def reset_ranges(self) -> tuple[float, float, float]:
        """Spawn attitude/velocity ranges for the current curriculum stage.

        The pitch range ramps with the stage; pitch rate and velocity do not.
        Measured 2026-08-07 on the stage-1 policy: spawn |pitch| is the single
        dominant predictor of an early fall (Cohen's d = 2.31, AUC = 0.956, with
        every other randomized parameter below d = 0.38), and the dose-response
        is a cliff rather than a slope — 0.0% fall rate below 7.6 deg across 640
        environments, 13.3% by 10.6 deg, 53.9% above 10.7 deg. The recoverable
        envelope is ~9 deg, so a flat 12 deg spawned a quarter of stage 1 outside
        what the policy could ever save. Everything else in this task already
        ramps by stage (command velocity, yaw rate, disturbance kind); this did
        not, and arrived at full magnitude on iteration 1 of stage 1.

        Expressed as a SCALE on reset_pitch_range_deg rather than per-stage
        absolutes so configs that deliberately set a small spawn keep it — the
        demo scene runs at curriculum_stage 5 with reset_pitch_range_deg = 2.0
        and must stay there.
        """
        scales = getattr(self.cfg, "reset_stage_pitch_scale", None)
        pitch_deg = self.cfg.reset_pitch_range_deg
        if scales:
            pitch_deg = pitch_deg * scales[max(0, min(self.stage, len(scales)) - 1)]
        return (
            math.radians(pitch_deg),
            self.cfg.reset_pitch_rate_range_radps,
            self.cfg.reset_velocity_range_mps,
        )


DIST_NONE = 0
DIST_HUMAN_PUSH = 1
DIST_DOUBLE_HUMAN_PUSH = 2
DIST_PAYLOAD = 3
DIST_PAYLOAD_PUSH = 4
DIST_SLOPE = 5
DIST_SINE_DIAGNOSTIC = 6

# Platform body-frame axis indices for external forces.
#
# Verified in sim (2026-07-28) and against the USD world frame (2026-08-05):
# driving moves the robot along the Y axis, and the platform body quaternion is
# identity, so body axes are world axes. Y is therefore the fore/aft axis (the
# one the robot can answer by driving) and X is lateral (which a differential
# drive cannot correct — it can only resist it through wheel friction). Z is
# the yaw axis.
#
# The 2026-07-28 note here claimed forward was +Y; per the USD it is -Y. That
# sign is immaterial to results — the robot is close to symmetric, every
# disturbance range below is symmetric about zero, and x_rel/velocity come from
# wheel odometry rather than world position, so it only flips which way the
# robot drives. Only the axis ASSIGNMENT matters, and that is correct.
#
# These were previously transposed: the +-8 N "human push" went to index 0 and
# so was applied sideways, while the fore/aft push the robot can actually
# recover from only ever got the +-2.5 N "lateral" magnitude.
AXIS_LATERAL = 0
AXIS_FOREAFT = 1
AXIS_VERTICAL = 2
# Torque axes: pitch is rotation about the wheel axle (lateral), yaw about up.
AXIS_TORQUE_PITCH = AXIS_LATERAL
AXIS_TORQUE_YAW = AXIS_VERTICAL


class DisturbanceGenerator:
    """Sample physically motivated sim2real disturbances.

    Disturbances are represented as force and torque events on the selected
    robot body/platform. Components are supplied directly to Isaac Lab's
    ``set_external_force_and_torque`` tensor API without frame conversion — the
    API applies them in the body-local frame, which for the platform is the
    world frame (identity quaternion at spawn).

    Axes are named via the AXIS_* constants above rather than written as bare
    indices, because the fore/aft and lateral force components were transposed
    here for the whole history of the task. See those constants for the sim
    measurement that pins the convention down.
    """

    def __init__(self, cfg, num_envs: int, device: torch.device):
        self.cfg = cfg
        self.device = device
        self.kind = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.push_force_1 = torch.zeros(num_envs, 3, device=device)
        self.push_torque_1 = torch.zeros(num_envs, 3, device=device)
        self.push_force_2 = torch.zeros(num_envs, 3, device=device)
        self.push_torque_2 = torch.zeros(num_envs, 3, device=device)
        self.push_start_1 = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.push_stop_1 = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.push_start_2 = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.push_stop_2 = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.payload_torque = torch.zeros(num_envs, 3, device=device)
        self.payload_start = torch.zeros(num_envs, device=device, dtype=torch.long)
        self.slope_force = torch.zeros(num_envs, 3, device=device)
        self.sine_amp = torch.zeros(num_envs, device=device)
        self.sine_freq = torch.zeros(num_envs, device=device)

    def reset(self, env_ids: torch.Tensor, stage: int, dt: float) -> None:
        n = len(env_ids)
        u = torch.rand(n, device=self.device)
        self._clear(env_ids)
        if stage >= 5:
            # 30% none, 30% human push, 25% payload/COM shift, 15% payload+push.
            self.kind[env_ids] = torch.where(
                u < 0.30,
                DIST_NONE,
                torch.where(u < 0.60, DIST_HUMAN_PUSH, torch.where(u < 0.85, DIST_PAYLOAD, DIST_PAYLOAD_PUSH)),
            )
            if self.cfg.enable_slope_disturbance and self.cfg.stage5_slope_probability > 0.0:
                slope_mask = torch.rand(n, device=self.device) < self.cfg.stage5_slope_probability
                self.kind[env_ids[slope_mask]] = DIST_SLOPE
        elif stage >= 4:
            # 40% none, 40% payload/COM shift, 20% payload+human push.
            self.kind[env_ids] = torch.where(
                u < 0.40, DIST_NONE, torch.where(u < 0.80, DIST_PAYLOAD, DIST_PAYLOAD_PUSH)
            )
        elif stage >= 3:
            # 50% none, 40% single human push, 10% double human push.
            self.kind[env_ids] = torch.where(
                u < 0.50, DIST_NONE, torch.where(u < 0.90, DIST_HUMAN_PUSH, DIST_DOUBLE_HUMAN_PUSH)
            )

        push_env_ids = env_ids[(self.kind[env_ids] == DIST_HUMAN_PUSH) | (self.kind[env_ids] == DIST_PAYLOAD_PUSH)]
        double_push_env_ids = env_ids[self.kind[env_ids] == DIST_DOUBLE_HUMAN_PUSH]
        payload_env_ids = env_ids[(self.kind[env_ids] == DIST_PAYLOAD) | (self.kind[env_ids] == DIST_PAYLOAD_PUSH)]
        slope_env_ids = env_ids[self.kind[env_ids] == DIST_SLOPE]

        if len(push_env_ids) > 0:
            self._sample_push(push_env_ids, first=True, dt=dt)
        if len(double_push_env_ids) > 0:
            self._sample_push(double_push_env_ids, first=True, dt=dt)
            self._sample_push(double_push_env_ids, first=False, dt=dt)
        if len(payload_env_ids) > 0:
            self._sample_payload(payload_env_ids, dt=dt)
        if len(slope_env_ids) > 0:
            # A slope biases the robot fore/aft, never sideways.
            self.slope_force[slope_env_ids, AXIS_FOREAFT] = torch.empty(
                len(slope_env_ids), device=self.device
            ).uniform_(*self.cfg.slope_fx_n_range)
        self.sine_amp[env_ids] = torch.empty(n, device=self.device).uniform_(*self.cfg.sine_force_n_range)
        self.sine_freq[env_ids] = torch.empty(n, device=self.device).uniform_(*self.cfg.sine_frequency_hz_range)

    def force_and_torque(self, step: torch.Tensor, t: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        force = self.slope_force.clone()
        torque = torch.zeros_like(force)

        push_1 = (step >= self.push_start_1) & (step < self.push_stop_1)
        push_2 = (step >= self.push_start_2) & (step < self.push_stop_2)
        payload = ((self.kind == DIST_PAYLOAD) | (self.kind == DIST_PAYLOAD_PUSH)) & (step >= self.payload_start)
        sine = self.kind == DIST_SINE_DIAGNOSTIC

        force = force + torch.where(push_1.view(-1, 1), self.push_force_1, torch.zeros_like(force))
        force = force + torch.where(push_2.view(-1, 1), self.push_force_2, torch.zeros_like(force))
        torque = torque + torch.where(push_1.view(-1, 1), self.push_torque_1, torch.zeros_like(torque))
        torque = torque + torch.where(push_2.view(-1, 1), self.push_torque_2, torch.zeros_like(torque))
        torque = torque + torch.where(payload.view(-1, 1), self.payload_torque, torch.zeros_like(torque))
        force[:, AXIS_FOREAFT] = torch.where(
            sine,
            force[:, AXIS_FOREAFT] + self.sine_amp * torch.sin(2.0 * math.pi * self.sine_freq * t),
            force[:, AXIS_FOREAFT],
        )
        return force, torque

    def active_for_recovery(self, step: torch.Tensor) -> torch.Tensor:
        push_1 = (step >= self.push_start_1) & (step < self.push_stop_1)
        push_2 = (step >= self.push_start_2) & (step < self.push_stop_2)
        # For payload+push benchmarks, recovery is measured after the push event;
        # for payload-only benchmarks, it is measured after the payload step.
        payload_step = (self.kind == DIST_PAYLOAD) & (step == self.payload_start)
        slope_start = (self.kind == DIST_SLOPE) & (step == 0)
        return push_1 | push_2 | payload_step | slope_start

    def _clear(self, env_ids: torch.Tensor) -> None:
        self.kind[env_ids] = DIST_NONE
        self.push_force_1[env_ids] = 0.0
        self.push_torque_1[env_ids] = 0.0
        self.push_force_2[env_ids] = 0.0
        self.push_torque_2[env_ids] = 0.0
        self.push_start_1[env_ids] = 0
        self.push_stop_1[env_ids] = 0
        self.push_start_2[env_ids] = 0
        self.push_stop_2[env_ids] = 0
        self.payload_torque[env_ids] = 0.0
        self.payload_start[env_ids] = 0
        self.slope_force[env_ids] = 0.0
        self.sine_amp[env_ids] = 0.0
        self.sine_freq[env_ids] = 0.0

    def _sample_push(self, env_ids: torch.Tensor, first: bool, dt: float) -> None:
        n = len(env_ids)
        start = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_start_s_range)
        duration = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_duration_s_range)
        force = torch.zeros(n, 3, device=self.device)
        torque = torch.zeros(n, 3, device=self.device)
        # fx is the fore/aft shove, fy the sideways one — see AXIS_* constants.
        force[:, AXIS_FOREAFT] = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_fx_n_range)
        force[:, AXIS_LATERAL] = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_fy_n_range)
        torque[:, AXIS_TORQUE_YAW] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.human_push_yaw_torque_nm_range
        )
        start_step = torch.ceil(start / dt - 1.0e-9).to(torch.long)
        stop_step = start_step + torch.ceil(duration / dt).to(torch.long)
        if first:
            self.push_start_1[env_ids], self.push_stop_1[env_ids] = start_step, stop_step
            self.push_force_1[env_ids], self.push_torque_1[env_ids] = force, torque
        else:
            self.push_start_2[env_ids], self.push_stop_2[env_ids] = start_step, stop_step
            self.push_force_2[env_ids], self.push_torque_2[env_ids] = force, torque

    def _sample_payload(self, env_ids: torch.Tensor, dt: float) -> None:
        n = len(env_ids)
        start = torch.empty(n, device=self.device).uniform_(*self.cfg.payload_start_s_range)
        self.payload_start[env_ids] = torch.ceil(start / dt - 1.0e-9).to(torch.long)
        self.payload_torque[env_ids, AXIS_TORQUE_PITCH] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.payload_pitch_torque_nm_range
        )

    def force(self, step: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        """Compatibility helper for older callers; returns the fore/aft force."""
        force, _ = self.force_and_torque(step, t)
        return force[:, AXIS_FOREAFT]


class CommandGenerator:
    """Joystick-style velocity / yaw-rate commands with slew limits and integrated references.

    Mirrors exactly what the STM32 firmware must do with the joystick input:

        v_cmd, w_cmd     <- joystick, slew-limited
        pos_ref          += v_cmd * dt          (longitudinal odometry reference, m)
        yaw_ref          += w_cmd * dt          (heading reference, rad), then
                             re-wrapped to (-pi, pi] -- numerical hygiene only,
                             mathematically inert (see step())
        pos_err          = clamp(x_odom - pos_ref, +-pos_err_clamp)
        yaw_err_sin/cos  = sin/cos(yaw - yaw_ref)

    The clamp on ``pos_err`` is the anti-windup that keeps real-world odometry
    drift from pushing the observation out of the training distribution.
    ``yaw_ref`` does NOT get the equivalent anti-windup treatment -- see
    ``step()`` and ``yaw_error_sin_cos()`` for why yaw does not need it.
    """

    def __init__(self, cfg, num_envs: int, device: torch.device):
        self.cfg = cfg
        self.device = device
        self.num_envs = num_envs
        self.v_target = torch.zeros(num_envs, device=device)
        self.w_target = torch.zeros(num_envs, device=device)
        self.v_cmd = torch.zeros(num_envs, device=device)
        self.w_cmd = torch.zeros(num_envs, device=device)
        self.pos_ref = torch.zeros(num_envs, device=device)
        self.yaw_ref = torch.zeros(num_envs, device=device)
        self.still_episode = torch.zeros(num_envs, device=device, dtype=torch.bool)
        self.next_resample_step = torch.zeros(num_envs, device=device, dtype=torch.long)

    @property
    def forced(self) -> bool:
        # Read live so benchmarks can flip forced commands between env.reset()
        # calls without rebuilding the environment.
        return self.cfg.forced_command_mode == "fixed"

    def _stage_limits(self, stage: int) -> tuple[float, float]:
        idx = max(0, min(stage - 1, len(self.cfg.cmd_stage_velocity_max_mps) - 1))
        return self.cfg.cmd_stage_velocity_max_mps[idx], self.cfg.cmd_stage_yaw_rate_max_radps[idx]

    def _sample_targets(self, env_ids: torch.Tensor, stage: int) -> None:
        n = len(env_ids)
        v_max, w_max = self._stage_limits(stage)
        if self.forced:
            self.v_target[env_ids] = self.cfg.forced_velocity_cmd_mps
            self.w_target[env_ids] = self.cfg.forced_yaw_rate_cmd_radps
            return
        v = torch.empty(n, device=self.device).uniform_(-v_max, v_max)
        w = torch.empty(n, device=self.device).uniform_(-w_max, w_max)
        # A slice of resamples commands zero on one or both axes so the policy
        # regularly practices pure driving, pure turning, and full stops.
        zero_v = torch.rand(n, device=self.device) < self.cfg.cmd_zero_axis_prob
        zero_w = torch.rand(n, device=self.device) < self.cfg.cmd_zero_axis_prob
        v = torch.where(zero_v, torch.zeros_like(v), v)
        w = torch.where(zero_w, torch.zeros_like(w), w)
        self.v_target[env_ids] = v
        self.w_target[env_ids] = w
        still = self.still_episode[env_ids]
        zeros = torch.zeros(n, device=self.device)
        self.v_target[env_ids] = torch.where(still, zeros, self.v_target[env_ids])
        self.w_target[env_ids] = torch.where(still, zeros, self.w_target[env_ids])

    def _schedule_resample(self, env_ids: torch.Tensor, current_step: torch.Tensor, dt: float) -> None:
        n = len(env_ids)
        lo, hi = self.cfg.cmd_resample_s_range
        interval = torch.empty(n, device=self.device).uniform_(lo, hi)
        self.next_resample_step[env_ids] = current_step + (interval / dt).to(torch.long).clamp(min=1)

    def reset(self, env_ids: torch.Tensor, stage: int, dt: float, initial_yaw: torch.Tensor) -> None:
        n = len(env_ids)
        self.v_cmd[env_ids] = 0.0
        self.w_cmd[env_ids] = 0.0
        self.pos_ref[env_ids] = 0.0
        self.yaw_ref[env_ids] = initial_yaw
        self.still_episode[env_ids] = torch.rand(n, device=self.device) < self.cfg.cmd_still_episode_prob
        if self.forced:
            self.still_episode[env_ids] = False
        self._sample_targets(env_ids, stage)
        self._schedule_resample(env_ids, torch.zeros(n, device=self.device, dtype=torch.long), dt)
        # Hold zero commands for a short settling window at episode start.
        settle_steps = int(self.cfg.cmd_settle_s / dt)
        self.next_resample_step[env_ids] = self.next_resample_step[env_ids].clamp(min=settle_steps)

    def step(
        self,
        step_buf: torch.Tensor,
        stage: int,
        dt: float,
        x_odom: torch.Tensor | None = None,
        yaw: torch.Tensor | None = None,
    ) -> None:
        """Advance commands and references by one control step.

        Pass ``x_odom``/``yaw`` to enable reference anti-windup (see
        ``_apply_reference_anti_windup``). They are optional so older callers
        that only advance the commands keep working.
        """
        due = step_buf >= self.next_resample_step
        due_ids = torch.nonzero(due, as_tuple=False).squeeze(-1)
        if len(due_ids) > 0:
            self._sample_targets(due_ids, stage)
            self._schedule_resample(due_ids, step_buf[due_ids], dt)
        # Settling window: force zero targets before cmd_settle_s.
        settle = step_buf < int(self.cfg.cmd_settle_s / dt)
        v_target = torch.where(settle, torch.zeros_like(self.v_target), self.v_target)
        w_target = torch.where(settle, torch.zeros_like(self.w_target), self.w_target)
        dv = (v_target - self.v_cmd).clamp(-self.cfg.cmd_velocity_slew_mps2 * dt, self.cfg.cmd_velocity_slew_mps2 * dt)
        dw = (w_target - self.w_cmd).clamp(-self.cfg.cmd_yaw_slew_radps2 * dt, self.cfg.cmd_yaw_slew_radps2 * dt)
        self.v_cmd = self.v_cmd + dv
        self.w_cmd = self.w_cmd + dw
        self.pos_ref = self.pos_ref + self.v_cmd * dt
        # Re-wrap yaw_ref to (-pi, pi] after integrating. This is NOT the
        # anti-windup position gets (that changes the reference's VALUE based
        # on the robot's measured state); this is a pure modulo reduction of
        # yaw_ref against itself, so it never changes sin/cos(yaw - yaw_ref) --
        # only keeps yaw_ref from growing to an arbitrarily large float over a
        # long-running deployment (radians accumulate without bound under
        # sustained turning commands otherwise).
        self.yaw_ref = wrap_angle_rad(self.yaw_ref + self.w_cmd * dt)
        self._apply_reference_anti_windup(x_odom)

    def _apply_reference_anti_windup(self, x_odom: torch.Tensor | None) -> None:
        """Back-calculate pos_ref so it can never outrun the robot.

        ``pos_ref`` integrates the joystick command, but the robot cannot
        always follow it. Without this, pos_ref runs away forever: pos_err
        pins at its clamp, a permanent, unclearable "you are behind" signal
        with zero gradient back toward zero once past it. This is standard
        integrator anti-windup by back-calculation.

        yaw_ref does NOT get the equivalent treatment (removed 2026-08-10).
        The original version existed for the same two reasons: (1) keep the
        observation-visible error bounded, and (2) avoid yaw_err sweeping past
        +-pi and wrapping, which is a step discontinuity that measured 100%
        fall rate at a sustained 1.2 rad/s command (vs 20% with the reference
        pinned). Both are now solved at the source instead: yaw_error_sin_cos
        encodes sin/cos of the RAW (unwrapped) difference, which is smooth and
        bounded in [-1, 1] for any magnitude of error, with no wrap point to
        hit regardless of how far yaw_ref runs ahead of an unachievable
        command. Pinning yaw_ref like pos_ref would reintroduce the exact
        silent-drift blind spot this replaced (see rew_hold_world_drift's
        history in nn_drive_env_cfg.py) -- for pitch tracking. Confirm the
        1.2 rad/s fall-rate result still holds under this encoding before
        trusting high sustained yaw-rate commands; that number was measured
        against the old representation, not re-derived for this one.
        """
        if x_odom is not None:
            pos_clamp = self.cfg.cmd_pos_err_clamp_m
            pos_err = x_odom - self.pos_ref
            self.pos_ref = self.pos_ref + (pos_err - pos_err.clamp(-pos_clamp, pos_clamp))

    def position_error(self, x_odom: torch.Tensor) -> torch.Tensor:
        clamp = self.cfg.cmd_pos_err_clamp_m
        return (x_odom - self.pos_ref).clamp(-clamp, clamp)

    def position_error_raw(self, x_odom: torch.Tensor) -> torch.Tensor:
        """Unclamped position error — reward-only, never fed to the policy.

        The observation must stay clamped for firmware parity, but a clamped
        reward is flat beyond +-cmd_pos_err_clamp_m, so a policy that has already
        drifted that far gets no incentive to come back. The reward uses this
        raw error to keep a pull toward home at any drift distance.
        """
        return x_odom - self.pos_ref

    def yaw_error_sin_cos(self, yaw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """sin/cos of the raw (unwrapped) heading error -- see step()'s docstring.

        Deliberately not run through wrap_angle_rad first: sin/cos of any real
        number is already smooth and periodic, so wrapping first would only
        reintroduce the discontinuity this representation exists to avoid.
        """
        diff = yaw - self.yaw_ref
        return torch.sin(diff), torch.cos(diff)


class DriveObservationBuilder:
    """Build the 21-value drive observation with fixed normalization.

    Layout (all values BEFORE dividing by ``drive_observation_scale``):
        [0]  pos_err          m,  clamp +-cmd_pos_err_clamp_m
        [1]  velocity         m/s (wheel odometry mean)
        [2]  pitch            rad
        [3]  pitch_rate       rad/s
        [4]  yaw_err_sin      sin(yaw - yaw_ref), raw/unwrapped difference
        [5]  yaw_err_cos      cos(yaw - yaw_ref), raw/unwrapped difference
        [6]  yaw_rate         rad/s
        [7]  velocity_cmd     m/s
        [8]  yaw_rate_cmd     rad/s
        [9-12]  cg_pos_norm   CyberGear joint angle / 90 deg (fl, fr, bl, br)
        [13-14] prev wheel current A (left, right)
        [15-18] prev cg action, tanh-squashed in [-1, 1] (fl, fr, bl, br)
        [19] roll             rad
        [20] roll_rate        rad/s

    ``yaw_err`` changed from a single clamped radian to an unclamped sin/cos
    pair 2026-08-10 (renumbering [4] onward -- this repo's firmware is still
    ours to define, so this trades the previous "indices 0-17 keep their
    meaning, append only" convention for actually fixing the representation).
    The old approach clamped yaw_err to +-cmd_yaw_err_clamp_rad via reference
    anti-windup (CommandGenerator._apply_reference_anti_windup used to handle
    yaw the same way it still handles position): needed both to keep the
    observation bounded and to avoid the raw wrapped error sweeping past +-pi,
    a step discontinuity in the "which way to turn" signal that measured 100%
    fall rate at a sustained 1.2 rad/s command. sin/cos of the RAW (unwrapped)
    difference solves both at once -- it is smooth and bounded in [-1, 1] for
    any error magnitude, with no wrap point to hit regardless of how far
    yaw_ref runs ahead of an unachievable command -- so the anti-windup on
    yaw_ref was removed rather than reused for a heading-drift patch. Unlike
    pos_err (which stays reward-only for world drift -- position is unbounded
    AND has no absolute real-world sensor, only drifting wheel odometry), yaw
    needed neither a clamp nor a privileged-reward workaround once encoded
    this way: it is bounded by construction (periodic) and, on this robot's
    BNO085, an actually measurable quantity, not a dead-reckoned one. yaw
    itself carries no simulated noise/bias (see _get_observations), matching a
    fused-IMU heading rather than raw gyro integration.

    Roll and roll_rate ([19]/[20]) were added 2026-08-04, appended after
    pitch/pitch_rate rather than grouped next to them so the firmware change
    was two extra values rather than a renumbering -- the convention the yaw
    change above deliberately breaks from, for reasons explained there. Before
    roll/roll_rate existed the reward penalized roll at
    rew_roll = 12.0 -- the heaviest weight in the config -- while roll was
    absent from this vector, so the policy was taxed on a quantity it could not
    sense while holding four leg actuators that directly control it. Measured
    consequence: roll left the reset value of 0.000 deg and settled to a
    systematic lean within 1.5 s, identical in sign and magnitude across all
    envs and unaffected by command, disturbance or terrain (-5.3 deg for the
    2026-07-29 policy, -11.0 deg for 2026-08-04). With zero actions the
    platform sits level, confirming the lean was policy-commanded.
    """

    def __init__(self, cfg, device: torch.device):
        self.cfg = cfg
        self.scale = torch.tensor(cfg.drive_observation_scale, device=device, dtype=torch.float32).view(1, -1)
        if len(cfg.drive_observation_scale) != cfg.observation_space:
            raise ValueError("drive_observation_scale length must match observation_space")

    def build(
        self,
        pos_err: torch.Tensor,
        velocity: torch.Tensor,
        pitch: torch.Tensor,
        pitch_rate: torch.Tensor,
        yaw_err_sin: torch.Tensor,
        yaw_err_cos: torch.Tensor,
        yaw_rate: torch.Tensor,
        velocity_cmd: torch.Tensor,
        yaw_rate_cmd: torch.Tensor,
        cg_pos_norm: torch.Tensor,
        previous_current: torch.Tensor,
        previous_cg_action: torch.Tensor,
        roll: torch.Tensor,
        roll_rate: torch.Tensor,
    ) -> torch.Tensor:
        scalars = torch.stack(
            [pos_err, velocity, pitch, pitch_rate, yaw_err_sin, yaw_err_cos, yaw_rate, velocity_cmd, yaw_rate_cmd],
            dim=1,
        )
        attitude = torch.stack([roll, roll_rate], dim=1)
        # With the legs pinned, cg_pos_norm and previous_cg_action are constants
        # (the fixed stance mapped back through the same path), so they are eight
        # dead inputs. Dropping them takes the vector 21 -> 13 without disturbing
        # indices 0-8, which keep their firmware meaning either way.
        if self.cfg.include_cg_obs:
            blocks = [scalars, cg_pos_norm, previous_current, previous_cg_action, attitude]
        else:
            blocks = [scalars, previous_current, attitude]
        obs = torch.cat(blocks, dim=1)
        return torch.nan_to_num(obs / self.scale, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)


class CyberGearStanceProcessor:
    """Convert raw CyberGear actions into slew-limited joint position targets.

    Deployment contract (must match the STM32 firmware):
        t = tanh(a)
        target = t * joint_hi if t >= 0 else (-t) * joint_lo
        target = clamp(target + calibration_bias, joint_lo, joint_hi)
        target = slew_limit(target, cg_target_slew_radps)

    A per-episode calibration bias models real CyberGear zero-offset error.
    """

    def __init__(self, cfg, num_envs: int, device: torch.device):
        self.cfg = cfg
        self.device = device
        self.tanh_action = torch.zeros(num_envs, 4, device=device)
        self.prev_tanh_action = torch.zeros(num_envs, 4, device=device)
        self.target_angle = torch.zeros(num_envs, 4, device=device)
        self.prev_target_angle = torch.zeros(num_envs, 4, device=device)
        self.applied_target = torch.zeros(num_envs, 4, device=device)
        self.calib_bias = torch.zeros(num_envs, 4, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        n = len(env_ids)
        self.tanh_action[env_ids] = 0.0
        self.prev_tanh_action[env_ids] = 0.0
        self.target_angle[env_ids] = 0.0
        self.prev_target_angle[env_ids] = 0.0
        self.applied_target[env_ids] = 0.0
        lo, hi = self.cfg.cg_calib_bias_rad_range
        self.calib_bias[env_ids] = torch.empty(n, 4, device=self.device).uniform_(lo, hi)

    def process(
        self,
        raw_cg_actions: torch.Tensor,
        joint_lo: torch.Tensor,
        joint_hi: torch.Tensor,
        dt: float,
    ) -> torch.Tensor:
        self.prev_tanh_action = self.tanh_action.clone()
        self.prev_target_angle = self.target_angle.clone()
        self.tanh_action = torch.tanh(raw_cg_actions)
        self.target_angle = map_cybergear_tanh_to_joint_target(self.tanh_action, joint_lo, joint_hi)
        desired = (self.target_angle + self.calib_bias).clamp(min=joint_lo, max=joint_hi)
        max_step = self.cfg.cg_target_slew_radps * dt
        delta = (desired - self.applied_target).clamp(-max_step, max_step)
        self.applied_target = self.applied_target + delta
        return self.applied_target

    def process_fixed(
        self,
        fixed_target_rad: tuple[float, float, float, float],
        joint_lo: torch.Tensor,
        joint_hi: torch.Tensor,
        dt: float,
    ) -> torch.Tensor:
        """Hold a physical leg stance through the same bias and slew path."""
        self.prev_tanh_action = self.tanh_action.clone()
        self.prev_target_angle = self.target_angle.clone()
        target = torch.as_tensor(fixed_target_rad, device=self.device, dtype=joint_lo.dtype).view(1, 4)
        self.target_angle = target.expand_as(joint_lo).clamp(min=joint_lo, max=joint_hi)
        # Preserve the observation meaning of "previous CG tanh action" even
        # though the diagnostic policy does not emit these four values.
        positive_scale = joint_hi.clamp_min(torch.finfo(joint_hi.dtype).eps)
        negative_scale = (-joint_lo).clamp_min(torch.finfo(joint_lo.dtype).eps)
        self.tanh_action = torch.where(
            self.target_angle >= 0.0,
            self.target_angle / positive_scale,
            self.target_angle / negative_scale,
        ).clamp(-1.0, 1.0)
        desired = (self.target_angle + self.calib_bias).clamp(min=joint_lo, max=joint_hi)
        max_step = self.cfg.cg_target_slew_radps * dt
        delta = (desired - self.applied_target).clamp(-max_step, max_step)
        self.applied_target = self.applied_target + delta
        return self.applied_target

    def delta_target_angle(self) -> torch.Tensor:
        return self.target_angle - self.prev_target_angle


class SteadinessTracker:
    """Exponential-moving-average reference for the AC (oscillating) part of a signal.

    Motivated by measured hardware, not by simulation. The deployed range+GRU
    policy holds a clean **0.83 Hz limit cycle** while station-keeping: pitch and
    forward velocity peak at the same frequency (one coupled rocking mode), +-2 deg
    and +-0.1 m/s, and the robot wanders 0.52 m over a 17 s hold
    (TwoWheeledRobot_Embedded, Results/paper/range_gru_segmented.csv, idle stage).
    Replaying that trajectory through DriveReward's weights shows why nothing
    suppressed it -- the whole oscillation costs about 0.07/step against a
    1.0/step alive bonus:

      * ``rew_pitch`` charges 0.0007/step. Its ``pitch_flat_deg = 3.0`` deadband
        is *wider than the oscillation*, so 75% of the samples sit in the
        zero-gradient region and the swing is very nearly free.
      * ``rew_delta_current`` charges 0.0020/step. It penalizes the step-to-step
        action change at the 66.7 Hz control rate, and one 15 ms step of a 1.2 s
        period moves the action by well under 1% of its amplitude. It is a
        chatter term measuring at the wrong timescale entirely.

    Subtracting an EMA is a first-order high-pass: with ``tau = 0.7 s`` the corner
    sits at 0.23 Hz, so a DC offset is removed completely while the measured
    0.83 Hz component passes at ~0.96 gain. That is the property the deadband
    argument actually needs. The deadband exists because the reward reads true
    pitch while the policy observes a copy carrying a per-episode +-3 deg IMU
    mounting bias, so demanding an exact angle trains a precision the hardware
    cannot deliver -- but a mounting bias is a *constant*, and this residual
    cancels constants by construction. Penalising it therefore asks only that
    the robot stop moving back and forth, which is measurable on any IMU
    regardless of how it is bolted on, and leaves the deadband intact for the
    absolute-angle term it was designed for.
    """

    def __init__(self, num_envs: int, width: int, device, tau_s: float):
        self._ema = torch.zeros(num_envs, width, device=device)
        self._primed = torch.zeros(num_envs, 1, device=device)
        self.tau_s = tau_s

    def update(self, value: torch.Tensor, step_dt: float) -> torch.Tensor:
        """Advance the EMA and return this step's AC residual (value - EMA).

        The EMA is seeded with the first post-reset sample rather than with zero.
        Decaying up from zero would report a large spurious residual for the
        first few tenths of a second of every episode -- a reset transient, not
        an oscillation -- and with short episodes that artefact would be a
        significant fraction of the training signal.
        """
        alpha = math.exp(-step_dt / max(self.tau_s, step_dt))
        seeded = self._primed * self._ema + (1.0 - self._primed) * value
        self._ema = alpha * seeded + (1.0 - alpha) * value
        self._primed = torch.ones_like(self._primed)
        return value - seeded

    def reset(self, env_ids: torch.Tensor) -> None:
        self._ema[env_ids] = 0.0
        self._primed[env_ids] = 0.0


class DriveReward:
    """Command-tracking reward for the NN drive task.

    Rewritten 2026-08-07 so that **every per-step reward is >= 0**. All goal terms
    are bounded non-negative bonuses; only actuation costs subtract, and their
    worst case (0.706) is below ``rew_alive``. With all rewards >= 0 and gamma < 1,
    a longer episode weakly dominates a shorter one, so diving for the floor
    cannot pay — by the shape of the reward, not by a clamp calibration. The
    previous design bounded each penalty against the ~2.3/step a fall forfeits;
    that argument had already failed once (docs/experiments/2026-07-28: an
    unbounded wrapped-yaw penalty reached 4.9/step and turn-in-place fell 98% of
    the time) and had to be re-derived every time a weight moved.

    Shapes, both module-level helpers:

    * ``tent_bonus`` — steepest at zero error, linear to zero at ``max_error``.
      Replaces the old ``-w*e^2`` penalties and keeps the restoring gradient that
      closes out the last few cm/s, which an exp kernel (flat at zero) cannot.
    * ``flat_top_bonus`` — full weight inside a band with no gradient, decaying
      outside. Used for pitch and roll.

    Attitude is a band rather than a target because the reward reads *true*
    pitch/roll while the policy observes a copy carrying a per-episode mounting
    bias (+-3 deg pitch, +-1 deg roll). Demanding an exact angle inside that band
    asks the policy to resolve what its sensor cannot, and trains a precision the
    hardware IMU can never deliver. Attitude is also no longer a termination
    criterion — falling is contact-based (see NNDriveEnvCfg.fall_mode) — so the
    3-to-15 deg shoulder is the only thing keeping the robot vertical.
    """

    def __init__(self, cfg):
        self.cfg = cfg

    def compute(
        self,
        pos_err_raw: torch.Tensor,
        velocity: torch.Tensor,
        pitch: torch.Tensor,
        pitch_rate: torch.Tensor,
        roll: torch.Tensor,
        roll_rate: torch.Tensor,
        yaw_err_cos: torch.Tensor,
        yaw_rate: torch.Tensor,
        velocity_cmd: torch.Tensor,
        yaw_rate_cmd: torch.Tensor,
        current: torch.Tensor,
        delta_current: torch.Tensor,
        cg_target_angle: torch.Tensor,
        cg_delta_target_angle: torch.Tensor,
        world_drift: torch.Tensor,
        pitch_ac: torch.Tensor,
        action_ac: torch.Tensor,
        terminal_penalty: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        # The policy only ever sees pos_err clamped to +-cmd_pos_err_clamp_m; the
        # reward reads the raw drift so the bonus keeps pulling home past the
        # clamp, all the way out to where it reaches zero.
        clamp = self.cfg.cmd_pos_err_clamp_m
        pos_err = pos_err_raw.clamp(-clamp, clamp)
        pos_far = (pos_err_raw.abs() - clamp).clamp(0.0, self.cfg.pos_err_far_max_m)

        vel_err = (velocity - velocity_cmd).clamp(-1.0, 1.0)
        yaw_rate_err = (yaw_rate - yaw_rate_cmd).clamp(-3.0, 3.0)
        # Every penalty below is bounded on purpose. An unbounded per-step
        # penalty can exceed what falling costs (-fall_penalty once, plus the
        # forgone alive/tracking reward), at which point diving for the floor is
        # the optimal policy. The wrapped yaw error was the live example: at
        # +-pi it cost 0.5*pi^2 = 4.9/step, roughly double the 2.3/step the robot
        # gives up by falling, and turn-in-place benchmarks fell 98% of the time.
        # yaw_error below is now 1-cos(diff) instead of a clamped square: it is
        # bounded in [0, 2] BY CONSTRUCTION for any error magnitude, so there is
        # no clamp value to pick or re-derive if a weight moves.
        rate_clamp = self.cfg.attitude_rate_pen_clamp_radps
        pitch_rate_pen = pitch_rate.clamp(-rate_clamp, rate_clamp)
        roll_rate_pen = roll_rate.clamp(-rate_clamp, rate_clamp)
        hold_velocity = velocity.clamp(-0.5, 0.5)
        # "Joystick centred" gate: the station-keeping requirement is only
        # meaningful when no motion was asked for.
        hold = (
            (velocity_cmd.abs() < self.cfg.hold_velocity_cmd_threshold_mps)
            & (yaw_rate_cmd.abs() < self.cfg.hold_yaw_rate_cmd_threshold_radps)
        ).float()

        # Attitude DEADBAND. The reward reads true pitch/roll while the policy
        # only ever observes a copy carrying a per-episode IMU mounting bias
        # (+-3 deg pitch, +-1 deg roll). Penalising anything inside that band
        # asks the policy to resolve an angle its sensor cannot resolve, and on
        # hardware you can never do better than your calibration -- so an exact
        # theta = 0 is a sim-only skill. Outside the band the original quadratic
        # (plus the linear roll partner) is unchanged, so the fall economics the
        # clamps above were calibrated against still hold.
        pitch_excess = (pitch.abs() - math.radians(self.cfg.pitch_flat_deg)).clamp(min=0.0)
        roll_excess = (roll.abs() - math.radians(self.cfg.roll_flat_deg)).clamp(min=0.0)

        components = {
            "alive": torch.ones_like(pitch) * self.cfg.rew_alive,
            "vel_track": self.cfg.rew_vel_track * torch.exp(-vel_err.pow(2) / self.cfg.vel_track_sigma**2),
            "yaw_rate_track": self.cfg.rew_yaw_rate_track
            * torch.exp(-yaw_rate_err.pow(2) / self.cfg.yaw_rate_track_sigma**2),
            "vel_err": -self.cfg.rew_vel_err * vel_err.pow(2),
            "yaw_rate_err": -self.cfg.rew_yaw_rate_err * yaw_rate_err.pow(2),
            "position": -self.cfg.rew_position * pos_err.pow(2),
            "position_far": -self.cfg.rew_position_far * pos_far,
            "hold_velocity": -self.cfg.rew_hold_velocity * hold * hold_velocity.pow(2),
            "hold_position": -self.cfg.rew_hold_position * hold * pos_err.pow(2),
            # Ground-truth world drift, not the anti-windup-mutable reference
            # error -- see rew_hold_world_drift in nn_drive_env_cfg.py.
            "hold_world_drift": -self.cfg.rew_hold_world_drift
            * hold
            * world_drift.clamp(0.0, self.cfg.hold_world_drift_clamp_m).pow(2),
            # ---- Station-keeping steadiness (2026-08-21) --------------------
            # Three terms against one measured failure: the deployed range+GRU
            # policy holds a 0.83 Hz pitch/velocity limit cycle while standing
            # still, which the reward above charges ~0.07/step for. See
            # SteadinessTracker for the measurement and the per-term autopsy.
            #
            # All three are hold-gated. Pitch legitimately swings while
            # accelerating -- leaning IS how this robot drives -- so an
            # always-on steadiness term would be taxing the correct behaviour.
            # "Smooth while driving" is a separate question from "still while
            # still", and only the second one is what the hardware runs failed.
            #
            # Two are bounded non-negative BONUSES, so they cannot make falling
            # attractive no matter how they are weighted -- the failure mode
            # rew_hold_world_drift's comment documents (individually-bounded
            # penalties stacking past the ~2.3/step a fall forfeits) is
            # unreachable by construction here, which is why the pitch signal
            # is shaped as a bonus rather than as the obvious penalty.
            #
            # Exp kernels for the same reason vel_track uses one: they are the
            # small-error discriminators. The existing quadratics (hold_velocity)
            # are steepest at large error and nearly flat at the last few cm/s,
            # which is precisely the regime "hold still" lives in.
            "hold_pitch_steady": self.cfg.rew_hold_pitch_steady
            * hold
            * torch.exp(-pitch_ac.pow(2) / math.radians(self.cfg.hold_pitch_ac_sigma_deg) ** 2),
            "hold_vel_steady": self.cfg.rew_hold_vel_steady
            * hold
            * torch.exp(-hold_velocity.pow(2) / self.cfg.hold_vel_steady_sigma_mps**2),
            # The one penalty, and the term rew_delta_current should have been:
            # it reads the wheel action's departure from its own 0.7 s average
            # instead of from last step's value, so it sees the 0.83 Hz band
            # that a step-to-step difference at 66.7 Hz cannot resolve.
            # Normalized by the clamp so the worst case is exactly
            # rew_hold_action_ac per step, with no cap-versus-weight arithmetic
            # to redo whenever either moves.
            "hold_action_ac": -self.cfg.rew_hold_action_ac
            * hold
            * (action_ac.clamp(-self.cfg.hold_action_ac_clamp, self.cfg.hold_action_ac_clamp)
               / self.cfg.hold_action_ac_clamp).pow(2).mean(dim=1),
            # No hold_world_heading_drift term: yaw_error below already reads
            # yaw_ref without anti-windup, so it has no blind spot to patch in
            # the first place (see CommandGenerator._apply_reference_anti_windup).
            "yaw_error": -self.cfg.rew_yaw_error * (1.0 - yaw_err_cos),
            "pitch": -self.cfg.rew_pitch * pitch_excess.pow(2),
            "pitch_rate": -self.cfg.rew_pitch_rate * pitch_rate_pen.pow(2),
            # Quadratic + linear on the band excess. The quadratic alone has
            # vanishing gradient at the band edge, which would leave a lean just
            # outside the band nearly free; the linear term keeps a constant
            # restoring gradient back to it. The robot is symmetric to 0.003 deg
            # airborne and its nominal COM is +0.29 mm fore/aft and -0.03 mm
            # lateral of the wheel axis (scripts/measure_nominal_com.py), so an
            # observed lean is the policy breaking a symmetry, not the plant.
            "roll": -self.cfg.rew_roll * roll_excess.pow(2) - self.cfg.rew_roll_abs * roll_excess,
            "roll_rate": -self.cfg.rew_roll_rate * roll_rate_pen.pow(2),
            "current": -self.cfg.rew_current * current.pow(2).sum(dim=1),
            "delta_current": -self.cfg.rew_delta_current * delta_current.pow(2).sum(dim=1),
            # Penalize the mapped physical targets in radians. The mapping has
            # already respected each joint's asymmetric limits, so an invalid
            # request hidden behind a clamp cannot distort either reward term.
            "cg_pos": -self.cfg.rew_cg_pos * cg_target_angle.pow(2).sum(dim=1),
            # Bounded by clamping the DELTA (kept from the 2026-08-07 work, a
            # genuine pre-existing bug): target_angle is not slew-limited --
            # cg_target_slew_radps limits applied_target one stage later -- so a
            # single action reversal can swing all four joints across their full
            # range for a -156/step penalty. It logged as -0.006 and never
            # surfaced until random-action probing hit -32/step.
            "cg_rate": -self.cfg.rew_cg_rate
            * cg_delta_target_angle.clamp(
                -self.cfg.cg_rate_delta_clamp_rad, self.cfg.cg_rate_delta_clamp_rad
            ).pow(2).sum(dim=1),
        }
        if terminal_penalty is None:
            terminal_penalty = torch.zeros_like(pitch)
        components["terminal"] = terminal_penalty
        reward = sum(components.values())
        reward = torch.nan_to_num(reward, nan=0.0, posinf=3.0, neginf=-100.0).clamp(-100.0, 3.0)
        components["total"] = reward
        return reward, components


class BalanceReward:
    def __init__(self, cfg):
        self.cfg = cfg

    def compute(
        self,
        position: torch.Tensor,
        pitch: torch.Tensor,
        pitch_rate: torch.Tensor,
        roll: torch.Tensor,
        roll_rate: torch.Tensor,
        velocity: torch.Tensor,
        yaw_error: torch.Tensor,
        yaw_rate: torch.Tensor,
        current: torch.Tensor,
        delta_current: torch.Tensor,
        terminal_penalty: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        components = {
            "alive": torch.ones_like(pitch) * self.cfg.rew_alive,
            "position": -self.cfg.rew_position * position.pow(2),
            "pitch": -self.cfg.rew_pitch * pitch.pow(2),
            "pitch_rate": -self.cfg.rew_pitch_rate * pitch_rate.pow(2),
            "roll": -self.cfg.rew_roll * roll.pow(2),
            "roll_rate": -self.cfg.rew_roll_rate * roll_rate.pow(2),
            "velocity": -self.cfg.rew_velocity * velocity.pow(2),
            "yaw_error": -self.cfg.rew_yaw_error * yaw_error.pow(2),
            "yaw_rate": -self.cfg.rew_yaw_rate * yaw_rate.pow(2),
            "current": -self.cfg.rew_current * current.pow(2).sum(dim=1),
            "delta_current": -self.cfg.rew_delta_current * delta_current.pow(2).sum(dim=1),
        }
        if terminal_penalty is None:
            terminal_penalty = torch.zeros_like(pitch)
        components["terminal"] = terminal_penalty
        reward = sum(components.values())
        reward = torch.nan_to_num(reward, nan=0.0, posinf=1.0, neginf=-100.0).clamp(-100.0, 1.0)
        components["total"] = reward
        return reward, components
