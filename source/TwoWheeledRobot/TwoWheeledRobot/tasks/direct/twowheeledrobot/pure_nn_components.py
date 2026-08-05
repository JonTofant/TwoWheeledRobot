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
        self.bias[env_ids] = torch.empty(n, 2, device=self.device).uniform_(bias_lo, bias_hi)
        self.tau_s[env_ids] = torch.empty(n, 2, device=self.device).uniform_(tau_lo, tau_hi)
        self.current_limit[env_ids] = torch.empty(n, 2, device=self.device).uniform_(limit_lo, limit_hi)

    def process(self, raw_actions: torch.Tensor, dt: float) -> torch.Tensor:
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
        motor_target = torch.sign(motor_target) * torch.clamp(motor_target.abs() - self.deadzone, min=0.0)
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
        return (
            math.radians(self.cfg.reset_pitch_range_deg),
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
            self.kind[env_ids] = torch.where(u < 0.40, DIST_NONE, torch.where(u < 0.80, DIST_PAYLOAD, DIST_PAYLOAD_PUSH))
        elif stage >= 3:
            # 50% none, 40% single human push, 10% double human push.
            self.kind[env_ids] = torch.where(u < 0.50, DIST_NONE, torch.where(u < 0.90, DIST_HUMAN_PUSH, DIST_DOUBLE_HUMAN_PUSH))

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
        yaw_ref          += w_cmd * dt          (heading reference, rad)
        pos_err          = clamp(x_odom - pos_ref, +-pos_err_clamp)
        yaw_err          = wrap(yaw - yaw_ref)

    The clamp on ``pos_err`` is the anti-windup that keeps real-world odometry
    drift from pushing the observation out of the training distribution.
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
        self.yaw_ref = self.yaw_ref + self.w_cmd * dt
        self._apply_reference_anti_windup(x_odom, yaw)

    def _apply_reference_anti_windup(self, x_odom: torch.Tensor | None, yaw: torch.Tensor | None) -> None:
        """Back-calculate the references so they can never outrun the robot.

        ``pos_ref``/``yaw_ref`` integrate the joystick command, but the robot
        cannot always follow it — the measured yaw-rate ceiling is ~0.75 rad/s
        against commands up to 2.0 rad/s. Without this, the reference runs away
        forever: ``pos_err`` pins at its clamp (a permanent, unclearable "you are
        behind" signal) and ``yaw_err`` sweeps past +-pi and *wraps*, which is a
        step discontinuity in the observation. Measured effect of the wrap: 100%
        fall rate at a sustained 1.2 rad/s, versus 20% with the reference pinned.

        This is standard integrator anti-windup by back-calculation, and it is
        the same job the ``cmd_pos_err_clamp_m`` clamp was doing on the
        observation — except done at the source, so the error never wraps and
        the reference stays recoverable.
        """
        if x_odom is not None:
            pos_clamp = self.cfg.cmd_pos_err_clamp_m
            pos_err = x_odom - self.pos_ref
            self.pos_ref = self.pos_ref + (pos_err - pos_err.clamp(-pos_clamp, pos_clamp))
        if yaw is not None:
            yaw_clamp = self.cfg.cmd_yaw_err_clamp_rad
            yaw_err = wrap_angle_rad(yaw - self.yaw_ref)
            self.yaw_ref = self.yaw_ref + (yaw_err - yaw_err.clamp(-yaw_clamp, yaw_clamp))

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

    def yaw_error(self, yaw: torch.Tensor) -> torch.Tensor:
        # Anti-windup keeps this within +-cmd_yaw_err_clamp_rad, so the wrap is
        # never reached; the clamp here makes that guarantee explicit and matches
        # what the firmware computes.
        clamp = self.cfg.cmd_yaw_err_clamp_rad
        return wrap_angle_rad(yaw - self.yaw_ref).clamp(-clamp, clamp)


class DriveObservationBuilder:
    """Build the 20-value drive observation with fixed normalization.

    Layout (all values BEFORE dividing by ``drive_observation_scale``):
        [0]  pos_err          m,  clamp +-cmd_pos_err_clamp_m
        [1]  velocity         m/s (wheel odometry mean)
        [2]  pitch            rad
        [3]  pitch_rate       rad/s
        [4]  yaw_err          rad, wrapped
        [5]  yaw_rate         rad/s
        [6]  velocity_cmd     m/s
        [7]  yaw_rate_cmd     rad/s
        [8-11]  cg_pos_norm   CyberGear extension fraction in [-1, 1] (fl, fr, bl, br)
        [12-13] prev wheel current A (left, right)
        [14-17] prev cg action, tanh-squashed in [-1, 1] (fl, fr, bl, br)
        [18] roll             rad
        [19] roll_rate        rad/s

    Roll and roll_rate are APPENDED rather than grouped next to pitch on
    purpose: indices 0-17 keep their meaning, so the STM32 firmware change is
    two added values at the end instead of renumbering fourteen entries.

    They were added 2026-08-04. Before that the reward penalized roll at
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
        yaw_err: torch.Tensor,
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
            [pos_err, velocity, pitch, pitch_rate, yaw_err, yaw_rate, velocity_cmd, yaw_rate_cmd], dim=1
        )
        attitude = torch.stack([roll, roll_rate], dim=1)
        obs = torch.cat([scalars, cg_pos_norm, previous_current, previous_cg_action, attitude], dim=1)
        return torch.nan_to_num(obs / self.scale, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)


class CyberGearStanceProcessor:
    """Convert raw CyberGear actions into slew-limited joint position targets.

    Deployment contract (must match the STM32 firmware):
        target = clamp(nominal + tanh(a) * authority, joint_lo, joint_hi)
        target = slew_limit(target, cg_target_slew_radps)

    A per-episode calibration bias models real CyberGear zero-offset error.
    """

    def __init__(self, cfg, num_envs: int, device: torch.device):
        self.cfg = cfg
        self.device = device
        self.tanh_action = torch.zeros(num_envs, 4, device=device)
        self.prev_tanh_action = torch.zeros(num_envs, 4, device=device)
        self.applied_target = torch.zeros(num_envs, 4, device=device)
        self.calib_bias = torch.zeros(num_envs, 4, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        n = len(env_ids)
        self.tanh_action[env_ids] = 0.0
        self.prev_tanh_action[env_ids] = 0.0
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
        self.tanh_action = torch.tanh(raw_cg_actions)
        desired = (self.tanh_action * self.cfg.cg_action_authority_rad + self.calib_bias).clamp(
            min=joint_lo, max=joint_hi
        )
        max_step = self.cfg.cg_target_slew_radps * dt
        delta = (desired - self.applied_target).clamp(-max_step, max_step)
        self.applied_target = self.applied_target + delta
        return self.applied_target

    def delta_tanh(self) -> torch.Tensor:
        return self.tanh_action - self.prev_tanh_action


class DriveReward:
    """Command-tracking reward for the NN drive task.

    Positive tracking terms (exp kernels) reward following the joystick;
    quadratic penalties keep pitch, position drift, and actuation smooth.
    Pitch is deliberately weighted lower than in the balance task because the
    equilibrium pitch is nonzero on inclines and while accelerating.

    An exp kernel is flat at zero error, so on its own it exerts almost no pull
    over the last few cm/s — which is exactly the regime that decides whether
    the robot holds station or creeps away. The kernels are therefore paired
    with quadratic ``vel_err``/``yaw_rate_err`` terms (maximum gradient at zero)
    plus ``hold_*`` terms that switch on only while the joystick is centred.
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
        yaw_err: torch.Tensor,
        yaw_rate: torch.Tensor,
        velocity_cmd: torch.Tensor,
        yaw_rate_cmd: torch.Tensor,
        current: torch.Tensor,
        delta_current: torch.Tensor,
        cg_tanh: torch.Tensor,
        cg_delta_tanh: torch.Tensor,
        terminal_penalty: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        # The policy only ever sees pos_err clamped to +-cmd_pos_err_clamp_m; the
        # reward keeps a bounded linear term on the excess so drift past the
        # clamp is still punished and still pulls home.
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
        yaw_err_pen = yaw_err.clamp(-self.cfg.yaw_error_pen_clamp_rad, self.cfg.yaw_error_pen_clamp_rad)
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
            "yaw_error": -self.cfg.rew_yaw_error * yaw_err_pen.pow(2),
            "pitch": -self.cfg.rew_pitch * pitch.pow(2),
            "pitch_rate": -self.cfg.rew_pitch_rate * pitch_rate_pen.pow(2),
            "roll": -self.cfg.rew_roll * roll.pow(2),
            "roll_rate": -self.cfg.rew_roll_rate * roll_rate_pen.pow(2),
            "current": -self.cfg.rew_current * current.pow(2).sum(dim=1),
            "delta_current": -self.cfg.rew_delta_current * delta_current.pow(2).sum(dim=1),
            "cg_pos": -self.cfg.rew_cg_pos * cg_tanh.pow(2).sum(dim=1),
            "cg_rate": -self.cfg.rew_cg_rate * cg_delta_tanh.pow(2).sum(dim=1),
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
