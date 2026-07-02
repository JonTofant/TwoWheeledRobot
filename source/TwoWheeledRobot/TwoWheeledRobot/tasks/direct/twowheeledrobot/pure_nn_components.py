"""Configurable components for pure neural-network balance control."""

from __future__ import annotations

import math

import torch


def pitch_from_projected_gravity(projected_gravity_body: torch.Tensor) -> torch.Tensor:
    return torch.atan2(projected_gravity_body[:, 1], -projected_gravity_body[:, 2])


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


class DisturbanceGenerator:
    """Sample physically motivated sim2real disturbances.

    Disturbances are represented as force and torque events on the selected
    robot body/platform. Components are supplied directly to Isaac Lab's
    ``set_external_force_and_torque`` tensor API without frame conversion. In
    this task convention, +X is the main forward/back push axis, +Y is lateral,
    torque X is the pitch-axis payload/COM-shift equivalent, and torque Z is yaw
    torque from an off-center human push.
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
            self.slope_force[slope_env_ids, 0] = torch.empty(len(slope_env_ids), device=self.device).uniform_(
                *self.cfg.slope_fx_n_range
            )
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
        force[:, 0] = torch.where(
            sine,
            force[:, 0] + self.sine_amp * torch.sin(2.0 * math.pi * self.sine_freq * t),
            force[:, 0],
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
        force[:, 0] = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_fx_n_range)
        force[:, 1] = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_fy_n_range)
        torque[:, 2] = torch.empty(n, device=self.device).uniform_(*self.cfg.human_push_yaw_torque_nm_range)
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
        self.payload_torque[env_ids, 0] = torch.empty(n, device=self.device).uniform_(
            *self.cfg.payload_pitch_torque_nm_range
        )

    def force(self, step: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        """Compatibility helper for older callers; returns only force."""
        force, _ = self.force_and_torque(step, t)
        return force[:, 0]


class BalanceReward:
    def __init__(self, cfg):
        self.cfg = cfg

    def compute(
        self,
        position: torch.Tensor,
        pitch: torch.Tensor,
        pitch_rate: torch.Tensor,
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
