"""Residual PPO controller wrapped around the analytical LQR balancer."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import torch

from .residual_lqr_env_cfg import ResidualLqrEnvCfg
from .sim_params import DDSM115_I_PEAK, DDSM115_KT, DDSM115_NO_LOAD_SPEED, DDSM115_TAU_PEAK
from .standup_env import StandupEnv


@dataclass(frozen=True)
class LqrPhysicalParams:
    body_mass_kg: float
    wheel_cart_mass_kg: float
    wheel_radius_m: float
    body_com_height_m: float
    body_pitch_inertia_kg_m2: float
    wheel_torque_constant_nm_per_a: float
    track_width_m: float
    body_yaw_inertia_kg_m2: float
    no_load_current_a: float = 0.25
    no_load_speed_rpm: float = 200.0
    gravity_m_s2: float = 9.81

    @property
    def wheel_internal_damping_nm_s_rad(self) -> float:
        omega_no_load_rad_s = self.no_load_speed_rpm * 2.0 * math.pi / 60.0
        return self.wheel_torque_constant_nm_per_a * self.no_load_current_a / omega_no_load_rad_s


@dataclass(frozen=True)
class LqrWeights:
    q_position: float
    q_wheel_velocity: float
    q_pitch: float
    q_pitch_rate: float
    r_force: float
    q_yaw: float = 5.0
    q_yaw_rate: float = 0.5
    r_left_current: float = 1.0
    r_right_current: float = 1.0


LQR_CURRENT_SIGN = -1.0
LQR_YAW_SIGN = -1.0
LQR_PHYSICAL_PARAMS = LqrPhysicalParams(
    body_mass_kg=2.6,
    wheel_cart_mass_kg=1.53,
    wheel_radius_m=0.05035,
    body_com_height_m=0.14,
    body_pitch_inertia_kg_m2=0.0129596588636,
    wheel_torque_constant_nm_per_a=0.75,
    track_width_m=0.382999941707,
    body_yaw_inertia_kg_m2=0.0315051945189,
)
LQR_WEIGHTS = LqrWeights(
    q_position=0.0,
    q_wheel_velocity=20.0,
    q_pitch=45.0,
    q_pitch_rate=2.0,
    r_force=1.0,
    q_yaw=10.0,
    q_yaw_rate=0.2,
    r_left_current=1.0,
    r_right_current=1.0,
)
R_WHEEL = LQR_PHYSICAL_PARAMS.wheel_radius_m


def _build_pitch_model(params: LqrPhysicalParams) -> tuple[torch.Tensor, torch.Tensor]:
    m = params.body_mass_kg
    m_cart = params.wheel_cart_mass_kg
    l = params.body_com_height_m
    inertia = params.body_pitch_inertia_kg_m2
    g = params.gravity_m_s2
    b = 2.0 * params.wheel_internal_damping_nm_s_rad / (params.wheel_radius_m**2)

    denominator = inertia * (m_cart + m) + m_cart * m * l**2
    a22 = -((inertia + m * l**2) * b) / denominator
    a23 = (m**2 * g * l**2) / denominator
    a42 = -(m * l * b) / denominator
    a43 = (m * g * l * (m_cart + m)) / denominator
    b2 = (inertia + m * l**2) / denominator
    b4 = (m * l) / denominator

    a = torch.tensor(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, a22, a23, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, a42, a43, 0.0],
        ],
        dtype=torch.float64,
    )
    b_mat = torch.tensor([[0.0], [b2], [0.0], [b4]], dtype=torch.float64)
    return a, b_mat


def _calculate_lqr6_gain(params: LqrPhysicalParams, weights: LqrWeights) -> torch.Tensor:
    # Torch does not expose CARE; use scipy here during env construction only.
    import numpy as np
    from scipy.linalg import solve_continuous_are

    a4_t, b_force_t = _build_pitch_model(params)
    a4 = a4_t.numpy()
    b_force = b_force_t.numpy()
    a6 = np.zeros((6, 6), dtype=float)
    b6 = np.zeros((6, 2), dtype=float)
    a6[0:4, 0:4] = a4

    force_per_current = params.wheel_torque_constant_nm_per_a / params.wheel_radius_m
    pitch_per_current = LQR_CURRENT_SIGN * b_force[:, 0] * force_per_current
    b6[0:4, 0] = pitch_per_current
    b6[0:4, 1] = pitch_per_current

    a6[4, 5] = 1.0
    yaw_gain = params.track_width_m * params.wheel_torque_constant_nm_per_a / (
        2.0 * params.wheel_radius_m * params.body_yaw_inertia_kg_m2
    )
    b6[5, 0] = -LQR_YAW_SIGN * yaw_gain
    b6[5, 1] = LQR_YAW_SIGN * yaw_gain

    q6 = np.diag([
        weights.q_position,
        weights.q_wheel_velocity,
        weights.q_pitch,
        weights.q_pitch_rate,
        weights.q_yaw,
        weights.q_yaw_rate,
    ])
    r6 = np.diag([weights.r_left_current, weights.r_right_current])
    p = solve_continuous_are(a6, b6, q6, r6)
    k6 = np.linalg.solve(r6, b6.T @ p)
    return torch.tensor(k6, dtype=torch.float32)


def _pitch_from_projected_gravity(projected_gravity_body: torch.Tensor) -> torch.Tensor:
    return torch.atan2(projected_gravity_body[:, 1], -projected_gravity_body[:, 2])


def _yaw_from_quat_wxyz(quat_wxyz: torch.Tensor) -> torch.Tensor:
    w, x, y, z = quat_wxyz.unbind(dim=1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle_rad(value: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(value), torch.cos(value))


class ResidualLqrEnv(StandupEnv):
    cfg: ResidualLqrEnvCfg

    def __init__(self, cfg: ResidualLqrEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._k6_lqr = _calculate_lqr6_gain(LQR_PHYSICAL_PARAMS, LQR_WEIGHTS).to(self.device)
        self._wheel_sign = torch.tensor([-1.0, 1.0], device=self.device, dtype=torch.float32)
        self._yaw_reference = torch.zeros(self.num_envs, device=self.device)
        self._prev_rl_current = torch.zeros(self.num_envs, 2, device=self.device)
        self._cur_rl_current = torch.zeros(self.num_envs, 2, device=self.device)
        self._lqr_current = torch.zeros(self.num_envs, 2, device=self.device)
        self._final_current = torch.zeros(self.num_envs, 2, device=self.device)
        self._disturbance_current = torch.zeros(self.num_envs, 2, device=self.device)
        self._disturbance_current_amp = torch.zeros(self.num_envs, device=self.device)
        self._disturbance_start_s = torch.zeros(self.num_envs, device=self.device)
        self._disturbance_start_step = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._disturbance_stop_step = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._episode_reward = torch.zeros(self.num_envs, device=self.device)
        print(
            "[ResidualLqrEnv] LQR active, residual_rl="
            f"{self.cfg.enable_residual_rl}, residual_action_limit={self.cfg.residual_action_limit:.3f} A"
        )

    def _compute_lqr_current(self) -> torch.Tensor:
        proj_grav = self.bno080.data.projected_gravity_b
        raw_pos = self.robot.data.joint_pos[:, self._wheel_ids]
        raw_vel = self.robot.data.joint_vel[:, self._wheel_ids]
        wheel_pos = raw_pos * self._wheel_sign
        wheel_vel = raw_vel * self._wheel_sign

        theta = _pitch_from_projected_gravity(proj_grav)
        theta_dot = -self.bno080.data.ang_vel_b[:, 0]
        wheel_position_m = 0.5 * wheel_pos.sum(dim=1) * R_WHEEL
        wheel_velocity_m_s = 0.5 * wheel_vel.sum(dim=1) * R_WHEEL
        yaw = _wrap_angle_rad(_yaw_from_quat_wxyz(self.robot.data.root_quat_w) - self._yaw_reference)
        yaw_rate = self.robot.data.root_ang_vel_w[:, 2]

        state6 = torch.stack([wheel_position_m, wheel_velocity_m_s, theta, theta_dot, yaw, yaw_rate], dim=1)
        return -(state6 @ self._k6_lqr.T)

    def _sample_disturbance(self, env_ids_t: torch.Tensor) -> None:
        choices = torch.tensor(self.cfg.disturbance_currents_a, device=self.device, dtype=torch.float32)
        choice_ids = torch.randint(len(self.cfg.disturbance_currents_a), (len(env_ids_t),), device=self.device)
        self._disturbance_current_amp[env_ids_t] = choices[choice_ids]
        start_lo, start_hi = self.cfg.disturbance_start_range_s
        starts = torch.empty(len(env_ids_t), device=self.device).uniform_(start_lo, start_hi)
        self._disturbance_start_s[env_ids_t] = starts
        start_steps = torch.ceil(starts / self.step_dt - 1.0e-9).to(torch.long)
        self._disturbance_start_step[env_ids_t] = start_steps
        self._disturbance_stop_step[env_ids_t] = start_steps + int(self.cfg.disturbance_samples)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        actions = actions.clamp(-1.0, 1.0)
        self._enforce_cybergear_joint_state_limits()

        self._prev_actions = self._cur_actions.clone()
        self._cur_actions = actions.clone()
        self._prev_rl_current = self._cur_rl_current.clone()
        if self.cfg.enable_residual_rl:
            self._cur_rl_current = (actions[:, 0:2] * self.cfg.residual_action_limit).clamp(
                -self.cfg.residual_action_limit,
                self.cfg.residual_action_limit,
            )
        else:
            self._cur_rl_current.zero_()

        zero_cg_targets = torch.zeros(self.num_envs, 4, device=self.device)
        zero_cg_targets = torch.max(self._cg_joint_lo, torch.min(self._cg_joint_hi, zero_cg_targets))
        self.robot.set_joint_position_target(zero_cg_targets, joint_ids=self._cg_ids)

        self._lqr_current = self._compute_lqr_current()
        active = (
            (self.episode_length_buf >= self._disturbance_start_step)
            & (self.episode_length_buf < self._disturbance_stop_step)
        )
        self._disturbance_current.zero_()
        self._disturbance_current[:, 0] = torch.where(active, self._disturbance_current_amp, 0.0)
        self._disturbance_current[:, 1] = torch.where(active, -self._disturbance_current_amp, 0.0)

        self._final_current = self._lqr_current + self._cur_rl_current + self._disturbance_current
        self._wheel_i_des = self._final_current
        self._wheel_i_cmd = self._wheel_i_des.clamp(-DDSM115_I_PEAK, DDSM115_I_PEAK)
        self._wheel_tau_current = self._wheel_i_cmd * DDSM115_KT
        self._wheel_velocity_raw = self.robot.data.joint_vel[:, self._wheel_ids].clone()
        self._wheel_velocity_used = self._wheel_velocity_raw.clone()
        self._wheel_omega_for_limiter = self._wheel_velocity_used.abs()
        self._wheel_tau_speed_limit = DDSM115_TAU_PEAK * (1.0 - self._wheel_omega_for_limiter / DDSM115_NO_LOAD_SPEED)
        self._wheel_tau_speed_limit = self._wheel_tau_speed_limit.clamp(0.0, DDSM115_TAU_PEAK)
        self._wheel_torque_cmd = torch.maximum(
            -self._wheel_tau_speed_limit,
            torch.minimum(self._wheel_tau_current, self._wheel_tau_speed_limit),
        )

        self._efforts_buf[:, 0] = -self._wheel_torque_cmd[:, 0]
        self._efforts_buf[:, 1] = self._wheel_torque_cmd[:, 1]
        self.robot.set_joint_effort_target(self._efforts_buf, joint_ids=self._wheel_ids)

    def _get_observations(self) -> dict:
        self._enforce_cybergear_joint_state_limits()
        raw_vel = self.robot.data.joint_vel[:, self._wheel_ids]
        wheel_vel = raw_vel * self._wheel_sign
        pitch = _pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)
        pitch_rate = -self.bno080.data.ang_vel_b[:, 0]
        base_position = 0.5 * self.robot.data.joint_pos[:, self._wheel_ids].mul(self._wheel_sign).sum(dim=1) * R_WHEEL
        base_velocity = 0.5 * wheel_vel.sum(dim=1) * R_WHEEL
        prev_rl_norm = self._prev_rl_current / max(self.cfg.residual_action_limit, 1.0e-6)
        obs = torch.cat(
            [
                pitch.unsqueeze(1),
                pitch_rate.unsqueeze(1),
                base_position.unsqueeze(1),
                base_velocity.unsqueeze(1),
                wheel_vel / self.cfg.wheel_velocity_norm,
                prev_rl_norm,
            ],
            dim=1,
        )
        return {"policy": torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0).clamp(-10.0, 10.0)}

    def _get_rewards(self) -> torch.Tensor:
        pitch = _pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)
        pitch_rate = -self.bno080.data.ang_vel_b[:, 0]
        position = 0.5 * self.robot.data.joint_pos[:, self._wheel_ids].mul(self._wheel_sign).sum(dim=1) * R_WHEEL
        rl_action_sq = self._cur_rl_current.pow(2).sum(dim=1)
        reward = (
            -pitch.pow(2)
            -0.1 * pitch_rate.pow(2)
            -0.05 * position.pow(2)
            -0.01 * rl_action_sq
        )
        reward = torch.nan_to_num(reward, nan=0.0, posinf=0.0, neginf=-10.0).clamp(-10.0, 0.0)
        self._episode_reward += reward
        self.extras["log"] = {
            "reward": reward.mean(),
            "episode_reward": self._episode_reward.mean(),
            "u_left_lqr": self._lqr_current[:, 0].mean(),
            "u_right_lqr": self._lqr_current[:, 1].mean(),
            "u_left_rl": self._cur_rl_current[:, 0].mean(),
            "u_right_rl": self._cur_rl_current[:, 1].mean(),
            "u_left_final": self._final_current[:, 0].mean(),
            "u_right_final": self._final_current[:, 1].mean(),
            "disturbance_current": self._disturbance_current_amp.mean(),
            "disturbance_start": self._disturbance_start_s.mean(),
        }
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        pitch = _pitch_from_projected_gravity(self.bno080.data.projected_gravity_b)
        body_z = torch.nan_to_num(self.robot.data.root_pos_w[:, 2], nan=-999.0)
        fallen = pitch.abs() > math.radians(self.cfg.floor_stop_pitch_deg)
        physics_broken = body_z < -1.0
        terminated = fallen | physics_broken
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, timeout

    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if not hasattr(self, "_yaw_reference"):
            return
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        env_ids_t = env_ids if isinstance(env_ids, torch.Tensor) else torch.tensor(env_ids, device=self.device, dtype=torch.long)

        pitch_rad = math.radians(self.cfg.floor_initial_pitch_deg)
        root_state = self.robot.data.default_root_state[env_ids_t].clone()
        root_state[:, :3] += self.scene.env_origins[env_ids_t]
        root_state[:, 2] = self.scene.env_origins[env_ids_t, 2] + self.cfg.spawn_upright_z
        root_state[:, 3] = math.cos(0.5 * pitch_rad)
        root_state[:, 4] = -math.sin(0.5 * pitch_rad)
        root_state[:, 5:7] = 0.0
        root_state[:, 7:] = 0.0
        self.robot.write_root_pose_to_sim(root_state[:, :7], env_ids_t)
        self.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids_t)
        self._spawn_pos_xy[env_ids_t] = root_state[:, :2]
        self._yaw_reference[env_ids_t] = _yaw_from_quat_wxyz(root_state[:, 3:7])

        joint_pos = self.robot.data.default_joint_pos[env_ids_t].clone()
        joint_vel = self.robot.data.default_joint_vel[env_ids_t].clone()
        joint_pos[:, self._cg_ids] = 0.0
        joint_vel[:, self._cg_ids] = 0.0
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids_t)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids_t)

        self._prev_rl_current[env_ids_t] = 0.0
        self._cur_rl_current[env_ids_t] = 0.0
        self._lqr_current[env_ids_t] = 0.0
        self._final_current[env_ids_t] = 0.0
        self._disturbance_current[env_ids_t] = 0.0
        self._episode_reward[env_ids_t] = 0.0
        self._sample_disturbance(env_ids_t)
