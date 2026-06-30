#!/usr/bin/env python3
"""Report the learned pure-NN balance sign convention from the current sim code.

This script does not train and does not change the environment implementation. It
creates a deterministic one-env test instance, applies fixed raw NN action values,
and optionally loads the exported TorchScript policy to inspect its initial
outputs for positive and negative pitch errors.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_EXTENSION_SOURCE_PATH = _REPO_ROOT / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))


def _latest_policy() -> Path | None:
    log_root = _REPO_ROOT / "logs" / "rsl_rl" / "pure_nn_balance_two_wheel"
    if not log_root.is_dir():
        return None
    runs = sorted((p for p in log_root.iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime, reverse=True)
    for run in runs:
        policy = run / "exported" / "policy.pt"
        if policy.is_file():
            return policy
    return None


_DEFAULT_POLICY = _latest_policy()

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Determine pure NN action/observation sign conventions.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-PureNNBalance-v0")
parser.add_argument("--policy", type=Path, default=_DEFAULT_POLICY, help="Exported TorchScript policy.pt to inspect.")
parser.add_argument("--steps", type=int, default=50, help="Steps per direct action pattern; 50 steps = 1 s at 50 Hz.")
parser.add_argument("--action-mag", type=float, default=0.3, help="Raw NN action magnitude to test before tanh.")
parser.add_argument("--pitch-deg", type=float, default=5.0, help="Pitch perturbation for policy behavior tests.")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg, multi_agent_to_single_agent
from isaaclab_tasks.utils.hydra import hydra_task_config

import isaaclab_tasks  # noqa: F401
import TwoWheeledRobot.tasks  # noqa: F401
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import yaw_from_quat_wxyz
from TwoWheeledRobot.tasks.direct.twowheeledrobot.residual_lqr_env import LQR_PHYSICAL_PARAMS, R_WHEEL


def _mean(value: torch.Tensor) -> float:
    return value.detach().mean().item()


def _sign_name(value: float, tol: float = 1.0e-4) -> str:
    if value > tol:
        return "+"
    if value < -tol:
        return "-"
    return "0"


def _classify(v_forward: float, yaw_rate: float, tol_v: float = 1.0e-3, tol_yaw: float = 1.0e-2) -> str:
    if abs(v_forward) >= abs(yaw_rate) * 0.02 and abs(v_forward) > tol_v:
        return "forward" if v_forward > 0.0 else "backward"
    if abs(yaw_rate) > tol_yaw:
        return "yaw-left/CCW" if yaw_rate > 0.0 else "yaw-right/CW"
    return "near-zero/mixed"


def _state(unwrapped):
    x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = unwrapped._state_terms()
    wheel_vel_raw = unwrapped.robot.data.joint_vel[:, unwrapped._wheel_ids]
    wheel_vel_nn = wheel_vel_raw * unwrapped._wheel_sign
    return {
        "x_rel": x_rel,
        "v_forward": velocity,
        "pitch": pitch,
        "pitch_rate": pitch_rate,
        "yaw_error": yaw_error,
        "yaw_rate": yaw_rate,
        "left_wheel_vel_nn": wheel_vel_nn[:, 0],
        "right_wheel_vel_nn": wheel_vel_nn[:, 1],
        "left_wheel_vel_raw": wheel_vel_raw[:, 0],
        "right_wheel_vel_raw": wheel_vel_raw[:, 1],
    }


def _set_pose_and_reset_buffers(env, pitch_rad: float = 0.0) -> None:
    unwrapped = env.unwrapped
    env_ids = torch.arange(unwrapped.num_envs, device=unwrapped.device, dtype=torch.long)
    root_state = unwrapped.robot.data.default_root_state[env_ids].clone()
    root_state[:, :3] += unwrapped.scene.env_origins[env_ids]
    root_state[:, 2] = unwrapped.scene.env_origins[env_ids, 2] + unwrapped.cfg.spawn_upright_z
    root_state[:, 3] = math.cos(0.5 * pitch_rad)
    root_state[:, 4] = -math.sin(0.5 * pitch_rad)
    root_state[:, 5:7] = 0.0
    root_state[:, 7:] = 0.0
    unwrapped.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
    unwrapped.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)
    unwrapped._yaw_reference[env_ids] = yaw_from_quat_wxyz(root_state[:, 3:7])

    joint_pos = unwrapped.robot.data.default_joint_pos[env_ids].clone()
    joint_vel = unwrapped.robot.data.default_joint_vel[env_ids].clone()
    joint_pos[:, unwrapped._cg_ids] = 0.0
    joint_vel[:, unwrapped._cg_ids] = 0.0
    joint_vel[:, unwrapped._wheel_ids] = 0.0
    unwrapped.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
    unwrapped.robot.set_joint_position_target(joint_pos, env_ids=env_ids)

    unwrapped._prev_actions[env_ids] = 0.0
    unwrapped._cur_actions[env_ids] = 0.0
    unwrapped._obs_now[env_ids] = 0.0
    unwrapped._obs_delay[env_ids] = 0.0
    unwrapped._obs_delay_samples[env_ids] = 0
    unwrapped._pitch_bias[env_ids] = 0.0
    unwrapped._action_processor.reset(env_ids)
    unwrapped._action_processor.action_delay_samples[env_ids] = 0


def _step_action(env, action_pair: tuple[float, float], steps: int):
    unwrapped = env.unwrapped
    action = torch.tensor(action_pair, device=unwrapped.device, dtype=torch.float32).view(1, 2).repeat(unwrapped.num_envs, 1)
    obs = None
    for _ in range(steps):
        out = env.step(action)
        obs = out[0]["policy"] if isinstance(out[0], dict) else out[0]
    return obs, _state(unwrapped)


def _print_action_trace(unwrapped) -> None:
    print("\nACTION PATH FROM CODE")
    print("Stage                         Left sign effect                 Right sign effect")
    print("policy action                 action[0]                        action[1]")
    print("after tanh/current scaling    tanh(action[0]) * i_max          tanh(action[1]) * i_max")
    print("after action processing       same sign unless random bias      same sign unless random bias")
    print("after torque conversion       command_current[0] * Kt           command_current[1] * Kt")
    print("sim joint effort              -torque_left                     +torque_right")
    print("NN wheel velocity sign        raw_left * -1                    raw_right * +1")
    print(f"wheel_sign tensor             {unwrapped._wheel_sign.detach().cpu().tolist()}")
    print(f"wheel_radius_m                {R_WHEEL:.8f}")
    print(f"track_width_m                 {LQR_PHYSICAL_PARAMS.track_width_m:.12f}")


def _print_observation_trace(unwrapped) -> None:
    print("\nOBSERVATION PATH FROM CODE")
    print("obs | name             | source/formula before normalization                         | scale")
    scales = unwrapped.cfg.observation_scale
    rows = [
        (0, "x_rel", "0.5 * ((joint_pos_L * -1) + (joint_pos_R * +1)) * R_WHEEL"),
        (1, "v_forward", "0.5 * ((joint_vel_L * -1) + (joint_vel_R * +1)) * R_WHEEL"),
        (2, "pitch", "atan2(projected_gravity_b.y, -projected_gravity_b.z) + pitch_bias + noise"),
        (3, "pitch_rate", "-bno080.ang_vel_b.x + noise"),
        (4, "yaw_error", "wrap(yaw_from_root_quat_wxyz(root_quat_w) - yaw_reference)"),
        (5, "yaw_rate", "root_ang_vel_w.z"),
        (6, "prev_left_current", "action_processor.command_current[0]"),
        (7, "prev_right_current", "action_processor.command_current[1]"),
    ]
    for idx, name, formula in rows:
        print(f"{idx:>3} | {name:<16} | {formula:<67} | / {scales[idx]}")


def _direct_action_tests(env) -> None:
    unwrapped = env.unwrapped
    mag = args_cli.action_mag
    patterns = [(+mag, +mag), (+mag, -mag), (-mag, +mag), (-mag, -mag)]
    print("\nDIRECT ACTION SIGN TESTS")
    print(
        "pattern      net_current_L net_current_R cmd_current_L cmd_current_R "
        "wheel_L_nn wheel_R_nn v_forward yaw_rate pitch pitch_rate result"
    )
    for pattern in patterns:
        _set_pose_and_reset_buffers(env, 0.0)
        obs, state = _step_action(env, pattern, args_cli.steps)
        proc = unwrapped._action_processor
        result = _classify(_mean(state["v_forward"]), _mean(state["yaw_rate"]))
        print(
            f"[{pattern[0]:+.1f},{pattern[1]:+.1f}] "
            f"{_mean(proc.net_current[:, 0]):+.6f} {_mean(proc.net_current[:, 1]):+.6f} "
            f"{_mean(proc.command_current[:, 0]):+.6f} {_mean(proc.command_current[:, 1]):+.6f} "
            f"{_mean(state['left_wheel_vel_nn']):+.6f} {_mean(state['right_wheel_vel_nn']):+.6f} "
            f"{_mean(state['v_forward']):+.6f} {_mean(state['yaw_rate']):+.6f} "
            f"{_mean(state['pitch']):+.6f} {_mean(state['pitch_rate']):+.6f} {result}"
        )


def _previous_current_test(env) -> None:
    unwrapped = env.unwrapped
    pattern = (args_cli.action_mag, -args_cli.action_mag)
    _set_pose_and_reset_buffers(env, 0.0)
    obs, _ = _step_action(env, pattern, 1)
    proc = unwrapped._action_processor
    scale = torch.tensor(unwrapped.cfg.observation_scale, device=unwrapped.device, dtype=torch.float32)
    unnormalized_obs = obs * scale.view(1, -1)
    print("\nPREVIOUS-CURRENT OBSERVATION TEST")
    print(f"input raw action                [{pattern[0]:+.6f}, {pattern[1]:+.6f}]")
    print(f"net_current=tanh(action)*i_max  [{_mean(proc.net_current[:, 0]):+.6f}, {_mean(proc.net_current[:, 1]):+.6f}]")
    print(f"filtered_current                [{_mean(proc.filtered_current[:, 0]):+.6f}, {_mean(proc.filtered_current[:, 1]):+.6f}]")
    print(f"command_current                 [{_mean(proc.command_current[:, 0]):+.6f}, {_mean(proc.command_current[:, 1]):+.6f}]")
    print(f"obs[6:8] normalized             [{_mean(obs[:, 6]):+.6f}, {_mean(obs[:, 7]):+.6f}]")
    print(f"obs[6:8] unnormalized           [{_mean(unnormalized_obs[:, 6]):+.6f}, {_mean(unnormalized_obs[:, 7]):+.6f}]")
    print("obs[6] and obs[7] are command_current before the left joint-effort sign inversion.")


def _pitch_and_yaw_tests(env) -> None:
    print("\nPITCH AND YAW SIGN TESTS")
    _set_pose_and_reset_buffers(env, math.radians(args_cli.pitch_deg))
    _, fwd_pitch = _step_action(env, (0.0, 0.0), 1)
    print(f"forward lean +{args_cli.pitch_deg:.1f} deg -> pitch obs sign {_sign_name(_mean(fwd_pitch['pitch']))}, pitch={_mean(fwd_pitch['pitch']):+.6f}")

    _set_pose_and_reset_buffers(env, 0.0)
    _, ccw = _step_action(env, (-args_cli.action_mag, +args_cli.action_mag), args_cli.steps)
    print(
        "[-,+] yaw test -> "
        f"yaw_error={_mean(ccw['yaw_error']):+.6f}, yaw_rate={_mean(ccw['yaw_rate']):+.6f}, "
        f"classification={_classify(_mean(ccw['v_forward']), _mean(ccw['yaw_rate']))}"
    )
    _set_pose_and_reset_buffers(env, 0.0)
    _, cw = _step_action(env, (+args_cli.action_mag, -args_cli.action_mag), args_cli.steps)
    print(
        "[+,-] yaw test -> "
        f"yaw_error={_mean(cw['yaw_error']):+.6f}, yaw_rate={_mean(cw['yaw_rate']):+.6f}, "
        f"classification={_classify(_mean(cw['v_forward']), _mean(cw['yaw_rate']))}"
    )


def _policy_pitch_tests(env) -> None:
    if args_cli.policy is None or not args_cli.policy.is_file():
        print("\nTRAINED POLICY TESTS")
        print("No policy.pt found or provided. Pass --policy /path/to/exported/policy.pt to run policy behavior tests.")
        return

    unwrapped = env.unwrapped
    policy = torch.jit.load(str(args_cli.policy), map_location=unwrapped.device).eval()
    print("\nTRAINED POLICY TESTS")
    print(f"policy: {args_cli.policy}")
    print("pitch_case obs[0:8 unnormalized] raw_policy_action current_after_tanh processed_command_current wheel_L_nn wheel_R_nn")
    scale = torch.tensor(unwrapped.cfg.observation_scale, device=unwrapped.device, dtype=torch.float32)
    for label, pitch_rad in (("theta>0", math.radians(args_cli.pitch_deg)), ("theta<0", math.radians(-args_cli.pitch_deg))):
        _set_pose_and_reset_buffers(env, pitch_rad)
        obs_dict = env.unwrapped._get_observations()
        obs = obs_dict["policy"]
        with torch.inference_mode():
            action = policy(obs)
        env.step(action)
        state = _state(unwrapped)
        current_after_tanh = torch.tanh(action[:, :2]) * unwrapped.cfg.i_max_a
        proc = unwrapped._action_processor
        obs_un = obs * scale.view(1, -1)
        print(
            f"{label:<8} "
            f"{[round(v, 6) for v in obs_un[0].detach().cpu().tolist()]} "
            f"{[round(v, 6) for v in action[0, :2].detach().cpu().tolist()]} "
            f"{[round(v, 6) for v in current_after_tanh[0].detach().cpu().tolist()]} "
            f"{[round(v, 6) for v in proc.command_current[0].detach().cpu().tolist()]} "
            f"{_mean(state['left_wheel_vel_nn']):+.6f} {_mean(state['right_wheel_vel_nn']):+.6f}"
        )


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    env_cfg.scene.num_envs = 1
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    env_cfg.curriculum_stage = 5
    env_cfg.training_mode = False
    env_cfg.evaluation_mode = True
    env_cfg.pitch_bias_rad_range = (0.0, 0.0)
    env_cfg.pitch_noise_std = 0.0
    env_cfg.pitch_rate_noise_std = 0.0
    env_cfg.velocity_noise_std = 0.0
    env_cfg.action_smoothing_alpha = 0.0
    env_cfg.enable_current_slew_limit = False
    env_cfg.hardware_safe_current_slew_limit = False
    env_cfg.motor_gain_range = (1.0, 1.0)
    env_cfg.motor_deadzone_a_range = (0.0, 0.0)
    env_cfg.motor_bias_a_range = (0.0, 0.0)
    env_cfg.motor_tau_s_range = (env_cfg.sim.dt * env_cfg.decimation, env_cfg.sim.dt * env_cfg.decimation)
    env_cfg.motor_current_limit_a_range = (env_cfg.i_max_a, env_cfg.i_max_a)
    env_cfg.benchmark_disturbance_kind = "none"

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env.reset()
    unwrapped = env.unwrapped

    print("PURE NN SIGN CONVENTION REPORT")
    print(f"task: {args_cli.task}")
    print(f"policy default/found: {args_cli.policy}")
    print(f"i_max_a: {unwrapped.cfg.i_max_a}")
    _print_action_trace(unwrapped)
    _print_observation_trace(unwrapped)
    _direct_action_tests(env)
    _previous_current_test(env)
    _pitch_and_yaw_tests(env)
    _policy_pitch_tests(env)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
