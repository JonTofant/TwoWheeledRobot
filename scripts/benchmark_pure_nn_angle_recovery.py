#!/usr/bin/env python3
"""
Isaac Sim angle-recovery timeseries logger for the pure NN balance policy.

Purpose:
    Release the robot from rest at a fixed initial pitch angle (no push, no
    payload, no slope — curriculum_stage=1 disables all external disturbance
    kinds) and log the full per-step trajectory into the shared
    timeseries_log_template.csv schema. Writes one CSV per angle, each
    containing --num_envs repeated runs (run_id suffixed _01, _02, ...).
    Isaac Sim only; the MuJoCo/hardware rows in that schema come from
    separate scripts.

obs_0..obs_7 column mapping (RAW physical units, not the network-normalized
values that actually feed the policy):
    obs_0 = x_rel (m), obs_1 = linear_velocity (m/s), obs_2 = pitch (rad),
    obs_3 = pitch_rate (rad/s), obs_4 = yaw_error (rad), obs_5 = yaw_rate
    (rad/s), obs_6 = previous left current command (A), obs_7 = previous
    right current command (A).
    This is PureNNBalanceEnvCfg's actual observation order (see the comment
    above `observation_scale` in pure_nn_balance_env_cfg.py, and
    NormalizedObservationBuilder.build() in pure_nn_components.py) — it does
    not necessarily match the illustrative numbers in
    timeseries_log_template.csv, which look pitch-first. Make sure the
    MuJoCo/hardware loggers use this same order so the isaac/mujoco/hardware
    rows are directly comparable.

Row timing convention:
    Row at t_s=0.00 is the released state before any control has acted
    (action=0, I_cmd=0), matching the template's release-moment row. Each
    subsequent row at t_s=(step+1)*dt pairs the state AFTER that control step
    with the action/current that was applied DURING that step (obs_6/obs_7
    still reflect the current from BEFORE this step, i.e. what the network
    actually saw as input when it produced this row's action).

held is always 0 for isaac: sim releases instantaneously, no physical hold
phase like the hardware rig.

Edit here when:
    You want to change the release angles, episode duration, run count, or
    output CSV schema/location.

Avoid changing here without also checking:
    timeseries_log_template.csv column names/order — every platform's
    logging script (isaac/mujoco/hardware) must populate the same schema.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

CSV_HEADER = [
    "t_s",
    "platform",
    "scenario",
    "run_id",
    "initial_theta_deg",
    "held",
    "obs_0",
    "obs_1",
    "obs_2",
    "obs_3",
    "obs_4",
    "obs_5",
    "obs_6",
    "obs_7",
    "action_L",
    "action_R",
    "I_L_cmd_A",
    "I_R_cmd_A",
    "I_L_meas_A",
    "I_R_meas_A",
]
DEFAULT_ANGLES_DEG = [5.0, 10.0, 15.0, 20.0, 25.0]
DEFAULT_OUTPUT_DIR = Path("outputs/isaac_angle_recovery")

parser = argparse.ArgumentParser(description="Isaac Sim angle-recovery timeseries logger for the pure NN balance policy.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-PureNNBalance-v0")
parser.add_argument(
    "--policy",
    required=True,
    type=Path,
    help="TorchScript actor .pt exported by play.py (the raw actor, NOT the tanh+current-scaled ONNX used on hardware).",
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=4,
    help="Parallel repeated runs logged per angle (run_id _01.._NN); kept small since each one logs a full timeseries.",
)
parser.add_argument("--angles-deg", nargs="+", type=float, default=DEFAULT_ANGLES_DEG, help="Release angle magnitudes to test.")
parser.add_argument("--negative", action="store_true", help="Release at negative angles instead of positive.")
parser.add_argument(
    "--episode-length-s",
    type=float,
    default=None,
    help="Override the task's episode_length_s (default: task's own configured value).",
)
parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
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


def _release_at_angle(env: gym.Env, angle_deg: float) -> None:
    """Force every env to an exact pitch angle at rest, with no external disturbance active."""
    unwrapped = env.unwrapped
    env_ids = torch.arange(unwrapped.num_envs, device=unwrapped.device, dtype=torch.long)
    pitch = torch.full((len(env_ids),), math.radians(angle_deg), device=unwrapped.device)

    root_state = unwrapped.robot.data.default_root_state[env_ids].clone()
    root_state[:, :3] += unwrapped.scene.env_origins[env_ids]
    root_state[:, 2] = unwrapped.scene.env_origins[env_ids, 2] + unwrapped.cfg.spawn_upright_z
    root_state[:, 3] = torch.cos(0.5 * pitch)
    root_state[:, 4] = -torch.sin(0.5 * pitch)
    root_state[:, 5:7] = 0.0
    root_state[:, 7:] = 0.0
    unwrapped.robot.write_root_pose_to_sim(root_state[:, :7], env_ids)
    unwrapped.robot.write_root_velocity_to_sim(root_state[:, 7:], env_ids)
    unwrapped._spawn_pos_xy[env_ids] = root_state[:, :2]
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
    if hasattr(unwrapped, "_fall_counter"):
        unwrapped._fall_counter[env_ids] = 0
        unwrapped._last_fall[env_ids] = False
        unwrapped._last_physics_broken[env_ids] = False
        unwrapped._last_invalid_state[env_ids] = False
        unwrapped._last_timeout[env_ids] = False
        unwrapped._last_terminal_penalty[env_ids] = 0.0
        unwrapped._termination_update_step[env_ids] = -1
    if hasattr(unwrapped, "_last_total_tilt"):
        unwrapped._last_total_tilt[env_ids] = 0.0


def _row(t_s: float, run_id: str, angle_deg: float, raw_obs: list[float], action: list[float], i_cmd: list[float]) -> dict[str, str]:
    row = {
        "t_s": f"{t_s:.4f}",
        "platform": "isaac",
        "scenario": "initial_pitch_release",
        "run_id": run_id,
        "initial_theta_deg": f"{angle_deg:.4f}",
        "held": 0,
        "I_L_meas_A": "",
        "I_R_meas_A": "",
    }
    for i, value in enumerate(raw_obs):
        row[f"obs_{i}"] = f"{value:.6f}"
    row["action_L"], row["action_R"] = f"{action[0]:.6f}", f"{action[1]:.6f}"
    row["I_L_cmd_A"], row["I_R_cmd_A"] = f"{i_cmd[0]:.6f}", f"{i_cmd[1]:.6f}"
    return row


def _run_angle(policy: torch.nn.Module, env: gym.Env, angle_deg: float) -> list[dict[str, str]]:
    unwrapped = env.unwrapped
    env.reset()
    _release_at_angle(env, angle_deg)
    obs = unwrapped._get_observations()["policy"]
    dt = unwrapped.step_dt
    steps = unwrapped.max_episode_length
    num_envs = unwrapped.num_envs

    run_ids = [f"isaac_rel_{abs(angle_deg):g}deg_{i + 1:02d}" for i in range(num_envs)]
    rows_per_env: list[list[dict[str, str]]] = [[] for _ in range(num_envs)]
    alive = torch.ones(num_envs, dtype=torch.bool, device=unwrapped.device)

    # Release-moment row: state as released, before any control action.
    x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = unwrapped._state_terms()
    for i in range(num_envs):
        raw = [x_rel[i].item(), velocity[i].item(), pitch[i].item(), pitch_rate[i].item(), yaw_error[i].item(), yaw_rate[i].item(), 0.0, 0.0]
        rows_per_env[i].append(_row(0.0, run_ids[i], angle_deg, raw, [0.0, 0.0], [0.0, 0.0]))

    for step in range(steps):
        alive_before = alive.clone()
        prev_current = unwrapped._action_processor.command_current.clone()
        # no_grad (not inference_mode): env.step() stores persistent buffers like
        # self._prev_actions/_cur_actions internally. Tensors created under
        # inference_mode become permanently un-mutatable in place outside of it,
        # which breaks the NEXT env.reset() (called for the next angle, reusing
        # this same env) when it tries `self._prev_actions[env_ids] = 0.0`.
        with torch.no_grad():
            action = policy(obs)
            step_out = env.step(action)
        if len(step_out) == 5:
            obs, _, terminated, truncated, _ = step_out
            dones = terminated | truncated
        else:
            obs, _, dones, _ = step_out
        if isinstance(obs, dict):
            obs = obs["policy"]

        x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = unwrapped._state_terms()
        new_current = unwrapped._action_processor.command_current.clone()
        t_s = (step + 1) * dt

        for i in range(num_envs):
            if not bool(alive_before[i]):
                continue  # this env already fell/timed out/auto-reset before this step
            raw = [
                x_rel[i].item(),
                velocity[i].item(),
                pitch[i].item(),
                pitch_rate[i].item(),
                yaw_error[i].item(),
                yaw_rate[i].item(),
                prev_current[i, 0].item(),
                prev_current[i, 1].item(),
            ]
            rows_per_env[i].append(
                _row(t_s, run_ids[i], angle_deg, raw, [action[i, 0].item(), action[i, 1].item()], [new_current[i, 0].item(), new_current[i, 1].item()])
            )

        alive = alive & ~dones

    return [row for env_rows in rows_per_env for row in env_rows]


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    repo_root = Path(__file__).resolve().parents[1]
    output_dir = args_cli.output_dir if args_cli.output_dir.is_absolute() else repo_root / args_cli.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    env_cfg.curriculum_stage = 1  # stage < 3 => DisturbanceGenerator never sets a disturbance kind
    env_cfg.benchmark_disturbance_kind = "none"
    if args_cli.episode_length_s is not None:
        env_cfg.episode_length_s = args_cli.episode_length_s

    policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()

    # Create the scene once and reuse it for every angle (release_at_angle() just
    # repositions/resets the robot). Repeatedly gym.make()/env.close()-ing a fresh
    # scene per angle is slow and can hang — Isaac Lab's stage teardown between
    # sequential env creations in one process is fragile (see the "USD stage detach
    # not called, holding a loose ptr to a stage!" warning it emits on close()).
    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    try:
        for angle_deg in args_cli.angles_deg:
            signed_angle = -angle_deg if args_cli.negative else angle_deg
            rows = _run_angle(policy, env, signed_angle)

            csv_path = output_dir / f"timeseries_isaac_rel_{angle_deg:g}deg.csv"
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
                writer.writeheader()
                writer.writerows(rows)

            print(f"[{angle_deg:g} deg] wrote {csv_path} ({len(rows)} rows across {args_cli.num_envs} runs)")
    finally:
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
