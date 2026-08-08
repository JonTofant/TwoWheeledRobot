#!/usr/bin/env python3
"""Run the four hardware-comparable measurement protocols against a frozen NNDrive checkpoint.

Purpose:
    Produce per-control-step CSV traces in the shared ``timeseries_log_template.csv``
    schema so that Isaac, MuJoCo and the physical rig reduce through one code
    path. Every protocol below is reproducible on hardware with the same
    joystick/hold rig, which is why the CSV carries raw physical units rather
    than the network-normalized observation the policy actually consumes.

Protocols (``--protocols``):
    angle_recovery      Release from rest at an exact pitch angle, zero command.
                        Scenario name ``initial_pitch_release``. One CSV per angle.
    station_keeping     Zero command, ordinary randomized spawn. This is the
                        correctness anchor: rms_pitch must land near 0.5 deg for
                        the nn_drive_revert seed42 model_599 checkpoint.
    velocity_step       Zero command for ``cmd_settle_s`` then a slew-limited ramp
                        to ``--velocity-mps``. The env produces this step itself;
                        unlike benchmark_nn_drive.py, which discards the settle
                        window as a transient, the whole window is logged here
                        because the step response IS the measurement.
    payload_disturbance A deterministic persistent pitch torque switched on at a
                        fixed time — the sim analogue of hanging a known mass at a
                        known lever arm (tau = m * g * offset).

obs_0..obs_11 column mapping (RAW physical units, matching
``DriveObservationBuilder``'s pre-scale layout for the 12-wide fixed-stance
vector; see its docstring in pure_nn_components.py):
    obs_0  pos_err (m, clamped to +-cmd_pos_err_clamp_m)
    obs_1  velocity (m/s, wheel odometry)
    obs_2  pitch (rad)
    obs_3  pitch_rate (rad/s)
    obs_4  yaw_err (rad, wrapped and clamped)
    obs_5  yaw_rate (rad/s)
    obs_6  velocity_cmd (m/s)
    obs_7  yaw_rate_cmd (rad/s)
    obs_8  previous left wheel current (A)
    obs_9  previous right wheel current (A)
    obs_10 roll (rad)
    obs_11 roll_rate (rad/s)
These are ground-truth values read from ``_state_terms()`` / the IMU, NOT the
biased+noised values fed to the network — the same choice the published
angle-recovery logger made, so the isaac rows stay directly comparable to a
hardware trace that has its own, different sensor errors.

Current columns:
    I_*_cmd_A   ``CurrentActionProcessor.net_current`` = tanh(action) * i_max_a.
                Exactly what the STM32 firmware computes from the same action,
                so this column means the same thing on both platforms.
    I_*_meas_A  ``CurrentActionProcessor.command_current`` = the current actually
                delivered after the per-unit motor model (gain, deadzone, bias,
                current-loop lag). The sim analogue of the motor's reported
                current. Note obs_8/obs_9 carry this same quantity from the
                PREVIOUS step, because that is what the network is fed.

Row timing convention:
    The row at ``t_s = 0`` is the state before any control has acted
    (action = 0, currents = 0). Each later row at ``t_s = (step + 1) * dt`` pairs
    the state AFTER that control step with the action/current applied DURING it.
    Rows stop for an environment once it terminates; ``held`` is always 0
    (sim releases instantaneously, there is no physical hold phase).

Edit here when:
    You want new protocols, different release angles, or a different run length.

Avoid changing here without also checking:
    ``timeseries_log_template.csv`` column names/order, and the MuJoCo/hardware
    loggers that must populate the identical schema.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

PROTOCOLS = ("angle_recovery", "station_keeping", "velocity_step", "payload_disturbance")
CSV_HEADER = [
    "t_s",
    "platform",
    "scenario",
    "run_id",
    "initial_theta_deg",
    "held",
    *[f"obs_{i}" for i in range(12)],
    "action_L",
    "action_R",
    "I_L_cmd_A",
    "I_R_cmd_A",
    "I_L_meas_A",
    "I_R_meas_A",
]

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDriveFixedStance-v0")
policy_group = parser.add_mutually_exclusive_group(required=True)
policy_group.add_argument("--policy", type=Path, help="TorchScript actor exported by play.py")
policy_group.add_argument("--checkpoint", type=Path, help="Raw RSL-RL model_*.pt checkpoint")
parser.add_argument("--num_envs", type=int, default=64, help="Parallel repeated runs, logged as run_id _01.._NN")
parser.add_argument("--duration-s", type=float, default=15.0, help="Logged duration per run")
parser.add_argument("--protocols", nargs="+", choices=PROTOCOLS, default=list(PROTOCOLS))
parser.add_argument("--angles-deg", nargs="+", type=float, default=[5.0, 10.0, 15.0])
parser.add_argument("--negative", action="store_true", help="Release at negative pitch angles instead of positive")
parser.add_argument("--velocity-mps", type=float, default=0.30, help="velocity_step target speed")
parser.add_argument(
    "--payload-torque-nm",
    type=float,
    default=0.4,
    help="payload_disturbance pitch torque in N*m; on the bench this is m * g * lever_arm",
)
parser.add_argument("--payload-start-s", type=float, default=3.0, help="payload_disturbance onset time")
parser.add_argument("--curriculum-stage", type=int, default=1, help="Stages 1-2 are guaranteed disturbance-free")
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--output-dir", type=Path, default=Path("outputs/protocol_bench"))
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math
import random

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from rsl_rl.runners import OnPolicyRunner
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import (
    roll_from_projected_gravity,
    yaw_from_quat_wxyz,
)

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config

RAD2DEG = 180.0 / math.pi


def _release_at_angle(env: gym.Env, angle_deg: float) -> None:
    """Force every env to an exact pitch angle at rest, with no external disturbance active.

    Ported unchanged in intent from the published
    ``benchmark_pure_nn_angle_recovery.py`` (branch feature/ERK2026), with the
    NNDrive command references re-anchored as well: ``CommandGenerator`` holds
    ``pos_ref``/``yaw_ref`` integrators that must restart from the released pose
    or the policy sees a phantom tracking error at t = 0.

    ``_pitch_bias`` is zeroed on purpose: the protocol's whole point is that the
    released angle is exactly the angle the policy measures. The roll/gyro/odometry
    biases are deliberately left randomized — they are in-distribution nuisance
    parameters, not the quantity under test.
    """
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
    yaw = yaw_from_quat_wxyz(root_state[:, 3:7])
    unwrapped._yaw_reference[env_ids] = yaw

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
    # NNDrive-only state the balance-task script had no equivalent of.
    unwrapped._commands.reset(env_ids, unwrapped.cfg.curriculum_stage, unwrapped.step_dt, yaw)
    unwrapped._cg_processor.reset(env_ids)
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


def _attitude(unwrapped) -> tuple[torch.Tensor, torch.Tensor]:
    roll = roll_from_projected_gravity(unwrapped.bno080.data.projected_gravity_b)
    roll_rate = unwrapped.bno080.data.ang_vel_b[:, 1]
    return roll, roll_rate


_VALUE_COLUMNS = [
    *[f"obs_{i}" for i in range(12)],
    "action_L",
    "action_R",
    "I_L_cmd_A",
    "I_R_cmd_A",
    "I_L_meas_A",
    "I_R_meas_A",
]


def _row(t_s: float, scenario: str, run_id: str, theta_deg: float, values: list[float]) -> dict[str, str]:
    row = {
        "t_s": f"{t_s:.4f}",
        "platform": "isaac",
        "scenario": scenario,
        "run_id": run_id,
        "initial_theta_deg": f"{theta_deg:.4f}",
        "held": "0",
    }
    for name, value in zip(_VALUE_COLUMNS, values):
        row[name] = f"{value:.6f}"
    return row


def _sample_block(
    unwrapped,
    prev_current: torch.Tensor,
    actions: torch.Tensor | None,
    state: tuple[torch.Tensor, ...],
) -> list[list[float]]:
    """One (num_envs, 18) block of loggable values, moved to host in a single sync.

    Per-element ``.item()`` here would cost a GPU synchronization per number —
    tens of millions across a full run — so everything is stacked on device and
    transferred once per control step.
    """
    x_rel, velocity, pitch, pitch_rate, yaw_error, yaw_rate = state
    roll, roll_rate = _attitude(unwrapped)
    zeros = torch.zeros_like(velocity)
    if actions is None:
        action_l, action_r = zeros, zeros
        i_cmd = torch.zeros_like(prev_current)
        i_meas = torch.zeros_like(prev_current)
    else:
        action_l, action_r = actions[:, 0], actions[:, 1]
        i_cmd = unwrapped._action_processor.net_current
        i_meas = unwrapped._action_processor.command_current
    block = torch.stack(
        [
            unwrapped._commands.position_error(x_rel),
            velocity,
            pitch,
            pitch_rate,
            yaw_error,
            yaw_rate,
            unwrapped._commands.v_cmd,
            unwrapped._commands.w_cmd,
            prev_current[:, 0],
            prev_current[:, 1],
            roll,
            roll_rate,
            action_l,
            action_r,
            i_cmd[:, 0],
            i_cmd[:, 1],
            i_meas[:, 0],
            i_meas[:, 1],
        ],
        dim=1,
    )
    return block.float().cpu().tolist()


def _run_protocol(policy, env, scenario: str, run_prefix: str, steps: int, settle_steps: int, release_deg=None):
    """Log one protocol run across all envs and return (rows, per-env summary metrics)."""
    unwrapped = env.unwrapped
    num_envs = unwrapped.num_envs
    device = unwrapped.device
    dt = unwrapped.step_dt

    reset_result = env.reset()
    obs = reset_result[0] if isinstance(reset_result, tuple) else reset_result
    if release_deg is not None:
        _release_at_angle(env, release_deg)
        # Re-read through the wrapper when there is one: RslRlVecEnvWrapper hands
        # the policy a TensorDict of observation groups, and the raw dict from
        # unwrapped._get_observations() is not interchangeable with it.
        obs = env.get_observations() if hasattr(env, "get_observations") else unwrapped._get_observations()
    if isinstance(obs, dict):
        obs = obs["policy"]

    run_ids = [f"{run_prefix}_{i + 1:02d}" for i in range(num_envs)]
    rows_per_env: list[list[dict[str, str]]] = [[] for _ in range(num_envs)]
    alive = torch.ones(num_envs, dtype=torch.bool, device=device)

    # t = 0 row. After _release_at_angle the IMU cache is stale: the sensor is
    # only refreshed by a real simulation step, not by write_root_pose_to_sim, so
    # the known commanded attitude is substituted for that one row. Wheel
    # odometry, yaw and the command references come off the articulation and the
    # command generator directly, so they are already correct.
    zero_current = torch.zeros(num_envs, 2, device=device)
    block = _sample_block(unwrapped, zero_current, None, unwrapped._state_terms())
    if release_deg is not None:
        for values in block:
            values[2], values[3], values[10], values[11] = math.radians(release_deg), 0.0, 0.0, 0.0
        theta = [release_deg] * num_envs
    else:
        theta = [values[2] * RAD2DEG for values in block]
    for i in range(num_envs):
        rows_per_env[i].append(_row(0.0, scenario, run_ids[i], theta[i], block[i]))

    fall = torch.zeros(num_envs, dtype=torch.bool, device=device)
    survival = torch.zeros(num_envs, device=device)
    pitch_sq_sum = torch.zeros(num_envs, device=device)
    roll_sq_sum = torch.zeros(num_envs, device=device)
    speed_abs_sum = torch.zeros(num_envs, device=device)
    velocity_sum = torch.zeros(num_envs, device=device)
    sample_count = torch.zeros(num_envs, device=device)

    for step in range(steps):
        alive_before = alive.clone()
        prev_current = unwrapped._action_processor.command_current.clone()
        actions = policy(obs)
        step_out = env.step(actions)
        if len(step_out) == 5:
            obs, _, terminated, truncated, _ = step_out
            dones = terminated | truncated
        else:
            obs, _, dones, _ = step_out
        dones = dones.view(-1).to(device=device, dtype=torch.bool)
        if isinstance(obs, dict):
            obs = obs["policy"]

        state = unwrapped._state_terms()
        _, velocity, pitch, _, _, _ = state
        roll, _ = _attitude(unwrapped)
        block = _sample_block(unwrapped, prev_current, actions, state)
        t_s = (step + 1) * dt

        alive_before_host = alive_before.cpu().tolist()
        for i in range(num_envs):
            if not alive_before_host[i]:
                continue
            rows_per_env[i].append(_row(t_s, scenario, run_ids[i], theta[i], block[i]))

        # Metrics use the same alive-masking and settle window as
        # benchmark_nn_drive.py, so the printed rms_pitch_deg is directly
        # comparable with the checkpoint's recorded benchmark JSON.
        alive_after = alive & ~dones
        if step >= settle_steps:
            zeros = torch.zeros_like(velocity)
            sample_count += alive_after.float()
            pitch_sq_sum += torch.where(alive_after, pitch.pow(2), zeros)
            roll_sq_sum += torch.where(alive_after, roll.pow(2), zeros)
            speed_abs_sum += torch.where(alive_after, velocity.abs(), zeros)
            velocity_sum += torch.where(alive_after, velocity, zeros)
        survival = torch.where(alive, torch.full_like(survival, t_s), survival)
        fall |= alive & unwrapped._last_step_fall
        alive &= ~dones

    sampled = sample_count > 0
    denom = sample_count.clamp_min(1.0)

    def sampled_mean(values: torch.Tensor) -> float:
        return values[sampled].mean().item() if bool(torch.any(sampled)) else 0.0

    metrics = {
        "fall_rate": fall.float().mean().item(),
        "survival_time_s": survival.mean().item(),
        "rms_pitch_deg": sampled_mean(torch.sqrt(pitch_sq_sum / denom)) * RAD2DEG,
        "rms_roll_deg": sampled_mean(torch.sqrt(roll_sq_sum / denom)) * RAD2DEG,
        "achieved_speed_mps": sampled_mean(speed_abs_sum / denom),
        "mean_velocity_mps": sampled_mean(velocity_sum / denom),
    }
    return [row for env_rows in rows_per_env for row in env_rows], metrics


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(rows)


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args_cli.seed)

    repo_root = Path(__file__).resolve().parents[1]
    output_dir = args_cli.output_dir
    if not output_dir.is_absolute():
        output_dir = repo_root / output_dir

    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.forced_command_mode = "fixed"
    env_cfg.forced_velocity_cmd_mps = 0.0
    env_cfg.forced_yaw_rate_cmd_radps = 0.0
    env_cfg.curriculum_stage = args_cli.curriculum_stage
    env_cfg.benchmark_disturbance_kind = "none"
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    # Built once. env.close() tears down the whole simulation app in this Isaac
    # build, so protocols are switched by mutating the live cfg, which every
    # env.reset() re-reads.
    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    if args_cli.checkpoint is not None:
        agent_cfg.seed = args_cli.seed
        agent_cfg.device = env_cfg.sim.device
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(str(args_cli.checkpoint))
        policy = runner.get_inference_policy(device=env.unwrapped.device)
    else:
        policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()

    dt = env.unwrapped.step_dt
    steps = min(int(round(args_cli.duration_s / dt)), int(env.unwrapped.max_episode_length))
    settle_steps = int((env_cfg.cmd_settle_s + 1.0) / dt)

    jobs = []
    if "angle_recovery" in args_cli.protocols:
        for angle in args_cli.angles_deg:
            signed = -angle if args_cli.negative else angle
            jobs.append(
                {
                    "scenario": "initial_pitch_release",
                    "run_prefix": f"isaac_rel_{abs(angle):g}deg",
                    "csv": f"timeseries_isaac_angle_recovery_{abs(angle):g}deg.csv",
                    "release_deg": signed,
                    "cfg": {"benchmark_disturbance_kind": "none", "forced_velocity_cmd_mps": 0.0},
                }
            )
    if "station_keeping" in args_cli.protocols:
        jobs.append(
            {
                "scenario": "station_keeping",
                "run_prefix": "isaac_station",
                "csv": "timeseries_isaac_station_keeping.csv",
                "release_deg": None,
                "cfg": {"benchmark_disturbance_kind": "none", "forced_velocity_cmd_mps": 0.0},
            }
        )
    if "velocity_step" in args_cli.protocols:
        jobs.append(
            {
                "scenario": "velocity_step",
                "run_prefix": f"isaac_vstep_{args_cli.velocity_mps:g}mps",
                "csv": f"timeseries_isaac_velocity_step_{args_cli.velocity_mps:g}mps.csv",
                "release_deg": None,
                "cfg": {"benchmark_disturbance_kind": "none", "forced_velocity_cmd_mps": args_cli.velocity_mps},
            }
        )
    if "payload_disturbance" in args_cli.protocols:
        jobs.append(
            {
                "scenario": "payload_disturbance",
                "run_prefix": f"isaac_payload_{args_cli.payload_torque_nm:g}nm",
                "csv": f"timeseries_isaac_payload_{args_cli.payload_torque_nm:g}nm.csv",
                "release_deg": None,
                # Both ranges are pinned to a single value so the disturbance is
                # exactly reproducible: the same torque, switched on at the same
                # time, in every env and every repeat of the run.
                "cfg": {
                    "benchmark_disturbance_kind": "payload",
                    "forced_velocity_cmd_mps": 0.0,
                    "payload_pitch_torque_nm_range": (args_cli.payload_torque_nm, args_cli.payload_torque_nm),
                    "payload_start_s_range": (args_cli.payload_start_s, args_cli.payload_start_s),
                },
            }
        )

    print(
        f"\n[protocol_bench] task={args_cli.task} envs={args_cli.num_envs} steps={steps} "
        f"({steps * dt:.2f} s at {1.0 / dt:.1f} Hz) stage={args_cli.curriculum_stage} terrain={args_cli.terrain}"
    )
    for job in jobs:
        live_cfg = env.unwrapped.cfg
        for key, value in job["cfg"].items():
            setattr(live_cfg, key, value)
        # Reset and stepping share one inference_mode block: stepping turns the
        # env's internal buffers into inference tensors, and a later reset
        # outside that context fails on their in-place writes.
        with torch.inference_mode():
            rows, metrics = _run_protocol(
                policy,
                env,
                job["scenario"],
                job["run_prefix"],
                steps,
                settle_steps,
                release_deg=job["release_deg"],
            )
        csv_path = output_dir / job["csv"]
        _write_csv(csv_path, rows)
        summary = "  ".join(f"{key}={value:.4g}" for key, value in metrics.items())
        print(f"[{job['scenario']}] {csv_path} ({len(rows)} rows / {args_cli.num_envs} runs)\n    {summary}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
