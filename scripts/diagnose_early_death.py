#!/usr/bin/env python3
"""Find what separates episodes that die in the first seconds from those that live.

The stage-1 A/B (docs/experiments/2026-08-07) showed that every termination in
the benchmark happens inside the 2 s settle window, and that zeroing the reset
attitude/velocity randomization removes only about a third of them. This script
answers the follow-up directly instead of bisecting configs: run one scenario,
snapshot every per-episode randomization draw at reset, then split environments
into "died early" and "survived" and compare the two distributions.

Output is a per-parameter table with group means and a separation statistic, plus
the mean pitch trajectory of each group so the divergence time is visible.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from nn_drive_benchmark_contract import SCENARIOS

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Diagnose early-episode terminations.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
parser.add_argument("--checkpoint", type=Path, required=True)
parser.add_argument("--num_envs", type=int, default=1024)
parser.add_argument("--num_steps", type=int, default=200, help="200 steps = 3.0 s at 66.7 Hz")
parser.add_argument("--early_steps", type=int, default=133, help="133 steps = 2.0 s settle window")
parser.add_argument("--scenario", type=str, default="station_keeping", choices=tuple(SCENARIOS))
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--json-output", type=Path, default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import json
import math
import random

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from rsl_rl.runners import OnPolicyRunner

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


def _draws(unwrapped) -> dict[str, torch.Tensor]:
    """Every per-episode randomized quantity, read back after reset.

    COM offset and mass scale are recovered from PhysX rather than the draw
    itself, so a mismatch between the sampled value and the applied value would
    show up here rather than being hidden (the PAPER-02 gate).
    """
    dev = unwrapped.device
    proc = unwrapped._action_processor
    cg = unwrapped._cg_processor

    coms = unwrapped.robot.root_physx_view.get_coms()
    com_delta = (coms - unwrapped._default_body_coms)[:, unwrapped._platform_body_col, :].to(dev)
    masses = unwrapped.robot.root_physx_view.get_masses().to(dev)
    default_masses = unwrapped._default_body_masses.to(dev)
    mass_scale = masses.sum(dim=1) / default_masses.sum(dim=1).clamp_min(1e-9)

    stiffness = unwrapped.robot.data.joint_stiffness
    damping = unwrapped.robot.data.joint_damping
    cg_cols = [unwrapped._cg_fl_ids[0], unwrapped._cg_fr_ids[0], unwrapped._cg_bl_ids[0], unwrapped._cg_br_ids[0]]
    wheel_cols = [unwrapped._left_wheel_ids[0], unwrapped._right_wheel_ids[0]]

    out = {
        "com_offset_y_m": com_delta[:, 1],
        "com_offset_z_m": com_delta[:, 2],
        "abs_com_offset_y_m": com_delta[:, 1].abs(),
        "body_mass_scale": mass_scale,
        "pitch_bias_deg": unwrapped._pitch_bias * 180.0 / math.pi,
        "abs_pitch_bias_deg": (unwrapped._pitch_bias * 180.0 / math.pi).abs(),
        "roll_bias_deg": unwrapped._roll_bias * 180.0 / math.pi,
        "pitch_rate_bias_radps": unwrapped._pitch_rate_bias,
        "yaw_rate_bias_radps": unwrapped._yaw_rate_bias,
        "roll_rate_bias_radps": unwrapped._roll_rate_bias,
        "odometry_scale": unwrapped._odometry_scale,
        "force_noise_amp_n": unwrapped._force_noise_amp,
        "motor_gain_left": proc.left_gain,
        "motor_gain_right": proc.right_gain,
        "motor_deadzone_mean_a": proc.deadzone.mean(dim=1),
        "motor_deadzone_asym_a": (proc.deadzone[:, 0] - proc.deadzone[:, 1]).abs(),
        "motor_tau_mean_s": proc.tau_s.mean(dim=1),
        "motor_current_limit_mean_a": proc.current_limit.mean(dim=1),
        "action_delay_samples": proc.action_delay_samples.float(),
        "cg_kp_mean": stiffness[:, cg_cols].mean(dim=1),
        "cg_kd_mean": damping[:, cg_cols].mean(dim=1),
        "wheel_damping_mean": damping[:, wheel_cols].mean(dim=1),
        "cg_calib_bias_absmax_deg": (cg.calib_bias.abs().max(dim=1).values) * 180.0 / math.pi,
    }
    return {k: v.detach().float().clone() for k, v in out.items()}


def _welch(a: torch.Tensor, b: torch.Tensor) -> tuple[float, float, float]:
    """Return (mean_died, mean_lived, Cohen's d) — d is the separation measure."""
    ma, mb = a.mean().item(), b.mean().item()
    va, vb = a.var(unbiased=True).item(), b.var(unbiased=True).item()
    pooled = math.sqrt(max(0.5 * (va + vb), 1e-24))
    return ma, mb, (ma - mb) / pooled


def _auc(a: torch.Tensor, b: torch.Tensor) -> float:
    """P(random died-env value > random survived-env value); 0.5 = no signal."""
    if a.numel() == 0 or b.numel() == 0:
        return 0.5
    combined = torch.cat([a, b])
    ranks = combined.argsort().argsort().float() + 1.0
    ra = ranks[: a.numel()].sum().item()
    na, nb = float(a.numel()), float(b.numel())
    return (ra - na * (na + 1.0) / 2.0) / (na * nb)


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args_cli.seed)
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.forced_command_mode = "fixed"

    stage, dist_kind, v_cmd, w_cmd = SCENARIOS[args_cli.scenario]
    env_cfg.curriculum_stage = stage
    env_cfg.benchmark_disturbance_kind = dist_kind
    env_cfg.forced_velocity_cmd_mps = v_cmd
    env_cfg.forced_yaw_rate_cmd_radps = w_cmd

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    agent_cfg.seed = args_cli.seed
    agent_cfg.device = env_cfg.sim.device
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    unwrapped = env.unwrapped

    # Reset and stepping share one inference_mode block: leaving it between the
    # two turns env buffers into inference tensors and the next reset crashes.
    with torch.inference_mode():
        reset_result = env.reset()
        obs = reset_result[0] if isinstance(reset_result, tuple) else reset_result
        if isinstance(obs, dict):
            obs = obs["policy"]

        draws = _draws(unwrapped)
        wd = draws["wheel_damping_mean"]
        print(f"\n[wheel_damping_mean post-reset] mean={wd.mean():.5f} min={wd.min():.5f} max={wd.max():.5f} "
              f"(expected range {unwrapped.cfg.wheel_viscous_damping_range})")
        # Spawn attitude/velocity are not stored on the env, so read them from
        # live state at t=0 rather than from the (unsaved) reset draw.
        x_rel_0, velocity_0, pitch_0, _, _, _ = unwrapped._state_terms()
        draws["spawn_pitch_deg"] = (pitch_0 * 180.0 / math.pi).detach().float().clone()
        draws["abs_spawn_pitch_deg"] = draws["spawn_pitch_deg"].abs()
        draws["spawn_velocity_mps"] = velocity_0.detach().float().clone()
        draws["abs_spawn_velocity_mps"] = draws["spawn_velocity_mps"].abs()

        n = unwrapped.num_envs
        dev = unwrapped.device
        alive = torch.ones(n, dtype=torch.bool, device=dev)
        died_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        pitch_trace = torch.zeros(args_cli.num_steps, n, device=dev)
        current_trace = torch.zeros(args_cli.num_steps, n, device=dev)

        for step in range(args_cli.num_steps):
            actions = policy(obs)
            step_out = env.step(actions)
            if len(step_out) == 5:
                obs, _, terminated, truncated, _ = step_out
                dones = terminated | truncated
            else:
                obs, _, dones, _ = step_out
            dones = dones.view(-1).to(device=dev, dtype=torch.bool)
            if isinstance(obs, dict):
                obs = obs["policy"]

            _, _, pitch, _, _, _ = unwrapped._state_terms()
            pitch_trace[step] = torch.where(alive, pitch * 180.0 / math.pi, torch.zeros_like(pitch))
            current_trace[step] = torch.where(
                alive, unwrapped._action_processor.command_current.abs().mean(dim=1), torch.zeros_like(pitch)
            )
            newly_dead = alive & dones
            died_step = torch.where(newly_dead, torch.full_like(died_step, step), died_step)
            alive &= ~dones

    early = (died_step >= 0) & (died_step < args_cli.early_steps)
    late = (died_step >= args_cli.early_steps)
    lived = died_step < 0
    n_early, n_late, n_lived = int(early.sum()), int(late.sum()), int(lived.sum())

    print(f"\nscenario={args_cli.scenario} envs={unwrapped.num_envs} steps={args_cli.num_steps}")
    print(f"died within {args_cli.early_steps} steps ({args_cli.early_steps * unwrapped.step_dt:.1f} s): {n_early}")
    print(f"died later: {n_late}")
    print(f"survived: {n_lived}")
    if n_early == 0 or n_lived == 0:
        print("Need both groups non-empty to compare. Stopping.")
        env.close()
        return

    dstep = died_step[early].float()
    print(f"death step among early deaths: mean={dstep.mean():.1f} p10={dstep.quantile(0.10):.0f} "
          f"median={dstep.median():.0f} p90={dstep.quantile(0.90):.0f} (dt={unwrapped.step_dt:.3f}s)")

    rows = []
    for key, values in draws.items():
        a, b = values[early], values[lived]
        ma, mb, d = _welch(a, b)
        rows.append((abs(d), key, ma, mb, d, _auc(a, b)))
    rows.sort(reverse=True)

    print(f"\n{'parameter':30}{'died<2s':>12}{'survived':>12}{'cohen_d':>10}{'auc':>8}")
    for _, key, ma, mb, d, auc in rows:
        print(f"{key:30}{ma:12.5f}{mb:12.5f}{d:10.3f}{auc:8.3f}")

    # Dose-response for the strongest separator: a real cause should show fall
    # rate rising monotonically with the parameter, not just a mean shift.
    top_key = rows[0][1]
    top_values = draws[top_key]
    died_any = died_step >= 0
    order = top_values.argsort()
    n_bins = 8
    print(f"\nfall rate vs {top_key} (deciles, {unwrapped.num_envs // n_bins} envs per bin)")
    print(f"{'bin':>4}{'range_lo':>12}{'range_hi':>12}{'n':>6}{'fall_rate':>11}")
    chunks = order.chunk(n_bins)
    dose = []
    for i, idx in enumerate(chunks):
        vals = top_values[idx]
        fr = died_any[idx].float().mean().item()
        dose.append({"bin": i, "lo": vals.min().item(), "hi": vals.max().item(),
                     "n": int(idx.numel()), "fall_rate": fr})
        print(f"{i:4d}{vals.min().item():12.5f}{vals.max().item():12.5f}{idx.numel():6d}{fr:11.3f}")

    print("\nmean |pitch| deg by step (alive-masked within each group)")
    print(f"{'step':>6}{'t_s':>7}{'died<2s':>12}{'survived':>12}")
    for step in range(0, args_cli.num_steps, max(1, args_cli.num_steps // 20)):
        pe = pitch_trace[step][early & (died_step > step)].abs()
        pl = pitch_trace[step][lived].abs()
        pe_v = pe.mean().item() if pe.numel() else float("nan")
        print(f"{step:6d}{step * unwrapped.step_dt:7.2f}{pe_v:12.3f}{pl.mean().item():12.3f}")

    if args_cli.json_output is not None:
        payload = {
            "scenario": args_cli.scenario,
            "checkpoint": str(args_cli.checkpoint),
            "num_envs": unwrapped.num_envs,
            "num_steps": args_cli.num_steps,
            "early_steps": args_cli.early_steps,
            "counts": {"died_early": n_early, "died_late": n_late, "survived": n_lived},
            "separation": [
                {"parameter": key, "mean_died": ma, "mean_survived": mb, "cohen_d": d, "auc": auc}
                for _, key, ma, mb, d, auc in rows
            ],
            "dose_response": {"parameter": top_key, "bins": dose},
        }
        args_cli.json_output.parent.mkdir(parents=True, exist_ok=True)
        args_cli.json_output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"\nWrote {args_cli.json_output}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
