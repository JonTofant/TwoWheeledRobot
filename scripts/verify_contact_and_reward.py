#!/usr/bin/env python3
"""Assert the contact sensor is live and the reward is non-negative.

Both properties fail silently if they fail at all, which is why they get their
own script rather than a comment:

* ``activate_contact_sensors=False`` does not raise — a ContactSensor built on
  top of it reports zeros forever, so every episode looks contact-free and
  nothing ever terminates. Wheel forces are the tell: they must be non-zero
  while the robot is standing on them.
* The reward's ">= 0 in every reachable state" invariant is what makes diving for
  the floor unable to pay. Nothing enforces it at runtime, so it is checked here
  against real rollouts including deliberately tipped states.

Exits non-zero if any check fails.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDrive-v0")
parser.add_argument("--num_envs", type=int, default=256)
parser.add_argument("--num_steps", type=int, default=400)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--terrain", type=str, default="flat", choices=["flat", "generator"])
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

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str) -> None:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if not ok:
        FAILURES.append(f"{name}: {detail}")


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg):
    random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terrain_mode = args_cli.terrain
    env_cfg.curriculum_stage = 1

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        raise RuntimeError("multi-agent env not supported here")
    u = env.unwrapped


    check(
        "fall_mode is contact",
        u.cfg.fall_mode == "contact",
        f"fall_mode={u.cfg.fall_mode!r}",
    )
    check(
        "contact sensor constructed",
        u.ground_contact is not None and u._contact_nonwheel_cols is not None,
        f"sensor={'present' if u.ground_contact is not None else 'MISSING'}",
    )

    spawn_steps = 5  # 75 ms: long enough to seat the 1 cm drop, too short to fall
    term_span: dict[str, list[float]] = {}

    def rollout(steps: int, random_actions: bool):
        """Return (wheel force peak, non-wheel force peak, min/max reward, max tilt)."""
        wheel_peak = torch.zeros(u.num_envs, device=u.device)
        nonwheel_peak = torch.zeros(u.num_envs, device=u.device)
        lo = torch.full((1,), float("inf"), device=u.device)
        hi = torch.full((1,), float("-inf"), device=u.device)
        max_tilt = 0.0
        # Reset and stepping share one inference_mode block: stepping turns env
        # buffers into inference tensors and a later reset outside it would fail.
        with torch.inference_mode():
            obs, _ = env.reset()
            # Spawn attitude read before any stepping, so this is the reset draw
            # itself rather than anything the dynamics did to it.
            _, _, spawn_pitch, _, _, _ = u._state_terms()
            self_spawn_pitch_max = math.degrees(spawn_pitch.abs().max().item())
            for _ in range(steps):
                if random_actions:
                    actions = torch.randn(u.num_envs, u.cfg.action_space, device=u.device)
                else:
                    actions = torch.zeros(u.num_envs, u.cfg.action_space, device=u.device)
                obs, _, _, _, _ = env.step(actions)

                forces = u.ground_contact.data.net_forces_w
                wheel_peak = torch.maximum(
                    wheel_peak,
                    torch.linalg.vector_norm(forces[:, u._contact_wheel_cols, :], dim=-1).max(dim=1).values,
                )
                nonwheel_peak = torch.maximum(
                    nonwheel_peak,
                    torch.linalg.vector_norm(forces[:, u._contact_nonwheel_cols, :], dim=-1)
                    .max(dim=1)
                    .values,
                )
                # reward_total is post-clamp; the pre-clamp sum is what must be
                # >= 0. terminal is excluded: it is a one-off episode-end cost,
                # not a per-step shaping term.
                parts = u._last_reward_components
                pre_clamp = sum(v for k, v in parts.items() if k not in ("total", "terminal"))
                lo = torch.minimum(lo, pre_clamp.min().reshape(1))
                hi = torch.maximum(hi, pre_clamp.max().reshape(1))
                for key, value in parts.items():
                    if key in ("total", "terminal", "alive"):
                        continue
                    span = term_span.setdefault(key, [float("inf"), float("-inf")])
                    span[0] = min(span[0], value.min().item())
                    span[1] = max(span[1], value.max().item())

                proj = u.bno080.data.projected_gravity_b
                tilt = torch.acos(torch.clamp(-proj[:, 2], -1.0, 1.0)).max().item()
                max_tilt = max(max_tilt, math.degrees(tilt))
        return wheel_peak, nonwheel_peak, lo, hi, max_tilt, self_spawn_pitch_max

    # Phase A — spawn geometry. Zero actions and only a few steps, so nothing has
    # had time to fall over: any non-wheel contact here is the spawn pose itself
    # intersecting the ground, which would terminate every episode at step 1.
    wheel_peak, nonwheel_peak, _, _, spawn_tilt, spawn_pitch_max = rollout(spawn_steps, random_actions=False)

    # The stage ramp is a config value until something proves it reached the
    # reset draw. This repo has twice shipped randomization that was configured
    # but not applied (the COM axis, the disturbance-force axes), so it is
    # asserted rather than assumed.
    scales = getattr(u.cfg, "reset_stage_pitch_scale", None)
    expected_deg = u.cfg.reset_pitch_range_deg
    if scales:
        expected_deg *= scales[max(0, min(int(u.cfg.curriculum_stage), len(scales)) - 1)]
    check(
        "stage spawn-pitch ramp is applied",
        spawn_pitch_max <= expected_deg + 0.25,
        f"stage {u.cfg.curriculum_stage} max |spawn pitch| = {spawn_pitch_max:.2f} deg "
        f"vs expected <= {expected_deg:.2f} deg "
        f"(base {u.cfg.reset_pitch_range_deg:.1f} deg, scale {scales})",
    )
    check(
        "wheels report contact force",
        bool((wheel_peak > 0.0).all()),
        f"{int((wheel_peak > 0.0).sum())}/{u.num_envs} envs saw non-zero wheel force, "
        f"peak={wheel_peak.max().item():.2f} N "
        "(all-zero means activate_contact_sensors did not take)",
    )
    spawn_hits = int((nonwheel_peak > u.cfg.contact_force_threshold_n).sum())
    check(
        "spawn does not self-terminate",
        spawn_hits == 0,
        f"{spawn_hits}/{u.num_envs} envs had non-wheel contact above "
        f"{u.cfg.contact_force_threshold_n:.2f} N within {spawn_steps} steps of reset "
        f"(spawn_extra_clearance_m={u.cfg.spawn_extra_clearance_m}, max tilt {spawn_tilt:.1f} deg)",
    )

    # Phase B — reward invariant. Random actions deliberately, to drive the robot
    # into tipped, high-rate, high-current, full-swing-leg states. That is exactly
    # where an unbounded penalty hides: it is how the uncapped cg_rate term
    # (-32/step) was found.
    _, _, lo, hi, stress_tilt, _ = rollout(args_cli.num_steps, random_actions=True)
    check(
        "per-step reward is non-negative",
        lo.item() >= 0.0,
        f"min pre-clamp per-step reward = {lo.item():.4f} over {args_cli.num_steps} steps "
        f"with random actions (max tilt reached {stress_tilt:.1f} deg)",
    )
    check(
        "reward ceiling is not truncating",
        hi.item() < u.cfg.reward_total_max,
        f"max observed {hi.item():.3f} vs reward_total_max {u.cfg.reward_total_max}",
    )

    # A term pinned to one value across a random-action rollout has no gradient
    # and is shaping nothing — it only shifts the reward by a constant, which
    # reads as harmless in TensorBoard. This has now cost two runs: cg_rate
    # saturated at its -0.10 cap for a whole run, then cg_pos at its -0.20 cap,
    # the latter letting the legs park at an extreme deflection whose COM shift
    # put the robot into a 15-24 deg lean. Both were output caps; there are none
    # left, and this check is what keeps it that way.
    dead = {k: v for k, v in term_span.items() if (v[1] - v[0]) < 1.0e-4}
    check(
        "no reward term has a dead gradient",
        not dead,
        "all terms vary across the rollout"
        if not dead
        else f"pinned to a constant: { {k: round(v[0], 4) for k, v in dead.items()} }",
    )

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
    if FAILURES:
        print(f"\n{len(FAILURES)} CHECK(S) FAILED:")
        for item in FAILURES:
            print(f"  - {item}")
        sys.exit(1)
    print("\nAll checks passed.")
