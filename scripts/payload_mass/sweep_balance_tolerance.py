"""How much payload, and how far off-centre, can the frozen hardware GRU carry?

Places one object per env on a mass x placement grid at t = 2 s, holds it for the
rest of the episode, and reports fall rate, settled lean and leg-signal change per
cell, once under station keeping and once under random stage-2 driving commands.
Also checks the payload composition against the expected whole-robot mass/COM.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "source" / "TwoWheeledRobot"))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--reps", type=int, default=8)
parser.add_argument("--seconds", type=float, default=14.0)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--out", type=Path, default=Path("outputs/payload_mass/tolerance.json"))
AppLauncher.add_app_launcher_args(parser)
args_cli, rest = parser.parse_known_args()
sys.argv = [sys.argv[0]] + rest
app = AppLauncher(args_cli).app

import torch  # noqa: E402
from common import make_env  # noqa: E402
from frozen_policy import HARDWARE_GRU_CHECKPOINT, FrozenGruPolicy  # noqa: E402

MASSES = [0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0]
OFF_Y = [-0.08, -0.04, 0.0, 0.04, 0.08]  # fore/aft
OFF_X = [-0.09, 0.0, 0.09]  # lateral


def run(env, policy, cells, reps, steps, forced):
    u = env.unwrapped
    u.cfg.forced_command_mode = "fixed" if forced else ""
    u.cfg.forced_velocity_cmd_mps = 0.0
    u.cfg.forced_yaw_rate_cmd_radps = 0.0
    n = u.num_envs
    dev = u.device
    cell_of_env = torch.arange(n, device=dev) // reps
    mass = torch.tensor([c[0] for c in cells], device=dev)[cell_of_env]
    off = torch.tensor([[c[2], c[1]] for c in cells], device=dev)[cell_of_env]
    size = torch.tensor([0.15, 0.12, 0.03], device=dev).expand(n, 3)
    place = torch.full((n,), int(2.0 / u.step_dt), device=dev)
    with torch.inference_mode():
        obs = env.reset()[0]["policy"]
        u.set_payload(torch.arange(n, device=dev), mass, off, size, place)
        policy.reset()
        fell = torch.zeros(n, dtype=torch.bool, device=dev)
        pitch_sum = torch.zeros(n, device=dev)
        pitch_cnt = torch.zeros(n, device=dev)
        leg_pre = torch.zeros(n, 8, device=dev)
        leg_post = torch.zeros(n, 8, device=dev)
        speed_sum = torch.zeros(n, device=dev)
        checked = False
        for step in range(steps):
            act = policy(obs)
            obs, _, term, trunc, _ = env.step(act)
            obs = obs["policy"]
            done = term | trunc
            if done.any():
                policy.reset(done.nonzero().flatten())
            fell |= term & ~fell
            alive = ~fell
            t = step * u.step_dt
            legs = u.leg_features()
            if 1.0 <= t < 2.0:
                leg_pre += legs / int(1.0 / u.step_dt)
            if t >= 6.0:
                leg_post += legs * alive.unsqueeze(1)
                pitch_sum += obs[:, 2] * 25.0 * alive
                pitch_cnt += alive
                speed_sum += obs[:, 1].abs() * alive
            if not checked and step == place[0].item() + 2:
                checked = True
                _check_composition(u, mass)
    pitch_mean = pitch_sum / pitch_cnt.clamp_min(1)
    leg_post = leg_post / pitch_cnt.clamp_min(1).unsqueeze(1)
    rows = []
    for i, (m, oy, ox) in enumerate(cells):
        sel = cell_of_env == i
        ok = sel & ~fell
        rows.append(dict(
            mass=m, off_y=oy, off_x=ox, fall_rate=fell[sel].float().mean().item(),
            lean_deg=pitch_mean[ok].mean().item() if ok.any() else None,
            mean_speed=(speed_sum / pitch_cnt.clamp_min(1))[ok].mean().item() if ok.any() else None,
            d_leg_pos_mrad=((leg_post - leg_pre)[ok, 0:4].mean(0) * 1000).tolist() if ok.any() else None,
            d_leg_tau_nm=(leg_post - leg_pre)[ok, 4:8].mean(0).tolist() if ok.any() else None,
        ))
    return rows


def _check_composition(u, mass):
    view = u.robot.root_physx_view
    masses = view.get_masses()
    base = u._default_body_masses.sum(dim=1)
    print("[check] env0 payload", mass[0].item(), "kg; total mass", masses[0].sum().item(),
          "default total", base[0].item(), "platform", masses[0, u._platform_body_col].item(),
          "platform com", view.get_coms()[0, u._platform_body_col, :3].tolist(),
          "payload pos", u.payload_pos_b[0].tolist())


def main():
    cells = list(itertools.product(MASSES, OFF_Y, OFF_X))
    n = len(cells) * args_cli.reps
    env = make_env(n, args_cli.seed)
    policy = FrozenGruPolicy(HARDWARE_GRU_CHECKPOINT, n, env.unwrapped.device)
    steps = int(args_cli.seconds / env.unwrapped.step_dt)
    out = {"hold": run(env, policy, cells, args_cli.reps, steps, forced=True),
           "drive": run(env, policy, cells, args_cli.reps, steps, forced=False)}
    for mode, rows in out.items():
        print(f"\n=== {mode}: fall rate by mass (all placements) / centred-only / worst cell ===")
        for m in MASSES:
            sel = [r for r in rows if r["mass"] == m]
            centre = [r for r in sel if r["off_x"] == 0 and r["off_y"] == 0][0]
            worst = max(sel, key=lambda r: r["fall_rate"])
            print(f"m={m:4.2f}  all {sum(r['fall_rate'] for r in sel) / len(sel):.3f}  centre {centre['fall_rate']:.3f}"
                  f"  worst {worst['fall_rate']:.3f} at y={worst['off_y']:+.2f} x={worst['off_x']:+.2f}"
                  f"  centre lean {centre['lean_deg']}  dtau {centre['d_leg_tau_nm']}  dpos {centre['d_leg_pos_mrad']}")
    args_cli.out.parent.mkdir(parents=True, exist_ok=True)
    args_cli.out.write_text(json.dumps(out, indent=1))
    print("wrote", args_cli.out)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
