"""Roll out the frozen hardware GRU with random payloads and record estimator data.

Every env starts a 20 s episode in lockstep; an object of random mass, size and
deck position is put on at 2-6 s (none in 15% of episodes) and taken off again in
25%. Commands come from the task's own stage-2 joystick generator (35% pure
station keeping, otherwise random drive/turn up to 0.30 m/s / 0.4 rad/s). An env
that falls is masked out from its first termination on, so each env contributes
one clean episode per round.

Output, one file per round: features (T, n, 23) fp16, mass (T, n), offset_xy
(T, n, 2), valid (T, n) bool -- see common.estimator_inputs for the layout.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "source" / "TwoWheeledRobot"))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--rounds", type=int, default=3)
parser.add_argument("--seed", type=int, default=100)
parser.add_argument("--tag", type=str, default="train")
parser.add_argument("--out_dir", type=Path, default=Path("outputs/payload_mass/data"))
AppLauncher.add_app_launcher_args(parser)
args_cli, rest = parser.parse_known_args()
sys.argv = [sys.argv[0]] + rest
app = AppLauncher(args_cli).app

import torch  # noqa: E402
from common import estimator_inputs, make_env  # noqa: E402
from frozen_policy import HARDWARE_GRU_CHECKPOINT, FrozenGruPolicy  # noqa: E402


def main():
    env = make_env(args_cli.num_envs, args_cli.seed, payload_mass_kg_range=(0.05, 2.0))
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    policy = FrozenGruPolicy(HARDWARE_GRU_CHECKPOINT, n, dev)
    steps = int(u.cfg.episode_length_s / u.step_dt) - 1
    args_cli.out_dir.mkdir(parents=True, exist_ok=True)
    centre = u._deck_center_b.to(dev)
    for rnd in range(args_cli.rounds):
        t0 = time.time()
        feats, mass, offset, valid = [], [], [], []
        with torch.inference_mode():
            u.episode_length_buf[:] = 0
            obs = env.reset()[0]["policy"]
            policy.reset()
            alive = torch.ones(n, dtype=torch.bool, device=dev)
            for _ in range(steps):
                act = policy(obs)
                # Features and labels describe the state the policy just acted on.
                feats.append(estimator_inputs(env, obs, act).half().cpu())
                mass.append(u.payload_mass_now.clone().cpu())
                offset.append((u.payload_pos_b[:, :2] - centre).cpu())
                valid.append(alive.clone().cpu())
                obs, _, term, trunc, _ = env.step(act)
                obs = obs["policy"]
                done = term | trunc
                alive &= ~done
                if done.any():
                    policy.reset(done.nonzero().flatten())
        path = args_cli.out_dir / f"{args_cli.tag}_s{args_cli.seed}_r{rnd}.pt"
        torch.save(dict(features=torch.stack(feats), mass=torch.stack(mass), offset_xy=torch.stack(offset),
                        valid=torch.stack(valid), dt=u.step_dt), path)
        print(f"[collect] round {rnd}: {steps} steps x {n} envs in {time.time() - t0:.0f} s, "
              f"survived {alive.float().mean().item():.3f}, wrote {path}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
