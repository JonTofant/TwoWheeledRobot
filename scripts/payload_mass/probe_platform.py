"""One-off probe for the payload-mass experiment.

Prints the platform geometry and mass properties a payload model needs, checks
how the exported GRU's hidden state is stored, and measures how much the leg
joint signals move when 1 kg is added to the platform under the frozen policy.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "source" / "TwoWheeledRobot"))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--policy", required=True)
parser.add_argument("--num_envs", type=int, default=16)
AppLauncher.add_app_launcher_args(parser)
args_cli, rest = parser.parse_known_args()
sys.argv = [sys.argv[0]] + rest
app = AppLauncher(args_cli).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import TwoWheeledRobot.tasks  # noqa: E402,F401

from pxr import Usd, UsdGeom  # noqa: E402

from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

TASK = "Template-Twowheeledrobot-NNDriveFixedStance-v0"


def main():
    cfg = load_cfg_from_registry(TASK, "env_cfg_entry_point")
    cfg.scene.num_envs = args_cli.num_envs
    cfg.terrain_mode = "flat"
    cfg.forced_command_mode = "fixed"
    cfg.curriculum_stage = 1
    cfg.benchmark_disturbance_kind = "none"
    cfg.enable_force_noise = False
    env = gym.make(TASK, cfg=cfg)
    u = env.unwrapped
    robot = u.robot
    view = robot.root_physx_view
    names = robot.body_names
    masses = view.get_masses()[0]
    coms = view.get_coms()[0]
    inertias = view.get_inertias()[0]
    print("\n=== BODIES ===")
    for i, n in enumerate(names):
        print(f"[{i}] {n}: mass={masses[i]:.4f} com={coms[i, :3].tolist()} quat={coms[i, 3:].tolist()}")
        print(f"     inertia={inertias[i].view(3, 3).tolist()}")
    print(f"total mass {masses.sum():.4f}; platform col {u._platform_body_col} = {names[u._platform_body_col]}")
    print("joint names", robot.joint_names)

    stage = u.sim.stage
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    root = stage.GetPrimAtPath("/World/envs/env_0/Robot")
    print("\n=== PRIM TREE (depth<=3) + world AABB ===")
    for prim in Usd.PrimRange(root):
        depth = len(str(prim.GetPath()).split("/")) - len(str(root.GetPath()).split("/"))
        if depth > 3:
            continue
        if not prim.IsA(UsdGeom.Imageable):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        print(f"{'  ' * depth}{prim.GetName()} [{prim.GetTypeName()}] min={tuple(round(v, 4) for v in box.GetMin())} "
              f"max={tuple(round(v, 4) for v in box.GetMax())}")

    policy = torch.jit.load(args_cli.policy, map_location=u.device).eval()
    print("\n=== POLICY ===")
    for n, b in policy.named_buffers():
        print("buffer", n, tuple(b.shape))
    print("methods:", [m for m in dir(policy) if not m.startswith("_")][:40])

    with torch.inference_mode():
        obs = env.reset()[0]["policy"]
        policy.reset()
        pos = robot.data.root_pos_w[0].tolist()
        print("root pos after reset", pos, "body pos platform", robot.data.body_pos_w[0, u._platform_body_col].tolist())
        rec = []
        for step in range(700):
            if step == 350:
                plat = u._platform_body_col
                m = view.get_masses().clone()
                m[:, plat] += 1.0
                view.set_masses(m, torch.arange(u.num_envs))
                print("added 1 kg to platform at step 350")
            act = policy(obs)
            obs, _, term, trunc, _ = env.step(act)
            obs = obs["policy"]
            if (term | trunc).any():
                print("done at step", step, (term | trunc).nonzero().flatten().tolist())
            cg = robot.data.joint_pos[:, u._cg_ids]
            tau = robot.data.applied_torque[:, u._cg_ids]
            ctau = robot.data.computed_torque[:, u._cg_ids]
            pitch = obs[:, 2] * 25.0
            rec.append(torch.cat([cg.mean(0), tau.mean(0), ctau.mean(0), act.abs().mean(0),
                                  pitch.mean().view(1)]).cpu())
        rec = torch.stack(rec)
        before = rec[250:350].mean(0)
        after = rec[600:700].mean(0)
        print("\n=== LEG SIGNALS, mean over envs: steps 250-350 (no payload) vs 600-700 (+1 kg) ===")
        print("cg_pos rad      ", before[0:4].tolist(), "->", after[0:4].tolist())
        print("applied tau Nm  ", before[4:8].tolist(), "->", after[4:8].tolist())
        print("computed tau Nm ", before[8:12].tolist(), "->", after[8:12].tolist())
        print("|action|        ", before[12:14].tolist(), "->", after[12:14].tolist())
        print("pitch obs deg   ", before[14].item(), "->", after[14].item())
        print("cg_pos std (time, 250-350)", rec[250:350, 0:4].std(0).tolist())
    env.close()


if __name__ == "__main__":
    main()
    app.close()
