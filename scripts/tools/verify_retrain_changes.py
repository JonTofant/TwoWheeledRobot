#!/usr/bin/env python3
"""Pre-flight check for the 2026-10 retrain: runs the live sim (no training) and
asserts every audit fix is active. Usage (host):
  docker exec isaac-lab-dev bash -c "cd /workspace/TwoWheeledRobot && \
    /isaac-sim/python.sh scripts/tools/verify_retrain_changes.py --headless" | grep verify
Expect: [verify] DONE failures=[]
"""

import argparse
import math
import sys

sys.path.insert(0, "/workspace/TwoWheeledRobot/source/TwoWheeledRobot")
from isaaclab.app import AppLauncher  # noqa: E402

p = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(p)
a, rest = p.parse_known_args()
sys.argv = [sys.argv[0]] + rest
app = AppLauncher(a).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import TwoWheeledRobot.tasks  # noqa: E402,F401
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

FAIL = []


def check(name, ok, detail=""):
    print(f"[verify] {'PASS' if ok else 'FAIL'} {name} {detail}", flush=True)
    if not ok:
        FAIL.append(name)


task = "Template-Twowheeledrobot-NNDriveFixedStance-v0"
cfg = load_cfg_from_registry(task, "env_cfg_entry_point")
cfg.scene.num_envs = 64
cfg.terrain_mode = "flat"
cfg.seed = 7
cfg.forced_command_mode = "fixed"
cfg.curriculum_stage = 1
cfg.benchmark_disturbance_kind = "none"
env = gym.make(task, cfg=cfg)
u = env.unwrapped
env.reset()

# 1. Contact friction
mats = u.robot.root_physx_view.get_material_properties()
per_env = mats[:, 0, :2].clone()
same_within_env = bool((mats[:, :, :2] == mats[:, :1, :2]).all())
check("friction: one pair per env across all shapes", same_within_env)
check(
    "friction: varies across envs",
    per_env[:, 0].unique().numel() > 10,
    f"unique static={per_env[:, 0].unique().numel()}",
)
check(
    "friction: static in range",
    bool(((per_env[:, 0] >= 0.5 - 1e-6) & (per_env[:, 0] <= 1.1 + 1e-6)).all()),
    f"min={per_env[:, 0].min():.3f} max={per_env[:, 0].max():.3f}",
)
check(
    "friction: dynamic <= static",
    bool((per_env[:, 1] <= per_env[:, 0] + 1e-6).all()),
    f"dyn min={per_env[:, 1].min():.3f} max={per_env[:, 1].max():.3f}",
)
check("friction: recorded == applied", torch.allclose(u._contact_friction.cpu(), per_env, atol=1e-6))
import omni.usd  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

gm = UsdPhysics.MaterialAPI(omni.usd.get_context().get_stage().GetPrimAtPath("/World/ground/terrain/physicsMaterial"))
check(
    "friction: ground neutral 1.0/1.0",
    abs(gm.GetStaticFrictionAttr().Get() - 1.0) < 1e-6 and abs(gm.GetDynamicFrictionAttr().Get() - 1.0) < 1e-6,
)
env.reset()
mats2 = u.robot.root_physx_view.get_material_properties()
check("friction: resampled on reset", not torch.equal(mats2[:, 0, :2], per_env))

# 2. Motor model: no lag, no bias, no limit -> command = sign*max(|g*i|-dz, 0) within the same step.
proc = u._action_processor
proc.action_delay_samples.zero_()
actions = torch.full((u.num_envs, 2), math.atanh(0.5), device=u.device)  # tanh -> 0.5 -> 1.0 A
with torch.no_grad():
    env.step(actions)
gain = torch.stack([proc.left_gain, proc.right_gain], dim=1)
expected = (1.0 * gain - proc.deadzone).clamp(min=0.0)
err = (proc.command_current - expected).abs().max().item()
check("motor: no lag, command = g*i - dz in the same step", err < 1e-5, f"max err={err:.2e}")
check("motor: no tau/bias/limit attributes", not any(hasattr(proc, k) for k in ("tau_s", "bias", "current_limit")))

# 2b. Damping fixed at the measured value; CyberGear gains fixed; COM fore-aft untouched.
damp = u.robot.data.joint_damping[:, u._wheel_ids]
check("damping: wheels at 0.0007", torch.allclose(damp, torch.full_like(damp, 0.0007)), f"{damp[0].tolist()}")
cg_cols = [u._cg_fl_ids[0], u._cg_fr_ids[0], u._cg_bl_ids[0], u._cg_br_ids[0]]
kp = u.robot.data.joint_stiffness[:, cg_cols]
kd = u.robot.data.joint_damping[:, cg_cols]
check(
    "cybergear: kp=30, kd=3 everywhere",
    torch.allclose(kp, torch.full_like(kp, 30.0)) and torch.allclose(kd, torch.full_like(kd, 3.0)),
)
coms = u.robot.root_physx_view.get_coms()
check(
    "COM fore-aft offset zero",
    torch.allclose(coms[:, u._platform_body_col, 1], u._default_body_coms[:, u._platform_body_col, 1]),
)
comps = u._last_reward_components
check("reward: no position_far term", "position_far" not in comps)
check("reward: cg terms zero", float(comps["cg_pos"].abs().max()) == 0.0 and float(comps["cg_rate"].abs().max()) == 0.0)

# 3. Obs 9-10 carry the command (net_current clamped), scaled by 2 A.
obs_now = u._obs_now
check(
    "obs[9:10] == policy command / 2",
    torch.allclose(obs_now[:, 9:11], proc.net_current.clamp(-2, 2) / 2.0, atol=1e-6),
    f"obs={obs_now[0, 9:11].tolist()} cmd={proc.net_current[0].tolist()}",
)

# 4. Limiter symmetry: equal logical wheel speed and equal motoring current on
# both wheels must give equal speed limits.
v_logical = 8.0  # rad/s, forward
jv = u.robot.data.joint_vel.clone()
jv[:, u._wheel_ids] = v_logical * u._wheel_sign
u.robot.write_joint_state_to_sim(u.robot.data.joint_pos.clone(), jv)
u.robot.update(0.0)
u._wheel_i_cmd = torch.full((u.num_envs, 2), 1.5, device=u.device)
u._apply_action()
lim = u._wheel_tau_speed_limit
check(
    "limiter: left == right == 270 rpm envelope",
    torch.allclose(lim[:, 0], lim[:, 1])
    and abs(lim[0, 0].item() - 2.0 * (1 - v_logical / (270 * 2 * math.pi / 60))) < 1e-4,
    f"left={lim[0, 0]:.3f} right={lim[0, 1]:.3f} (expect {2.0 * (1 - v_logical / (270 * 2 * math.pi / 60)):.3f})",
)
u._wheel_i_cmd = torch.full((u.num_envs, 2), -1.5, device=u.device)
u._apply_action()
lim = u._wheel_tau_speed_limit
check(
    "limiter: braking not derated on either wheel",
    bool((lim == 2.0).all()),
    f"left={lim[0, 0]:.3f} right={lim[0, 1]:.3f}",
)

# 5. Delays follow the cfg.
u.cfg.action_delay_steps_range = (0, 0)
u.cfg.obs_delay_steps_range = (0, 0)
env.reset()
check(
    "delays: (0,0) gives no delay", int(proc.action_delay_samples.sum()) == 0 and int(u._obs_delay_samples.sum()) == 0
)
u.cfg.action_delay_steps_range = (0, 1)
u.cfg.obs_delay_steps_range = (0, 1)
env.reset()
frac = proc.action_delay_samples.float().mean().item()
check("delays: (0,1) gives a mix", 0.2 < frac < 0.8, f"action-delay fraction {frac:.2f}")

# 6. Actor parameter counts for all three runner cfgs.
wrapped = RslRlVecEnvWrapper(env, clip_actions=None)
actor_counts = {}
for t in (
    "Template-Twowheeledrobot-NNDriveFixedStance-v0",
    "Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0",
    "Template-Twowheeledrobot-NNDriveFixedStanceWide-v0",
):
    acfg = load_cfg_from_registry(t, "rsl_rl_cfg_entry_point")
    check(
        f"{t}: restored exploration floor",
        tuple(acfg.action_std_floor) == (0.15, 0.15),
        f"floor={acfg.action_std_floor}",
    )
    acfg.device = str(u.device)
    r = OnPolicyRunner(wrapped, acfg.to_dict(), log_dir=None, device=acfg.device)
    pol = r.alg.policy
    n = sum(x.numel() for x in pol.actor.parameters())
    if hasattr(pol, "memory_a"):
        n += sum(x.numel() for x in pol.memory_a.parameters())
    actor_counts[t] = n
    print(f"[verify] {t.split('-')[-2]}: actor params {n}, std floor {acfg.action_std_floor}", flush=True)
gru_count = actor_counts["Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0"]
wide_count = actor_counts["Template-Twowheeledrobot-NNDriveFixedStanceWide-v0"]
check("wide actor within 1% of GRU actor", abs(wide_count - gru_count) / gru_count < 0.01)

# 7. Station-keeping drift reference latches when the command reaches zero.
env.reset()
with torch.no_grad():
    env.step(torch.zeros(u.num_envs, 2, device=u.device))  # settle window: hold from the first step
    ref0 = u._hold_ref_xy.clone()
    root = u.robot.data.root_state_w.clone()

    def teleport(dx):
        r = root.clone()
        r[:, 0] += dx
        u.robot.write_root_pose_to_sim(r[:, :7])
        u.robot.write_root_velocity_to_sim(torch.zeros_like(r[:, 7:]))

    def reward_step(v):
        u._commands.v_cmd[:] = v
        u.episode_length_buf += 1  # new termination-update step
        u._get_rewards()
        return u._last_reward_components["hold_world_drift"].clone()

    teleport(0.5)
    d_hold = reward_step(0.0)
    check(
        "hold drift: reference stays while holding",
        torch.allclose(u._hold_ref_xy, ref0),
        f"drift term {d_hold[0].item():+.3f} (0.5 m -> -0.25)",
    )
    reward_step(0.3)
    check("hold drift: no hold while commanded", not bool(u._hold_prev.any()))
    teleport(1.2)
    d_new = reward_step(0.0)
    pos = u.robot.data.root_pos_w[:, :2]
    gap = (u._hold_ref_xy - pos).norm(dim=1).max().item()
    check(
        "hold drift: re-latched where the new hold began",
        torch.allclose(u._hold_ref_xy, pos, atol=1e-5) and float(d_new.abs().max()) < 1e-6,
        f"ref-pos gap {gap:.2e} m, drift term {d_new.abs().max().item():.2e}",
    )

print(f"[verify] DONE failures={FAIL}", flush=True)
env.close()
