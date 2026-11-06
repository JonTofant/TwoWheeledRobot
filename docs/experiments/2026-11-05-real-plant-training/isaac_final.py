"""Frozen finalists in Isaac with the real-plant effects, previous-current observation as the firmware feeds it."""
from __future__ import annotations
import json, sys
from pathlib import Path
ROOT = Path("/workspace/TwoWheeledRobot")
sys.path[:0] = [str(ROOT / "source" / "TwoWheeledRobot"), str(ROOT / "scripts" / "payload_mass"), str(ROOT / "scripts")]
from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser(); AppLauncher.add_app_launcher_args(parser)
args_cli, rest = parser.parse_known_args(); sys.argv = [sys.argv[0]] + rest
app = AppLauncher(args_cli).app
import numpy as np, torch, gymnasium as gym, TwoWheeledRobot.tasks  # noqa
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry
from frozen_policy import FrozenGruPolicy
from nn_drive_benchmark_contract import SCENARIOS
TASK = "Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0"
L = ROOT / "logs/rsl_rl/nn_drive_fixed_stance"
FC = L / "2026-11-05_14-03-46_real_fresh_cmd_s43_20261105T113720Z_stage5"; FM = L / "2026-11-05_16-10-48_real_fresh_model_s43_20261105T113720Z_stage5"
POL = {"AUG 775": ROOT / "ExportedPolicy/fixedstance_range_gru_2026-08-10/2026-08-10_23-51-29_range_gru_stage5/model_775.pt",
       "cmd 1150": FC / "model_1150.pt", "cmd 1200": FC / "model_1200.pt", "model 1125": FM / "model_1125.pt", "model 1200": FM / "model_1200.pt", "model 1225": FM / "model_1225.pt"}
SCEN = ["station_keeping", "drive_forward", "drive_backward", "turn_in_place", "push_while_still", "push_while_driving", "payload_while_still", "payload_while_driving"]
n = 128
cfg = load_cfg_from_registry(TASK, "env_cfg_entry_point")
cfg.seed = 77; cfg.scene.num_envs = n; cfg.terrain_mode = "flat"; cfg.forced_command_mode = "fixed"
cfg.wheel_coulomb_friction_nm_range = (.01, .06); cfg.velocity_obs_filter = "ddsm115"; cfg.obs_delay_mode = "sensors"; cfg.prev_current_obs_source = "command"
env = gym.make(TASK, cfg=cfg); u = env.unwrapped
out = {}
for name, ck in POL.items():
    pol = FrozenGruPolicy(ck, n, u.device)
    for sc in SCEN:
        stage, kind, v, w = SCENARIOS[sc]
        c = u.cfg; c.curriculum_stage = stage; c.benchmark_disturbance_kind = kind; c.forced_velocity_cmd_mps = v; c.forced_yaw_rate_cmd_radps = w
        with torch.inference_mode():
            obs = env.reset()[0]["policy"]; pol.reset(); tr = []; fell = torch.zeros(n, dtype=torch.bool, device=u.device); act = []
            for s in range(1000):
                a = pol(obs); obs, _, term, trunc, _ = env.step(a); obs = obs["policy"]; fell |= term
                if (term | trunc).any(): pol.reset((term | trunc).nonzero().flatten())
                tr.append(torch.stack([obs[:, 2] * 25, obs[:, 1], a.mean(1) * 2], 1).cpu().numpy())
        x = np.stack(tr)[200:]; ok = ~fell.cpu().numpy(); p = x[:, ok, 0]
        r = dict(fall=float(fell.float().mean()), sd=float(p.std(0).mean()) if ok.any() else None, v=float(x[:, ok, 1].mean()) if ok.any() else None, dI=float(np.abs(np.diff(x[:, ok, 2], axis=0)).mean()) if ok.any() else None)
        out[f"{name} | {sc}"] = r; print(f"RESULT {name:11s} {sc:22s} fall {r['fall']*100:5.1f}%  SD {r['sd'] if r['sd'] is None else round(r['sd'],2)}  v {r['v'] if r['v'] is None else round(r['v'],3)}  dI {r['dI'] if r['dI'] is None else round(r['dI'],3)}", flush=True)
        (ROOT / "outputs/regression_nov/isaac_final.json").write_text(json.dumps(out, indent=1))
print("DONE", flush=True); app.close()
