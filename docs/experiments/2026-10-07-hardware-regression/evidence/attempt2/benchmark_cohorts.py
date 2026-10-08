#!/usr/bin/env python3
"""All canonical scenarios, frozen policies in separate randomized cohorts.
Initial random draws differ between policies. No training or model selection here.
"""
from pathlib import Path
import argparse,sys
R=Path('/workspace/TwoWheeledRobot');sys.path.insert(0,str(R/'source/TwoWheeledRobot'));sys.path.insert(0,str(R/'scripts'))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser();p.add_argument('--models',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--cohort',type=int,default=64)
AppLauncher.add_app_launcher_args(p);args,rest=p.parse_known_args();sys.argv=[sys.argv[0]]+rest
app=AppLauncher(args).app
import hashlib,json,random,time
import torch,numpy as np,gymnasium as gym
import TwoWheeledRobot.tasks
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry
from rsl_rl.runners import OnPolicyRunner
from nn_drive_benchmark_contract import SCENARIOS,stage_gate_failures,checkpoint_score
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import roll_from_projected_gravity
def run_scenario(wrapped, policies, slices, steps, settle_steps):
    """benchmark_nn_drive._scenario_metrics_impl, vectorised over all envs,
    plus limiter-binding counters. Returns per-env tensors."""
    u = wrapped.unwrapped
    obs, _ = wrapped.reset()
    n, dev = u.num_envs, u.device
    dt = u.step_dt
    z = lambda: torch.zeros(n, device=dev)  # noqa: E731
    alive = torch.ones(n, dtype=torch.bool, device=dev)
    survival = z()
    fall = torch.zeros(n, dtype=torch.bool, device=dev)
    physics_broken, invalid_state, timeout = fall.clone(), fall.clone(), fall.clone()
    acc = {k: z() for k in (
        "vel_err_sq", "yaw_rate_err_sq", "pitch_sq", "roll_sq", "current_sq", "pos_err_abs_max",
        "pos_err_abs_final", "speed_abs", "velocity", "yaw_rate", "world_drift_final", "count",
        "bind_left", "bind_right", "obs_cur_gap", "command_delta",
    )}
    actions = torch.zeros(n, 2, device=dev)
    previous_command = actions.clone()
    for step in range(steps):
        for name, sl in slices.items():
            actions[sl] = policies[name](obs[sl])
        command = 2.0 * torch.tanh(actions)
        command_delta = (command - previous_command).abs().mean(dim=1)
        previous_command = command.clone()
        obs, _, dones, _ = wrapped.step(actions)
        dones = dones.view(-1).to(torch.bool)
        x_rel, velocity, pitch, _, _, yaw_rate = u._state_terms()
        a = alive & ~dones
        if step >= settle_steps:
            w = lambda v: torch.where(a, v, torch.zeros_like(v))  # noqa: E731
            acc["count"] += a.float()
            acc["command_delta"] += w(command_delta)
            acc["vel_err_sq"] += w((velocity - u._commands.v_cmd).pow(2))
            acc["yaw_rate_err_sq"] += w((yaw_rate - u._commands.w_cmd).pow(2))
            roll = roll_from_projected_gravity(u.bno080.data.projected_gravity_b)
            acc["pitch_sq"] += w(pitch.pow(2))
            acc["roll_sq"] += w(roll.pow(2))
            acc["current_sq"] += w(u._action_processor.command_current.pow(2).mean(dim=1))
            acc["speed_abs"] += w(velocity.abs())
            acc["velocity"] += w(velocity)
            acc["yaw_rate"] += w(yaw_rate)
            drift = torch.linalg.vector_norm(u.robot.data.root_pos_w[:, :2] - u._spawn_pos_xy, dim=1)
            acc["world_drift_final"] = torch.where(a, drift, acc["world_drift_final"])
            pos_err = u._commands.position_error_raw(x_rel).abs()
            acc["pos_err_abs_max"] = torch.where(a, torch.maximum(acc["pos_err_abs_max"], pos_err), acc["pos_err_abs_max"])
            acc["pos_err_abs_final"] = torch.where(a, pos_err, acc["pos_err_abs_final"])
            # Extra: how often the torque-speed limiter clipped each wheel this step,
            # and how far the trained obs[9:10] quantity sits from firmware semantics.
            clipped = u._wheel_torque_cmd.abs() < u._wheel_tau_current.abs() - 1e-7
            acc["bind_left"] += w(clipped[:, 0].float())
            acc["bind_right"] += w(clipped[:, 1].float())
            gap = (u._action_processor.command_current - u._action_processor.net_current.clamp(-2.0, 2.0)).abs().mean(dim=1)
            acc["obs_cur_gap"] += w(gap)
        survival = torch.where(alive, torch.full_like(survival, (step + 1) * dt), survival)
        fall |= alive & u._last_step_fall
        physics_broken |= alive & u._last_step_physics_broken
        invalid_state |= alive & u._last_step_invalid_state
        timeout |= alive & u._last_step_timeout
        alive &= ~dones
    acc.update(survival=survival, fall=fall.float(), physics_broken=physics_broken.float(),
               invalid_state=invalid_state.float(), timeout=timeout.float())
    return acc

def aggregate(acc, sl) -> dict[str, float]:
    """benchmark_nn_drive's final reduction, applied to one policy's slice."""
    g = {k: v[sl] for k, v in acc.items()}
    sampled = g["count"] > 0
    denom = g["count"].clamp_min(1.0)

    def sm(values):
        return values[sampled].mean().item() if torch.any(sampled) else 0.0

    termination = ((g["fall"] + g["physics_broken"] + g["invalid_state"]) > 0).float()
    return {
        "survival_time_s": g["survival"].mean().item(),
        "fall_rate": g["fall"].mean().item(),
        "physics_broken_rate": g["physics_broken"].mean().item(),
        "invalid_state_rate": g["invalid_state"].mean().item(),
        "timeout_rate": g["timeout"].mean().item(),
        "termination_rate": termination.mean().item(),
        "rms_vel_err_mps": sm(torch.sqrt(g["vel_err_sq"] / denom)),
        "rms_yaw_rate_err_radps": sm(torch.sqrt(g["yaw_rate_err_sq"] / denom)),
        "rms_pitch_deg": sm(torch.sqrt(g["pitch_sq"] / denom)) * 180.0 / np.pi,
        "rms_roll_deg": sm(torch.sqrt(g["roll_sq"] / denom)) * 180.0 / np.pi,
        "max_pos_err_m": sm(g["pos_err_abs_max"]),
        "final_pos_err_m": sm(g["pos_err_abs_final"]),
        "mean_velocity_mps": sm(g["velocity"] / denom),
        "mean_yaw_rate_radps": sm(g["yaw_rate"] / denom),
        "achieved_speed_mps": sm(g["speed_abs"] / denom),
        "world_drift_m": sm(g["world_drift_final"]),
        "rms_current_a": sm(torch.sqrt(g["current_sq"] / denom)),
        "sampled_env_fraction": sampled.float().mean().item(),
        "mean_scored_samples": g["count"].mean().item(),
        "mean_abs_command_delta_a": sm(g["command_delta"] / denom),
        "limiter_bind_frac_left": sm(g["bind_left"] / denom),
        "limiter_bind_frac_right": sm(g["bind_right"] / denom),
        "obs_current_gap_a": sm(g["obs_cur_gap"] / denom),
    }
models=json.loads(args.models.read_text());random.seed(args.seed);np.random.seed(args.seed);torch.manual_seed(args.seed)
task='Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0'
cfg=load_cfg_from_registry(task,'env_cfg_entry_point');cfg.seed=args.seed;cfg.scene.num_envs=len(models)*args.cohort;cfg.terrain_mode='generator';cfg.forced_command_mode='fixed';cfg.obs_delay_mode='sensors'
env=gym.make(task,cfg=cfg);wrapped=RslRlVecEnvWrapper(env,clip_actions=None);u=env.unwrapped
# TerrainImporter assigns columns by the GLOBAL environment index. Explicitly
# give each policy the canonical N=cohort layout; otherwise contiguous cohorts
# receive different terrain columns. Shared origins are normal in Isaac
# (thousands of robots share a small tile grid); cross-env collisions are filtered.
terrain=u._terrain
common_types=torch.div(torch.arange(args.cohort,device=u.device),args.cohort/terrain.terrain_origins.shape[1],rounding_mode="floor").long()
common_levels=terrain.terrain_levels[:args.cohort].clone()
common_origins=terrain.terrain_origins[common_levels,common_types].clone()
for i in range(len(models)):
 sl=slice(i*args.cohort,(i+1)*args.cohort)
 terrain.terrain_types[sl]=common_types
 terrain.terrain_levels[sl]=common_levels
 terrain.env_origins[sl]=common_origins
assert all(torch.equal(terrain.env_origins[i*args.cohort:(i+1)*args.cohort],common_origins) for i in range(len(models)))
policies={};mods={};slices={}
for i,(name,path) in enumerate(models.items()):
 ac=load_cfg_from_registry(task,'rsl_rl_cfg_entry_point');ac.device=str(u.device)
 runner=OnPolicyRunner(wrapped,ac.to_dict(),log_dir=None,device=ac.device);runner.load(path,load_optimizer=False)
 policies[name]=runner.get_inference_policy(device=u.device);mods[name]=runner.alg.policy;slices[name]=slice(i*args.cohort,(i+1)*args.cohort)
out={'meta':{'seed':args.seed,'cohort':args.cohort,'steps':1000,'dt':u.step_dt,'terrain':'generator','obs_delay_mode':'sensors','matched_initial_states':False,'terrain_assignment':'identical canonical cohort origin layout for every policy','terrain_types':common_types.cpu().tolist(),'terrain_levels':common_levels.cpu().tolist(),'models':{k:{'path':v,'sha256':hashlib.sha256(Path(v).read_bytes()).hexdigest()} for k,v in models.items()}},'results':{k:{} for k in models},'gates':{}}
settle=int((cfg.cmd_settle_s+1)/u.step_dt);out['meta']['settle_steps']=settle
for i,(scen,(stage,kind,v,w)) in enumerate(SCENARIOS.items()):
 u.cfg.curriculum_stage=stage;u.cfg.benchmark_disturbance_kind=kind;u.cfg.forced_velocity_cmd_mps=v;u.cfg.forced_yaw_rate_cmd_radps=w
 random.seed(args.seed+i);np.random.seed(args.seed+i);torch.manual_seed(args.seed+i)
 for m in mods.values():m.reset()
 with torch.inference_mode():acc=run_scenario(wrapped,policies,slices,1000,settle)
 for name,sl in slices.items():out['results'][name][scen]=aggregate(acc,sl)
 args.out.write_text(json.dumps(out,indent=2)+'\n')
 print('SCENARIO_DONE',scen,{k:round(out['results'][k][scen]['fall_rate'],3) for k in models},flush=True)
for name,results in out['results'].items():out['gates'][name]={'failures':stage_gate_failures(5,results),'score':checkpoint_score(results)}
args.out.write_text(json.dumps(out,indent=2)+'\n');print('COHORT_BENCHMARK_SUCCESS',out['gates'],flush=True)
env.close();app.close()
