#!/usr/bin/env python3
"""Frozen policies, simultaneous independent robot cohorts. Diagnostic, not paper test."""
import argparse, sys
from pathlib import Path
R=Path('/workspace/TwoWheeledRobot')
sys.path.insert(0,str(R/'source/TwoWheeledRobot'))
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser(); p.add_argument('--out',type=Path,required=True); p.add_argument('--steps',type=int,default=1000);p.add_argument('--cohort',type=int,default=32);p.add_argument('--seed',type=int,default=3001)
AppLauncher.add_app_launcher_args(p);args,rest=p.parse_known_args();sys.argv=[sys.argv[0]]+rest
app=AppLauncher(args).app
import json,random,hashlib,time
import torch,numpy as np,gymnasium as gym
import TwoWheeledRobot.tasks
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner
TASK='Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0'
RUNS=R/'logs/rsl_rl/nn_drive_fixed_stance'
MODELS={
'old':RUNS/'2026-08-10_23-51-29_range_gru_stage5/model_775.pt',
'recovery':RUNS/'2026-10-06_07-07-54_gru_recovery_floor015_s43_2026-10-06_retry1_stage5/model_924.pt',
'pilot':RUNS/'2026-10-06_12-07-53_r3_cd_pilot_20261006T101555Z_range_gru_s43_stage5/model_1150.pt'}
# motor delay, sensor delay, whether commands/history stay fresh, damping
CASES=[('as_trained',-1,-1,False,.0007)]
for motor,sensor in [(0,0),(1,0),(2,0),(3,0),(1,1),(2,1),(3,1),(2,2)]:
 CASES.append((f'a{motor}_sensor{sensor}_fresh',motor,sensor,True,.0007))
CASES += [('a1_o1_whole',1,1,False,.0007),('a2_o1_whole',2,1,False,.0007),('a2_sensor1_old_damping',2,1,True,.01)]
random.seed(args.seed);np.random.seed(args.seed);torch.manual_seed(args.seed)
cfg=load_cfg_from_registry(TASK,'env_cfg_entry_point');cfg.seed=args.seed;cfg.scene.num_envs=len(CASES)*len(MODELS)*args.cohort
cfg.terrain_mode='flat';cfg.forced_command_mode='fixed';cfg.forced_velocity_cmd_mps=0.;cfg.forced_yaw_rate_cmd_radps=0.;cfg.curriculum_stage=5;cfg.benchmark_disturbance_kind='none';cfg.episode_length_s=25.
cfg.obs_delay_steps_range=(0,0);cfg.action_delay_steps_range=(0,0)
env=gym.make(TASK,cfg=cfg);w=RslRlVecEnvWrapper(env,clip_actions=None);u=env.unwrapped;dev=u.device;n=u.num_envs
policies={};mods={};slices={}
for i,(name,path) in enumerate(MODELS.items()):
 ac=load_cfg_from_registry(TASK,'rsl_rl_cfg_entry_point');ac.device=str(dev)
 runner=OnPolicyRunner(w,ac.to_dict(),log_dir=None,device=str(dev));runner.load(str(path),load_optimizer=False)
 policies[name]=runner.get_inference_policy(device=dev);mods[name]=runner.alg.policy;slices[name]=slice(i*len(CASES)*args.cohort,(i+1)*len(CASES)*args.cohort)
adel=torch.zeros(n,dtype=torch.long,device=dev);odel=adel.clone();fresh=torch.zeros(n,dtype=torch.bool,device=dev);damp=torch.zeros(n,device=dev);trained=torch.zeros_like(fresh)
for mi,name in enumerate(MODELS):
 for ci,(_,a,o,f,d) in enumerate(CASES):
  s=slice((mi*len(CASES)+ci)*args.cohort,(mi*len(CASES)+ci+1)*args.cohort);adel[s]=max(a,0);odel[s]=max(o,0);fresh[s]=f;damp[s]=d;trained[s]=a<0
rows=torch.arange(n,device=dev);ah=torch.zeros(n,4,2,device=dev);oh=torch.zeros(n,3,13,device=dev)
orig_reset=u._reset_idx;orig_obs=u._get_observations;ap=u._action_processor;orig_proc=ap.process

def reset(ids):
 orig_reset(ids)
 if ids is None: ids=rows
 ids=torch.as_tensor(ids,device=dev,dtype=torch.long)
 ah[ids]=0;oh[ids]=0
 t=ids[trained[ids]];adel[t]=torch.randint(0,2,(len(t),),device=dev);odel[t]=torch.randint(0,2,(len(t),),device=dev)
 jd=u.robot.data.joint_damping[ids].clone();jd[:,u._wheel_ids]=damp[ids,None]
 u.robot.write_joint_damping_to_sim(jd,env_ids=ids.cpu())

def process(raw):
 ah[:,1:]=ah[:,:-1].clone();ah[:,0]=raw[:,:2]
 val=orig_proc(ah[rows,adel]);ap.net_current=torch.tanh(raw[:,:2])*u.cfg.i_max_a
 return val

def observations():
 out=orig_obs();now=out['policy'];oh[:,1:]=oh[:,:-1].clone();oh[:,0]=now
 obs=oh[rows,odel].clone()
 # Firmware reference/commands and own previous action are computed at the current tick.
 obs[fresh,0]=now[fresh,0];obs[fresh,4:6]=now[fresh,4:6];obs[fresh,7:11]=now[fresh,7:11]
 out['policy']=obs;return out
u._reset_idx=reset;u._get_observations=observations;ap.process=process
out={'meta':{'source_commit':'98658ede576b64a9d19ad7715622e45a3748e59a','seed':args.seed,'cohort':args.cohort,'steps':args.steps,'dt':u.step_dt,'paired_initial_states':False,'note':'Independent randomized cohorts; frozen sensitivity, not proof of physical latency. Sensor delay also changes state-derived odometry but references/current remain current.','models':{k:{'path':str(v),'sha256':hashlib.sha256(v.read_bytes()).hexdigest()} for k,v in MODELS.items()},'cases':CASES},'results':{}}
with torch.inference_mode():
 obs,_=w.reset()
 for m in mods.values():m.reset()
 alive=torch.ones(n,dtype=torch.bool,device=dev);fall=torch.zeros_like(alive);invalid=fall.clone();broken=fall.clone();surv=torch.zeros(n,device=dev);count=surv.clone();vs=surv.clone();ps=surv.clone();cs=surv.clone();ds=surv.clone();peakv=surv.clone();drift=surv.clone();prev=torch.zeros(n,2,device=dev);act=prev.clone()
 for step in range(args.steps):
  for name,sl in slices.items():act[sl]=policies[name](obs[sl])
  cur=torch.tanh(act)*2;delta=(cur-prev).abs().mean(1);prev=cur.clone()
  obs,_,done,_=w.step(act);done=done.flatten().bool()
  for name,sl in slices.items():mods[name].reset(done[sl])
  _,v,pitch,_,_,_=u._state_terms();a=alive&~done
  fall |= alive & u._last_step_fall;invalid |= alive & u._last_step_invalid_state;broken |= alive & u._last_step_physics_broken
  surv[alive]=(step+1)*u.step_dt;count+=a;vs+=torch.where(a,v*v,0);ps+=torch.where(a,pitch*pitch,0);cs+=torch.where(a,cur.square().mean(1),0);ds+=torch.where(a,delta,0);peakv=torch.where(a,torch.maximum(peakv,v.abs()),peakv)
  drift=torch.where(a,torch.linalg.vector_norm(u.robot.data.root_pos_w[:,:2]-u._spawn_pos_xy,dim=1),drift);alive &= ~done
  if step%100==0:print('STEP',step,'alive',alive.sum().item(),'time',time.time(),flush=True)
 for mi,name in enumerate(MODELS):
  out['results'][name]={}
  for ci,(case,*_) in enumerate(CASES):
   sl=slice((mi*len(CASES)+ci)*args.cohort,(mi*len(CASES)+ci+1)*args.cohort);den=count[sl].clamp_min(1)
   out['results'][name][case]={'fall_rate':fall[sl].float().mean().item(),'invalid_rate':invalid[sl].float().mean().item(),'physics_broken_rate':broken[sl].float().mean().item(),'survival_s':surv[sl].mean().item(),'sample_coverage':(count[sl]/args.steps).mean().item(),'rms_velocity_mps':torch.sqrt(vs[sl]/den).mean().item(),'rms_pitch_deg':torch.sqrt(ps[sl]/den).mean().item()*180/np.pi,'rms_command_a':torch.sqrt(cs[sl]/den).mean().item(),'mean_abs_command_delta_a':(ds[sl]/den).mean().item(),'mean_peak_speed_mps':peakv[sl].mean().item(),'final_drift_m':drift[sl].mean().item()}
args.out.write_text(json.dumps(out,indent=2)+'\n');print('LATENCY_SWEEP_SUCCESS',flush=True)
env.close();app.close()
