import csv,json,hashlib
from pathlib import Path
import torch,numpy as np
R=Path('/workspace/TwoWheeledRobot');torch.set_num_threads(1)
rows=[r for r in csv.DictReader((R/'logs/hardware_c_pilot_2026-10-07/C_pilot_test.csv').open()) if not int(r['flags'])&2]
runs=R/'logs/rsl_rl/nn_drive_fixed_stance'
models={'old':runs/'2026-08-10_23-51-29_range_gru_stage5/model_775.pt','recovery':runs/'2026-10-06_07-07-54_gru_recovery_floor015_s43_2026-10-06_retry1_stage5/model_924.pt'}
for d in sorted(runs.glob('*r3_cd_pilot*range_gru_s43_stage*')):
 p=max(d.glob('model_*.pt'),key=lambda p:int(p.stem.split('_')[-1]));models[d.name.split('_')[-1]]=p
models['pilot_selected']=runs/'2026-10-06_12-07-53_r3_cd_pilot_20261006T101555Z_range_gru_s43_stage5/model_1150.pt'
models['timing_recovery']=runs/'2026-10-07_13-38-09_gru_sensor_timing_recovery_s43_2026-10-07_stage5/model_874.pt'
result={}
for name,path in models.items():
 s=torch.load(path,map_location='cpu',weights_only=False)['model_state_dict'];gru=torch.nn.GRU(13,64);head=torch.nn.Sequential(torch.nn.Linear(64,64),torch.nn.ReLU(),torch.nn.Linear(64,64),torch.nn.ReLU(),torch.nn.Linear(64,2))
 gru.load_state_dict({k.removeprefix('memory_a.rnn.'):v for k,v in s.items() if k.startswith('memory_a.rnn.')});head.load_state_dict({k.removeprefix('actor.'):v for k,v in s.items() if k.startswith('actor.')})
 h=torch.zeros(1,1,64);actions=[];jacs=[]
 for row in rows:
  obs=torch.tensor([float(row[f'obs_{i}']) for i in range(13)]).reshape(1,1,13)
  # Direct action-history response at the observed input and replayed state;
  # NOT a closed-loop physical stability estimate.
  if int(row['sequence'])%10==0:
   def fun(prev):
    x=obs.clone();x[0,0,9:11]=prev;y,_=gru(x,h);return torch.tanh(head(y[0]))[0]
   j=torch.autograd.functional.jacobian(fun,obs[0,0,9:11]);jacs.append(float(torch.linalg.eigvals(j).abs().max()))
  with torch.no_grad():
   y,h=gru(obs,h);a=2*torch.tanh(head(y[0]));actions.append(a[0].numpy())
 a=np.array(actions);seq=np.array([int(r['sequence']) for r in rows]);window=(seq>=918)&(seq<=948)
 result[name]={'checkpoint':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'replay_adjacent_delta_A':float(abs(np.diff(a,axis=0)).mean()),'replay_window_seq918_948_delta_A':float(abs(np.diff(a[window],axis=0)).mean()),'direct_history_jacobian_radius_median':float(np.median(jacs)),'direct_history_jacobian_radius_max':max(jacs),'note':'Open-loop on failed pilot trajectory, not an alternative physical trajectory or proof of stability.'}
 print(name,result[name],flush=True)
(R/'logs/regression_diagnosis_2026-10-07/checkpoint_feedback_with_recovery.json').write_text(json.dumps(result,indent=2)+'\n')
