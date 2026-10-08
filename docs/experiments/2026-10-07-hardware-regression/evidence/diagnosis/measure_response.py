import json
from pathlib import Path
import numpy as np,pandas as pd
ROOT=Path('logs/hardware_c_pilot_2026-10-07');out={}
for fn in ['C_pilot_test.csv','gru_testbench.csv','gru_test.csv']:
 d=pd.read_csv(ROOT/fn);res={}
 for side in ['left','right']:
  cols=[f'wheel_{side}_current_command_A',f'wheel_{side}_current_measured_A']
  if not set(cols)<=set(d):continue
  cmd=d[cols[0]].to_numpy();meas=d[cols[1]].to_numpy();valid=(d['flags'].to_numpy().astype(int)&2)==0
  for vmax in [.2,.5,10]:
   fits=[]
   for lag in range(6):
    idx=np.arange(max(lag,1),len(d));ok=valid[idx]&valid[idx-lag]&(abs(d['velocity_mps'].to_numpy()[idx])<vmax)&(abs(cmd[idx-lag])>.04)
    ids=idx[ok];x=cmd[ids-lag];y=meas[ids]
    if len(ids)<20:continue
    X=np.column_stack([x,np.ones_like(x)]);slope,bias=np.linalg.lstsq(X,y,rcond=None)[0];resid=y-X@np.array([slope,bias]);r2=1-(resid@resid)/np.sum((y-y.mean())**2)
    fits.append(dict(lag_ticks=lag,n=len(x),slope=slope,bias_A=bias,r2=r2,rms_residual_A=float(np.sqrt(np.mean(resid**2)))))
   res[f'{side}_v_below_{vmax}']=fits
 out[fn]=res
Path('logs/regression_diagnosis_2026-10-07/current_response_fit.json').write_text(json.dumps(out,indent=2)+'\n')
for fn,res in out.items():
 print(fn)
 for k,fits in res.items():
  if fits:print(k,max(fits,key=lambda x:x['r2']))
