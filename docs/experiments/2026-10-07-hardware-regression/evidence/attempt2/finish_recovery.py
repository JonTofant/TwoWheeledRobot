"""Post-training audit, fixed-final validation/test and export. Never retrains/reselects."""
from pathlib import Path
import hashlib,json,subprocess,sys,time
R=Path('/workspace/TwoWheeledRobot');D=R/'logs/regression_pilot_adaptation_2026-10-08';protocol=json.loads((D/'recovery_protocol.json').read_text());runs=R/'logs/rsl_rl/nn_drive_fixed_stance'
found=list(runs.glob('*_'+protocol['tag']+'_stage5'));assert len(found)==1,found
run=found[0];checkpoint=run/'model_1249.pt';assert checkpoint.is_file(),checkpoint
for f,h in protocol['source_sha256'].items():assert hashlib.sha256((R/f).read_bytes()).hexdigest()==h,f
state={'status':'evaluating','checkpoint':str(checkpoint),'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),'source_parent':protocol['source_parent'],'source_changes_sha256':hashlib.sha256((D/'source_changes.diff').read_bytes()).hexdigest(),'commands':[],'hardware_validated':False}

def save(): (D/'recovery_status.json').write_text(json.dumps(state,indent=2)+'\n')
def cmd(label,*args,marker=None):
 state['phase']=label;state['commands'].append(list(args));save();print('START',label,flush=True)
 with (D/(label+'.log')).open('w') as f:rc=subprocess.run(args,cwd=R,stdout=f,stderr=subprocess.STDOUT).returncode
 if rc or (marker is not None and marker not in (D/(label+'.log')).read_text()):
  state['status']='failed';state['failed_phase']=label;state['returncode']=rc;save();raise SystemExit(label+' failed')
 print('DONE',label,flush=True)
models={'old':str(runs/'2026-08-10_23-51-29_range_gru_stage5/model_775.pt'),'first_recovery':str(runs/'2026-10-07_13-38-09_gru_sensor_timing_recovery_s43_2026-10-07_stage5/model_874.pt'),'pilot':str(runs/'2026-10-06_12-07-53_r3_cd_pilot_20261006T101555Z_range_gru_s43_stage5/model_1150.pt'),'timing_recovery':str(checkpoint)}
(D/'comparison_models.json').write_text(json.dumps(models,indent=2)+'\n')
for label,seed in [('validation',6001),('test',7001)]:
 cmd(label,sys.executable,str(D/'benchmark_cohorts.py'),'--headless','--models',str(D/'comparison_models.json'),'--out',str(D/(label+'.json')),'--seed',str(seed),marker='COHORT_BENCHMARK_SUCCESS')
 result=json.loads((D/(label+'.json')).read_text());state[label+'_gates']=result['gates'];save()
passed=not state['validation_gates']['timing_recovery']['failures'] and not state['test_gates']['timing_recovery']['failures']
state['passes_both_gates']=passed
state['symptom_screen']={}
for label in ['validation','test']:
 result=json.loads((D/(label+'.json')).read_text())['results']
 parent=result['pilot']['station_keeping']['mean_abs_command_delta_a']
 candidate=result['timing_recovery']['station_keeping']['mean_abs_command_delta_a']
 state['symptom_screen'][label]={'parent_delta_a':parent,'candidate_delta_a':candidate,'ratio':candidate/parent,'passes':candidate <= .75*parent}
passed = passed and all(v['passes'] for v in state['symptom_screen'].values())
state['passes_deployment_screen']=passed;save()
if not passed:
 state['status']='completed_not_qualified';save();print('RECOVERY_NOT_QUALIFIED',flush=True);sys.exit(0)
cmd('export_actor',sys.executable,'scripts/rsl_rl/play.py','--task','Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0','--checkpoint',str(checkpoint),'--num_envs','1','--num_steps','1','--headless')
out=D/'exports';out.mkdir(exist_ok=True)
cmd('export_current',sys.executable,'scripts/export_pure_nn_current_onnx.py','--policy',str(run/'exported/policy.pt'),'--actor-onnx',str(run/'exported/policy.onnx'),'--output',str(out/'policy_drive.onnx'),'--obs-dim','13','--cg-outputs','0','--i-max-a','2.0','--require-validation')
cmd('expand_gru',sys.executable,'scripts/tools/expand_gru_for_stm32ai.py',str(out/'policy_drive.onnx'),str(out/'policy_drive_stm32ai.onnx'),'--steps','1024')
state['status']='completed_exported_candidate';state['native_onnx_sha256']=hashlib.sha256((out/'policy_drive.onnx').read_bytes()).hexdigest();state['stm32_onnx_sha256']=hashlib.sha256((out/'policy_drive_stm32ai.onnx').read_bytes()).hexdigest();save();print('RECOVERY_EVALUATION_EXPORT_SUCCESS',flush=True)
