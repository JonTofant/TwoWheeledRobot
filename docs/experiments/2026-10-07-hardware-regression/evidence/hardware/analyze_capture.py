from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

work=Path(__file__).resolve().parent
d=pd.read_csv(work/'C_pilot_test.csv')
fallen=(d['flags'].astype(int)&2)!=0
active=d.loc[~fallen].copy()
start=float(active.time_s.iloc[0]);fall=float(d.loc[active.index[-1]+1,'time_s'])
replay=json.loads((work/'model_replay.json').read_text())
probe=json.loads((work/'action_history_probe.json').read_text())
summary={
 'telemetry_commit':'5b4b481',
 'firmware_commit':subprocess.check_output(['git','-C','/home/jon/Projects/TwoWheeledRobot_Embedded','rev-parse','origin/feature/mdpi-actuators'],text=True).strip(),
 'csv_sha256':hashlib.sha256((work/'C_pilot_test.csv').read_bytes()).hexdigest(),
 'rows':len(d),'recording_duration_s':float(d.time_s.iloc[-1]-d.time_s.iloc[0]),
 'active_rows':len(active),'rearm_time_s':start,'fall_time_s':fall,'rearm_to_fall_s':fall-start,
 'upright_threshold_rearm_pitch_deg':float(np.degrees(active.pitch_rad.iloc[0])),
 'fall_pitch_deg':float(np.degrees(d.loc[active.index[-1]+1,'pitch_rad'])),
 'command_velocity_max_abs_mps':float(abs(active.obs_7).max()),
 'command_yaw_max_abs_radps':float(abs(2*active.obs_8).max()),
 'testbench_running_frames':int(d.testbench_running.sum()),
 'physical_fall_transitions':int(((~fallen.shift(fill_value=True))&fallen).sum()),
 'testbench_reported_falls':int(d.testbench_fall_count.max()),
 'max_abs_velocity_mps':float(abs(active.velocity_mps).max()),
 'velocity_rms_zero_command_mps':float(np.sqrt(np.mean(active.velocity_mps**2))),
 'pitch_rms_deg':float(np.sqrt(np.mean(np.degrees(active.pitch_rad)**2))),
 'pre_fall_odometry_change_m':float(active.position_m.iloc[-1]-active.position_m.iloc[0]),
 'pre_fall_unwrapped_yaw_change_deg':float(np.degrees(np.unwrap(active.yaw_rad)[-1]-np.unwrap(active.yaw_rad)[0])),
 'max_abs_command_current_A':float(abs(active[['action_left','action_right']]).max().max()),
 'last_current_commands_logical_A':active[['action_left','action_right']].iloc[-1].tolist(),
 'last_current_feedback_logical_A':[-float(active.wheel_right_current_measured_A.iloc[-1]),float(active.wheel_left_current_measured_A.iloc[-1])],
 'inference_mean_us':float(active.inference_last_us.mean()),'inference_max_us':float(active.inference_last_us.max()),
 'control_body_max_us':float(active.total_last_us.max()),
 'control_period_min_us':float(active.control_period_last_us.min()),'control_period_max_us':float(active.control_period_last_us.max()),
 'wheel_age_median_us':float(active.wheel_age_last_us.median()),
 'finite_numeric_values':bool(np.isfinite(d.select_dtypes(include=np.number)).all().all()),
 'sequence_complete':bool((d.sequence.diff().dropna()==1).all()),
 'telemetry_dropped_frames_max':int(d.dropped_frames.max()),
 'replay_C_max_abs_current_difference_A':replay['C_pilot']['max_abs_difference_A'],
 'replay_C_rms_difference_A':replay['C_pilot']['rms_difference_A'],
 'observations_verified':{},'references':{},
}
for name,col,source,scale in [('pitch','obs_2','pitch_rad',.43633231),('pitch_rate','obs_3','pitch_rate_radps',4),('velocity','obs_1','velocity_mps',1),('roll','obs_11','roll_rad',.43633231),('yaw_rate','obs_6','yaw_rate_world_radps',4)]:
 summary['observations_verified'][name+'_max_abs_error']=float(abs(active[col]-active[source]/scale).max())
for wheel,col in [('left','obs_9'),('right','obs_10')]:
 summary['observations_verified']['previous_'+wheel+'_max_abs_error']=float(abs(active[col].iloc[1:].to_numpy()-active['action_'+wheel].iloc[:-1].to_numpy()/2).max())
for name in ['gru_testbench.csv','range_gru_segmented.csv']:
 old=pd.read_csv(work/name);f=(old['flags'].astype(int)&2)!=0;running=(old['flags'].astype(int)&8)!=0
 summary['references'][name]={'fall_frames':int(f.sum()),'testbench_running_sample_s':float(running.sum()*.015),'exact_model_build_identity':'not supplied; contextual historical recording, not matched replicate'}
(work/'analysis_summary.json').write_text(json.dumps(summary,indent=2)+'\n')

plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
x=d.time_s-start;show=(x>=-.25)&(x<=fall-start+.3)
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
axs[0].plot(x[show],np.degrees(d.pitch_rad[show]),color='#ae2837',label='Pitch')
axs[0].axhline(np.degrees(.60),color='#777',ls=':',label='Fall latch threshold')
axs[0].set_ylabel('Pitch (degrees)');axs[0].legend(loc='upper left',ncol=2)
axs[1].plot(x[show],d.velocity_mps[show],color='#216aa0',label='Measured wheel-odometry velocity')
axs[1].axhline(0,color='#666',ls=':',label='Velocity command = 0')
axs[1].set_ylabel('Velocity (m/s)');axs[1].legend(loc='upper left')
command=(d.action_left+d.action_right)/2
# Current feedback converted back from the established physical destination/sign mapping.
feedback=(d.wheel_left_current_measured_A-d.wheel_right_current_measured_A)/2
cmd=command.where(~fallen,0)
axs[2].plot(x[show],cmd[show],label='Logical common current command',color='#ae7c16')
axs[2].plot(x[show],feedback[show],label='Logical common current feedback',color='#358758',alpha=.85)
axs[2].set_ylabel('Current (A)');axs[2].legend(loc='upper left');axs[2].set_xlabel('Seconds since upright rearm')
for ax in axs:
 ax.axvline(fall-start,color='#ae2837',ls='--');ax.grid(alpha=.15);ax.set_xlim(-.25,fall-start+.3)
fig.suptitle('C pilot: zero-command standing failed after 5.336 s\nNo automatic testbench stage started')
fig.savefig(work/'standing_failure.png',dpi=170);fig.savefig(work/'standing_failure.svg');plt.close(fig)

# Comparison is open-loop on one recorded trajectory, not a prediction of counterfactual balance.
window=(active.time_s>=18.2)&(active.time_s<=18.65)
fig,axs=plt.subplots(2,1,figsize=(10,5.5),sharex=True,layout='constrained')
for label,color,legend in [('C_pilot','#ae2837','New C'),('original_GRU','#216aa0','Original GRU (offline replay)')]:
 pred=np.array(replay[label]['outputs_on_recorded_observations_A'])
 for i,ax in enumerate(axs):ax.plot(active.time_s[window]-start,pred[window,i],color=color,label=legend)
for i,ax in enumerate(axs):ax.set_ylabel(('Left' if i==0 else 'Right')+' current (A)');ax.legend(loc='upper left');ax.grid(alpha=.15)
axs[1].set_xlabel('Seconds since upright rearm')
fig.suptitle('Every-other-tick current variation before the fall\nSame recorded observations; reference replay does not simulate hardware recovery')
fig.savefig(work/'current_alternation.png',dpi=170);plt.close(fig)
print(json.dumps({k:summary[k] for k in ['rearm_to_fall_s','max_abs_velocity_mps','pre_fall_odometry_change_m','replay_C_max_abs_current_difference_A','inference_max_us']},indent=2))
