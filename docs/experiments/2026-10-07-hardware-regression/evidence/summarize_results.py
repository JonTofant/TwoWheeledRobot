"""Archive completed frozen experiments; never selects checkpoints or changes gates."""
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path('/home/jon/Projects/TwoWheeledRobot')
DOC = ROOT / 'docs/experiments/2026-10-07-hardware-regression'
folders = [ROOT / 'logs/regression_diagnosis_2026-10-07', ROOT / 'logs/regression_pilot_adaptation_2026-10-08']
labels = {'old': 'Original GRU 775', 'recovery': 'Earlier recovery 924', 'first_recovery': 'First adaptation 874', 'pilot': 'Failed fresh pilot 1150', 'timing_recovery': 'Adaptation final'}
lines = ['# Frozen recovery results', '', 'These are development adaptations evaluated under the corrected sensor-timing contract. Original models are evaluated under that same corrected simulator; these figures do not reproduce their historical training conditions or rank their physical performance. Cohorts share the same terrain layout but have independent initial-state and motor draws. No intermediate checkpoint selection or gate relaxation was used.', '', 'Falls, survival and coverage precede alive-masked tracking. There are 64 robots per scenario, 11 scenarios and 15 s per trial. Each split has 704 initial episodes. Current RMS is simulated post-actuator-model effective current; command change is the logical network-output change. Neither is a direct measurement of hardware torque.', '']
summary = {}
for i, folder in enumerate(folders, 1):
    state = json.loads((folder / 'recovery_status.json').read_text())
    assert state['status'] in ('completed_not_qualified', 'completed_exported_candidate'), state['status']
    lines += [f'## Attempt {i}', '', f"Final checkpoint: `{Path(state['checkpoint']).name}`. SHA256: `{state['checkpoint_sha256']}`. Status: **{state['status']}**.", '']
    summary[str(i)] = state
    for split in ['validation', 'test']:
        d = json.loads((folder / f'{split}.json').read_text())
        assert 'COHORT_BENCHMARK_SUCCESS' in (folder / f'{split}.log').read_text()
        n = d['meta']['cohort']; steps = d['meta']['steps']; scored = steps - d['meta']['settle_steps']
        lines += [f"### {split.capitalize()} — base seed {d['meta']['seed']}", '', '| Model | Falls / 704 | Mean survival (s) | Minimum sample coverage (%) | Gate failures |', '|---|---:|---:|---:|---|']
        for name, results in d['results'].items():
            assert len(results) == 11
            vals = list(results.values())
            falls = sum(round(v['fall_rate']*n) for v in vals)
            survival = sum(v['survival_time_s'] for v in vals) / len(vals)
            coverage = min(v['mean_scored_samples']/scored for v in vals)*100
            invalid = sum(v['invalid_state_rate'] + v['physics_broken_rate'] for v in vals)
            assert invalid == 0, (name, invalid)
            failed = '; '.join(d['gates'][name]['failures']) or 'None'
            lines.append(f'| {labels[name]} | {falls} | {survival:.3f} | {coverage:.1f} | {failed} |')
        lines += ['', 'No invalid-state or physics-broken terminations occurred in this split. Coverage is the smallest scored-sample fraction across scenarios, after the common settling exclusion.', '', '| Model / scenario | Falls (%) | Survival (s) | Velocity error RMS (m/s) | Yaw-rate error RMS (rad/s) | Final world drift (m) | Effective current RMS (A) | Command change (A/tick) |', '|---|---:|---:|---:|---:|---:|---:|---:|']
        for name, results in d['results'].items():
            for scenario, v in results.items():
                lines.append(f"| {labels[name]} / {scenario} | {100*v['fall_rate']:.1f} | {v['survival_time_s']:.3f} | {v['rms_vel_err_mps']:.4f} | {v['rms_yaw_rate_err_radps']:.4f} | {v['world_drift_m']:.4f} | {v['rms_current_a']:.4f} | {v['mean_abs_command_delta_a']:.4f} |")
        lines.append('')
    if 'symptom_screen' in state:
        lines += ['### Predeclared standing-current screen', '', 'Required: candidate mean absolute command change no greater than 75% of the parent pilot on both splits, in addition to all stage-5 gates.', '']
        for split, s in state['symptom_screen'].items():
            lines += [f"- {split}: parent {s['parent_delta_a']:.4f} A/tick; candidate {s['candidate_delta_a']:.4f} A/tick; ratio {s['ratio']:.3f}; pass={s['passes']}."]
        lines.append('')
    evidence = DOC / 'evidence' / f'attempt{i}'
    evidence.mkdir(parents=True, exist_ok=True)
    for name in ['recovery_protocol.json', 'recovery_status.json', 'validation.json', 'test.json', 'benchmark_cohorts.py', 'finish_recovery.py', 'source_changes.diff']:
        shutil.copy2(folder / name, evidence / name)
lines += ['## Physical interpretation', '', 'The supplied fresh-C capture contains one unforced standing failure, 5.336 s after rearm, before testbench start. Its network outputs match the expected graph to 7.2e-7 A. The observation-timing mismatch is a confirmed implementation issue; its complete causal contribution to the fall remains unproven. No new physical trial was performed by this investigation.', '', 'The unequal-terrain results from the first harness version are excluded and retained under `logs/regression_diagnosis_2026-10-07/invalid_unbalanced_terrain`. All tables above use the repaired, explicitly balanced terrain layout.', '', 'The original hardware archive remains usable as implementation-bound feasibility evidence with disclosed limitations. See PAPER_PATH.md. No failed seed, failed capture or unsuccessful adaptation is removed. Warm-start adaptation is not an independent training replicate.']
(DOC / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
(DOC / 'evidence/summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(DOC / 'RESULTS.md')
