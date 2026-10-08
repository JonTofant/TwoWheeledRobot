import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import onnxruntime as ort
root=Path('/workspace/TwoWheeledRobot')
work=root/'logs/hardware_c_pilot_2026-10-07'
rows=list(csv.DictReader((work/'C_pilot_test.csv').open()))
active=[r for r in rows if not(int(r['flags'])&2)]
pkg=root/'ExportedPolicy/cd_pilot_s43_2026-10-06'
models={'C_pilot':pkg/'C_GRU/policy_drive_stm32ai.onnx','D_pilot':pkg/'D_Wide_MLP/policy_drive_stm32ai.onnx','original_GRU':pkg/'Rollback/rollback_old_gru_stm32ai.onnx'}
recovery=root/'ExportedPolicy/gru_recovery_floor015_s43_2026-10-06_retry1'
for p in recovery.rglob('*stm32ai.onnx'):
    if 'rollback' not in p.name:
        models['recovered_GRU']=p
        break
out={}
for label,path in models.items():
    options=ort.SessionOptions();options.intra_op_num_threads=1
    session=ort.InferenceSession(str(path),options,providers=['CPUExecutionProvider'])
    hidden_inputs=[i for i in session.get_inputs() if i.name!='obs']
    h=np.zeros(hidden_inputs[0].shape,dtype=np.float32) if hidden_inputs else None
    predictions=[]
    for r in active:
        obs=np.array([[float(r[f'obs_{j}']) for j in range(13)]],dtype=np.float32)
        feed={'obs':obs}
        if h is not None: feed[hidden_inputs[0].name]=h
        result=session.run(None,feed);predictions.append(result[0][0].tolist())
        if h is not None: h=result[1]
    p=np.array(predictions);actual=np.array([[float(r['action_left']),float(r['action_right'])] for r in active])
    error=p-actual
    out[label]={'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'max_abs_difference_A':float(np.max(np.abs(error))),'rms_difference_A':float(np.sqrt(np.mean(error**2))),'first_predicted_A':predictions[0],'first_recorded_A':actual[0].tolist(),'outputs_on_recorded_observations_A':predictions}
    print(label, 'max difference',out[label]['max_abs_difference_A'],'rms',out[label]['rms_difference_A'],'first',predictions[0])
(work/'model_replay.json').write_text(json.dumps(out,indent=2)+'\n')
