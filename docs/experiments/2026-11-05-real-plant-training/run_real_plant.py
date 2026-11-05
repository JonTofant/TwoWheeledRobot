"""Train on the Isaac plant with wheel Coulomb friction and the DDSM115 velocity filter.

Protocol fixed before any result (2026-11-05). Run inside isaac-lab-dev:
    /isaac-sim/python.sh docs/experiments/2026-11-05-real-plant-training/run_real_plant.py
Do not edit source while this runs: every stage starts a fresh process that imports it.
Arms run in order; a failed arm is recorded and the next still runs. No selection here: all
checkpoints are kept and ranked afterwards on the independent MuJoCo plant.
"""
import json, subprocess, sys, time
from pathlib import Path
repo = Path(__file__).resolve().parents[3]
py = sys.executable
TASK = "Template-Twowheeledrobot-NNDriveFixedStanceGRU-v0"
EXP = "nn_drive_fixed_stance"
tag = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
out = repo / "logs" / f"real_plant_{tag}"; out.mkdir(parents=True)
SEED = 43
PLANT = ["env.wheel_coulomb_friction_nm_range=[0.01,0.06]", "env.velocity_obs_filter=ddsm115", "env.obs_delay_mode=sensors", "agent.action_std_floor=[0.15,0.15]"]
CMD = ["env.prev_current_obs_source=command"]; MODEL = ["env.prev_current_obs_source=model"]
BUDGET = ["300", "350", "200", "250", "150"]
curric = lambda prefix, extra: [py, "scripts/train_nn_drive_curriculum.py", "--task", TASK, "--experiment-name", EXP, "--run-name-prefix", prefix, "--num_envs", "4096", "--iterations", *BUDGET,
                                "--selection", "last", "--seed", str(SEED), "--skip-export", "--headless", *PLANT, *extra]
ARMS = [("warm", [py, "scripts/rsl_rl/train.py", "--task", TASK, "--num_envs", "4096", "--max_iterations", "300", "--run_name", f"real_warm_s{SEED}_{tag}", "--seed", str(SEED), "--headless",
                  "--resume", "--load_run", "2026-08-10_23-51-29_range_gru_stage5", "--checkpoint", "model_775.pt", "--reset_optimizer",
                  "env.curriculum_stage=5", "env.terrain_mode=generator", "agent.algorithm.learning_rate=5e-5", "agent.algorithm.schedule=fixed", *PLANT, *CMD]),
        ("fresh_command", curric(f"real_fresh_cmd_s{SEED}_{tag}", CMD)),
        ("fresh_model", curric(f"real_fresh_model_s{SEED}_{tag}", MODEL))]
status = {"tag": tag, "seed": SEED, "plant": PLANT, "arms": {}}
(out / "source.diff").write_bytes(subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=repo))
for name, cmd in ARMS:
    t0 = time.time(); status["arms"][name] = {"cmd": cmd, "state": "running"}
    (out / "status.json").write_text(json.dumps(status, indent=1))
    with open(out / f"{name}.log", "wb") as f:
        rc = subprocess.run(cmd, cwd=repo, stdout=f, stderr=subprocess.STDOUT).returncode
    status["arms"][name].update(state="done" if rc == 0 else f"failed rc={rc}", hours=(time.time() - t0) / 3600)
    (out / "status.json").write_text(json.dumps(status, indent=1))
    print(name, status["arms"][name]["state"], flush=True)
print("ALL_ARMS_FINISHED", flush=True)
