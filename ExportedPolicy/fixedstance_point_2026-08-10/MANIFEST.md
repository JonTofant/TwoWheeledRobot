# fixedstance_point_2026-08-10

Archived from: /workspace/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_11-43-33_point_stage1, /workspace/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_12-20-49_point_stage2, /workspace/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_13-08-59_point_stage3, /workspace/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_13-44-25_point_stage4, /workspace/TwoWheeledRobot/logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_14-52-51_point_stage5
Repo commit at archive time: 591e77b17d9c3b8bd3c23df489846bacdd1140bc
Repo dirty at archive time: yes -- see below

```
M scripts/archive_run_for_paper.py
 M scripts/export_pure_nn_current_onnx.py
?? .claude/
?? ExportedPolicy/fixedstance_point_2026-08-10/
?? ExportedPolicy/sim2real/
```

Each subfolder below is one training run (one curriculum stage, or a
standalone run). See its params/env.yaml and params/agent.yaml for the
exact resolved config and seed, git/*.diff for the exact code state,
and selected_checkpoint.json / benchmark_selection/*.json for raw
benchmark results. See scripts/archive_run_for_paper.py for what was
deliberately excluded (intermediate model_*.pt checkpoints).
