# fresh_model model_1125, seed 43

Candidate for the next hardware session. **Not yet run on the robot.** Results and reasoning:
`docs/experiments/2026-11-05-real-plant-training/README.md`.

- Flash `policy_drive_stm32ai.onnx`; 13 observations, 2 wheel currents, 64-unit GRU, same contract as before.
- Trained with prev_current=model; firmware feeds the command. Both test plants used the firmware behaviour.
- `export.log` and `stm32ai_equivalence.log` show a FAILED tolerance check on white-noise input. Understood,
  see the README: agreement on realistic observations is 2.2e-6 A and 7.5e-7 A. This was not bypassed.
- Expect far more current switching than the August GRU (0.24-0.32 vs 0.02-0.09 A per tick). Stop if motors
  heat up or the robot buzzes.
