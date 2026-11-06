# Real-plant retrain, seed 43, 2026-11-06

Not flashed and not tested on hardware. Full evidence: `docs/experiments/2026-11-05-real-plant-training/README.md`.

| Folder | Source | Status |
|---|---|---|
| `primary_cmd_1200` | fresh curriculum, `prev_current=command`, `model_1200.pt` | Passes ONNX and STM32AI validation. Use this one first. |
| `backup_model_1125_failed_validation` | fresh curriculum, `prev_current=model`, `model_1125.pt` | Both validations FAILED. Do not deploy without a new decision. |

Each folder holds `policy_drive.onnx` (native GRU, `h_in` shape [1,1,64]) and `policy_drive_stm32ai.onnx`
(expanded GRU, `h_in` shape [1,64]), 13 observations, same layout as the current FixedStance firmware.
