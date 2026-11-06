# Retraining on a plant with wheel friction and the DDSM115 velocity filter

Run tag `20261105T113720Z`, seed 43, protocol in `run_real_plant.py`. Follows
`../2026-11-05-hardware-regression-mujoco/`. **No policy from this run has been on the robot.**

## Outcome

All three arms finished without errors (noise std 0.15). The two fresh curricula reduce predicted station-keeping
wobble to about half of the August GRU, on two independent plants. The warm start does not.

The first candidate is `fresh_command` `model_1200.pt`, exported to `ExportedPolicy/real_plant_s43_2026-11-06/primary_cmd_1200`.

Commit `fb745b5` names `model 1125` as selected in its message. That is superseded: `cmd 1200` is primary, chosen for equal
pitch SD, 2-3 times less current switching and a passing export, on the condition that the first test uses a centred deck.
Its export was rechecked independently: closed-loop MuJoCo pitch SD 1.09 deg for the ONNX and 1.10 for the checkpoint, and
ONNX against STM32 graph within 2.5e-7 A over 60 s of realistic observations.

| Check | AUG 775 | cmd 1200 (primary) | model 1125 (backup) |
|---|---|---|---|
| MuJoCo fit plant, pitch SD deg | 2.69 | 1.11 | 1.31 |
| MuJoCo, 5 ms motor lag | 2.64 | 1.29 | 1.44 |
| MuJoCo, friction 0.075 Nm | 5.73 | 2.62 | 2.22 |
| MuJoCo, friction 0.01 Nm | 1.22 | 0.69 | 0.68 |
| MuJoCo, deadzone 0.08 A | 3.06 | 1.60 | 1.55 |
| MuJoCo, COM +1 cm | 2.32 | 1.09, 8% falls | 1.36 |
| MuJoCo, COM +-2 cm (20 s start test) | no falls | 100% falls | no falls |
| Isaac station keeping, pitch SD | 2.56 | 1.02 | 1.04 |
| Isaac payload while still, pitch SD | 3.36 | 1.98 | 2.13 |
| Isaac, speed at +0.30 m/s | 0.295 | 0.379 | 0.291 |
| Isaac, speed at -0.30 m/s | -0.261 | -0.432 | -0.278 |
| Command change per tick, MuJoCo / Isaac, A | 0.021 / 0.090 | 0.138 / 0.105 | 0.319 / 0.241 |
| MuJoCo, 3-tick velocity spike of 3 m/s, falls | 0% | 33% | 0% |
| MuJoCo, 10 ticks of zero observations, falls | 67% | 17% | 67% |
| Export validation | n/a | passed | **failed** |

MuJoCo figures use six held-out seeds, two starting pitches, 30 s. Isaac uses 128 envs, sampled friction 0.01-0.06 Nm
and the filter, with `prev_current=command` as the firmware feeds it. No Isaac scenario fell for any fresh finalist.

## Why the lowest number was not chosen

`model 1225` scored 0.16 deg on the fit plant, by far the best. All of its command power is above 8 Hz: the current
flips every tick, a 33 Hz dither that frees the wheels from friction. In Isaac the same checkpoint gives 1.25 deg, and
5 ms of motor lag takes it to 0.92 deg. The number depends on this one friction model and was not used for selection.

`fresh_model` checkpoints also have a hidden-state problem. With strong white noise on the observations (SD 1) two
copies of the GRU that start 1e-5 apart end up to 2.4 A apart, on every one of the five checkpoints tested. August,
warm and `fresh_command` stay below 1e-3 A. Below noise SD 0.6 the same policies are stable. The export check found
this first: it feeds random inputs to `model 1125` and failed at 0.098 A. On in-envelope inputs the export matches the
checkpoint to 1e-5 A, and its closed-loop MuJoCo result matches too. So the export is faithful; the policy itself is
sensitive outside its normal envelope. That is why it is only the backup.

## What this says about the earlier suspects

- **`prev_current=command` is not the cause.** Rejected as an explanation of the wobble: both fresh arms end with
  negative self-feedback (dI/dprevI -0.72 to -0.91), like every post-fix GRU, and both reach about 1 deg. `prev_current=model`
  does not restore the August sign. August's positive sign was correlation only.
- **What changed instead** is velocity feedback: dI/dvel is 0.19 A per m/s on August, 1.8 on cmd 1200 and 2.9 on model
  1125. Trained with the filter in the loop, the policies can use a signal that was unusable when training ignored its lag.
- **Warm start does not help.** 300 updates left it at 2.6-2.9 deg, like August, with forward speed down to 0.05-0.07 m/s
  at a 0.30 m/s command, and it fell in every run at 30 ms lag.

## Motor response, measured on the robot

Same estimator on hardware logs (August GRU and C pilot): reported current follows the command up to 32 Hz at gain about
0.5 relative to 0.8 at low frequency, coherence 0.93-0.99, with a constant report age of about 30 ms. A first-order fit
gives a current lag of at most about 7 ms, so the 15 and 30 ms MuJoCo variants are pessimistic and 5 ms is the realistic
one. At 15 ms and above the gap to August closes (1.38 vs 1.48 deg), but the logs do not support that much lag.

Commanded chatter therefore reaches the wheels. Its effect on the structure is unknown: rigid MuJoCo cannot show it, and
heat, noise and wear are unmeasured.

There is no evidence of a structural resonance. A peak near 16 Hz appeared only in the derivative of gyro rate against
command for the C pilot, from 5.3 s of closed-loop data (4 windows, the weakest coherence of its neighbours at that bin).
Differentiating tilts the spectrum. The peak is absent from the gyro spectrum and from rate against command, and from
125 s of August data. A bench chirp would settle the question.

## Limits

- One seed. Enough to pick a hardware candidate, not a paper result. Differences between arms can be seed effects.
- Checkpoint-to-checkpoint scatter is large: `cmd 1245` falls in 15% of ranking runs, 1200 in none.
- MuJoCo is fitted to the robot on friction, and overstates August (2.69 vs 1.89-2.63 deg measured). It also did not
  reproduce the two hardware falls of the C pilot and the point MLP, so a clean MuJoCo run does not rule out a fall.
- Friction 0.03 Nm is a closed-loop fit: ASSUMED for paper section 2.4. The velocity filter is IDENTIFIED. Neither is
  in Notion yet.
- Six runs per glitch case, so those fall rates are coarse. Across the six `fresh_command` checkpoints tested, five fell
  on the velocity spike. Ten ticks of random observations caused falls in 50-100% of runs for every policy, August included.
- The `fresh_command` start-up falls happen within about 1 s at static leans near 5 deg or more. Isaac did not train on
  this and did not test it. Payload offset on the robot would be a test of exactly this.

## First hardware run

1. Flash `primary_cmd_1200/policy_drive_stm32ai.onnx`. Keep the August GRU as rollback.
2. Start with the deck empty and centred, with a hand near the robot for the first seconds.
3. Station keeping for 60 s, then log pitch SD and oscillation frequency, and check for vibration and motor heat.
4. Compare with the August GRU on 2026-11-03: 2.63 deg, 0.73 Hz.
5. Only then drive at +-0.3 m/s (expect overshoot, up to -0.43 m/s) and try an off-centre payload.
