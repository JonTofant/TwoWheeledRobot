#!/usr/bin/env python3
"""Check SteadinessTracker's filter does what nn_drive_env_cfg.py claims it does.

Runs on the HOST, without Isaac -- the tracker needs nothing but torch, so it is
loaded straight from its file to bypass the package __init__ that imports
isaaclab. That is the point of having this: the station-keeping steadiness terms
rest on a frequency-response argument (DC out, 0.83 Hz in), and an argument that
can be checked in two seconds on the host should not have to wait 75 s of Kit
startup and a training run to be believed.

Verifies three claims:
  1. A constant offset -- the +-3 deg IMU mounting bias the pitch deadband exists
     for -- leaves NO residual. This is what makes penalising the residual
     different from shrinking the deadband.
  2. The 0.83 Hz limit cycle measured on hardware passes at ~0.96, and the
     response matches the first-order high-pass 1/sqrt(1+(fc/f)^2) with
     fc = 1/(2*pi*tau) across the whole band.
  3. reset() re-seeds from the next sample instead of zeroing, so a new episode's
     spawn attitude is not read as a step change away from the old episode.

Usage:  python3 scripts/probe_steadiness_tracker.py
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import torch

_SRC = (
    Path(__file__).resolve().parents[1]
    / "source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_components.py"
)
_spec = importlib.util.spec_from_file_location("pure_nn_components_standalone", _SRC)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
SteadinessTracker = _mod.SteadinessTracker

DT = 0.015  # 66.7 Hz control rate (PureNNBalanceEnvCfg.decimation = 15)
TAU = 0.7  # must track NNDriveEnvCfg.hold_ac_tau_s
MEASURED_HZ = 0.83  # deployed range+GRU station-keeping limit cycle
MEASURED_RMS_DEG = 1.89

failures: list[str] = []


def residual(signal: np.ndarray) -> np.ndarray:
    tracker = SteadinessTracker(1, 1, torch.device("cpu"), TAU)
    return np.array([tracker.update(torch.tensor([[float(v)]]), DT).item() for v in signal])


def check(name: str, ok: bool, detail: str) -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if not ok:
        failures.append(name)


n = 4000
t = np.arange(n) * DT

print("SteadinessTracker (tau = %.2f s, dt = %.3f s)" % (TAU, DT))

# 1 -- a mounting bias must leave nothing behind.
worst = np.degrees(np.abs(residual(np.full(n, math.radians(3.0)))[500:])).max()
check("DC rejection", worst < 1e-4, f"3.0 deg constant -> {worst:.2e} deg residual")

# 2 -- the measured cycle, riding on that same bias, must survive it.
amp = math.radians(MEASURED_RMS_DEG) * math.sqrt(2)
sig = math.radians(3.0) + amp * np.sin(2 * math.pi * MEASURED_HZ * t)
gain = np.degrees(residual(sig)[500:]).std() / MEASURED_RMS_DEG
check(
    "measured cycle passes",
    0.90 < gain < 1.05,
    f"{MEASURED_HZ} Hz on a 3 deg bias -> gain {gain:.3f}",
)

fc = 1.0 / (2 * math.pi * TAU)
worst_err = 0.0
for f in (0.05, 0.23, 0.45, 0.83, 2.0, 5.0):
    measured = residual(np.sin(2 * math.pi * f * t))[1000:].std() * math.sqrt(2)
    theory = 1.0 / math.sqrt(1 + (fc / f) ** 2)
    worst_err = max(worst_err, abs(measured - theory))
    print(f"        {f:5.2f} Hz  measured {measured:.3f}  first-order {theory:.3f}")
check("first-order high-pass", worst_err < 0.02, f"worst deviation {worst_err:.4f} over 0.05-5 Hz")

# 3 -- reset re-seeds rather than zeroing, and only for the given envs.
tracker = SteadinessTracker(2, 1, torch.device("cpu"), TAU)
for _ in range(300):
    tracker.update(torch.tensor([[0.5], [0.5]]), DT)
tracker.reset(torch.tensor([0]))
res = tracker.update(torch.tensor([[-0.4], [-0.4]]), DT)
check("reset re-seeds", abs(res[0, 0].item()) < 1e-6, f"reset env residual {res[0, 0].item():.2e}")
check("reset is per-env", res[1, 0].item() < -0.85, f"untouched env residual {res[1, 0].item():.3f}")

print("\n" + ("FAILED: " + ", ".join(failures) if failures else "All checks passed."))
raise SystemExit(1 if failures else 0)
