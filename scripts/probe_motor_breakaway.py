#!/usr/bin/env python3
"""Check the Stribeck breakaway term in CurrentActionProcessor.

Runs on the HOST without Isaac -- the processor needs only torch, so it is
loaded straight from its file to bypass the package __init__.

Two claims, both load-bearing:

  1. The DEFAULT multiplier range (1.0, 1.0) reproduces the previous
     constant-deadzone plant EXACTLY, at every wheel speed, and identically to
     not passing wheel_omega at all. The 2026-08-10 paper arms were trained
     against that plant; if this drifts, their results silently stop being
     reproducible.
  2. When enabled, the effective deadzone follows
     dz(w) = dz_kinetic * (1 + (mult - 1) * exp(-|w| / w_s))
     i.e. full breakaway from rest, decaying to the measured kinetic value as
     the wheel spins up.

NOTE for anyone extending this: CurrentActionProcessor.reset() draws a 50/50
action delay (`randint(0, 2)`), so a single process() call after reset returns a
delayed ZERO half the time. Pin `action_delay_samples` and step a few times, or
you will measure the delay draw and think you are measuring the deadzone.

Usage:  python3 scripts/probe_motor_breakaway.py
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import torch

_SRC = (
    Path(__file__).resolve().parents[1]
    / "source/TwoWheeledRobot/TwoWheeledRobot/tasks/direct/twowheeledrobot/pure_nn_components.py"
)
_spec = importlib.util.spec_from_file_location("pure_nn_components_standalone", _SRC)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
CurrentActionProcessor = _mod.CurrentActionProcessor

DT = 0.015
KINETIC_A = 0.0534  # EMB-18 population mean
REQUEST_A = 0.5
R_WHEEL = 0.0505


class _Cfg:
    i_max_a = 2.0
    action_smoothing_alpha = 0.0
    enable_current_slew_limit = False
    hardware_safe_current_slew_limit = False
    current_slew_limit_a = 1.0
    motor_gain_range = (1.0, 1.0)
    motor_deadzone_a_range = (KINETIC_A, KINETIC_A)
    motor_bias_a_range = (0.0, 0.0)
    motor_tau_s_range = (0.005, 0.005)
    motor_current_limit_a_range = (2.0, 2.0)
    motor_breakaway_multiplier_range = (1.0, 1.0)
    motor_breakaway_speed_radps = 0.5


def commanded(cfg, omega: float, pass_omega: bool = True) -> float:
    proc = CurrentActionProcessor(cfg, 1, torch.device("cpu"))
    proc.reset(torch.tensor([0]))
    proc.action_delay_samples[:] = 0  # see module docstring
    action = torch.atanh(torch.tensor([[REQUEST_A / cfg.i_max_a] * 2]))
    w = torch.full((1, 2), omega)
    out = None
    for _ in range(3):
        out = proc.process(action, DT, w if pass_omega else None)
    return out[0, 0].item()


failures: list[str] = []


def check(name: str, ok: bool, detail: str) -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if not ok:
        failures.append(name)


print(f"CurrentActionProcessor breakaway (kinetic {KINETIC_A*1000:.1f} mA, w_s "
      f"{_Cfg.motor_breakaway_speed_radps} rad/s)\n")

# Two separate claims. The with-vs-without comparison must be EXACT: at
# multiplier 1.0 the excess is 0.0 and the deadzone is multiplied by exactly
# 1.0, which is bit-preserving in float. The comparison against the analytic
# value is only good to float32, because the test's own atanh/tanh round trip
# loses ~1e-8 on a 0.5 A request -- tightening that tolerance tests the harness,
# not the plant.
expected = REQUEST_A - KINETIC_A
worst_path, worst_analytic, speeds = 0.0, 0.0, (0.0, 0.5, 2.0, 5.0)
for omega in speeds:
    with_w, without_w = commanded(_Cfg, omega), commanded(_Cfg, omega, False)
    worst_path = max(worst_path, abs(with_w - without_w))
    worst_analytic = max(worst_analytic, abs(with_w - expected))
check("default is bit-identical with/without wheel_omega", worst_path == 0.0,
      f"exact match at {len(speeds)} speeds (deviation {worst_path:.1e})")
check("default matches the old constant plant", worst_analytic < 1e-6,
      f"{expected:.4f} A at every speed, worst {worst_analytic:.2e} (float32 floor ~3e-8)")

_Cfg.motor_breakaway_multiplier_range = (2.5, 2.5)
worst = 0.0
print()
for omega in (0.0, 0.25, 0.5, 1.0, 2.0, 5.0):
    dz = REQUEST_A - commanded(_Cfg, omega)
    theory = KINETIC_A * (1 + 1.5 * math.exp(-omega / _Cfg.motor_breakaway_speed_radps))
    worst = max(worst, abs(dz - theory))
    print(f"        {omega:4.2f} rad/s ({omega*R_WHEEL:5.3f} m/s)  dz {dz*1000:6.2f} mA   "
          f"Stribeck {theory*1000:6.2f} mA")
check("Stribeck decay", worst < 1e-6, f"worst deviation {worst*1000:.2e} mA over 0-5 rad/s")
check("breakaway at rest", abs((REQUEST_A - commanded(_Cfg, 0.0)) - KINETIC_A * 2.5) < 1e-6,
      f"{KINETIC_A*2.5*1000:.1f} mA from rest")

print("\n" + ("FAILED: " + ", ".join(failures) if failures else "All checks passed."))
raise SystemExit(1 if failures else 0)
