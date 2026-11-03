"""Package the payload estimator for the laptop-side hardware test.

Writes, into --out (normally the firmware repo's Tools/payload_estimator/model/):
  estimator.onnx       single-step graph (see export_estimator_onnx.py)
  sim_reference.json   what the laptop tool needs to map hardware onto the sim:
    - feature names/order and the deck geometry
    - per-feature empty-deck mean/std (the tare target)
    - per-leg-channel load signature d(feature) = a*m + b*m*x + c*m*y, fitted
      on the recorded rollouts; the calibration step matches hardware against it

No Isaac needed: everything comes from outputs/payload_mass/data and the
trained estimator checkpoint.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from estimator import FEATURE_GROUPS  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--estimator", type=Path, default=Path("outputs/payload_mass/estimator_imu_wheels_legs_h32.pt"))
parser.add_argument("--data", type=Path, nargs="+",
                    default=[Path(f"outputs/payload_mass/data/train_s100_r{i}.pt") for i in range(4)])
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args()

LEGS = ["fl", "fr", "bl", "br"]
FEATURE_NAMES = ([f"obs_{i}" for i in range(13)] + ["current_cmd_left_A", "current_cmd_right_A"]
                 + [f"leg_angle_{k}_rad" for k in LEGS] + [f"leg_torque_{k}_Nm" for k in LEGS])
# Measured by NNDrivePayloadEnv._measure_deck (2026-11-03): offsets are relative to this.
DECK = dict(size_lateral_x_m=0.227, size_foreaft_y_m=0.200, forward_is_negative_y=True)
SETTLE_S = 2.0


def episode_windows(mass: torch.Tensor, valid: torch.Tensor, dt: float):
    """Per env: (pre-placement window, settled loaded window), as step ranges, or None."""
    T, N = mass.shape
    out = []
    for n in range(N):
        m, v = mass[:, n], valid[:, n]
        on = torch.nonzero(m > 0).flatten()
        if len(on) == 0:
            continue
        place = int(on[0])
        # loaded run is contiguous from placement; it ends at removal or episode end
        off_after = torch.nonzero(m[place:] == 0).flatten()
        end = place + int(off_after[0]) if len(off_after) else T
        end = min(end, int(v.float().argmin()) if not v.all() else T)
        pre = (int(1.0 / dt), place)
        loaded = (place + int(SETTLE_S / dt), end)
        if pre[1] - pre[0] >= int(0.5 / dt) and loaded[1] - loaded[0] >= int(1.0 / dt):
            out.append((n, pre, loaded))
    return out


def main():
    feats_all, rows, empty_parts = [], [], []
    dt = None
    for path in args.data:
        d = torch.load(path, map_location="cpu")
        dt = d["dt"]
        f, m, off, v = d["features"].float(), d["mass"], d["offset_xy"], d["valid"]
        empty = v & (m == 0)
        empty[: int(1.0 / dt)] = False  # skip the reset transient
        empty_parts.append(f[empty])
        for n, pre, loaded in episode_windows(m, v, dt):
            delta = f[loaded[0]:loaded[1], n].mean(0) - f[pre[0]:pre[1], n].mean(0)
            mass = float(m[loaded[0], n])
            x, y = off[loaded[0], n].tolist()
            rows.append((mass, x, y, delta))
        print(f"[kit] {path.name}: {len(rows)} loaded episodes so far", flush=True)
    empty = torch.cat(empty_parts)
    mass = torch.tensor([r[0] for r in rows])
    xy = torch.tensor([[r[1], r[2]] for r in rows])
    delta = torch.stack([r[3] for r in rows])

    # delta_j = a*m + b*m*x + c*m*y, least squares, leg channels only
    A = torch.stack([mass, mass * xy[:, 0], mass * xy[:, 1]], dim=1)
    leg_ids = list(range(15, 23))
    coef = torch.linalg.lstsq(A, delta[:, leg_ids]).solution  # (3, 8)
    resid = delta[:, leg_ids] - A @ coef
    r2 = 1 - resid.var(0) / delta[:, leg_ids].var(0)
    signature = {FEATURE_NAMES[j]: dict(per_kg=coef[0, i].item(), per_kg_per_m_lateral=coef[1, i].item(),
                                        per_kg_per_m_foreaft=coef[2, i].item(), r2=r2[i].item(),
                                        resid_std=resid[:, i].std().item())
                 for i, j in enumerate(leg_ids)}
    for k, s in signature.items():
        print(f"[kit] {k:22s} {s['per_kg']:+.4f}/kg  {s['per_kg_per_m_lateral']:+.3f}/(kg m) lat  "
              f"{s['per_kg_per_m_foreaft']:+.3f}/(kg m) f/a  r2 {s['r2']:.3f}", flush=True)

    ckpt = torch.load(args.estimator, map_location="cpu", weights_only=False)
    assert list(ckpt["feature_ids"]) == list(FEATURE_GROUPS["imu_wheels_legs"]), "kit expects the 23-feature model"
    args.out.mkdir(parents=True, exist_ok=True)
    onnx = args.estimator.with_suffix(".onnx")
    if not onnx.exists():
        subprocess.run([sys.executable, str(Path(__file__).parent / "export_estimator_onnx.py"),
                        "--estimator", str(args.estimator)], check=True)
    shutil.copy(onnx, args.out / "estimator.onnx")
    ref = dict(
        source=dict(estimator=str(args.estimator), data=[str(p) for p in args.data],
                    balance_policy="fixedstance_range_gru_2026-08-10 seed 42 model_775 (firmware e6d55c5)"),
        control_dt_s=dt, hidden_size=int(ckpt["hidden"]), feature_names=FEATURE_NAMES, deck=DECK,
        leg_order=LEGS, cybergear_kp=30.0,
        empty_mean=empty.mean(0).tolist(), empty_std=empty.std(0).tolist(),
        leg_signature=signature, n_episodes=len(rows),
    )
    (args.out / "sim_reference.json").write_text(json.dumps(ref, indent=1))
    print(f"[kit] wrote {args.out}/estimator.onnx and sim_reference.json ({len(rows)} episodes)", flush=True)


if __name__ == "__main__":
    main()
