"""Train the payload-mass estimator offline on collect_rollouts.py data.

Plain PyTorch, no Isaac needed. Training mixes full 20 s episodes (estimator
started at power-on) with random mid-episode crops started from a zero hidden
state, so the estimate converges no matter when the estimator was (re)armed or
whether an object was already on the deck at that moment.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import torch
import torch.nn.functional as F

sys.path.insert(0, str(Path(__file__).resolve().parent))
from estimator import FEATURE_GROUPS, PayloadMassEstimator  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--data_dir", type=Path, default=Path("outputs/payload_mass/data"))
parser.add_argument("--train", nargs="+", default=["train_s100_r0", "train_s100_r1", "train_s100_r2"])
parser.add_argument("--val", nargs="+", default=["train_s100_r3"])
parser.add_argument("--features", choices=list(FEATURE_GROUPS), default="imu_wheels_legs")
parser.add_argument("--hidden", type=int, default=32)
parser.add_argument("--iters", type=int, default=4000)
parser.add_argument("--batch", type=int, default=128)
parser.add_argument("--crop", type=int, default=600, help="crop length in steps (9 s)")
parser.add_argument("--lr", type=float, default=3e-3)
parser.add_argument("--offset_weight", type=float, default=0.5)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--out", type=Path, default=None)
args = parser.parse_args()
dev = "cuda"


def load(names):
    parts = [torch.load(args.data_dir / f"{n}.pt") for n in names]
    cat = {k: torch.cat([p[k] for p in parts], dim=1) for k in ("features", "mass", "offset_xy", "valid")}
    return cat, parts[0]["dt"]


def steps_since_event(mass: torch.Tensor) -> torch.Tensor:
    since = torch.zeros_like(mass)
    for t in range(1, mass.shape[0]):
        changed = (mass[t] - mass[t - 1]).abs() > 1e-6
        since[t] = torch.where(changed, torch.zeros_like(since[t]), since[t - 1] + 1)
    since[0] = 1e6  # nothing happened "before" the episode: treat as settled
    return since


def masked_mae(pred, target, mask):
    return ((pred - target).abs() * mask).sum().item() / mask.sum().clamp_min(1).item()


def evaluate(model, data, dt):
    model.eval()
    feats, mass, valid = data["features"], data["mass"], data["valid"]
    since = steps_since_event(mass)
    preds = []
    with torch.no_grad():
        for i in range(0, feats.shape[1], 512):
            m, _, _ = model(feats[:, i:i + 512].to(dev).float())
            preds.append(m.clamp_min(0).cpu())
    pred = torch.cat(preds, dim=1)
    v = valid.float()
    settled = v * (since * dt >= 3.0).float()
    loaded = settled * (mass > 0).float()
    rel = ((pred - mass).abs() / mass.clamp_min(1e-6)) * loaded
    model.train()
    return dict(
        mae_all=masked_mae(pred, mass, v),
        mae_settled=masked_mae(pred, mass, settled),
        mae_settled_loaded=masked_mae(pred, mass, loaded),
        mre_settled_loaded=rel.sum().item() / loaded.sum().clamp_min(1).item(),
        mae_settled_empty=masked_mae(pred, mass, settled * (mass == 0).float()),
        baseline_mae_settled=masked_mae(torch.full_like(mass, mass[valid].mean().item()), mass, settled),
    )


def main():
    torch.manual_seed(args.seed)
    train, dt = load(args.train)
    val, _ = load(args.val)
    ids = FEATURE_GROUPS[args.features]
    valid = train["valid"]
    x = train["features"][..., ids].float()[valid]
    mean, std = x.mean(0), x.std(0).clamp_min(1e-4)
    model = PayloadMassEstimator(ids, args.hidden, mean, std).to(dev)
    print(f"[train] features={args.features} ({len(ids)}), hidden={args.hidden}, params={model.num_params()}, "
          f"train envs={valid.shape[1]}, val envs={val['valid'].shape[1]}", flush=True)

    feats = train["features"].to(dev)
    mass = train["mass"].to(dev)
    offset = train["offset_xy"].to(dev)
    vmask = valid.to(dev).float()
    T, N = mass.shape
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.LambdaLR(opt, lambda i: 0.5 * (1 + math.cos(math.pi * i / args.iters)))
    t0 = time.time()
    history = []
    for it in range(args.iters):
        b = torch.randint(0, N, (args.batch,), device=dev)
        if it % 2 == 0:
            s, L = 0, T
        else:
            L = args.crop
            s = int(torch.randint(0, T - L, (1,)))
        xb = feats[s:s + L, b].float()
        mb, ob, vb = mass[s:s + L, b], offset[s:s + L, b], vmask[s:s + L, b]
        pm, po, _ = model(xb)
        loss_m = (F.smooth_l1_loss(pm, mb, beta=0.05, reduction="none") * vb).sum() / vb.sum().clamp_min(1)
        on = vb * (mb > 0).float()
        loss_o = (F.smooth_l1_loss(po, ob, beta=0.01, reduction="none").sum(-1) * on).sum() / on.sum().clamp_min(1)
        loss = loss_m + args.offset_weight * loss_o
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step()
        sched.step()
        if it % 500 == 0 or it == args.iters - 1:
            metrics = evaluate(model, val, dt)
            metrics.update(iter=it, loss=loss.item(), loss_mass=loss_m.item(), loss_offset=loss_o.item())
            history.append(metrics)
            print(f"[train] it {it:5d} loss {loss.item():.4f} | val MAE settled {metrics['mae_settled']:.4f} kg "
                  f"(loaded {metrics['mae_settled_loaded']:.4f}, rel {metrics['mre_settled_loaded']:.3f}; "
                  f"empty {metrics['mae_settled_empty']:.4f}; predict-mean baseline "
                  f"{metrics['baseline_mae_settled']:.3f}) {time.time() - t0:.0f}s", flush=True)
    out = args.out or Path(f"outputs/payload_mass/estimator_{args.features}_h{args.hidden}.pt")
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(dict(state_dict=model.cpu().state_dict(), feature_ids=ids, hidden=args.hidden,
                    features=args.features, args={k: str(v) for k, v in vars(args).items()}, history=history), out)
    out.with_suffix(".json").write_text(json.dumps(history, indent=1))
    print(f"[train] wrote {out}", flush=True)


if __name__ == "__main__":
    main()
