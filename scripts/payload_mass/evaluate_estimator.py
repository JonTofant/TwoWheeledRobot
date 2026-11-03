"""Closed-loop Isaac evaluation of payload-mass estimators.

The frozen hardware GRU balances; every estimator given runs online in the same
loop on the same signals (it never feeds back into control). Each env gets one
object from a mass x deck-placement grid, put on at --place_s and taken off at
--remove_s, under station keeping and then under random stage-2 driving.
Seeds differ from data collection, so every rollout here is unseen.

Per env: settled estimate (mean over [place+5 s, remove]), its jitter, settling
time to within max(50 g, 10 %), the empty-deck estimate before placement and
after removal, and the settled location estimate (the auxiliary offset head,
object COM relative to the deck centre). Written to --out/results.json plus
figures.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "source" / "TwoWheeledRobot"))

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--estimators", nargs="+", type=Path, required=True)
parser.add_argument("--reps", type=int, default=6)
parser.add_argument("--place_s", type=float, default=3.0)
parser.add_argument("--remove_s", type=float, default=33.0)
parser.add_argument("--seconds", type=float, default=45.0)
parser.add_argument("--seed", type=int, default=2001)
parser.add_argument("--size_scale", type=float, default=1.0, help="scale object size ranges (OOD test)")
parser.add_argument("--out", type=Path, default=Path("outputs/payload_mass/eval"))
parser.add_argument("--cfg", nargs="*", default=[], help="env cfg overrides key=value (robustness tests)")
AppLauncher.add_app_launcher_args(parser)
args_cli, rest = parser.parse_known_args()
sys.argv = [sys.argv[0]] + rest
app = AppLauncher(args_cli).app

import ast  # noqa: E402

import torch  # noqa: E402
from common import estimator_inputs, make_env  # noqa: E402
from estimator import PayloadMassEstimator  # noqa: E402
from frozen_policy import HARDWARE_GRU_CHECKPOINT, FrozenGruPolicy  # noqa: E402

MASSES = [0.0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
OFF_X = [-0.09, -0.045, 0.0, 0.045, 0.09]  # lateral
OFF_Y = [-0.08, -0.04, 0.0, 0.04, 0.08]  # fore/aft
DECIMATE = 5  # store every 5th estimate for plots
LOC_PLOT_MASSES = [0.1, 0.25, 1.0, 2.0]


def load_estimator(path: Path, dev):
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    model = PayloadMassEstimator(ckpt["feature_ids"], ckpt["hidden"])
    model.load_state_dict(ckpt["state_dict"])
    return model.to(dev).eval()


def run(env, policy, estimators, cells, mode):
    u = env.unwrapped
    u.cfg.forced_command_mode = "fixed" if mode == "hold" else ""
    u.cfg.forced_velocity_cmd_mps = 0.0
    u.cfg.forced_yaw_rate_cmd_radps = 0.0
    n, dev, dt = u.num_envs, u.device, u.step_dt
    reps = args_cli.reps
    cell = torch.arange(n, device=dev) // reps
    mass = torch.tensor([c[0] for c in cells], device=dev)[cell]
    off = torch.tensor([[c[1], c[2]] for c in cells], device=dev)[cell]
    cfg = u.cfg
    k = args_cli.size_scale
    size = torch.stack([torch.empty(n, device=dev).uniform_(lo * k, hi * k) for lo, hi in
                        (cfg.payload_size_x_m_range, cfg.payload_size_y_m_range, cfg.payload_size_z_m_range)], 1)
    place, remove = int(args_cli.place_s / dt), int(args_cli.remove_s / dt)
    steps = int(args_cli.seconds / dt)
    traces = {name: [] for name in estimators}
    loc_traces = {name: [] for name in estimators}
    with torch.inference_mode():
        u.episode_length_buf[:] = 0
        obs = env.reset()[0]["policy"]
        u.set_payload(torch.arange(n, device=dev), mass, off, size, torch.full((n,), place, device=dev),
                      torch.full((n,), remove, device=dev))
        policy.reset()
        hidden = {name: None for name in estimators}
        alive = torch.ones(n, dtype=torch.bool, device=dev)
        for step in range(steps):
            act = policy(obs)
            x = estimator_inputs(env, obs, act)
            for name, model in estimators.items():
                est, loc, hidden[name] = model.step(x, hidden[name])
                if step % DECIMATE == 0:
                    traces[name].append(est.cpu())
                    loc_traces[name].append(loc.cpu())
            obs, _, term, trunc, _ = env.step(act)
            obs = obs["policy"]
            alive &= ~(term | trunc)
    t = torch.arange(0, steps, DECIMATE) * dt
    alive = alive.cpu()
    result = dict(mode=mode, alive=alive.tolist(), mass=mass.cpu().tolist(), off=off.cpu().tolist(),
                  size=size.cpu().tolist(), estimators={})
    for name, tr in traces.items():
        est = torch.stack(tr)  # (S, n)
        loc = torch.stack(loc_traces[name])  # (S, n, 2)
        win = (t >= args_cli.place_s + 5.0) & (t < args_cli.remove_s)
        pre = (t >= args_cli.place_s - 1.5) & (t < args_cli.place_s)
        post = t >= args_cli.remove_s + 5.0
        m = mass.cpu()
        tol = torch.maximum(torch.full_like(m, 0.05), 0.1 * m)
        inside = ((est - m).abs() <= tol) | ~((t >= args_cli.place_s) & (t < args_cli.remove_s)).unsqueeze(1)
        # settle time: last time it was outside the band during the loaded phase
        outside_idx = torch.where(~inside, torch.arange(len(t)).unsqueeze(1), torch.zeros(1, dtype=torch.long))
        settle = (t[outside_idx.max(0).values] - args_cli.place_s + DECIMATE * dt).clamp_min(0)
        result["estimators"][name] = dict(
            settled=est[win].mean(0).tolist(), jitter=est[win].std(0).tolist(), settle_s=settle.tolist(),
            pre=est[pre].mean(0).tolist(), post=est[post].mean(0).tolist(),
            settled_loc=loc[win].mean(0).tolist(), loc_jitter=loc[win].std(0).norm(dim=-1).tolist(),
            example=est[:, ::max(1, n // 64)].tolist(),
        )
    result["t"] = t.tolist()
    return result


def summarize(res, name):
    alive = torch.tensor(res["alive"])
    m = torch.tensor(res["mass"])
    off = torch.tensor(res["off"])
    e = res["estimators"][name]
    est = torch.tensor(e["settled"])
    err = est - m
    lines = []
    s = dict(fall_rate=1 - alive.float().mean().item())
    loaded = alive & (m > 0)
    s["mae_kg"] = err[alive].abs().mean().item()
    s["mae_loaded_kg"] = err[loaded].abs().mean().item()
    s["mre_loaded"] = (err[loaded].abs() / m[loaded]).mean().item()
    s["p95_abs_err_kg"] = err[alive].abs().quantile(0.95).item()
    s["empty_before_kg"] = torch.tensor(e["pre"])[alive].mean().item()
    s["after_removal_kg"] = torch.tensor(e["post"])[alive].mean().item()
    s["jitter_kg"] = torch.tensor(e["jitter"])[alive].mean().item()
    s["settle_s_median"] = torch.tensor(e["settle_s"])[loaded].median().item()
    loc_err = torch.tensor(e["settled_loc"]) - off
    s["loc_mae_lateral_m"] = loc_err[loaded, 0].abs().mean().item()
    s["loc_mae_foreaft_m"] = loc_err[loaded, 1].abs().mean().item()
    s["loc_p95_radial_m"] = loc_err[loaded].norm(dim=1).quantile(0.95).item()
    s["loc_jitter_m"] = torch.tensor(e["loc_jitter"])[loaded].mean().item()
    per_mass = {}
    for mass in MASSES:
        sel = alive & (m == mass)
        cell_means, cell_loc_bias = [], []
        for ox, oy in itertools.product(OFF_X, OFF_Y):
            c = sel & ((off[:, 0] - ox).abs() < 1e-4) & ((off[:, 1] - oy).abs() < 1e-4)
            if c.any():
                cell_means.append(est[c].mean().item())
                cell_loc_bias.append(loc_err[c].mean(0).norm().item())
        cm = torch.tensor(cell_means)
        per_mass[str(mass)] = dict(
            mean_est=est[sel].mean().item(), mae=err[sel].abs().mean().item(),
            placement_spread_std=cm.std().item(), placement_worst_bias=(cm - mass).abs().max().item(),
            settle_s_median=torch.tensor(e["settle_s"])[sel].median().item() if mass > 0 else None)
        line = (f"  m={mass:4.2f}: mean est {per_mass[str(mass)]['mean_est']:.3f}  MAE "
                f"{per_mass[str(mass)]['mae']:.3f}  placement std {cm.std().item():.3f}  worst-cell bias "
                f"{per_mass[str(mass)]['placement_worst_bias']:.3f}")
        if mass > 0:
            per_mass[str(mass)].update(
                loc_mae_lateral_m=loc_err[sel, 0].abs().mean().item(),
                loc_mae_foreaft_m=loc_err[sel, 1].abs().mean().item(),
                loc_p95_radial_m=loc_err[sel].norm(dim=1).quantile(0.95).item(),
                loc_worst_cell_bias_m=max(cell_loc_bias))
            line += (f"  | loc MAE lat {per_mass[str(mass)]['loc_mae_lateral_m'] * 100:.2f} cm, f/a "
                     f"{per_mass[str(mass)]['loc_mae_foreaft_m'] * 100:.2f} cm, worst-cell bias "
                     f"{max(cell_loc_bias) * 100:.2f} cm")
        lines.append(line)
    s["per_mass"] = per_mass
    grid = torch.zeros(len(OFF_Y), len(OFF_X))
    for (i, oy), (j, ox) in itertools.product(enumerate(OFF_Y), enumerate(OFF_X)):
        c = loaded & ((off[:, 0] - ox).abs() < 1e-4) & ((off[:, 1] - oy).abs() < 1e-4)
        grid[i, j] = (err[c].abs() / m[c]).mean() if c.any() else float("nan")
    s["rel_err_by_placement"] = grid.tolist()
    return s, lines


def figures(results, out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(results[0]["estimators"])
    for res in results:
        mode = res["mode"]
        fig, axes = plt.subplots(2, len(names), figsize=(5.2 * len(names), 9), squeeze=False)
        for j, name in enumerate(names):
            s = res["summary"][name]
            alive = torch.tensor(res["alive"])
            m = torch.tensor(res["mass"])[alive]
            est = torch.tensor(res["estimators"][name]["settled"])[alive]
            ax = axes[0, j]
            ax.scatter(m + (torch.rand_like(m) - 0.5) * 0.04, est, s=4, alpha=0.4)
            ax.plot([0, 2.1], [0, 2.1], "k--", lw=1)
            ax.set_xlabel("true payload mass (kg)")
            ax.set_ylabel("settled estimate (kg)")
            ax.set_title(f"{name}\n{mode}: MAE {s['mae_kg'] * 1000:.0f} g, rel {s['mre_loaded'] * 100:.1f}%")
            ax = axes[1, j]
            im = ax.imshow(torch.tensor(s["rel_err_by_placement"]) * 100, origin="lower", cmap="viridis",
                           extent=[OFF_X[0] - 0.0225, OFF_X[-1] + 0.0225, OFF_Y[0] - 0.02, OFF_Y[-1] + 0.02])
            ax.set_xlabel("lateral offset (m)")
            ax.set_ylabel("fore/aft offset (m)")
            ax.set_title("mean relative error by placement (%)")
            fig.colorbar(im, ax=ax)
        fig.tight_layout()
        fig.savefig(out / f"accuracy_{mode}.png", dpi=110)
        plt.close(fig)

        fig, axes = plt.subplots(1, len(names), figsize=(6 * len(names), 4), squeeze=False)
        t = res["t"]
        for j, name in enumerate(names):
            ex = torch.tensor(res["estimators"][name]["example"])
            n = len(res["mass"])
            idx = list(range(0, n, max(1, n // 64)))
            ax = axes[0, j]
            for k, i in enumerate(idx[: ex.shape[1]]):
                if not res["alive"][i] or res["mass"][i] not in (0.25, 1.0, 2.0):
                    continue
                truth = [res["mass"][i] if args_cli.place_s <= tt < args_cli.remove_s else 0.0 for tt in t]
                (line,) = ax.plot(t, ex[:, k], lw=0.8, alpha=0.8)
                ax.plot(t, truth, "--", color=line.get_color(), lw=0.8)
            ax.set_xlabel("time (s)")
            ax.set_ylabel("estimate (kg)")
            ax.set_title(f"{name}, {mode}: example traces (dashed = truth)")
        fig.tight_layout()
        fig.savefig(out / f"traces_{mode}.png", dpi=110)
        plt.close(fig)

        # Location map: true grid point (cross) vs each env's settled estimate (dots).
        fig, axes = plt.subplots(len(names), len(LOC_PLOT_MASSES), figsize=(3.6 * len(LOC_PLOT_MASSES),
                                                                            3.6 * len(names)), squeeze=False)
        alive = torch.tensor(res["alive"])
        m = torch.tensor(res["mass"])
        off = torch.tensor(res["off"])
        gx, gy = zip(*itertools.product(OFF_X, OFF_Y))
        for i, name in enumerate(names):
            loc = torch.tensor(res["estimators"][name]["settled_loc"])
            for j, mass in enumerate(LOC_PLOT_MASSES):
                ax = axes[i, j]
                sel = alive & (m == mass)
                # deck outline, 0.227 x 0.200 m about the deck centre (measured by the env)
                ax.plot([-0.1135, 0.1135, 0.1135, -0.1135, -0.1135], [-0.1, -0.1, 0.1, 0.1, -0.1], "k-", lw=0.6)
                ax.scatter(loc[sel, 0], loc[sel, 1], s=5, alpha=0.5)
                for k in torch.nonzero(sel).flatten().tolist():
                    ax.plot([off[k, 0], loc[k, 0]], [off[k, 1], loc[k, 1]], "-", color="0.6", lw=0.4)
                ax.scatter(gx, gy, marker="x", c="k", s=25, lw=1)
                e = (loc[sel] - off[sel]).norm(dim=1).mean().item()
                ax.set_title(f"{name}\n{mode}, {mass:g} kg: mean err {e * 100:.1f} cm", fontsize=9)
                ax.set_aspect("equal")
                ax.set_xlim(-0.13, 0.13)
                ax.set_ylim(-0.12, 0.12)
                ax.set_xlabel("lateral (m)")
                ax.set_ylabel("fore/aft (m)")
        fig.tight_layout()
        fig.savefig(out / f"location_{mode}.png", dpi=110)
        plt.close(fig)


def main():
    cells = list(itertools.product(MASSES, OFF_X, OFF_Y))
    n = len(cells) * args_cli.reps
    overrides = {k: ast.literal_eval(v) for k, v in (kv.split("=", 1) for kv in args_cli.cfg)}
    env = make_env(n, args_cli.seed, episode_length_s=args_cli.seconds + 5.0, **overrides)
    dev = env.unwrapped.device
    policy = FrozenGruPolicy(HARDWARE_GRU_CHECKPOINT, n, dev)
    estimators = {p.stem.removeprefix("estimator_"): load_estimator(p, dev) for p in args_cli.estimators}
    args_cli.out.mkdir(parents=True, exist_ok=True)
    results = []
    for mode in ("hold", "drive"):
        res = run(env, policy, estimators, cells, mode)
        res["summary"] = {}
        for name in estimators:
            s, lines = summarize(res, name)
            res["summary"][name] = s
            print(f"\n[eval] {mode} / {name}: fall {s['fall_rate']:.3f}, MAE {s['mae_kg']:.4f} kg, loaded rel "
                  f"{s['mre_loaded']:.3f}, p95 {s['p95_abs_err_kg']:.3f}, empty before {s['empty_before_kg']:.3f}, "
                  f"after removal {s['after_removal_kg']:.3f}, jitter {s['jitter_kg']:.4f}, settle median "
                  f"{s['settle_s_median']:.2f} s | location MAE lat {s['loc_mae_lateral_m'] * 100:.2f} cm, f/a "
                  f"{s['loc_mae_foreaft_m'] * 100:.2f} cm, p95 radial {s['loc_p95_radial_m'] * 100:.2f} cm", flush=True)
            print("\n".join(lines), flush=True)
        results.append(res)
    (args_cli.out / "results.json").write_text(json.dumps(
        [{k: v for k, v in r.items() if k != "estimators"}
         | {"settled": {n: r["estimators"][n]["settled"] for n in r["estimators"]},
            "settled_loc": {n: r["estimators"][n]["settled_loc"] for n in r["estimators"]}}
         for r in results], indent=1))
    figures(results, args_cli.out)
    print(f"[eval] wrote {args_cli.out}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
