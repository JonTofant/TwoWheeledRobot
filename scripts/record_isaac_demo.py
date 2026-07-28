#!/usr/bin/env python3
"""Record a showcase video + stills of the NN drive policy in Isaac Sim.

Runs the exported NNDrive policy on the presentation scene
(``Template-Twowheeledrobot-NNDriveDemo-v0``): a grid of identical training
pads, one robot per pad, all balancing under the same controller, each staged
on the lip of its drop ledge.

The hero robot (env 0) can be commanded forward with ``--drive-start`` /
``--drive-speed`` to drive off its ledge. Note that as of the
2026-07-27 stage-5 checkpoint no trained policy actually executes this: they
station-keep well but track velocity commands at roughly 2% of the commanded
speed (``benchmark_nn_drive.py`` drive_forward: 0.40 m/s commanded, ~0.06 m/s
achieved), so the hero holds its position on the lip instead. The trace summary
printed at the end reports what actually happened rather than assuming it.

The camera is keyframed in world space and moved every control step, so the
clip is a directed shot rather than a debug capture. Nothing about the policy
interface is special-cased here — the same 18-dim observation and 6-dim action
contract used for training and STM32 deployment drives every robot.

Example:
    /isaac-sim/python.sh scripts/record_isaac_demo.py \
        --policy logs/rsl_rl/nn_drive_two_wheel/2026-07-27_16-24-44_stage5/exported/policy.pt \
        --headless --out outputs/demo
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EXTENSION_SOURCE_PATH = Path(__file__).resolve().parents[1] / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Record an Isaac Sim showcase clip of the NN drive policy.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-NNDriveDemo-v0")
parser.add_argument("--policy", required=True, type=Path, help="TorchScript actor exported by play.py")
parser.add_argument("--num_envs", type=int, default=6, help="number of training pads / robots")
parser.add_argument("--out", type=Path, default=Path("outputs/demo"), help="output directory")
parser.add_argument("--tag", type=str, default="training_ground", help="output filename stem")
parser.add_argument("--ledge", type=float, default=0.10, help="drop-ledge height (m)")
parser.add_argument("--seconds", type=float, default=16.0, help="clip length")
parser.add_argument("--shot", type=str, default="hero",
                    choices=["hero", "wide", "ledge", "poised", "grid", "sweep", "topdown"])
parser.add_argument("--still-shots", type=str, nargs="*", default=None,
                    help="camera presets to also capture stills from at every --stills time")
parser.add_argument("--drive-start", type=float, default=4.0, help="time the hero joystick goes forward (s)")
parser.add_argument("--drive-stop", type=float, default=7.6, help="time the hero joystick re-centres (s)")
parser.add_argument("--drive-speed", type=float, default=0.35, help="hero forward command (m/s)")
parser.add_argument("--stills", type=float, nargs="*", default=None, help="times (s) to save PNG stills")
parser.add_argument("--slowmo", type=float, nargs=2, default=None, metavar=("T0", "T1"),
                    help="also emit a slow-motion clip of this window (s)")
parser.add_argument("--slowmo-fps", type=int, default=15)
parser.add_argument("--resolution", type=int, nargs=2, default=(1920, 1080))
parser.add_argument("--no-video", action="store_true", help="stills + trace only")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
# Offscreen rendering is what produces the frames.
args_cli.enable_cameras = True
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import csv
import math

import gymnasium as gym
import numpy as np
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from TwoWheeledRobot.tasks.direct.twowheeledrobot.pure_nn_components import roll_from_projected_gravity

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config

# ── Camera shots ─────────────────────────────────────────────────────────────
# Keyframes are (time_s, eye_xyz, lookat_xyz) in world metres. The hero pad's
# lip centre is the world origin, the hero robot spawns at x = -run_up_x_m on
# the deck and drives towards +x. Interpolation is smoothstepped so the moves
# ease in and out instead of starting/stopping abruptly.
SHOTS: dict[str, list[tuple[float, tuple[float, float, float], tuple[float, float, float]]]] = {
    # Showcase move: establish the whole rig, glide down into the grid, settle
    # on the hero robot holding station on its ledge.
    "hero": [
        (0.0,  (-2.05, -4.05, 2.95), (1.75, 1.75, 0.05)),
        (4.0,  (-1.55, -3.00, 2.05), (1.55, 1.60, 0.05)),
        (7.0,  (-1.25, -2.30, 1.35), (1.05, 1.20, 0.06)),
        (10.0, (-0.85, -1.60, 0.72), (0.35, 0.45, 0.08)),
        (13.0, (0.05, -1.55, 0.30), (0.10, 0.05, 0.10)),
        (16.0, (0.10, -1.15, 0.22), (0.08, 0.02, 0.10)),
    ],
    # Static wide: the whole rig, good for a single "we train in Isaac Sim" still.
    "wide": [
        (0.0,  (-1.80, -3.60, 2.75), (1.75, 1.75, 0.05)),
        (16.0, (-1.80, -3.60, 2.75), (1.75, 1.75, 0.05)),
    ],
    # Static close on the hero pad: deck, drop and landing all in frame.
    "ledge": [
        (0.0,  (-0.70, -1.30, 0.40), (0.10, 0.00, 0.10)),
        (16.0, (-0.70, -1.30, 0.40), (0.10, 0.00, 0.10)),
    ],
    # True profile of the ledge: the deck, the vertical drop face and the
    # landing floor are all edge-on, so the step height is unambiguous.
    "poised": [
        (0.0,  (0.10, -1.40, 0.24), (0.10, 0.00, 0.10)),
        (16.0, (0.10, -1.40, 0.24), (0.10, 0.00, 0.10)),
    ],
    # Into the grid at robot height: many identical pads, many robots.
    "grid": [
        (0.0,  (-1.25, -2.30, 1.65), (1.35, 1.55, 0.05)),
        (16.0, (-1.25, -2.30, 1.65), (1.35, 1.55, 0.05)),
    ],
    # Slow orbit around the hero pad, for a loopable background clip.
    "sweep": [
        (0.0,  (-1.60, -1.90, 1.05), (0.30, 0.20, 0.06)),
        (8.0,  (1.90, -1.70, 0.95), (0.30, 0.20, 0.06)),
        (16.0, (2.60, 0.90, 1.10), (0.30, 0.20, 0.06)),
    ],
    # Plan view of the whole training ground: the parallel-envs layout is
    # unmistakable, useful as a "this is an RL rig" diagram frame.
    "topdown": [
        (0.0,  (1.90, 1.35, 9.00), (1.90, 1.95, 0.03)),
        (16.0, (1.90, 1.35, 9.00), (1.90, 1.95, 0.03)),
    ],
}


def _smoothstep(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def camera_at(keys: list, t: float) -> tuple[tuple, tuple]:
    """Smoothstep-interpolated (eye, lookat) for time ``t``."""
    if t <= keys[0][0]:
        return keys[0][1], keys[0][2]
    if t >= keys[-1][0]:
        return keys[-1][1], keys[-1][2]
    for (t0, eye0, look0), (t1, eye1, look1) in zip(keys, keys[1:]):
        if t0 <= t <= t1:
            u = _smoothstep((t - t0) / max(t1 - t0, 1e-6))
            eye = tuple(a + (b - a) * u for a, b in zip(eye0, eye1))
            look = tuple(a + (b - a) * u for a, b in zip(look0, look1))
            return eye, look
    return keys[-1][1], keys[-1][2]


def hero_command(t: float) -> tuple[float, float]:
    """Joystick for the hero robot: hold, drive off the ledge, hold again."""
    if args_cli.drive_start <= t < args_cli.drive_stop:
        return args_cli.drive_speed, 0.0
    return 0.0, 0.0


def ambient_command(i: int, t: float) -> tuple[float, float]:
    """Small bounded sway/turn so the other robots read as actively training.

    Zero-mean sinusoids with per-robot phase: they stay on their decks (the
    station-keeping reward pulls them back to their spawn point) but never look
    like frozen props.
    """
    ph = 0.9 * i
    v = 0.075 * math.sin(2.0 * math.pi * t / 6.0 + ph)
    w = 0.30 * math.sin(2.0 * math.pi * t / 4.5 + 1.7 * ph)
    return v, w


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.ledge_height_m = args_cli.ledge
    env_cfg.viewer.resolution = tuple(args_cli.resolution)
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    out_dir: Path = args_cli.out
    out_dir.mkdir(parents=True, exist_ok=True)
    policy = torch.jit.load(str(args_cli.policy), map_location=env_cfg.sim.device).eval()

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array")
    if isinstance(env.unwrapped, DirectMARLEnv):
        raise RuntimeError("multi-agent env is not supported by this recorder")
    unwrapped = env.unwrapped
    dt = unwrapped.step_dt
    fps = int(round(1.0 / dt))
    steps = int(round(args_cli.seconds / dt))
    keys = SHOTS[args_cli.shot]
    still_times = sorted(args_cli.stills) if args_cli.stills else []
    still_steps = {int(round(t / dt)): t for t in still_times}
    slowmo_range = None
    if args_cli.slowmo is not None:
        slowmo_range = (int(round(args_cli.slowmo[0] / dt)), int(round(args_cli.slowmo[1] / dt)))

    print(
        f"[record] shot={args_cli.shot} ledge={args_cli.ledge * 100:.0f}cm "
        f"{steps} steps @ {fps} fps -> {args_cli.seconds:.1f}s, "
        f"resolution={tuple(args_cli.resolution)}, pads={args_cli.num_envs}"
    )

    writer = None
    slowmo_writer = None
    if not args_cli.no_video:
        import imageio.v2 as imageio

        video_path = out_dir / f"{args_cli.tag}_{args_cli.shot}.mp4"
        writer = imageio.get_writer(
            str(video_path), fps=fps, codec="libx264", quality=9, macro_block_size=None,
            ffmpeg_params=["-pix_fmt", "yuv420p"],
        )
        if slowmo_range is not None:
            slowmo_writer = imageio.get_writer(
                str(out_dir / f"{args_cli.tag}_{args_cli.shot}_slowmo.mp4"),
                fps=args_cli.slowmo_fps, codec="libx264", quality=9, macro_block_size=None,
                ffmpeg_params=["-pix_fmt", "yuv420p"],
            )

    trace: list[dict] = []
    with torch.inference_mode():
        obs, _ = env.reset()
        if isinstance(obs, dict):
            obs = obs["policy"]
        # Frame the opening keyframe before the first render so frame 0 is
        # already composed.
        eye, look = camera_at(keys, 0.0)
        unwrapped.sim.set_camera_view(eye, look)

        for step in range(steps):
            t = step * dt
            v_hero, w_hero = hero_command(t)
            unwrapped.demo_v_target[0] = v_hero
            unwrapped.demo_w_target[0] = w_hero
            for i in range(1, unwrapped.num_envs):
                v_i, w_i = ambient_command(i, t)
                unwrapped.demo_v_target[i] = v_i
                unwrapped.demo_w_target[i] = w_i

            actions = policy(obs)
            step_out = env.step(actions)
            obs = step_out[0]
            if isinstance(obs, dict):
                obs = obs["policy"]
            terminated = step_out[2]

            eye, look = camera_at(keys, t + dt)
            unwrapped.sim.set_camera_view(eye, look)
            frame = unwrapped.render()

            if frame is not None:
                if writer is not None:
                    writer.append_data(frame)
                if slowmo_writer is not None and slowmo_range[0] <= step <= slowmo_range[1]:
                    slowmo_writer.append_data(frame)
                if step in still_steps:
                    from PIL import Image

                    stamp = f"{still_steps[step]:05.2f}".replace(".", "_")
                    name = f"{args_cli.tag}_{args_cli.shot}_t{stamp}s.png"
                    Image.fromarray(frame).save(out_dir / name)
                    print(f"[record] still saved: {name}")
                    # Same instant, other camera angles: park the camera on each
                    # extra preset, re-render, then restore the shot camera.
                    for preset in args_cli.still_shots or []:
                        p_eye, p_look = camera_at(SHOTS[preset], t)
                        unwrapped.sim.set_camera_view(p_eye, p_look)
                        extra = unwrapped.render()
                        if extra is not None:
                            extra_name = f"{args_cli.tag}_{preset}_t{stamp}s.png"
                            Image.fromarray(extra).save(out_dir / extra_name)
                            print(f"[record] still saved: {extra_name}")
                    if args_cli.still_shots:
                        unwrapped.sim.set_camera_view(eye, look)

            # Hero telemetry: this is the evidence that the drop was survived.
            root = unwrapped.robot.data.root_pos_w[0]
            x_rel, velocity, pitch, _, _, _ = unwrapped._state_terms()
            roll = roll_from_projected_gravity(unwrapped.bno080.data.projected_gravity_b)
            trace.append(
                {
                    "t_s": round(t, 4),
                    "x_m": float(root[0]),
                    "y_m": float(root[1]),
                    "z_m": float(root[2]),
                    "pitch_deg": float(pitch[0]) * 180.0 / math.pi,
                    "roll_deg": float(roll[0]) * 180.0 / math.pi,
                    "velocity_mps": float(velocity[0]),
                    "v_joystick_mps": v_hero,
                    # The slewed command the policy actually sees, and the
                    # tracking error it is being asked to close.
                    "v_cmd_mps": float(unwrapped._commands.v_cmd[0]),
                    "pos_err_m": float(unwrapped._commands.position_error_raw(x_rel)[0]),
                    "terminated": int(bool(terminated.view(-1)[0])),
                }
            )

    if writer is not None:
        writer.close()
        print(f"[record] video: {out_dir / f'{args_cli.tag}_{args_cli.shot}.mp4'}")
    if slowmo_writer is not None:
        slowmo_writer.close()
        print(f"[record] slow-motion: {out_dir / f'{args_cli.tag}_{args_cli.shot}_slowmo.mp4'}")

    csv_path = out_dir / f"{args_cli.tag}_{args_cli.shot}_hero_trace.csv"
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(trace[0].keys()))
        w.writeheader()
        w.writerows(trace)
    _report(trace, csv_path)
    env.close()


def _report(trace: list[dict], csv_path: Path) -> None:
    """Summarise the hero run: did it actually survive the drop?"""
    t = np.array([r["t_s"] for r in trace])
    z = np.array([r["z_m"] for r in trace])
    x = np.array([r["x_m"] for r in trace])
    pitch = np.array([r["pitch_deg"] for r in trace])
    roll = np.array([r["roll_deg"] for r in trace])
    term = np.array([r["terminated"] for r in trace])

    # The drop is the steepest sustained fall in platform height.
    drop_i = int(np.argmin(np.diff(z))) if len(z) > 1 else 0
    drop_t = float(t[drop_i])
    after = slice(drop_i, len(t))
    peak_pitch_i = drop_i + int(np.argmax(np.abs(pitch[after])))
    settle_t = None
    for i in range(peak_pitch_i, len(t)):
        window = slice(i, min(i + 50, len(t)))  # 1 s of control steps
        if np.all(np.abs(pitch[window]) < 10.0) and np.all(np.abs(roll[window]) < 10.0):
            settle_t = float(t[i])
            break

    tail = slice(max(len(t) - 100, 0), len(t))  # last 2 s
    print("\n================ hero robot: ledge drop ================")
    print(f"drop at t={drop_t:.2f} s, platform z {z[max(drop_i - 5, 0)]:.3f} -> {z.min():.3f} m")
    print(f"peak |pitch| after drop: {np.abs(pitch[after]).max():.1f} deg at t={t[peak_pitch_i]:.2f} s")
    print(f"peak |roll|  after drop: {np.abs(roll[after]).max():.1f} deg")
    if settle_t is not None:
        print(f"recovered (|pitch|,|roll| < 10 deg held 1 s) at t={settle_t:.2f} s"
              f"  -> {settle_t - drop_t:.2f} s after the drop")
    else:
        print("NOT recovered within the clip (pitch/roll never held under 10 deg for 1 s)")
    print(f"final 2 s: rms pitch {np.sqrt((pitch[tail] ** 2).mean()):.2f} deg, "
          f"rms roll {np.sqrt((roll[tail] ** 2).mean()):.2f} deg")
    print(f"travelled x: {x[0]:.3f} -> {x[-1]:.3f} m (drift over final 2 s: {x[-1] - x[tail.start]:+.3f} m)")
    print(f"terminations during clip: {int(term.sum())}")
    print(f"survived: {'YES' if term.sum() == 0 and settle_t is not None else 'NO'}")
    print(f"trace: {csv_path}")
    print("=======================================================\n")


if __name__ == "__main__":
    main()
    simulation_app.close()
