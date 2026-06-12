# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Extract physical parameters for the analytical two-wheeled robot LQR model.

The script spawns the same robot configuration used by the Isaac Lab task,
reads the resolved PhysX mass properties, and writes:

    outputs/robot_physical_parameters.csv

Run from the repository root with the same Python launcher used for Isaac Lab
scripts, for example:

    python scripts/tools/extract_robot_physical_parameters.py --headless
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_EXTENSION_SOURCE_PATH = _REPO_ROOT / "source" / "TwoWheeledRobot"
if _EXTENSION_SOURCE_PATH.is_dir():
    sys.path.insert(0, str(_EXTENSION_SOURCE_PATH))

from isaaclab.app import AppLauncher

# These are the rotating DDSM115 wheel/motor rigid bodies resolved by PhysX.
# DDSM115_Levi and DDSM115_Desni are the corresponding joint names.
WHEEL_LINK_NAMES = ["DDSM115_Simplified", "DDSM115_Simplified_01"]

# At the upright zero-joint pose in this USD, the wheels are separated along X,
# the robot moves along Y, Z is vertical, and the wheel axle/pitch axis is X.
WHEEL_SPIN_AXIS_WORLD = (1.0, 0.0, 0.0)

parser = argparse.ArgumentParser(description="Extract robot mass properties for 4-state/6-state LQR models.")
parser.add_argument("--task", type=str, default="Template-Twowheeledrobot-Standup-v0")
parser.add_argument("--output", type=str, default="outputs/robot_physical_parameters.csv")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.num_envs = 1
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Everything below runs after Isaac Sim has launched."""

import csv
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import torch
import TwoWheeledRobot.tasks  # noqa: F401
from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg
import isaaclab_tasks  # noqa: F401
from isaaclab.utils import math as math_utils
from isaaclab_tasks.utils.hydra import hydra_task_config
from pxr import Usd, UsdGeom


@dataclass(frozen=True)
class LinkProperties:
    name: str
    prim_path: str
    mass_kg: float
    com_world_m: torch.Tensor
    origin_world_m: torch.Tensor
    inertia_world_kg_m2: torch.Tensor


def _force_upright_zero_pose(env: Any) -> None:
    """Make the StandupEnv reset use an upright reference pose."""

    def upright_pose(count: int) -> tuple[torch.Tensor, torch.Tensor]:
        quat = torch.zeros(count, 4, device=env.device)
        quat[:, 0] = 1.0
        height = torch.full((count,), env.cfg.robot_cfg.init_state.pos[2], device=env.device)
        return quat, height

    if not hasattr(env, "_sample_fallen_poses"):
        raise RuntimeError("The selected task does not expose the StandupEnv reset pose sampler.")
    env._sample_fallen_poses = upright_pose


def _read_link_properties(robot: Any) -> list[LinkProperties]:
    """Read PhysX mass properties and express all inertia tensors in world axes."""
    masses = robot.root_physx_view.get_masses()[0].to(robot.device)
    inertias_com = robot.root_physx_view.get_inertias()[0].to(robot.device).reshape(-1, 3, 3)
    com_pos_w = robot.data.body_com_pos_w[0]
    com_quat_w = robot.data.body_com_quat_w[0]
    origin_pos_w = robot.data.body_link_pos_w[0]
    rotations_w = math_utils.matrix_from_quat(com_quat_w)
    inertias_w = rotations_w @ inertias_com @ rotations_w.transpose(-1, -2)
    prim_paths = robot.root_physx_view.link_paths[0]

    return [
        LinkProperties(
            name=name,
            prim_path=str(prim_paths[index]),
            mass_kg=float(masses[index].item()),
            com_world_m=com_pos_w[index].detach().cpu().double(),
            origin_world_m=origin_pos_w[index].detach().cpu().double(),
            inertia_world_kg_m2=inertias_w[index].detach().cpu().double(),
        )
        for index, name in enumerate(robot.body_names)
    ]


def _combine_body_properties(body_links: list[LinkProperties]) -> tuple[float, torch.Tensor, torch.Tensor]:
    """Combine body links at their shared COM using the parallel-axis theorem."""
    total_mass = sum(link.mass_kg for link in body_links)
    if total_mass <= 0.0:
        raise RuntimeError("Combined non-wheel body mass is not positive.")
    body_com = sum((link.mass_kg * link.com_world_m for link in body_links), start=torch.zeros(3)) / total_mass
    identity = torch.eye(3, dtype=torch.float64)
    total_inertia = torch.zeros((3, 3), dtype=torch.float64)
    for link in body_links:
        displacement = link.com_world_m - body_com
        parallel_axis = link.mass_kg * (
            torch.dot(displacement, displacement) * identity - torch.outer(displacement, displacement)
        )
        total_inertia += link.inertia_world_kg_m2 + parallel_axis
    return total_mass, body_com, total_inertia


def _wheel_radius_from_geometry(stage: Usd.Stage, wheel_links: list[LinkProperties]) -> tuple[float | None, str]:
    """Estimate wheel radius from bounds perpendicular to the configured spin axis."""
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    spin_axis_index = max(range(3), key=lambda index: abs(WHEEL_SPIN_AXIS_WORLD[index]))
    radial_axis_indices = [index for index in range(3) if index != spin_axis_index]
    axis_names = "XYZ"
    radial_axis_names = "/".join(axis_names[index] for index in radial_axis_indices)
    radii: list[float] = []
    details: list[str] = []
    for link in wheel_links:
        prim = stage.GetPrimAtPath(link.prim_path)
        if not prim or not prim.IsValid():
            return None, f"manual required: wheel prim not found at {link.prim_path}"
        size = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetSize()
        radial_sizes = [0.5 * float(size[index]) for index in radial_axis_indices]
        radii.append(sum(radial_sizes) / len(radial_sizes))
        details.append(f"{link.name} {radial_axis_names} radii={radial_sizes[0]:.9g}/{radial_sizes[1]:.9g} m")
        if max(radial_sizes) <= 0.0 or abs(radial_sizes[0] - radial_sizes[1]) / max(radial_sizes) > 0.05:
            return None, f"manual required: wheel {radial_axis_names} bounds are not circular; " + "; ".join(details)
    return sum(radii) / len(radii), f"from spawned wheel rigid-body {radial_axis_names} bounds; " + "; ".join(details)


def _append_row(rows: list[dict[str, str]], section: str, name: str, value: float | str, unit: str, notes: str) -> None:
    rows.append(
        {
            "section": section,
            "name": name,
            "value": f"{value:.12g}" if isinstance(value, float) else value,
            "unit": unit,
            "notes": notes,
        }
    )


def _build_csv_rows(links: list[LinkProperties], stage: Usd.Stage) -> list[dict[str, str]]:
    wheel_names = set(WHEEL_LINK_NAMES)
    available_names = {link.name for link in links}
    missing_names = wheel_names - available_names
    if missing_names:
        available = "\n  ".join(sorted(available_names))
        missing = ", ".join(sorted(missing_names))
        raise RuntimeError(f"Configured wheel link(s) not found: {missing}\nAvailable rigid body names:\n  {available}")

    wheel_links = [link for link in links if link.name in wheel_names]
    body_links = [link for link in links if link.name not in wheel_names]
    body_mass, body_com, body_inertia = _combine_body_properties(body_links)
    axle_center = sum((link.origin_world_m for link in wheel_links), start=torch.zeros(3)) / len(wheel_links)
    axis = torch.tensor(WHEEL_SPIN_AXIS_WORLD, dtype=torch.float64)
    axis /= torch.linalg.vector_norm(axis)
    yaw_axis = torch.tensor((0.0, 0.0, 1.0), dtype=torch.float64)
    wheel_spin_inertias = [float(axis @ link.inertia_world_kg_m2 @ axis) for link in wheel_links]
    track_width = float(torch.linalg.vector_norm(wheel_links[0].origin_world_m - wheel_links[1].origin_world_m))
    body_yaw_inertia = float(yaw_axis @ body_inertia @ yaw_axis)
    wheel_radius, wheel_radius_notes = _wheel_radius_from_geometry(stage, wheel_links)

    rows: list[dict[str, str]] = []
    for link in links:
        category = "wheel rotating body" if link.name in wheel_names else "non-wheel body link"
        prefix = link.name
        _append_row(rows, "link", f"{prefix}.mass_kg", link.mass_kg, "kg", category)
        for index, axis_name in enumerate("xyz"):
            _append_row(rows, "link", f"{prefix}.center_of_mass_{axis_name}", float(link.com_world_m[index]), "m", f"{category}; world frame")
        inertia = link.inertia_world_kg_m2
        for name, row, column in (("xx", 0, 0), ("yy", 1, 1), ("zz", 2, 2), ("xy", 0, 1), ("xz", 0, 2), ("yz", 1, 2)):
            _append_row(rows, "link", f"{prefix}.inertia_{name}", float(inertia[row, column]), "kg*m^2", f"{category}; about link COM in world axes")
        for index, axis_name in enumerate("xyz"):
            _append_row(rows, "link", f"{prefix}.world_position_{axis_name}", float(link.origin_world_m[index]), "m", f"{category}; rigid-body frame origin")

    wheel_mass_average = sum(link.mass_kg for link in wheel_links) / len(wheel_links)
    combined_values = [
        ("body_mass_excluding_wheels_kg", body_mass, "kg", "all non-wheel links"),
        ("one_wheel_mass_kg", wheel_mass_average, "kg", "average of left/right rotating wheel bodies"),
        ("total_body_com_x", float(body_com[0]), "m", "non-wheel body COM; world frame"),
        ("total_body_com_y", float(body_com[1]), "m", "non-wheel body COM; world frame"),
        ("total_body_com_z", float(body_com[2]), "m", "non-wheel body COM; world frame"),
        ("wheel_axle_center_x", float(axle_center[0]), "m", "average wheel rigid-body origin; world frame"),
        ("wheel_axle_center_y", float(axle_center[1]), "m", "average wheel rigid-body origin; world frame"),
        ("wheel_axle_center_z", float(axle_center[2]), "m", "average wheel rigid-body origin; world frame"),
        ("com_height_above_wheel_axle_m", float(body_com[2] - axle_center[2]), "m", "LQR l parameter; world Z"),
        ("body_pitch_inertia_kg_m2", float(axis @ body_inertia @ axis), "kg*m^2", "I_b about wheel axle/pitch axis world X at combined non-wheel COM"),
        ("track_width_m", track_width, "m", "left/right wheel rigid-body origin distance; yaw moment arm uses half this value"),
        ("body_yaw_inertia_kg_m2", body_yaw_inertia, "kg*m^2", "I_z about vertical world Z at combined non-wheel COM"),
        (
            "wheel_rotational_inertia_kg_m2",
            sum(wheel_spin_inertias) / len(wheel_spin_inertias),
            "kg*m^2",
            "I_w average about wheel spin axis world X; individual values="
            + ", ".join(f"{link.name}:{value:.12g}" for link, value in zip(wheel_links, wheel_spin_inertias, strict=True)),
        ),
    ]
    for name, value, unit, notes in combined_values:
        _append_row(rows, "combined", name, value, unit, notes)
    _append_row(
        rows,
        "combined",
        "wheel_radius_m",
        wheel_radius if wheel_radius is not None else "manual_required",
        "m",
        wheel_radius_notes,
    )
    return rows


def _write_csv(rows: list[dict[str, str]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=["section", "name", "value", "unit", "notes"])
        writer.writeheader()
        writer.writerows(rows)


def _print_rows(rows: list[dict[str, str]]) -> None:
    for row in rows:
        print(f"{row['section']:8s} {row['name']:55s} {row['value']:>16s} {row['unit']:8s} {row['notes']}")


@hydra_task_config(args_cli.task, None)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, _agent_cfg: Any | None):
    env_cfg.scene.num_envs = 1
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    env = gym.make(args_cli.task, cfg=env_cfg)
    try:
        unwrapped = env.unwrapped
        _force_upright_zero_pose(unwrapped)
        env.reset()
        unwrapped.robot.update(0.0)

        links = _read_link_properties(unwrapped.robot)
        print(f"[INFO]: Wheel links: {', '.join(WHEEL_LINK_NAMES)}")
        print("[INFO]: Body links: " + ", ".join(link.name for link in links if link.name not in WHEEL_LINK_NAMES))

        stage = unwrapped.scene.stage
        rows = _build_csv_rows(links, stage)
        output_path = Path(args_cli.output)
        if not output_path.is_absolute():
            output_path = _REPO_ROOT / output_path
        _write_csv(rows, output_path)
        _print_rows(rows)
        print(f"[INFO]: Wrote {len(rows)} rows to {output_path}")
    finally:
        env.close()


if __name__ == "__main__":
    main()  # type: ignore[reportCallIssue]
    simulation_app.close()
